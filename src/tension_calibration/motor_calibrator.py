#############################################################################
#                           The Motor Calibrator Class.                     #
# This class implement a calibration algorithm to tune the motors           #
# of a cable-driven continuum soft arm.                                     #              
# This algorithm try to be agnostic on motors' type/model, using            #
# the modularity of ROS. The output of the class is a timeseries of         #
# ROS topic (std_msgs/Float32MultiArray) of the n° of turns.                #
# If the n° of turns is equal to -1, it does means to turn off the motor.   #
#############################################################################

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

import os
import matplotlib.pyplot as plt

### Global Variables ###
proboscis_ns            = "/proboscis"
turns_topic_name        = proboscis_ns + "/cmd_turns"
current_topic_name      = proboscis_ns + "/read_currents"
class_ns                = "/motor_calibrator"
QUEUE_SIZE              = 1
DISABLE_TORQUE_REQUEST  = -1
NODE_FREQUENCY          = rospy.get_param(class_ns + "/node_frequency")    # [Hz]
SLEEP_TIME              = rospy.get_param(class_ns + "/sleep_time")        # [s]
FLOATING_NUMBER         = 2
PACKAGE_PATH            = os.path.expanduser('~') + "/catkin_ws/src/proboscis_full/tension_calibration"
CURRENT_CSV_FILENAME    = "/data/current_turns.csv"
CURRENT_DATA_SKIP       = rospy.get_param(class_ns + "/current_data_skip")
BROKEN_MOTOR_ID         = 5
MEAN_SAMPLES            = rospy.get_param(class_ns + "/mean_sample")

"""
Idea of the algorithm: 
The ROS Node is structured in 2 parts:
- 1. Callback that save the raw data in a .csv file;
- 2. Main Loop that periodically publish the commanded turns, using a state machine function.

After that, a shutdown_callback will be called, analyzing the csv file. [UNDER DEVELOP]
"""

### Class Definition ###
class Motor_Calibrator:
    def __init__(self, n_motors):
        # Publisher
        self.pub_obj = rospy.Publisher(turns_topic_name, Float32MultiArray, queue_size=QUEUE_SIZE)
        # Shutdown Callback
        rospy.on_shutdown(self.shutdown_callback)
        
        # Init n° of motors
        self.n_motors = n_motors

        # Calibration Parameters
        self.max_turns = rospy.get_param(class_ns + "/max_turns")
        self.turns_trial = rospy.get_param(class_ns + "/turns_trial")
        self.turns_resolution = np.round(self.max_turns/self.turns_trial, 3)

        # Init time to skip current data
        self.data_counter = 0
        
        # To Calibrate Motors Queue
        self.uncalibrated_motors = [*range(self.n_motors)]
        # Until Motor 5 is fixed
        self.uncalibrated_motors.remove(BROKEN_MOTOR_ID - 1)
        self.calibrated_motors = []

        # Actual Commanded Turns
        self.cmd_turns = Float32MultiArray()
        self.cmd_turns.data = [DISABLE_TORQUE_REQUEST]*self.n_motors    # Init to zeros
        self.publish_turns()
        rospy.sleep(1/NODE_FREQUENCY)
        
        # List to collect current data and compute average value
        self.current_data = []

        # draw motor from the queue
        self.draw_motor_from_queue()

        # Publish and Subscribe request
        self.pub_req = False
        self.sub_req = True

        ## Define Callbacks
        # Subscriber
        self.sub_obj = rospy.Subscriber(current_topic_name, Float32MultiArray, self.currents_callback)
        # Main Loop
        self.timer_obj = rospy.Timer(rospy.Duration(1/NODE_FREQUENCY), self.main_loop)

    def currents_callback(self, msg):
        if self.sub_req:
            # Increment data counter
            self.data_counter += 1
            self.current_data.append(list(msg.data) + self.cmd_turns.data)

            # Wait for the (MEAN_SAMPLES + CURRENT_DATA_SKIP)-sample
            if(self.data_counter > MEAN_SAMPLES + CURRENT_DATA_SKIP):
                # reset to zero the counter & list
                self.data_counter = 0
                # Wait for the publishing
                self.sub_req = False
                self.pub_req = True

    def main_loop(self, event):
        # Next step only if there are enough samples
        if self.pub_req:
            self.state_machine()
            self.sub_req = True
            self.pub_req = False

    def plot_data(self, np_array):
        # Only for Debug
        plt.figure()

        for i in range(self.n_motors):
            plt.plot(range(len(np_array)), np_array[:, i], label=f'Motor {i+1}')
        
        # Figure Properties
        plt.title('Currents')
        plt.xlabel('Indeces')
        plt.ylabel('Current [A]')
        plt.legend()
        plt.grid(True)
        plt.show()

    def save2csv(self, np_array):
        file_path = PACKAGE_PATH + CURRENT_CSV_FILENAME
        # Save the NumPy array to a CSV file
        np.savetxt(file_path, np_array, delimiter=',')

    def compute_lines(self):
        pass

    def shutdown_callback(self):
        # Turn OFF Motors
        rospy.logwarn("Calibration terminated. Killing the node and turn off the motors.")
        self.cmd_turns.data = [DISABLE_TORQUE_REQUEST]*self.n_motors
        self.publish_turns()

        ## Debug: Plot timeseries ##
        # Convert in numpy array
        current_ds = np.array(self.current_data)
        self.save2csv(current_ds)
        self.plot_data(current_ds)

    def calibration_single_motor(self, motor_id):
        for i in range(self.n_motors):
            # Compute the commanded turns for the motor_id-th motor
            if (i == motor_id):
                if (self.cmd_turns.data[i] >= 0) and (np.round(self.cmd_turns.data[i], FLOATING_NUMBER) < self.max_turns):  # [0, MAX_TURNS]
                    self.cmd_turns.data[i] += self.turns_resolution
                
                elif (self.cmd_turns.data[i] == DISABLE_TORQUE_REQUEST):
                    self.cmd_turns.data[i] = 0.0
                
                elif (np.round(self.cmd_turns.data[i], FLOATING_NUMBER) == np.round(self.max_turns, FLOATING_NUMBER)):
                    rospy.loginfo("Maximum turns reached (%f) of the motor %d", self.cmd_turns.data[i], i + 1)

                    # Turn off all the motors & publish
                    # First initial position
                    self.cmd_turns.data[i] = 0.0
                    self.publish_turns()
                    rospy.sleep(SLEEP_TIME)

                    # Then disable torque request
                    self.cmd_turns.data[i] = DISABLE_TORQUE_REQUEST
                    self.publish_turns()
                    # wait...
                    rospy.loginfo("Wait for the rest configuration of the robot...")
                    rospy.sleep(SLEEP_TIME)

                    # Then draw next motor from the queue
                    rospy.loginfo("Sleep phase ended. Switch to the next motor.")
                    self.draw_motor_from_queue()
                
                else:
                    rospy.logerr("Invalid value of cmd_turn = %f",  self.cmd_turns.data[i])
                    break

            # Request Torque Disable for the other motors
            else:
                self.cmd_turns.data[i] = DISABLE_TORQUE_REQUEST
    
    def draw_motor_from_queue(self):
        # if there are uncalibrated motors
        if self.uncalibrated_motors:
            rand_idx = np.random.randint(len(self.uncalibrated_motors))
            # Add at the bottom the result
            self.calibrated_motors.append(self.uncalibrated_motors[rand_idx])
            # Then remove from the queue the motor
            self.uncalibrated_motors.pop(rand_idx)
        else:
            rospy.signal_shutdown("Node terminated.")
            pass 

    def publish_turns(self):
        self.pub_obj.publish(self.cmd_turns)

    def state_machine(self):
        if self.calibrated_motors:
            self.calibration_single_motor(self.calibrated_motors[-1])
            self.publish_turns()