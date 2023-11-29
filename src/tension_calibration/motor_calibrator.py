#############################################################################
#                           The Motor Calibrator Class.                     #
# This class implement a calibration algorithm to tune the motors           #
# of a cable-driven continuum soft arm.                                     #              
# This algorithm try to be agnostic on motors' type/model, using            #
# the modularity of ROS. The output of the class is a timeseries of         #
# ROS topic (std_msgs/Float32MultiArray) of the n° of turns.                #
#If the n° of turns is equal to -1, it does means to turn off the motor.    #
#############################################################################

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

import os
import csv
import pandas as pd

### Global Variables ###
proboscis_ns            = "/proboscis"
turns_topic_name        = proboscis_ns + "/cmd_turns"
current_topic_name      = proboscis_ns + "/read_currents"
class_ns                = "/motor_calibrator"
QUEUE_SIZE              = 10
DISABLE_TORQUE_REQUEST  = -1
NODE_FREQUENCY          = rospy.get_param(class_ns + "/node_frequency")    # [Hz]
SLEEP_TIME              = rospy.get_param(class_ns + "/sleep_time")
MOTOR_INNER_TIME        = 0.5                                               #[s]
FLOATING_NUMBER         = 2
FLOAT_TOLERANCE         = 1e-2
PACKAGE_PATH            = os.path.expanduser('~') + "/catkin_ws/src/tension_calibration"
CURRENT_CSV_FILENAME    = "/data/current_position.csv"
DYNAMIXEL_FREQ          = 30.0                                              #[Hz]
CURRENT_DATA_SKIP       = rospy.get_param(class_ns + "/current_data_skip")

### Class Definition ###
class Motor_Calibrator:
    def __init__(self, n_motors):
        # Subscriber/Publisher
        self.pub_obj = rospy.Publisher(turns_topic_name, Float32MultiArray, queue_size=QUEUE_SIZE)
        self.sub_obj = rospy.Subscriber(current_topic_name, Float32MultiArray, self.currents_callback)

        # Shutdown Callback
        rospy.on_shutdown(self.shutdown_callback)
        
        # Init n° of motors
        self.n_motors = n_motors

        # Calibration Parameters
        self.max_turns = rospy.get_param(class_ns + "/max_turns")
        self.turns_trial = rospy.get_param(class_ns + "/turns_trial")
        self.turns_resolution = np.round(self.max_turns/self.turns_trial, 3)

        # Currents storing timeseries
        self.read_currents = Float32MultiArray()

        # Create csv file to store currents
        self.fieldnames = ['Current']*self.n_motors
        for i in range(len(self.fieldnames)):
            self.fieldnames[i] += str(i + 1)
        
        self.init_dataset()

        # Init time to skip current data
        self.data_counter = 0

        # Actual Commanded Turns
        self.cmd_turns = Float32MultiArray()
        self.cmd_turns.data = [DISABLE_TORQUE_REQUEST]*self.n_motors    # Init to zeros
        self.publish_turns()

        # To Calibrate Motors Queue
        self.uncalibrated_motors = [*range(self.n_motors)]
        self.calibrated_motors = []

        # draw motor from the queue
        self.draw_motor_from_queue()

        # Start the Main Loop | Removing for now
        # self.timer_obj = rospy.Timer(rospy.Duration(1/NODE_FREQUENCY), self.main_loop)
    
    def init_dataset(self):        
        with open(PACKAGE_PATH + CURRENT_CSV_FILENAME, mode='w', newline='') as file:            
            # Create writer obj
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)

            # Create the dataset
            writer.writeheader()

        # Close the file
        file.close()

    def current2csv(self):
        new_data = {}

        # Create Dictionary
        for i in range(len(self.fieldnames)):
            new_data[self.fieldnames[i]] = self.read_currents.data[i]
       
        new_df = pd.DataFrame(new_data.items())

        # Append the new row to the existing csv file
        # new_df.to_csv(PACKAGE_PATH + CURRENT_CSV_FILENAME, mode='a', index=False, header=False)        

    def currents_callback(self, msg):
        # Increment data counter
        self.data_counter += 1

        # Wait the CURRENT_DATA_SKIP-th sample
        if(self.data_counter >  CURRENT_DATA_SKIP):
            # Storing currents in the private attribute
            self.read_currents = msg
            self.current2csv()
            self.data_counter = 0   # reset to zero the counter

            # Start new position
            self.state_machine()

    def shutdown_callback(self):
        rospy.logwarn("Calibration terminated. Killing the node and turn off the motors.")
        self.cmd_turns.data = [DISABLE_TORQUE_REQUEST]*self.n_motors
        self.publish_turns()

    def calibration_single_motor(self, motor_id):
        for i in range(self.n_motors):
            # Compute the commanded turns for the motor_id-th motor
            if (i == motor_id):
                if (self.cmd_turns.data[i] >= 0) and (np.round(self.cmd_turns.data[i], FLOATING_NUMBER) < self.max_turns):  # [0, MAX_TURNS]
                    self.cmd_turns.data[i] += self.turns_resolution
                
                elif (self.cmd_turns.data[i] == DISABLE_TORQUE_REQUEST):
                    self.cmd_turns.data[i] = 0.0
                
                elif (np.round(self.cmd_turns.data[i], FLOATING_NUMBER) == np.round(self.max_turns, FLOATING_NUMBER)):
                    rospy.loginfo("Maximum turns reached (%f) of the motor %d", self.cmd_turns.data[i], i)

                    # Turn off all the motors & publish
                    # First initial position
                    self.cmd_turns.data[i] = 0.0
                    self.publish_turns()
                    rospy.sleep(1/NODE_FREQUENCY)

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
        if len(self.uncalibrated_motors) != 0:
            rand_idx = np.random.randint(len(self.uncalibrated_motors))
            # Add at the bottom the result
            self.calibrated_motors.append(self.uncalibrated_motors[rand_idx])
            # Then remove from the queue the motor
            self.uncalibrated_motors.pop(rand_idx)
        else:
            rospy.signal_shutdown("Node terminated.")

    def publish_turns(self):
        self.pub_obj.publish(self.cmd_turns)

    def state_machine(self):
        if (len(self.calibrated_motors) != 0):
            self.calibration_single_motor(self.calibrated_motors[-1])
            self.publish_turns()
        else:
            pass

    # def main_loop(self, event):
    #     self.state_machine()