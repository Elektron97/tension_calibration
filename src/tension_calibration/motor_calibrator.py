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

### Global Variables ###
turns_topic_name        = "/cmd_turns"
current_topic_name      = "/read_currents"
class_ns                = "/motor_calibrator"
QUEUE_SIZE              = 10
DISABLE_TORQUE_REQUEST  = -1
NODE_FREQUENCY          = rospy.get_param(class_ns + "/node_frequency")    # [Hz]
SLEEP_TIME              = rospy.get_param(class_ns + "/sleep_time")
MOTOR_INNER_TIME        = 0.1                                               #[s]

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
        self.turns_resolution = self.max_turns/self.turns_trial

        # Currents storing timeseries
        self.read_currents = Float32MultiArray()

        # Actual Commanded Turns
        self.cmd_turns = Float32MultiArray()
        self.cmd_turns.data = [DISABLE_TORQUE_REQUEST]*self.n_motors    # Init to zeros
        self.publish_turns()

        # To Calibrate Motors Queue
        self.uncalibrated_motors = [*range(self.n_motors)]
        self.calibrated_motors = []

        # draw motor from the queue
        self.draw_motor_from_queue()

        # Start the Main Loop
        self.timer_obj = rospy.Timer(rospy.Duration(1/NODE_FREQUENCY), self.main_loop)
    
    def currents_callback(self, msg):
        # Storing currents in the private attribute
        self.read_currents = msg
    
    def shutdown_callback(self):
        rospy.logwarn("Calibration terminated. Killing the node and turn off the motors.")
        self.cmd_turns.data = [DISABLE_TORQUE_REQUEST]*self.n_motors
        self.publish_turns()

    def calibration_single_motor(self, motor_id):
        for i in range(self.n_motors):
            # Compute the commanded turns for the motor_id-th motor
            if (i == motor_id):
                if (self.cmd_turns.data[i] >= 0) and (self.cmd_turns.data[i] < self.max_turns):  # [0, MAX_TURNS]
                    self.cmd_turns.data[i] += self.turns_resolution
                
                elif (self.cmd_turns.data[i] == DISABLE_TORQUE_REQUEST):
                    self.cmd_turns.data[i] = 0.0
                
                elif (self.cmd_turns.data[i] == self.max_turns):
                    rospy.loginfo("Maximum turns reached (%f) of the motor %d", self.cmd_turns.data[i], i)

                    # Turn off all the motors & publish
                    # First initial position
                    self.cmd_turns.data = [0.0]*self.n_motors
                    self.publish_turns()
                    rospy.sleep(MOTOR_INNER_TIME)   # TODO: to tune

                    # Then disable torque request
                    self.cmd_turns.data = [DISABLE_TORQUE_REQUEST]*self.n_motors
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

    def main_loop(self, event):
        if (len(self.calibrated_motors) != 0):
            self.calibration_single_motor(self.calibrated_motors[-1])
            self.publish_turns()
        else:
            pass