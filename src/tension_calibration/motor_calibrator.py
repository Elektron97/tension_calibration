#############################################################################
#                           The Motor Calibrator Class.                     #
# This class implement a calibration algorithm to tune the motors           #
# of a cable-driven continuum soft arm.                                     #              
# This algorithm try to be agnostic on motors' type/model, using            #
# the modularity of ROS. The output of the class is a timeseries of         #
# ROS topic (std_msgs/Float32MultiArray) of the n° of turns.                #
#If the n° of turns is equal to -1, it does means to turn off the motor.    #
#############################################################################

import rospy
from std_msgs.msg import Float32MultiArray

### Global Variables ###
turns_topic_name        = "/cmd_turns"
current_topic_name      = "/read_currents"
class_ns                = "/motor_calibrator"
QUEUE_SIZE              = 10
DISABLE_TORQUE_REQUEST  = -1
NODE_FREQUENCY          = rospy.get_param(class_ns + "/node_frequency")    # [Hz]

### Class Definition ###
class Motor_Calibrator:
    def __init__(self, n_motors):
        # Timer for Main Loop
        self.timer_obj = rospy.Timer(rospy.Duration(1/NODE_FREQUENCY), self.main_loop)

        # Subscriber/Publisher
        self.pub_obj = rospy.Publisher(turns_topic_name, Float32MultiArray, QUEUE_SIZE)
        self.sub_obj = rospy.Subscriber(current_topic_name, Float32MultiArray, self.currents_callback)
        
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
        self.cmd_turns.data = [0.0]*n_motors    # Init to zeros
    
    def currents_callback(self, msg):
        # Storing currents in the private attribute
        self.read_currents = msg

    def calibration_single_motor(self, motor_id):
        for i in range(self.n_motors):
            # Compute the commanded turns for the motor_id-th motor
            if (i == motor_id):
                if (self.cmd_turns >= 0) and (self.cmd_turns < self.max_turns):  # [0, MAX_TURNS]
                    self.cmd_turns.data[i] += self.turns_resolution
                elif (self.cmd_turns == DISABLE_TORQUE_REQUEST):
                    self.cmd_turns.data[i] = 0.0
                else:
                    rospy.logerr("Invalid value of cmd_turn, i = %d",  i)
                    break

            # Request Torque Disable for the other motors
            else:
                self.cmd_turns.data[i] = DISABLE_TORQUE_REQUEST
    
    def publish_turns(self):
        self.pub_obj.publish(self.cmd_turns)

    def main_loop(self, event):
        self.calibration_single_motor(0)
        self.publish_turns()
