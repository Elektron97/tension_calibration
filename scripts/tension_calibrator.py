#!/usr/bin/env python
import rospy
from tension_calibration.motor_calibrator import Motor_Calibrator

# Global Variables
N_MOTORS = 7

## Main ##
def main():
    rospy.init_node("tension_calibrator_node", anonymous=True)
    Motor_Calibrator(N_MOTORS)
    rospy.spin()

## Launch Main ##
if __name__ == '__main__':
    main()

