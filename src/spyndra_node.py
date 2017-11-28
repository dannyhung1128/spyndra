#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from spyndra.msg import MotorSignal
#from spyndra import motorModule
import ast

# spyndraMotor = motorModule.SpyndraMotor()

def callback(msg):
    motor_type = msg.motor_type
    chassis_1, chassis_2, chassis_3, chassis_4 = msg.chassis_1, msg.chassis_2, msg.chassis_3, msg.chassis_4
    tibia_1,   tibia_2,   tibia_3,   tibia_4   = msg.tibia_1, msg.tibia_2, msg.tibia_3, msg.tibia_4
    
    rospy.loginfo(motor_type)
    
    spyndraMotor.set_motor_type(motor_type)
    spyndraMotor.output_motor(chassis_1, tibia_1, 0, 1)
    spyndraMotor.output_motor(chassis_2, tibia_2, 2, 3)
    spyndraMotor.output_motor(chassis_3, tibia_3, 4, 5)
    spyndraMotor.output_motor(chassis_4, tibia_4, 6, 7)
    

def main():
    rospy.init_node('spyndra', anonymous=True)
    rospy.Subscriber('/motor_signal', MotorSignal, callback)
    rospy.spin()

if __name__ == '__main__':
    main()