#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from spyndra import motorModule
import ast

# spyndraMotor = motorModule.SpyndraMotor()

def parse_args(raw_args):
    motor_type = raw_args['motor_type']
    motor_output_1 = raw_args['motor_output_1']['chassisOutput'], raw_args['motor_output_1']['tibiaOutput'], raw_args['motor_output_1']['chassisNum'],raw_args['motor_output_1']['tibiaNum'] 
    motor_output_2 = raw_args['motor_output_2']['chassisOutput'], raw_args['motor_output_2']['tibiaOutput'], raw_args['motor_output_2']['chassisNum'],raw_args['motor_output_2']['tibiaNum']
    motor_output_3 = raw_args['motor_output_3']['chassisOutput'], raw_args['motor_output_3']['tibiaOutput'], raw_args['motor_output_3']['chassisNum'],raw_args['motor_output_3']['tibiaNum']
    motor_output_4 = raw_args['motor_output_4']['chassisOutput'], raw_args['motor_output_4']['tibiaOutput'], raw_args['motor_output_4']['chassisNum'],raw_args['motor_output_4']['tibiaNum'] 
    return motor_type, motor_output_1, motor_output_2, motor_output_3,motor_output_4

def callback(data):
    args = ast.literal_eval(data.data)
    motor_type, motor_output_1, motor_output_2, motor_output_3,motor_output_4 = parse_args(args)
    # successfully parsing the arguments1)
    #spyndraMotor.set_motor_type(motor_type)
    # spyndraMotor.output_motor(motor_output_1)
    # spyndraMotor.output_motor(motor_output_2)
    # spyndraMotor.output_motor(motor_output_3)
    # spyndraMotor.output_motor(motor_output_4)
    

def signal_receiver():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('signal_receiver', anonymous=True)
    rospy.Subscriber('motor/signal', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    signal_receiver()
