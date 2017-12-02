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

    bag = rosbag.Bag('motors.bag', 'w')
        
    seq = 0
    while not rospy.is_shutdown():
        # publishing imu data
        motor_data.header.stamp = rospy.Time.now()
        motor_data.header.frame_id = "spyndra_node"
        motor_data.header.seq = seq
        motor_data.motor0 = msg.chassis_1
        motor_data.motor1 = msg.tibia_1
        motor_data.motor2 = msg.chassis_2
        motor_data.motor3 = msg.tibia_2
        motor_data.motor4 = msg.chassis_3
        motor_data.motor5 = msg.tibia_3
        motor_data.motor6 = msg.chassis_4
        motor_data.motor7 = msg.tibia_4
       
        bag.write('/motor/data', motor_data)
        seq += 1
        rate.sleep()
    bag.close()
    

def main():
    rospy.init_node('spyndra', anonymous=True)
    rospy.Subscriber('/motor_signal', MotorSignal, callback)
    rospy.spin()
    """
    #begin collecting motor data
    motor_data = MotorSignal() #unsure how to initialize 
  
    """

if __name__ == '__main__':
    main()
