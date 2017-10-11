#!/usr/bin/env python
import rospy
from spyndra import standingGait
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    femur = [.3, .5, .3, .1]
    tibia = [.2, .4, .2, .0]
    while not rospy.is_shutdown():
        hello_str = "%s" % standingGait.spline_gen(femur, tibia, 5, 2)
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
