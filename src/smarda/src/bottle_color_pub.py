#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def SMARDA():
    pub = rospy.Publisher('bottle_color', String, queue_size=10)
    rospy.init_node('SMARDA', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    color_file = open("color_file.txt","r")
    bottle_color = color_file.readline()
    while not rospy.is_shutdown():
        rospy.loginfo(bottle_color)
        pub.publish(bottle_color)
        rate.sleep()

if __name__ == '__main__':
    try:
        SMARDA()
    except rospy.ROSInterruptException:
        pass

