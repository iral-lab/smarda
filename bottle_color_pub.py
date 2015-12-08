#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('bottle_color', String, queue_size=10)
    rospy.init_node('SMARDA', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    color_file = open(‘color_file.txt’,’r’)
    while not rospy.is_shutdown():
        bottle_color = (color_file.readline()).strip()
        rospy.loginfo(bottle_color)
        pub.publish(bottle_color)
        rate.sleep()

if __name__ == '__main__':
    try:
        SMARDA()
    except rospy.ROSInterruptException:
        pass

