#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('reset_arduino', String, queue_size=10)
    rospy.init_node('reset_arduino', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    reset_var = "arduino_reset" 
    rospy.loginfo(reset_var)
    pub.publish(reset_var)
    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
