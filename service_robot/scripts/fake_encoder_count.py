#!/usr/bin/env python
from std_msgs.msg import Int32
import rospy
import time
from std_msgs.msg import String
names = []
def talker():
    pub = rospy.Publisher('left_encoder_count', Int32, queue_size=50)
    pub2 = rospy.Publisher('right_encoder_count', Int32, queue_size=50)
    rospy.init_node('fake_count_node', anonymous=True)
    rate = rospy.Rate(2)
    left_count = 0
    right_count = 0
    while not rospy.is_shutdown():
 
        #rospy.loginfo(left_count)
        pub.publish(left_count)
        #rospy.loginfo(right_count)
        pub2.publish(right_count)
        right_count = right_count+1
        left_count = left_count+1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
