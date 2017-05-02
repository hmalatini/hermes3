#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt8

def talker():
    pub = rospy.Publisher('send_velocity', UInt8, queue_size=10)
    rospy.init_node('velocity_request', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        pub.publish(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
