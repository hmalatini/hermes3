#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

toggle = False

def callback(data):
	global toggle
	twist = Twist()
	twist.linear.x = 1.5*data.axes[1]
	twist.linear.y = -1.5*data.axes[0]
	twist.angular.z = 1.5*data.axes[3]
	if(data.buttons[4] == 1):
		toggle = True
		pub.publish(twist)
	elif(toggle == True):
		twist.linear.x = 0
		twist.linear.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		toggle = False

# Intializes everything
def start():
	# publishing to "turtle1/cmd_vel" to control turtle1
	global pub
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	# subscribed to joystick inputs on topic "joy"
	rospy.Subscriber("joy", Joy, callback)
	# starts the node
	rospy.init_node('Xbox360Joy')
	rospy.spin()

if __name__ == '__main__':
	start()
