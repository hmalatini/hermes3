#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Bool

pubEnableLeft = rospy.Publisher('/left_wheel/left_wheel_pid_activate', Bool, queue_size=10)
pubEnableRight = rospy.Publisher('/right_wheel/right_wheel_pid_activate', Bool, queue_size=10)
pubEnableFront = rospy.Publisher('/front_wheel/front_wheel_pid_activate', Bool, queue_size=10)
pubEnableBack = rospy.Publisher('/back_wheel/back_wheel_pid_activate', Bool, queue_size=10)

contadorLeft = 0
contadorRight = 0
contadorFront = 0
contadorBack = 0

oldEffortLeft = 0
oldEffortRight = 0
oldEffortFront = 0
oldEffortBack = 0

def comprobarEffortLeft(value):
	global oldEffortLeft, contadorLeft
	if( (abs(value.data - oldEffortLeft) <= 0.0001) and (abs(oldEffortLeft) < 1) ):
		contadorLeft += 1
	else:
		oldEffortLeft = value.data
		contadorLeft = 0

	if(contadorLeft >= 3):
		pubEnableLeft.publish(False)

def comprobarEffortRight(value):
	global oldEffortRight, contadorRight
	if( (abs(value.data - oldEffortRight) <= 0.0001) and (abs(oldEffortRight) < 1) ):
		contadorRight += 1
	else:
		oldEffortRight = value.data
		contadorRight = 0

	if(contadorRight >= 3):
		pubEnableRight.publish(False)

def comprobarEffortFront(value):
	global oldEffortFront, contadorFront
	if( (abs(value.data - oldEffortFront) <= 0.0001) and (abs(oldEffortFront) < 1) ):
		contadorFront += 1
	else:
		oldEffortFront = value.data
		contadorFront = 0

	if(contadorFront >= 3):
		pubEnableFront.publish(False)

def comprobarEffortBack(value):
	global oldEffortBack, contadorBack
	if( (abs(value.data - oldEffortBack) <= 0.0001) and (abs(oldEffortBack) < 1) ):
		contadorBack += 1
	else:
		oldEffortBack = value.data
		contadorBack = 0

	if(contadorBack >= 3):
		pubEnableBack.publish(False)

def listener():
    rospy.init_node('fix_integralError', anonymous=True)
    rospy.Subscriber("/left_wheel/control_effort", Float64, comprobarEffortLeft)
    rospy.Subscriber("/right_wheel/control_effort", Float64, comprobarEffortRight)
    rospy.Subscriber("/front_wheel/control_effort", Float64, comprobarEffortFront)
    rospy.Subscriber("/back_wheel/control_effort", Float64, comprobarEffortBack)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass