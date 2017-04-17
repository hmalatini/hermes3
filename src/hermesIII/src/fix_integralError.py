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

def comprobarEffortLeft(data):
	if( abs(data - oldEffortLeft) <= 0.0000001 ):
		contadorLeft++
	else:
		oldEffortLeft = data
		contadorLeft = 0

	if(contadorLeft >= 3):
		pubEnableLeft.publish(False)

def comprobarEffortRight(data):
	if( abs(data - oldEffortRight) <= 0.0000001 ):
		contadorRight++
	else:
		oldEffortRight = data
		contadorRight = 0

	if(contadorRight >= 3):
		pubEnableRight.publish(False)

def comprobarEffortFront(data):
	if( abs(data - oldEffortFront) <= 0.0000001 ):
		contadorFront++
	else:
		oldEffortFront = data
		contadorFront = 0

	if(contadorFront >= 3):
		pubEnableFront.publish(False)

def comprobarEffortBack(data):
	if( abs(data - oldEffortBack) <= 0.0000001 ):
		contadorBack++
	else:
		oldEffortBack = data
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