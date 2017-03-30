#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

pubLeft = rospy.Publisher('/setpoint_left', Float64, queue_size=10)
pubRigth = rospy.Publisher('/setpoint_right', Float64, queue_size=10)
pubFront = rospy.Publisher('/setpoint_front', Float64, queue_size=10)
pubBack = rospy.Publisher('/setpoint_back', Float64, queue_size=10)

def setDesiredVel(data):
    #print "Me llego mensaje Twist:\n    Lineal:\n        X: ",data.linear.x,"\n        Y: ",data.linear.y,"\n        Z: ",data.linear.z,"\n    ANGULAR:\n        X: ",data.angular.x,"\n        Y: ",data.angular.y,"\n        Z: ",data.angular.z
    radioRobot = 0.45
    vectorDeseado = [[data.linear.x],[data.linear.y],[data.angular.z * radioRobot]]
    matrizMotores = [[-1*math.sin(0.785398),math.cos(0.785398),1],[-1*math.sin(2.35619),math.cos(2.35619),1],[-1*math.sin(3.92699),math.cos(3.92699),1],[-1*math.sin(5.49779),math.cos(5.49779),1]]
    velMotores=[[0],[0],[0],[0]]
    for columna in range(0,len(vectorDeseado)):
      for fila in range(0,len(matrizMotores)):
        velMotores[fila][0] += matrizMotores[fila][columna] * vectorDeseado[columna][0]
    
    pubLeft.publish(velMotores[0][0])
    pubFront.publish(velMotores[1][0])
    pubRigth.publish(velMotores[2][0])
    pubBack.publish(velMotores[3][0])

    
def listener():
    rospy.init_node('PID_General', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, setDesiredVel)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass