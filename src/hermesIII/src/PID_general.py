#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import math

pubLeft = rospy.Publisher('/setpoint_left', Float64, queue_size=10)
pubRigth = rospy.Publisher('/setpoint_right', Float64, queue_size=10)
pubFront = rospy.Publisher('/setpoint_front', Float64, queue_size=10)
pubBack = rospy.Publisher('/setpoint_back', Float64, queue_size=10)

pubEnableLeft = rospy.Publisher('/left_wheel/left_wheel_pid_activate', Bool, queue_size=10)
pubEnableRigth = rospy.Publisher('/right_wheel/right_wheel_pid_activate', Bool, queue_size=10)
pubEnableFront = rospy.Publisher('/front_wheel/front_wheel_pid_activate', Bool, queue_size=10)
pubEnableBack = rospy.Publisher('/back_wheel/back_wheel_pid_activate', Bool, queue_size=10)

velMinima = 0.4

def setDesiredVel(data):
    #print "Me llego mensaje Twist:\n    Lineal:\n        X: ",data.linear.x,"\n        Y: ",data.linear.y,"\n        Z: ",data.linear.z,"\n    ANGULAR:\n        X: ",data.angular.x,"\n        Y: ",data.angular.y,"\n        Z: ",data.angular.z
    #ANALIZAR EL CASO EN EL QUE EL MENSAJE EN X ES IGUAL AL Y, ENTONCES DOS RUEDAS SE ANULAN Y FUNCIONAN SOLO DOS...
    radioRobot = 0.45
    vectorDeseado = [[data.linear.x],[data.linear.y],[data.angular.z * radioRobot]]
    matrizMotores = [[-1*math.sin(0.785398),math.cos(0.785398),1],[-1*math.sin(2.35619),math.cos(2.35619),1],[-1*math.sin(3.92699),math.cos(3.92699),1],[-1*math.sin(5.49779),math.cos(5.49779),1]]
    velMotores=[[0],[0],[0],[0]]
    for columna in range(0,len(vectorDeseado)):
        for fila in range(0,len(matrizMotores)):
            velMotores[fila][0] += matrizMotores[fila][columna] * vectorDeseado[columna][0]
    
    menor = False
    if(abs(data.linear.x) != abs(data.linear.y) and (data.linear.x) != 0 and (data.linear.y)):
        for i in range(0,4):
            print("IF num: " + str(i) + " vel: " + str(velMotores[i][0]))
            if( ((velMotores[i][0] > (velMinima*-1)) and (velMotores[i][0] < velMinima)) ):
                menor = True
                break
    else:
        for i in range(0,4):
            print("ELSE num: " + str(i) + " vel: " + str(velMotores[i][0]))
            if( (velMotores[i][0] > (velMinima*-1)) and (velMotores[i][0] < velMinima) ):
                if( (velMotores[i][0] > 0.1) or (velMotores[i][0] < -0.1) ):
                    menor = True
                    break

    if(not menor):
        pubEnableLeft.publish(True)
        pubEnableRigth.publish(True)
        pubEnableFront.publish(True)
        pubEnableBack.publish(True)
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