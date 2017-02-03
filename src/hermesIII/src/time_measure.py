#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32

def tomar_tiempo(data):
	print data
	#t2 = rospy.get_time()
	print rospy.get_time()



if __name__=="__main__":
	"""
		Lo que va a hacer este script, es mandarle un mensaje a Arduino, mediante un publisher llamado "probando_tiempo", y tomamos el tiempo
		Luego, con la Arduino ya suscripta, cuando reciba, automaticamente publica a un publisher que nosotros estamos como receiver,
		y medimos de nuevo el timepo. Por ulitmo, imprimimos la diferencia de tiempo en pantalla
	"""
	#Creamos el publisher
	pub = rospy.Publisher('probando_tiempo', UInt32, queue_size = 1)
	rospy.init_node('time_measure')

	t2 = rospy.get_time() #Tomamos un tiempo para empezar

	#Nos suscribimos al topico de arduino
	rospy.Subscriber("respuesta_arduino", UInt32, tomar_tiempo)

	#Creamos el mensaje UInt32
	int = UInt32()
	int.data = 20;


	#Lo enviamos
	pub.publish(UInt32)
	t1 = rospy.get_time()
	print t1