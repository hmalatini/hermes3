/*
 * rosserial Subscriber Example
 * Se manejará el PWM de los motores a través de mensajes enviados desde NVIDIA JETSON y ROS
*/

#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle  nh;

//Pines conectado al driver de los motores
int IN3 = 5;    // Input3 conectada al pin 5
int IN4 = 4;    // Input4 conectada al pin 4 
int ENB = 3;    // ENB conectada al pin 3 de Arduino

void adjustPWM( int PWM_value){
   analogWrite(ENB,PWM_value);
}

void messageCb( const std_msgs::UInt8& PWM_motor){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  adjustPWM(PWM_motor.data);
}

void cambiarGiro( const std_msgs::UInt8& PWM_motor){
  digitalWrite (IN3, LOW);
  delay(500); //Esperamos medio segundo para que la se detenga en caso que lo siga haciendo...
  digitalWrite (IN4, HIGH);
}

ros::Subscriber<std_msgs::UInt8> sub("PWM_motor", &messageCb );
ros::Subscriber<std_msgs::UInt8> sub2("Cambio_Giro", &cambiarGiro );

void setup()
{ //PWM pins config
  pinMode (ENB, OUTPUT); 
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
//----------------------
//Preparamos la salida para que el motor gire en un sentido
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}