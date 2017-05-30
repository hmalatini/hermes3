/*
 * rosserial Subscriber Example
 * Se manejará el PWM de los motores a través de mensajes enviados desde un joystick a la NVIDIA JETSON y de alli a la Arduino
*/

#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>

#define ledPin 13

byte I2CH;
byte I2CL;

unsigned long resultadoMotor1;
unsigned long resultadoMotor2;
unsigned long resultadoMotor3;
unsigned long resultadoMotor4;

ros::NodeHandle  nh;

//Pines del motor 4
int IN3Motor4Controlador2 = 22;    // Input3 conectada al pin 5
int IN4Motor4Controlador2 = 23;    // Input4 conectada al pin 4 
int ENBControlador2 = 2;    // ENBControlador2 conectada al pin 2 de Arduino

//Pines del motor 3
int IN1Motor4Controlador2 = 24;    // Input3 conectada al pin 5
int IN2Motor4Controlador2 = 25;    // Input4 conectada al pin 4 
int ENAControlador2 = 3;    // ENAControlador2 conectada al pin 3 de Arduino

//Pines del motor 2
int IN3Motor4Controlador1 = 26;    // Input3 conectada al pin 5
int IN4Motor4Controlador1 = 27;    // Input4 conectada al pin 4 
int ENBControlador1 = 4;    // ENBControlador1 conectada al pin 4 de Arduino

//Pines del motor 1
int IN1Motor4Controlador1 = 28;    // Input3 conectada al pin 5
int IN2Motor4Controlador1 = 29;    // Input4 conectada al pin 4 
int ENAControlador1 = 5;    // ENBControlador1 conectada al pin 5 de Arduino

//Predeterminadamente deberian arrancar moviendose hacia adelante
bool haciaAdelante = true;
bool sentidoHorarioM1 = true;
bool sentidoHorarioM2 = true;
bool sentidoHorarioM3 = true;
bool sentidoHorarioM4 = true;
float divisorX = 0.5;

float PWM_LeftWheel;
float PWM_RightWheel;
float PWM_BackWheel;
float PWM_FrontWheel;

float vel1Anterior = 0;
float vel2Anterior = 0;
float vel3Anterior = 0;
float vel4Anterior = 0;

std_msgs::Float64 vel;
std_msgs::Int16 PWM;
ros::Publisher state1("/right_wheel/state", &vel);
ros::Publisher state2("/back_wheel/state", &vel);
ros::Publisher state3("/left_wheel/state", &vel);
ros::Publisher state4("/front_wheel/state", &vel);

ros::Publisher PWM_Left_Value("/PWM_LeftValue", &PWM);
ros::Publisher PWM_Right_Value("/PWM_RightValue", &PWM);
ros::Publisher PWM_Back_Value("/PWM_BackValue", &PWM);
ros::Publisher PWM_Front_Value("/PWM_FrontValue", &PWM);

void adjustLeftPWM( byte PWM_value){ 
   analogWrite(ENAControlador2,PWM_value);//Motor M3
}
void adjustRightPWM( byte PWM_value){
   analogWrite(ENAControlador1, PWM_value);//Motor M1
}
void adjustFrontPWM( byte PWM_value){
   analogWrite(ENBControlador2,PWM_value);//Motor M4
}
void adjustBackPWM( byte PWM_value){
   analogWrite(ENBControlador1,PWM_value);//Motor M2
}

void setSentidoGiro(int cualMotor, bool sentidoHorario){
	switch(cualMotor){
		case 1: //Motor M1
			if(sentidoHorario){
				digitalWrite (IN2Motor4Controlador1, LOW) ;
				digitalWrite (IN1Motor4Controlador1, HIGH);
			} else {
        digitalWrite (IN1Motor4Controlador1, LOW);
				digitalWrite (IN2Motor4Controlador1, HIGH);
			}
      break;
		case 2: //Motor M2
			if(sentidoHorario){
				digitalWrite (IN4Motor4Controlador1, LOW);
				digitalWrite (IN3Motor4Controlador1, HIGH);
			} else {
        digitalWrite (IN3Motor4Controlador1, LOW);
				digitalWrite (IN4Motor4Controlador1, HIGH);
			}
      break;
		case 3: //Motor M3
			if(sentidoHorario){
				digitalWrite (IN2Motor4Controlador2, LOW);
				digitalWrite (IN1Motor4Controlador2, HIGH);
			} else {
        digitalWrite (IN1Motor4Controlador2, LOW);
				digitalWrite (IN2Motor4Controlador2, HIGH);
			}
      break;
		case 4: //Motor M4
			if(sentidoHorario){
				digitalWrite (IN4Motor4Controlador2, LOW);
				digitalWrite (IN3Motor4Controlador2, HIGH);
			} else {
        digitalWrite (IN3Motor4Controlador2, LOW);
				digitalWrite (IN4Motor4Controlador2, HIGH);
			}
      break;
	}
}

void publicarVelocidades( const std_msgs::UInt8& publicarVelocidades)
{
  /*-------------------------- LEEMOS LAS VELOCIDADES DE LOS DOS PRIMEROS MOTORES ------------------*/
  Wire.requestFrom(8, 4);    // request 4 bytes from slave device #8
  while (Wire.available()) { // slave may send less than requested
    I2CH = Wire.read(); // receive a byte
    I2CL = Wire.read(); // receive a byte
    resultadoMotor3 = (I2CH<<8) + I2CL; //LEFT MOTOR

    I2CH = Wire.read(); // receive a byte
    I2CL = Wire.read(); // receive a byte 
    resultadoMotor4 = (I2CH<<8) + I2CL; //FRONT MOTOR
  }
  
  /*-------------------------- LEEMOS LAS VELOCIDADES DE LOS OTROS DOS MOTORES ------------------*/
  Wire.requestFrom(9, 4);    // request 4 bytes from slave device #9
  while (Wire.available()) { // slave may send less than requested
    I2CH = Wire.read(); // receive a byte
    I2CL = Wire.read(); // receive a byte
    resultadoMotor1 = (I2CH<<8) + I2CL; //RIGHT MOTOR

    I2CH = Wire.read(); // receive a byte
    I2CL = Wire.read(); // receive a byte 
    resultadoMotor2 = (I2CH<<8) + I2CL; //BACK MOTOR
  }

  /*----------- CONVERTIMOS LAS VELOCIDADES EN METROS/SEGUNDOS --------------*/
  float vel1 = 0;
  if(resultadoMotor1 != 0){
    vel1 = (0.0078*1000000)/(resultadoMotor1*32);
  }
  float vel2 = 0;
  if(resultadoMotor2 != 0){
    vel2 = (0.0078*1000000)/(resultadoMotor2*32);
  }
  float vel3 = 0;
  if(resultadoMotor3 != 0){
    vel3 = (0.0078*1000000)/(resultadoMotor3*32);
  }
  float vel4 = 0;
  if(resultadoMotor4 != 0){
    vel4 = (0.0078*1000000)/(resultadoMotor4*32);
  }

  /* -------------------------- FILTRAMOS DATOS ATIPICOS (VELOCIDADES MAYORES A 0.9) --------------------------- */
  /* -------------------------- Y PUBLICAMOS LAS VELOCIDADES --------------------------------------------------- */
  if (vel1 > 0.9){ //Comprobacion que se realiza por los datos erroneos que suelen obtenerse... La rueda por cuestiones fisicas no puede ir a mas de 0.9 m/s
    vel1 = vel1Anterior;
  }
  else{
    vel1Anterior = vel1;
  }

  if (abs(PWM_RightWheel) <= 10){ //Si el PWM de ésta rueda es un valor bajo, Arduino supone que no se mueve y envia 0 (evitando la espera de los dos overflow en las Atmega)
    vel.data = 0;
  }
  else if (sentidoHorarioM1){
    vel.data = vel1;
  }
  else {
    vel.data = vel1 * -1;
  }
  
  state1.publish( &vel );
//------------------------------------------  
  if (vel2 > 0.9){
    vel2 = vel2Anterior;
  }
  else{
    vel2Anterior = vel2;
  }
  
  if (abs(PWM_BackWheel) <= 10){ //Si el PWM de ésta rueda es un valor bajo, Arduino supone que no se mueve y envia 0 (evitando la espera de los dos overflow en las Atmega)
    vel.data = 0;
  }
  else if (sentidoHorarioM2){
    vel.data = vel2;
  }
  else{
    vel.data = vel2 * -1;
  }
  state2.publish( &vel );
//------------------------------------------  
  if (vel3 > 0.9){
    vel3 = vel3Anterior;
  }
  else{
    vel3Anterior = vel3;
  }
  
  if (abs(PWM_LeftWheel) <= 10){ //Si el PWM de ésta rueda es un valor bajo, Arduino supone que no se mueve y envia 0 (evitando la espera de los dos overflow en las Atmega)
    vel.data = 0;
  }
  else if (sentidoHorarioM3){
    vel.data = vel3;
  }
  else{
    vel.data = vel3 * -1;
  }
  state3.publish( &vel );
//------------------------------------------    
  if (vel4 > 0.9){
    vel4 = vel4Anterior;
  }
  else{
    vel4Anterior = vel4;
  }
  
  if (abs(PWM_FrontWheel) <= 10){ //Si el PWM de ésta rueda es un valor bajo, Arduino supone que no se mueve y envia 0 (evitando la espera de los dos overflow en las Atmega)
    vel.data = 0;
  }
  else if (sentidoHorarioM4){
    vel.data = vel4;
  }
  else{
    vel.data = vel4 * -1;
  }
  state4.publish( &vel );
}
//------------------------------------------  


void setLeftPWM( const std_msgs::Float64& leftPWM){ //"leftPWM" es un valor entre -1 y 1 y define la intensidad del ajuste a realizar
  float pwm = PWM_LeftWheel + leftPWM.data;
  PWM_LeftWheel = pwm;
  if(pwm < 0){
    if (sentidoHorarioM3){// Si se esta moviendo en sentido horario debemos cambiar su sentido de giro
      sentidoHorarioM3 = false; //Seteamos el sentido antihorario
      setSentidoGiro(3, sentidoHorarioM3); //Lo hacemos girar en sentido antihorario
    }
    if(pwm < -80){
      PWM_LeftWheel = -80;
      pwm = -80;
    }
    pwm = pwm * -1;   //Hacemos que "pwm" sea positivo. Porque siempre lo debe ser
  }
  else{
    if (!sentidoHorarioM3){// Si se esta moviendo en sentido antihorario debemos cambiar su sentido de giro
      sentidoHorarioM3 = true; //Seteamos el sentido horario
      setSentidoGiro(3, sentidoHorarioM3); //Lo hacemos girar en sentido horario
    }
    if(pwm > 80){
      PWM_LeftWheel = 80;
      pwm = 80;
    }
  }

  adjustLeftPWM(pwm);
  //Publicamos el valor de PWM que desea enviar la placa
  PWM.data = PWM_LeftWheel;
  PWM_Left_Value.publish( &PWM );
}
void setFrontPWM( const std_msgs::Float64& frontPWM){
  float pwm = PWM_FrontWheel + frontPWM.data;
  PWM_FrontWheel = pwm;
  if(pwm < 0){
    if (sentidoHorarioM4){// Si se esta moviendo en sentido horario debemos cambiar su sentido de giro
      sentidoHorarioM4 = false; //Seteamos el sentido antihorario
      setSentidoGiro(4, sentidoHorarioM4); //Lo hacemos girar en sentido antihorario
    }
    if(pwm < -80){
      PWM_FrontWheel = -80;
      pwm = -80;
    }
    pwm = pwm * -1;   //Hacemos que "pwm" sea positivo. Porque siempre lo debe ser
  }
  else{
    if (!sentidoHorarioM4){// Si se esta moviendo en sentido antihorario debemos cambiar su sentido de giro
      sentidoHorarioM4 = true; //Seteamos el sentido horario
      setSentidoGiro(4, sentidoHorarioM4); //Lo hacemos girar en sentido horario
    }
    if(pwm > 80){
      PWM_FrontWheel = 80;
      pwm = 80;
    }
  }

  adjustFrontPWM(pwm);

    //Publicamos el valor de PWM que desea enviar la placa
  PWM.data = PWM_FrontWheel;
  PWM_Front_Value.publish( &PWM );
}
void setRightPWM( const std_msgs::Float64& rightPWM){
  float pwm = PWM_RightWheel + rightPWM.data;
  PWM_RightWheel = pwm;
  if(pwm < 0){
    if (sentidoHorarioM1){// Si se esta moviendo en sentido horario debemos cambiar su sentido de giro
      sentidoHorarioM1 = false; //Seteamos el sentido antihorario
      setSentidoGiro(1, sentidoHorarioM1); //Lo hacemos girar en sentido antihorario
    }
    if(pwm < -80){
      PWM_RightWheel = -80;
      pwm = -80;
    }
    pwm = pwm * -1;   //Hacemos que "pwm" sea positivo. Porque siempre lo debe ser
  }
  else{
    if (!sentidoHorarioM1){// Si se esta moviendo en sentido antihorario debemos cambiar su sentido de giro
      sentidoHorarioM1 = true; //Seteamos el sentido horario
      setSentidoGiro(1, sentidoHorarioM1); //Lo hacemos girar en sentido horario
    }
    if(pwm > 80){
      PWM_RightWheel = 80;
      pwm = 80;
    }
  }
  
  adjustRightPWM(pwm);

    //Publicamos el valor de PWM que desea enviar la placa
  PWM.data = PWM_RightWheel;
  PWM_Right_Value.publish( &PWM );
}
void setBackPWM( const std_msgs::Float64& backPWM){
  float pwm = PWM_BackWheel + backPWM.data;
  PWM_BackWheel = pwm;
  if(pwm < 0){
    if (sentidoHorarioM2){// Si se esta moviendo en sentido horario debemos cambiar su sentido de giro
      sentidoHorarioM2 = false; //Seteamos el sentido antihorario
      setSentidoGiro(2, sentidoHorarioM2); //Lo hacemos girar en sentido antihorario
    }
    if(pwm < -80){
      PWM_BackWheel = -80;
      pwm = -80;
    }
    pwm = pwm * -1;   //Hacemos que "pwm" sea positivo. Porque siempre lo debe ser
  }
  else{
    if (!sentidoHorarioM2){// Si se esta moviendo en sentido antihorario debemos cambiar su sentido de giro
      sentidoHorarioM2 = true; //Seteamos el sentido horario
      setSentidoGiro(2, sentidoHorarioM2); //Lo hacemos girar en sentido horario
    }
    if(pwm > 80){
      PWM_BackWheel = 80;
      pwm = 80;
    }
  }

  adjustBackPWM(pwm);

    //Publicamos el valor de PWM que desea enviar la placa
  PWM.data = PWM_BackWheel;
  PWM_Back_Value.publish( &PWM );
}


ros::Subscriber<std_msgs::UInt8> subVelRequest("/send_velocity", &publicarVelocidades );

ros::Subscriber<std_msgs::Float64> subLeftControlEffort("/left_wheel/control_effort", &setLeftPWM );
ros::Subscriber<std_msgs::Float64> subFrontControlEffort("/front_wheel/control_effort", &setFrontPWM );
ros::Subscriber<std_msgs::Float64> subRightControlEffort("/right_wheel/control_effort", &setRightPWM );
ros::Subscriber<std_msgs::Float64> subBackControlEffort("/back_wheel/control_effort", &setBackPWM );

void setup()
{
  pinMode(ledPin, OUTPUT);
  Wire.begin();        // join i2c bus (address optional for master)
  
  //PWM pins config
  pinMode (ENBControlador2, OUTPUT); 
  pinMode (IN3Motor4Controlador2, OUTPUT);
  pinMode (IN4Motor4Controlador2, OUTPUT);

  pinMode (ENAControlador2, OUTPUT); 
  pinMode (IN1Motor4Controlador2, OUTPUT);
  pinMode (IN2Motor4Controlador2, OUTPUT);

  pinMode (ENBControlador1, OUTPUT); 
  pinMode (IN3Motor4Controlador1, OUTPUT);
  pinMode (IN4Motor4Controlador1, OUTPUT);

  pinMode (ENAControlador1, OUTPUT); 
  pinMode (IN1Motor4Controlador1, OUTPUT);
  pinMode (IN2Motor4Controlador1, OUTPUT);
//----------------------
//Preparamos la salida para que el robot este seteado para mover todos sus motores en sentido horario
//MOTORES M3 y M4 giran en sentido horario
  digitalWrite (IN3Motor4Controlador2, HIGH);
  digitalWrite (IN4Motor4Controlador2, LOW);

  digitalWrite (IN1Motor4Controlador2, HIGH);
  digitalWrite (IN2Motor4Controlador2, LOW);
  
//MOTORES M1 y M2 giran en sentido horario
  digitalWrite (IN3Motor4Controlador1, HIGH);
  digitalWrite (IN4Motor4Controlador1, LOW);

  digitalWrite (IN1Motor4Controlador1, HIGH);
  digitalWrite (IN2Motor4Controlador1, LOW);
  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(subVelRequest);
  
  nh.subscribe(subLeftControlEffort);
  nh.subscribe(subRightControlEffort);
  nh.subscribe(subFrontControlEffort);
  nh.subscribe(subBackControlEffort);
  
  nh.advertise(state1);
  nh.advertise(state2);
  nh.advertise(state3);
  nh.advertise(state4);

  nh.advertise(PWM_Left_Value);
  nh.advertise(PWM_Right_Value);
  nh.advertise(PWM_Back_Value);
  nh.advertise(PWM_Front_Value);
}

void loop()
{  
  nh.spinOnce(); 
}
