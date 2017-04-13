#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

unsigned int timerCapture1 = 0;
unsigned int result1 = 0;
boolean descartar1 = false;

unsigned int timerCapture3 = 0;
unsigned int result3 = 0;
boolean descartar3 = false;

void setup() {
  //Configuramos para que trabaje con fuente de clock externa (para capturador de eventos)
  TIFR = 0;
  TCCR1B = 0b11000100; //Noise Canceler Capture (bit 7), Rising edge (bit 6) and Prescaler 256 (bits 2:0)
  TIMSK = 0b00100100; //Habilitamos Input Interrupt Capture

  TCCR1A = 0;
  
  // Inicializamos contador en 0
  TCNT1 = 0;

  
  //Configuramos para que trabaje con fuente de clock externa (para capturador de eventos)
  ETIFR = 0;
  TCCR3B = 0b11000100; //Noise Canceler Capture (bit 7), Rising edge (bit 6) and Prescaler 256 (bits 2:0)
  ETIMSK = 0b00100100; //Habilitamos Input Interrupt Capture

  TCCR3A = 0;
  
  // Inicializamos contador en 0
  TCNT3 = 0;
  
  Wire.begin(9);                // join i2c bus with address #9
  Wire.onRequest(requestEvent); // register event

  pinMode(44, OUTPUT);
  sei();
}

ISR(TIMER1_CAPT_vect){
  if (!descartar1){
    result1 = (ICR1 - timerCapture1); //result1 DEBERA SER MULTIPLICADO POR 32 (microsegundos escalon, minima resolucion) EN LA ARDUINO MEGA PARA ESTAR EN MICROSEGUNDOS
  }
  else{
    descartar1 = false;
  }
  timerCapture1 = ICR1;
}

ISR(TIMER1_OVF_vect){
  digitalWrite(44, !digitalRead(44));
  if (descartar1){
    result1 = 0;
  }
  descartar1 = true;
}

ISR(TIMER3_CAPT_vect){
  if (!descartar3){
    result3 = (ICR3 - timerCapture3); //result3 DEBERA SER MULTIPLICADO POR 32 (microsegundos escalon, minima resolucion) EN LA ARDUINO MEGA PARA ESTAR EN MICROSEGUNDOS
  }
  else{
    descartar3 = false;
  }
  timerCapture3 = ICR3;
}

ISR(TIMER3_OVF_vect){
  if (descartar3){
    result3 = 0;
  }
  descartar3 = true;
}

void loop() {
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() { //Mandamos 4 bytes
  byte resultados[4] = {result1>>8,result1,result3>>8,result3};
  Wire.write(resultados, 4);
}
