#include <QTRSensors.h>

#define led_indicador 9
#define start_button 8
#define pwmi  3   //PWM LEFT MOTOR
#define izq1  5   //LEFT1 MOTOR
#define izq2  4   //LEFT2 MOTOR
#define pwmd  11  //PWM RIGHT MOTOR
#define der1  6   //RIGHT1 MOTOR
#define der2  7   //RIGHT2 MOTOR
#define GO 10 //RRT GO
#define RDY 12 //RRT RDY

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Settings
int tipo_de_pista = 1; //1 para pista negra, 0 para pista blanca

// float KP = 0.05;
// float KD = 0.1;
// float KI = 0.02;
// int max_speed = 100;

// float KP = 0.09;
// float KD = 0.275;
// float KI = 0.02;
// int max_speed = 170;

// float KP = 0.045;
// float KD = 0.09;
// float KI = 0.001;
// int max_speed = 145;

// float KP = 0.05;
// float KD = 0.09;
// float KI = 0.001;
// int max_speed = 100;

float KP = 0.09;
float KD = 0.15;
float KI = 0.01;
int max_speed = 150;

int setpoint = 3500;

// Variables
float proporcional = 0;
float last_error = 0;
float derivativo = 0;
float integral = 0;
float diferencial = 0;
float error = 0;

//Errores
float error1 = 0.0;
float error2 = 0.0;
float error3 = 0.0;
float error4 = 0.0;
float error5 = 0.0;
float error6 = 0.0;



void setup() {
  // TCCR2B = TCCR2B & B11111000 | B00000011; //980.39 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000101; //245.10 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000110; //122.55 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000010; //3921.16 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000111; //30.64 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000001; //31372.55 Hz
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);
  qtr.setEmitterPin(2);

  pinMode(led_indicador, OUTPUT);
  pinMode(start_button, INPUT_PULLUP);
  pinMode(izq1, OUTPUT);
  pinMode(izq2, OUTPUT);
  pinMode(der1, OUTPUT);
  pinMode(der2, OUTPUT);
  pinMode(GO, INPUT);
  pinMode(RDY, INPUT);
  
  while(digitalRead(start_button) == HIGH);
  // Calibrate sensors
  for (int i = 0; i < 100; i++) {
    if(i%5 == 0){
      digitalWrite(led_indicador, !digitalRead(led_indicador));      
    }
    qtr.calibrate();
  }

  digitalWrite(led_indicador, LOW);
  while(digitalRead(RDY) == LOW);
}

void(* resetFunc) (void) = 0;

void loop() {
  while(true){
    // int lectura = position();
    // diferencial = PID(lectura);
    // controlMotores(diferencial, 0);
    if(digitalRead(RDY) == LOW && digitalRead(GO) == HIGH){  
      break;
    }
  }
  // 
  digitalWrite(led_indicador, HIGH);
  while(true){
    // frenos();
    int lectura = position();
    diferencial = PID(lectura);
    controlMotores(diferencial, max_speed);
    if(digitalRead(GO) == LOW){
      moverMotores(-30, -30);       
      break;
    }
  }
  digitalWrite(led_indicador, LOW);
  moverMotores(0,0);
  resetFunc();    
}

int position() {
  qtr.read(sensorValues);
  int position;
  if(tipo_de_pista){
    position = qtr.readLineBlack(sensorValues);
  }
  else{
    position = qtr.readLineWhite(sensorValues);
  }
  return position;
}

float PID(int lectura) {
  error = setpoint - lectura;
  proporcional = error;
  integral = (error1 + error2 + error3 + error4 + error5 + error6 + (integral + proporcional)) * (integral*proporcional > 0); 
  derivativo = error - last_error;
  float diff = KP * proporcional + KD * derivativo + KI * integral;
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = error;
  last_error = error;
  return diff;
}

void controlMotores(int diferencial, int speed) {
  // if(speed == 0){
  if(diferencial > speed*2) diferencial = speed*2;
  if(diferencial < -speed*2) diferencial = -(speed*2);
  // } else{
  //   if (diferencial > speed) diferencial = speed;
  //   if (diferencial < -speed) diferencial = -speed;
  // }
  // if(error < 300 && error > -300 && speed != 0){
  //   speed = 150;
  // }

  if (diferencial < 0) {
    moverMotores(speed, speed+diferencial);
  } else {
    moverMotores(speed-diferencial, speed);
  }
}

void moverMotores(int izq, int der) {
  // Mueve los motores con la velocidad indicada
  // izq: velocidad del motor izquierdo (valor entre -255 y 255)
  // der: velocidad del motor derecho (valor entre -255 y 255)

  // Control del motor IZQUIERDO
  if (izq >= 0) {
    digitalWrite(izq1, HIGH);
    digitalWrite(izq2, LOW);
  } else {
    digitalWrite(izq1, LOW);
    digitalWrite(izq2, HIGH);
    izq = -izq;
  }
  analogWrite(pwmi, izq);

  // Control del motor DERECHO
  if (der >= 0) {
    digitalWrite(der1, LOW);
    digitalWrite(der2, HIGH);
  } else {
    digitalWrite(der1, HIGH);
    digitalWrite(der2, LOW);
    der = -der;
  }
  analogWrite(pwmd, der);
}

//TODO
void frenos() {
  if(position<=150){
    moverMotores(200, -100);    
  }
  if(position>=6850){
    moverMotores(-100, 200);
  }
}
