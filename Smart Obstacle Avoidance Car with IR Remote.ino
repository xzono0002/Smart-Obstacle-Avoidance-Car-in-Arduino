#include <IRremote.h>
#include <IRremoteInt.h>

#include <Servo.h>

#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  6    //pin of controlling speed---- ENB of motor driver board
#define IR_Go      0x00FF18E7
#define IR_Back    0x00FF4AB5
#define IR_Left    0x00FF10EF
#define IR_Right   0x00FF5AA5
#define IR_Stop    0x00FF38C7

int pinLB=2;             //pin of controlling turning---- IN1 of motor driver board
int pinLF=4;             //pin of controlling turning---- IN2 of motor driver board
int pinRB=7;            //pin of controlling turning---- IN3 of motor dzdsriver board
int pinRF=8;            //pin of controlling turning---- IN4 of motor driver board
int RECV_PIN = 12;
unsigned char Lpwm_val = 150;
unsigned char Rpwm_val = 150;
IRrecv irrecv(RECV_PIN);
decode_results results;
volatile int D_mix;
volatile int D_mid;
volatile int D_max;
volatile int left_IR_Sensor;
volatile int right_IR_Sensor;
volatile int Front_Distance;
volatile int Left_Distance;
volatile int Right_Distance;

Servo myservo;
volatile int DL;
volatile int DM;
volatile int DR;

void M_Control_IO_config(void)
{
  pinMode(pinLB,OUTPUT); // pin2
  pinMode(pinLF,OUTPUT); // pin4
  pinMode(pinRB,OUTPUT); // pin7 
  pinMode(pinRF,OUTPUT); // pin8
  pinMode(Lpwm_pin,OUTPUT); // pin5 (PWM) 
  pinMode(Rpwm_pin,OUTPUT); // pin6 (PWM)   
}

void Sensor_Config(void){
  pinMode(9, INPUT);
  pinMode(A1, OUTPUT);
  pinMode(A0, INPUT);
}

float checkdistance() {
  digitalWrite(A1, LOW);
  delayMicroseconds(2);
  digitalWrite(A1, HIGH);
  delayMicroseconds(10);
  digitalWrite(A1, LOW);
  float distance = pulseIn(A0, HIGH) / 58.00;
  delay(10);
  return distance;
}

void Detect_Left_and_Right__distance() {
  myservo.write(165);
  delay(500);
  Serial.println(Left_Distance);
  delay(100);
  Left_Distance = checkdistance();
  myservo.write(15);
  delay(500);
  Serial.println(Right_Distance);
  delay(100);
  Right_Distance = checkdistance();
  myservo.write(90);
}

void Ultrasonic_obstacle_avoidance() {
  if (Front_Distance <= D_mid) {
    stopp();
    if (Front_Distance <= D_mix || left_IR_Sensor == 0 && right_IR_Sensor == 0) {
      back();
      Set_Speed(Lpwm_val,Rpwm_val);
      delay(300);
      stopp();
    }
    Detect_Left_and_Right__distance();
    if ((D_mix < Left_Distance && Left_Distance < D_max) && (D_mix < Right_Distance && Right_Distance < D_max)) {
      if (Right_Distance > Left_Distance) {
        turnR();
        Set_Speed(200,200);
        delay(150);

      } else {
        turnL();
        Set_Speed(200,200);
        delay(150);
        
      }
      stopp();
      IR_Control();

    } else if (D_mix < Left_Distance && Left_Distance < D_max || D_mix < Right_Distance && Right_Distance < D_max) {
      if (D_mix < Left_Distance && Left_Distance < D_max) {
        turnL();
        Set_Speed(Lpwm_val,Rpwm_val);
        delay(150);

      } else if (D_mix < Right_Distance && Right_Distance < D_max) {
        turnR();
        Set_Speed(Lpwm_val,Rpwm_val);
        delay(150);
      }
      stopp();
      IR_Control();
    } else if (Right_Distance < D_mix && Left_Distance < D_mix) {
      advance();
      Set_Speed(0,200);
      delay(250);
      stopp();
      IR_Control();
    }
    stopp();
    IR_Control();
  }
}

void Infrared_Obstacle_Avoidance() {
//  if (left_IR_Sensor == 0 && right_IR_Sensor == 1) {
//    turnR();
//    Set_Speed(Lpwm_val,Rpwm_val);
//
//  } else if (left_IR_Sensor == 1 && right_IR_Sensor == 0) {
//    turnL();
//    Set_Speed(Lpwm_val,Rpwm_val);
//  }
//    else {
//    advance();
//    Set_Speed(Lpwm_val,Rpwm_val);
//  }
    if (left_IR_Sensor == 0) {
      turnR();
      Set_Speed(Lpwm_val,Rpwm_val);
      delay(250);
      stopp();
      IR_Control();
    }
    else {
      IR_Control();
      Set_Speed(Lpwm_val,Rpwm_val);
    }
}

void Set_Speed(unsigned char Left,unsigned char Right) //function of setting speed
{
  analogWrite(Lpwm_pin,Left);
  analogWrite(Rpwm_pin,Right);
}
void advance()    //  going forward
    {
     digitalWrite(pinRB,LOW);  // making motor move towards right rear
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,LOW);  // making motor move towards left rear
     digitalWrite(pinLF,HIGH); 
   
    }
void turnR()        //turning right(dual wheel)
    {
     digitalWrite(pinRB,HIGH);  //making motor move towards right rear
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,LOW);
     digitalWrite(pinLF,HIGH);  //making motor move towards left front
  
    }
void turnL()         //turning left(dual wheel)
    {
     digitalWrite(pinRB,LOW);
     digitalWrite(pinRF,HIGH );   //making motor move towards right front
     digitalWrite(pinLB,HIGH);   //making motor move towards left rear
     digitalWrite(pinLF,LOW);
    
    }    
void stopp()        //stop
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,HIGH);
    
    }
void back()         //back up
    {
     digitalWrite(pinRB,HIGH);  //making motor move towards right rear     
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,HIGH);  //making motor move towards left rear
     digitalWrite(pinLF,LOW);
      
    }

void Obstacle_Avoidance_Main() {
  left_IR_Sensor = digitalRead(9);
  right_IR_Sensor = digitalRead(10);
  Front_Distance = checkdistance();
  Serial.println(Front_Distance);
  Infrared_Obstacle_Avoidance();
  Ultrasonic_obstacle_avoidance();
}

void IR_Control(void)
{
   unsigned long Key;
   if(irrecv.decode(&results)) //judging if serial port receives data   
 {
     Key = results.value;
    switch(Key)
     {
       case IR_Go :
       advance();     //UP
       break;
       case IR_Back: back();   //back
       break;
       case IR_Left:turnL();   //Left   
       break;
       case IR_Right:turnR(); //Right
       break;
       case IR_Stop:stopp();   //stop
       break;
       default: 
       break;      
     } 
    irrecv.resume(); // Receive the next value
    } 
}

void setup(){
  M_Control_IO_config();
  Sensor_Config();
  irrecv.enableIRIn();
  Set_Speed(Lpwm_val,Rpwm_val);
  myservo.attach(A2);
  D_mix = 7;
  D_mid = 13;
  D_max = 400;
  Front_Distance = 0;
  Left_Distance = 0;
  Right_Distance = 0;
  right_IR_Sensor = 1;
  left_IR_Sensor = 1;
  myservo.write(90);
  Serial.begin(9600);  
  stopp();  
}


void loop(){
  Obstacle_Avoidance_Main();
}
