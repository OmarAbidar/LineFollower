#include <QTRSensors.h> 
/*************************************************************************
*  Sensor Array object initialisation 
*************************************************************************/
QTRSensors qtr;  
const uint8_t SensorCount = 8; 
uint16_t sensorValues[SensorCount];
/*************************************************************************
*  PID control system variables 
*************************************************************************/
int16_t P = 0, I = 0, D = 0;  //int
float Kp = 0.12 ,Ki = 0.00002, Kd = 1.7; //ki 0.00004    kd 1.3
//float Kp = 0.07 ,Ki = 0.0008, Kd = 0.6;
/*************************************************************************
*  Global variables
*************************************************************************/ 
int lastError = 0;  

/*************************************************************************
*  DRV8835 GPIO pins declaration
*************************************************************************/
int ENA=5;      //Branché sur Arduino pin 5(output pwm)
int IN1=14;     //Sur Arduino pin 14
int IN2=15;     //Sur Arduino pin 15
int ENB=6;      //Sur Arduino pin 6(output pwm)
int IN3=16;     //Sur Arduino pin 16
int IN4=17;     //Sur Arduino pin 17
/*************************************************************************
*  Buttons pins declaration
*************************************************************************/
int button_start = 18;   //Sur Arduino pin 18
int button_calibre = 26; //Sur Arduino pin 26
 
void calibration(); 
  
void setup() 
{   // put your setup code here, to run once:  
  qtr.setTypeRC();   
  qtr.setSensorPins((const uint8_t[]){3, 4, 7, 8, 9, 10, 11, 12}, SensorCount);  
  qtr.setEmitterPin(2);

  pinMode(LED_BUILTIN, OUTPUT);
  delay(500);

  pinMode(ENA,OUTPUT);  //pin output
  pinMode(ENB,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);//stop driving
  digitalWrite(IN1,LOW); //HIGH
  digitalWrite(IN2,LOW);//setting motorA's direction
  digitalWrite(IN3,LOW); //HIGH
  digitalWrite(IN4,LOW);//setting motorB's direction

  Serial.begin(9600);
  Serial.print("OMAR_PID");

   boolean Ok = false;
    while (Ok == false) {         // the main function won't start  until the robot is calibrated
    if(digitalRead(button_calibre) == HIGH) {
      calibration();              //calibrate for 10 seconds
      Ok = true;
    }
  }
    
  pinMode(button_start, INPUT);       //10 seconds    
 while(digitalRead(button_start) == LOW) {} 
}  

void loop()
 {   // put your main code here, to run repeatedly:   
 PID_control(); 
 }  

 
void PID_control() 
{  
  uint16_t positionLine = qtr.readLineBlack(sensorValues); 
  int error = 3500 - positionLine; // 3500 is the ideal position (the center)
  
  P = error;   
  I = error + I;   
  D = error - lastError;   
  lastError = error;   
  
  int motorSpeedChange = P*Kp + I*Ki + D*Kd; 
  int motorSpeedA = 155 ,  motorSpeedB = 155; //150

  if (motorSpeedChange < -90) // Turn left
  {
      digitalWrite(IN1, HIGH);   // Set motor A to reverse
      digitalWrite(IN2, LOW);
      
      motorSpeedA = abs(motorSpeedChange) + 155;  //150
      if(motorSpeedA > 255) motorSpeedA = 255;
      
      digitalWrite(IN3, LOW);  // Set motor B to forward
      digitalWrite(IN4, HIGH);
  }
  else if(motorSpeedChange > 135) // Turn right
  {
      digitalWrite(IN1, LOW);  // Set motor A to forward
      digitalWrite(IN2, HIGH);
      
      motorSpeedB = abs(motorSpeedChange) + 155; //150
      if(motorSpeedB > 255) motorSpeedB = 255;
      
      digitalWrite(IN3, HIGH);   // Set motor B to reverse
      digitalWrite(IN4, LOW);
      
  }
  else // Move forward
  {        
      digitalWrite(IN1, HIGH);  // Set motor A to forward
      digitalWrite(IN2, LOW);
      
      digitalWrite(IN3, HIGH);  // Set motor B to forward
      digitalWrite(IN4, LOW);
      
      motorSpeedA = 155; 
      motorSpeedB = 155;
  }
/*
  Serial.print("POSITION:"); 
  Serial.print(positionLine);
  
  Serial.println();
  delay(300);
  
  Serial.print("MOTOR_A:");   
  Serial.print(motorSpeedA);
  Serial.println();
  delay(100);
  Serial.print("MOTOR_B:"); 
  Serial.print(motorSpeedB);
  Serial.println();
     
  Serial.print("PID:"); 
  Serial.print(motorSpeedChange);
  Serial.println();*/
    forward_movement(motorSpeedA, motorSpeedB); 
}
  
void forward_movement(int speedA, int speedB) 
 {        
              analogWrite(ENA, speedA); // Turn on motor A in direction 1  
              analogWrite(ENB, speedB); // Turn on motor B in direction 1                           
 } 

void calibration() {

  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  for (uint16_t i = 0; i < 400; i++)  //400
    {   qtr.calibrate();
    delay(20);} 
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
  digitalWrite(IN1,HIGH);
  digitalWrite(IN3,HIGH);  //calibration terminer
   
}
