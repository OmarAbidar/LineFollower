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
float Kp = 0.05 ,Ki = 0.00001, Kd = 0.8;   
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
                         // read calibrated sensor values and obtain a measure of the line position
                         // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t positionLine = qtr.readLineBlack(sensorValues); 

  int error = 3500 - positionLine; //3500 is the ideal position  (the centre)
  P = error;   I = error + I;   D = error - lastError;   lastError = error;   
  int motorSpeedChange = P*Kp + I*Ki + D*Kd; 
  int motorSpeedA = 0 ,  motorSpeedB = 0;
    
   if ( motorSpeedChange < -125 )
      {
         digitalWrite(IN1, HIGH);digitalWrite(IN3, LOW);
          
         motorSpeedA = (abs (motorSpeedChange)/2 ) + 20  ; 
         if(motorSpeedA > 75) motorSpeedA = 90 ;              //adapt speed with motor sensitivity
          motorSpeedB = 0;    
      } 
   else if( motorSpeedChange > 80 )
      {
          digitalWrite(IN1, LOW);digitalWrite(IN3, HIGH);
          
          motorSpeedB = ( motorSpeedChange/2 )  + 20;   
          if(motorSpeedB > 75) motorSpeedB = 90 ;            //adapt speed with motor sensitivity
          motorSpeedA = 0  ;  
      }
   else                
      {        
         digitalWrite(IN1, HIGH);digitalWrite(IN3, HIGH);
          motorSpeedA = 90;       //motorSpeedChange + 50;    //adapt speed with motor sensitivity
          motorSpeedB = 90;       //motorSpeedChange - 50;               
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
  for (uint16_t i = 0; i < 400; i++)  
    {   qtr.calibrate();  } 
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
  digitalWrite(IN1,HIGH);
  digitalWrite(IN3,HIGH);  //calibration terminer
   
}
