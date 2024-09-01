# H-Bridge proof of concept

//minimale hard- & software die aantoont dat 2 motoren onafhankelijk van elkaar kunnen draaien, en (traploos) regelbaar zijn in snelheid en draairichting.

#define rightMotor1 2
#define rightMotor2 3
#define rightMotorPWM 7

#define leftMotor1 4
#define leftMotor2 5
#define leftMotorPWM 6

#define MaxSpeed 255
#define BaseSpeed 255

int pot1 = A0;
int SetSpeed = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  SetSpeed = 200;
  
}

void loop() {
  // put your main code here, to run repeatedly:

  motorKiri();
  delay(1500);
  motorKanan();
  delay(1500);
  motorKiri();
  delay(1500);
  motorMaju();
  delay(1500);
  motorMundur();
  delay(1000);
  motorStop();
  delay(1000);
  
}

void move(int motor, int speed, int direction) {

  boolean inPin1=HIGH;
  boolean inPin2=LOW;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  
  if(direction == 0){
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if(motor == 0) {
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin1);
    analogWrite(leftMotorPWM, speed);
  }

  if(motor == 1) {
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin1);
    analogWrite(rightMotorPWM, speed);
  }
}

void motorStop(){
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, 0);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, 0);
}

void motorKanan(){
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, 120);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, 120);
}

void motorKiri(){
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, 120);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, 120);
}

void motorMaju(){
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, SetSpeed);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, SetSpeed);
}

void motorMundur(){
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, SetSpeed);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, SetSpeed);
}
