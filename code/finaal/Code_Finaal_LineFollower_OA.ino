#include "SerialCommand.h"
#include "EEPROMAnything.h"

#define SerialPort Serial
#define Baudrate 9600

#define MotorRechtsVooruit 9
#define MotorRechtsAchteruit 3
#define MotorLinksVooruit 10
#define MotorLinksAchteruit 11


SerialCommand sCmd(SerialPort);
bool debug;
bool run;
unsigned long previous, calculationTime;


const int sensor[] = {A7, A6, A5, A4, A3, A2, A1, A0};


struct param_t
{
  unsigned long cycleTime;
  int black[8];
  int white[8];
  float diff;
  float kp;
  int power;
  float ki;
  float kd;

} params;

int normalised[8];
float debugposition;
float output;
float error;
float iTerm = 0;
float lastErr;


void setup()
{


  pinMode(9, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  
  SerialPort.begin(Baudrate);

  sCmd.addCommand("set", onSet);
  sCmd.addCommand("debug", onDebug);
  sCmd.addCommand("calibrate", onCalibrate);
  sCmd.addCommand("run", onRun);
  sCmd.addCommand("stop", onStop);
  sCmd.setDefaultHandler(onUnknownCommand);

  EEPROM_readAnything(0, params);
  SerialPort.println("ready");
}

void loop()
{
  sCmd.readSerial();
 
  unsigned long current = micros();
  if (current - previous >= params.cycleTime)
  {
    previous = current;


    


    for (int i = 0; i < 8; i++)
    {
      normalised[i] = map(analogRead(sensor[i]), params.black[i], params.white[i], 0, 1000);

      
    }


    int index = 0;
    for (int i = 1; i < 8; i++) if (normalised[i] < normalised[index]) index = i;
    

    float position;

    if (index == 0) position = -30;
    else if (index == 7) position = 30;
    else
    {
    int sNul = normalised[index];
    int sMinEen = normalised[index-1];
    int sPlusEen = normalised[index+1];

    float b = sPlusEen - sMinEen;
    b = b / 2;

    float a = sPlusEen - b - sNul;

    position = -b / (2 * a);
    position += index;
    position -= 3.5;

    position *= 9.525; //afstand sensoren uit elkaar waarschijnlijk aanpassen

    }
     
debugposition = position;

error = -position;


//P-regelaar
output = error * params.kp;

//I-Regelaar

iTerm += params.ki * error;
iTerm = constrain(iTerm, -510, 510);
output += iTerm;

//D-Regelaar

output += params.kd * (error - lastErr);
lastErr = error;

output = constrain(output, -510, 510);

int powerLeft = 0;
int powerRight = 0;


if (run) if (output >= 0)
{
  powerLeft = constrain(params.power + params.diff * output, -255, 255);
  powerRight = constrain(powerLeft - output, -255, 255);
  powerLeft = powerRight + output;
}
else
{
  
  powerRight = constrain(params.power - params.diff * output, -255, 255);
  powerLeft = constrain(powerRight + output, -255, 255);
  powerRight = powerLeft - output;
  
}

analogWrite(MotorLinksVooruit, powerLeft > 0 ? powerLeft : 0);
analogWrite(MotorLinksAchteruit, powerLeft < 0 ? -powerLeft : 0);
analogWrite(MotorRechtsVooruit, powerRight > 0 ? powerRight : 0);
analogWrite(MotorRechtsAchteruit, powerRight < 0 ? -powerRight : 0);
}


  unsigned long difference = micros() - current;
  if (difference > calculationTime) calculationTime = difference;
}

void onUnknownCommand(char *command)
{
  SerialPort.print("unknown command: \"");
  SerialPort.print(command);
  SerialPort.println("\"");
}

void onSet()
{
  char* param = sCmd.next();
  char* value = sCmd.next();  
  
 if (strcmp(param, "cycle") == 0)
  {
    long newCycleTime = atol(value);
    float ratio = ((float) newCycleTime) / ((float) params.cycleTime);

    params.ki *= ratio;
    params.kd /= ratio;

    params.cycleTime = newCycleTime;
  }
  else if (strcmp(param, "ki") == 0)
  {
    float cycleTimeInSec = ((float) params.cycleTime) / 1000000;
    params.ki = atof(value) * cycleTimeInSec;
  }
  else if (strcmp(param, "kd") == 0)
  {
    float cycleTimeInSec = ((float) params.cycleTime) / 1000000;
    params.kd = atof(value) / cycleTimeInSec;
  }
  else if (strcmp(param, "power") == 0) params.power = atol(value);
  else if (strcmp(param, "diff") == 0) params.diff = atof(value);
  else if (strcmp(param, "kp") == 0) params.kp = atof(value);

  
  
  EEPROM_writeAnything(0, params);
}

void onDebug()
{
  SerialPort.print("cycle time: ");
  SerialPort.println(params.cycleTime);
  
  /* parameters weergeven met behulp van het debug commando doe je hier ... */
  SerialPort.print("black: ");
  for (int i = 0; i < 8; i++)
  {
    SerialPort.print(params.black[i]);
    SerialPort.print(" ");
  }
  SerialPort.println(" ");

  SerialPort.print("white: ");
  for (int i = 0; i < 8; i++)
  {
    SerialPort.print(params.white[i]);
    SerialPort.print(" ");
  }
SerialPort.println(" ");

  SerialPort.print("Normalised: ");
  for (int i = 0; i < 8; i++)
  {
    SerialPort.print(normalised[i]);
    SerialPort.print(" ");
  }
  SerialPort.println(" ");

  SerialPort.print("position: ");
  SerialPort.println(debugposition);

  SerialPort.print("Power: ");
  SerialPort.println(params.power);

  SerialPort.print("diff: ");
  SerialPort.println(params.diff);

  SerialPort.print("kp: ");
  SerialPort.println(params.kp);

  float cycleTimeInSec = ((float) params.cycleTime) / 1000000;
  float ki = params.ki / cycleTimeInSec;
  SerialPort.print("Ki: ");
  SerialPort.println(ki);

  float kd = params.kd * cycleTimeInSec;
  SerialPort.print("Kd: ");
  SerialPort.println(kd);

  SerialPort.print("RUN: ");
  SerialPort.println(params.kp);
  
  SerialPort.println(" ");
  SerialPort.print("calculation time: ");
  SerialPort.println(calculationTime);
  calculationTime = 0;
}

void onCalibrate()
{
  char* param = sCmd.next();

  if (strcmp(param, "black") == 0)
  {
    SerialPort.print("start calibrating black...");
    for (int i = 0; i < 8; i++) params.black[i] = analogRead(sensor[i]);
    SerialPort.println("done");
  }
  else if (strcmp(param, "white") == 0)
  {
    SerialPort.print("start calibrating white...");
    for (int i =0; i < 8; i++) params.white[i] = analogRead(sensor[i]);
    SerialPort.println("done");
  }

   EEPROM_writeAnything(0, params);
}

void onRun()
{
     run = HIGH;
}

void onStop()
{
  run = LOW;
  iTerm = 0;
}
