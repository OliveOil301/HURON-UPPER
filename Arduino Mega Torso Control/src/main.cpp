#include <Arduino.h>
#include "actuatorCon.h"

//* MOTOR PIN ASSIGNMENT *//
/*
    x = Motor Number
    MxINTERR = Interrupt Pin for Motor x
    MxREAD = Read Pin for Motor x
    MxPWM = PWM Pin for Motor x
    MxDIR1 = Directional Pin 1 for Motor x
    MxDIR2 = Directional Pin 2 for Motor x
*/

// motor 1 pins
#define M1INTERR 2
#define M1READ 30
#define M1PWM 0
#define M1DIR1 4
#define M1DIR2 16
// motor 2 pins
#define M2INTERR 17
#define M2READ 5
#define M2PWM 18
#define M2DIR1 19
#define M2DIR2 21
// motor 3 pins
#define M3INTERR 34
#define M3READ 35
#define M3PWM 32
#define M3DIR1 33
#define M3DIR2 25
// motor 4 pins
#define M4INTERR 26
#define M4READ 27
#define M4PWM 11
#define M4DIR1 12
#define M4DIR2 13

actuatorCon act1 = actuatorCon(M1INTERR, M1READ, M1PWM, M1DIR1, M1DIR2);
actuatorCon act2 = actuatorCon(M2INTERR, M2READ, M2PWM, M2DIR1, M2DIR2);
actuatorCon act3 = actuatorCon(M3INTERR, M3READ, M3PWM, M3DIR1, M3DIR2);
actuatorCon act4 = actuatorCon(M4INTERR, M4READ, M4PWM, M4DIR1, M4DIR2);

int *readData()
{
  int *tempArr = {-1,-1,-1, -1,-1,-1, -1,-1,-1, -1,-1,-1}; // Four ints with length of 3 
  while (Serial.available())
  {
    int numOfAct = 4;
    int i = 0;

    // converting string to char*
    // String data = Serial.readStringUntil('\n');
    // char d2[data.length() + 1];

    // strcpy(d2, data.c_str());

    // char *token = strtok(d2, " ");

    // while (token != NULL)
    // {
    //   tempArr[i] = atoi(token);
    //   token = strtok(NULL, " ");
    //   i++;
    // }

    // if (i == numOfAct)
    // {
    //   return tempArr;
    // }

    char inputChar = Serial.read();
    if (inputChar == "\n")
    {
      break;
    }
    else
    {
      tempArr[i] = inputChar;
      i++;
    }
  }

  return -1;
}

void motor1ISR()
{
  if (digitalRead(M2READ))
  {
    act1.setTics(act1.getTics() + 1);
  }
  else
  {
    act1.setTics(act1.getTics() - 1);
  }
}

void motor2ISR()
{
  if (digitalRead(M2READ))
  {
    act2.setTics(act2.getTics() + 1);
  }
  else
  {
    act2.setTics(act2.getTics() - 1);
  }
}

void motor3ISR()
{
  if (digitalRead(M2READ))
  {
    act3.setTics(act3.getTics() + 1);
  }
  else
  {
    act3.setTics(act3.getTics() - 1);
  }
}

void motor4ISR()
{
  if (digitalRead(M2READ))
  {
    act4.setTics(act4.getTics() + 1);
  }
  else
  {
    act4.setTics(act4.getTics() - 1);
  }
}

void setup()
{
  Serial.begin(9600);

  //* Interrupt Service Routine initialization *//
  attachInterrupt(digitalPinToInterrupt(M1INTERR), motor1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M2INTERR), motor2ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M3INTERR), motor3ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M4INTERR), motor4ISR, RISING);
}

void loop()
{
  int q = readData();
  if (q != -1)
  {

    Serial.println();
  }
}