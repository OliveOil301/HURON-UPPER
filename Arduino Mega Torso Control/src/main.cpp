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
#define M1PWM 13
#define M1DIR1 15
#define M1DIR2 16
// motor 2 pins
#define M2INTERR 3
#define M2READ 28
#define M2PWM 12
#define M2DIR1 17
#define M2DIR2 18
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
#define M4DIR1 10
#define M4DIR2 0

actuatorCon act1 = actuatorCon(M1INTERR, M1READ, M1PWM, M1DIR1, M1DIR2);
actuatorCon act2 = actuatorCon(M2INTERR, M2READ, M2PWM, M2DIR1, M2DIR2);
actuatorCon act3 = actuatorCon(M3INTERR, M3READ, M3PWM, M3DIR1, M3DIR2);
actuatorCon act4 = actuatorCon(M4INTERR, M4READ, M4PWM, M4DIR1, M4DIR2);

int *readData()
{
  int i = 0;
  int tempArr[] = {0, 0, -1, 0, 0, -1, 0, 0, -1, 0, 0, -1}; // Four "ints" with length of 3
  static int finalArray[] = {-1, -1, -1, -1};               // Four actual ints (3 digits)

  while (Serial.available())
  {
    char inputChar = Serial.read();
    delay(10);
    // Serial.println(inputChar);
    tempArr[i] = inputChar - 48;
    i++;
  }

  for (int j = 0; j <= 9; j += 3)
  {
    finalArray[j / 3] = tempArr[j] * 100 + tempArr[j + 1] * 10 + tempArr[j + 2];
  }

  // Serial.println(finalArray[0]);
  // Serial.println(finalArray[1]);
  // Serial.println(finalArray[2]);
  // Serial.println(finalArray[3]);

  return finalArray;
}

// I reversed this ISR in order to get it working, but this might be because I wired it incorrectly
void motor1ISR()
{
  if (digitalRead(M1READ))
  {
    act1.setTics(act1.getTics() - 1);
  }
  else
  {
    act1.setTics(act1.getTics() + 1);
  }
}

void motor2ISR()
{
  if (digitalRead(M2READ))
  {
    act2.setTics(act2.getTics() - 1);
  }
  else
  {
    act2.setTics(act2.getTics() + 1);
  }
}

void motor3ISR()
{
  if (digitalRead(M3READ))
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
  if (digitalRead(M4READ))
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

  // Each actuator DESPERATELY needs to have a starting point of reference
  // For simplicity's sake for now, I just put it at the midpoint. This can be tweaked when the robot is assembled.
  act1.setTics(22821);
  act2.setTics(22821);



}

void loop()
{
  int *q;
  q = readData();

  act2.moveAct(303);
  act1.moveAct(303);


  // if (q[0] != -1)
  // {
  //   for (int i = 0; i < 4; i++)
  //   {
  //     Serial.println(q[i]);
  //   }
  // }


}