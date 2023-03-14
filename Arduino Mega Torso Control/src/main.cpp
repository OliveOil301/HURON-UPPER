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

//Communication global variables:
enum Command{
  NONE,
  MOVE,
  GET
};
/** Command Start Characters:
 * 'M' - MOVE, move to the position indicated in the following digits
 * 'G' - GET, get data on the current position
*/

int commandDigits[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//This array holds the digits of the most recent command. If there is no command, it should be all 0

int actuatorGoalPosition[] = {0, 0, 0, 0};//the goal position of the actuators, in millimeters
//TODO: Change this to the values that result in a home posiiton

/** void clearSerialUntilCommand()
 * clears each character until a valid command start character is recieved.
 * This helps to stop a slight communication error from making the whole system crash.
 * Only the affected command will be lost, not each subsequent one
*/
void clearSerialUntilCommand(){
  char currentChar = Serial.read();
  while ((currentChar != 'M' || currentChar != 'G') && Serial.read() >= 1){
    currentChar = Serial.read();
  }
}

/** bool readCommand()
 * @return true if a command was recieved, false otherwise
 * If a command was recieved, the command type is stored in the global variable "currentCommand"
 * If a move command was recieved, the actuatorGoalPosition list is updated with the requested position
*/
Command readCommand()
{
  if (Serial.available() < 13){
    //if there are less than 13 characters in the serial port
    //return an int array that shows there is no command or it isnt a full one yet
    return NONE;
  } else {

    //If we're here, there are at least 13 characters in the serial bus
    //we now need to check if the first one is a valid command character (M or G)
    char firstChar = Serial.read();
    if (firstChar != 'G' || firstChar != 'M'){
      //If it's not a valid command start character,
      //clear the serial bus until a valid start character is found
      //then return false and the next loop can try to read the command
      clearSerialUntilCommand();
      return NONE;
    } else {

      //At this point, we have enough characters and it's a valid 
      //command start character at the beginning
      //We'll read through the chars and add them to the global array before returning the 
      //command type
      
      //Read the 12 command digits
      for (int i = 0; i<12; i++){
        commandDigits[i] = Serial.read() - 48;
      }

      if (firstChar == 'M'){
        return MOVE;
      } else if (firstChar == 'G'){
        return GET;
      } else {
        return NONE; //This shouldn't happen but it's here for safety
      }
    
    }
  }
  // for (int j = 0; j <= 9; j += 3)
  // {
  //   finalArray[j / 3] = tempArr[j] * 100 + tempArr[j + 1] * 10 + tempArr[j + 2];
  // }

  // // Serial.println(finalArray[0]);
  // // Serial.println(finalArray[1]);
  // // Serial.println(finalArray[2]);
  // // Serial.println(finalArray[3]);

  // return finalArray;
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
  pinMode(LED_BUILTIN, OUTPUT);

  //* Interrupt Service Routine initialization *//
  attachInterrupt(digitalPinToInterrupt(M1INTERR), motor1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M2INTERR), motor2ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M3INTERR), motor3ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M4INTERR), motor4ISR, RISING);
}


int state = 0;
unsigned long lastOnTime = 0;

Command comm = NONE;

void loop()
{
  Command tempComm = readCommand();
  if(tempComm != NONE){
    comm = tempComm;
  }
  
  unsigned long currentTime = millis();
  switch (comm){
    case NONE:
      //
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case MOVE:
      //
      if (currentTime >= lastOnTime+1000){
        digitalWrite(LED_BUILTIN, HIGH);
        if(currentTime >= lastOnTime+2000){
          lastOnTime = currentTime;
        }
      } else {
        digitalWrite(LED_BUILTIN, LOW);
      }
      break;
    case GET:
      //
      if (currentTime >= lastOnTime+200){
        digitalWrite(LED_BUILTIN, HIGH);
        if(currentTime >= lastOnTime+400){
          lastOnTime = currentTime;
        }
      } else {
        digitalWrite(LED_BUILTIN, LOW);
      }
      break;
  }
}