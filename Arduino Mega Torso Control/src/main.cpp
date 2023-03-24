#include <Arduino.h>
#include "actuatorCon.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Screen stuff for debugging:
#define DEBUG_MODE true

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);




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
#define M1INTERR 3
#define M1READ 40
#define M1PWM 13
#define M1DIR1 28
#define M1DIR2 29
#define M1POT A1
// motor 2 pins
#define M2INTERR 2
#define M2READ 41
#define M2PWM 12
#define M2DIR1 30
#define M2DIR2 31
#define M2POT A2
// motor 3 pins
#define M3INTERR 18
#define M3READ 42
#define M3PWM 11
#define M3DIR1 32
#define M3DIR2 33
#define M3POT A3
// motor 4 pins
#define M4INTERR 19
#define M4READ 43
#define M4PWM 10
#define M4DIR1 34
#define M4DIR2 35
#define M4POT A4

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

int actuatorPositionGoals[10][4];//the goal position of the actuators, in millimeters
int currentPositionGoalIndex = 0; //The index of the above array for the current position goal.
int newPositionGoalIndex = 1; // This stores the index for the next goal to be set to. This is used to keep track of the new goals in the system 
//TODO: Add a goal in setup() that adds a goal for the home position.

/** void clearSerialUntilCommand()
 * clears each character until a valid command start character is recieved.
 * This helps to stop a slight communication error from making the whole system crash.
 * Only the affected command will be lost, not each subsequent one
*/
void clearSerialUntilCommand(){
  char nextChar = Serial.peek();
  while (nextChar != 'M' && nextChar != 'G' && Serial.available() > 0){
    Serial.read();
  }
  //Just read the first char to clear it from the stack
}

void printDebugToScreen(String line){
  if (DEBUG_MODE == true){
    display.clearDisplay();
    display.setTextSize(1); 
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 0);
    display.println(line);
    display.display();     
  }
}

void printActuatorGoals(){
  // Only works on newPositionGoalIndex from 1-9, not 0
  // Can be fixed but would make it a bit more complicated. 
  // Not worth it since this is jsut for testing
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.print(String(actuatorPositionGoals[newPositionGoalIndex-1][0]));
  display.print(", ");
  display.print(String(actuatorPositionGoals[newPositionGoalIndex-1][1]));
  display.print(", ");
  display.print(String(actuatorPositionGoals[newPositionGoalIndex-1][2]));
  display.print(", ");
  display.print(String(actuatorPositionGoals[newPositionGoalIndex-1][3]));
  display.display(); 

}

/** void setHomePositionGoal()
 * sets the first position goal to be the home position of the robot
*/
void setHomePositionGoal(){
  actuatorPositionGoals[0][0] = 276;
  actuatorPositionGoals[0][1] = 276;
  actuatorPositionGoals[0][2] = 266;
  actuatorPositionGoals[0][3] = 266;
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
    //return NONE since there isn't a command or isn't one yet
    return NONE;
  } else {
    printDebugToScreen("Command Recieved (117)");

    //If we're here, there are at least 13 characters in the serial bus
    //we now need to check if the first one is a valid command character (M or G)
    char firstChar = Serial.read();
    if (firstChar != 'G' && firstChar != 'M'){
      printDebugToScreen("No valid st char (123)");
      printDebugToScreen(String(firstChar));
      //If it's not a valid command start character,
      //clear the serial bus until a valid start character is found
      //then return false and the next loop can try to read the command
      clearSerialUntilCommand();
      return NONE;
    } else {
      printDebugToScreen("Valid st char (130)");

      //At this point, we have enough characters and it's a valid 
      //command start character at the beginning
      //We'll read through the chars and add them to the global array before returning the 
      //command type
      
      //Read the 12 command digits
      for (int i = 0; i<12; i++){
        commandDigits[i] = Serial.read() - 48;
      }

      if (firstChar == 'M'){
        //Serial.println("MOVE");
        return MOVE;
      } else if (firstChar == 'G'){
        //Serial.println("GET");
        return GET;
      } else {
        return NONE; //This shouldn't happen but it's here for safety
      }
    }
  }
}

/** bool addMoveGoal(int act1, int act2, int act3, int act4)
 * @param act1: an integer representing the goal for the first actuator
 * @param act2: an integer representing the goal for the second actuator
 * @param act3: an integer representing the goal for the third actuator
 * @param act4: an integer representing the goal for the fourth actuator
 * @returns: a boolean representing whether the goal was added to the list. 
 *      If there are already 10 goals that have not yet been completed, the
 *      goal cannot be added to the list and false will be returned. 
 *      Otherwise, true is returned
*/
bool addMoveGoal(int* commandNumbers){
  if(newPositionGoalIndex == currentPositionGoalIndex){
    //If we will overwrite the oldest goal, return false and disregard the movement goal
    return false;
  } else { //If we are good to add the goal to the array:

    //Parse the commandNumbers to get the actual actuator lengths
    int actuatorPositions[4] = {0, 0, 0, 0};
    for (int j = 0; j <= 9; j += 3)
    {
      actuatorPositions[j / 3] = commandNumbers[j] * 100 + commandNumbers[j + 1] * 10 + commandNumbers[j + 2];
    }

    //add the new goal to the correct position in the array
    actuatorPositionGoals[newPositionGoalIndex][0] = actuatorPositions[0];
    actuatorPositionGoals[newPositionGoalIndex][1] = actuatorPositions[1];
    actuatorPositionGoals[newPositionGoalIndex][2] = actuatorPositions[2];
    actuatorPositionGoals[newPositionGoalIndex][3] = actuatorPositions[3];

    //index the position while keepping it constrained from 0 to 9 with modulo
    newPositionGoalIndex = (newPositionGoalIndex+1)%10;
    
    return true;// Now return true since everything worked out correctly
  }
}


//*Motor functions:----------------

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
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for(;;); // Don't proceed, loop forever
  }
  printDebugToScreen("Setup Starting");

  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  //* Interrupt Service Routine initialization *//
  attachInterrupt(digitalPinToInterrupt(M1INTERR), motor1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M2INTERR), motor2ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M3INTERR), motor3ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M4INTERR), motor4ISR, RISING);

  setHomePositionGoal();


  printDebugToScreen("Setup Complete");
}



void loop()
{
  Command tempComm = readCommand();

  if(tempComm == NONE){

  } else if(tempComm == MOVE){
    printDebugToScreen("MOVE");
    if(addMoveGoal(commandDigits)){
      printActuatorGoals();
    }
  } else {
    printDebugToScreen("GET");
  }

  // if(tempComm != NONE){
  //   comm = tempComm;
  // }
  
  // unsigned long currentTime = millis();
  // switch (comm){
  //   case NONE:
  //     //
  //     digitalWrite(LED_BUILTIN, LOW);
  //     break;
  //   case MOVE:
  //     //
  //     if (currentTime >= lastOnTime+1000){
  //       digitalWrite(LED_BUILTIN, HIGH);
  //       if(currentTime >= lastOnTime+2000){
  //         lastOnTime = currentTime;
  //       }
  //     } else {
  //       digitalWrite(LED_BUILTIN, LOW);
  //     }
  //     break;
  //   case GET:
  //     //
  //     if (currentTime >= lastOnTime+200){
  //       digitalWrite(LED_BUILTIN, HIGH);
  //       if(currentTime >= lastOnTime+400){
  //         lastOnTime = currentTime;
  //       }
  //     } else {
  //       digitalWrite(LED_BUILTIN, LOW);
  //     }
  //     break;
  // }
}