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
#define M1_MIN_POT 870
#define M1_MAX_POT 990

// motor 2 pins
#define M2INTERR 2
#define M2READ 41
#define M2PWM 12
#define M2DIR1 30
#define M2DIR2 31
#define M2POT A2
#define M2_MIN_POT 870
#define M2_MAX_POT 990

// motor 3 pins
#define M3INTERR 18
#define M3READ 42
#define M3PWM 11
#define M3DIR1 32
#define M3DIR2 33
#define M3POT A3
#define M3_MIN_POT 870
#define M3_MAX_POT 990

// motor 4 pins
#define M4INTERR 19
#define M4READ 43
#define M4PWM 10
#define M4DIR1 34
#define M4DIR2 35
#define M4POT A4
#define M4_MIN_POT 870
#define M4_MAX_POT 990


actuatorCon act1 = actuatorCon(M1INTERR, M1READ, M1PWM, M1DIR1, M1DIR2, M1POT, M1_MIN_POT, M1_MAX_POT);
actuatorCon act2 = actuatorCon(M2INTERR, M2READ, M2PWM, M2DIR1, M2DIR2, M2POT, M2_MIN_POT, M2_MAX_POT);
actuatorCon act3 = actuatorCon(M3INTERR, M3READ, M3PWM, M3DIR1, M3DIR2, M3POT, M3_MIN_POT, M3_MAX_POT);
actuatorCon act4 = actuatorCon(M4INTERR, M4READ, M4PWM, M4DIR1, M4DIR2, M4POT, M4_MIN_POT, M4_MAX_POT);


void motor1ISR()
{
  if (digitalRead(M1READ))
  {
    act1.decrementTicks();
  }
  else
  {
    act1.incrementTicks();
  }
}

void motor2ISR()
{
  if (digitalRead(M2READ))
  {
    act2.decrementTicks();
  }
  else
  {
    act2.incrementTicks();
  }
}

void motor3ISR()
{
  if (digitalRead(M3READ))
  {
    act3.decrementTicks();
  }
  else
  {
    act3.incrementTicks();
  }
}

void motor4ISR()
{
  if (digitalRead(M4READ))
  {
    act4.decrementTicks();
  }
  else
  {
    act4.incrementTicks();
  }
}
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

unsigned long currentTime = 0;

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


void setup()
{
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    //for(;;); // Don't proceed, loop forever
      // Uncomment this if you're using an I2C screen to debug. 
  }

  printDebugToScreen("Setup Starting");

  //Begin serial communication so we can connect to the MATLAB script
  Serial.begin(9600);

  // attaching interrupts so we can use the encoders for positioning
  attachInterrupt(digitalPinToInterrupt(M1INTERR), motor1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M2INTERR), motor2ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M3INTERR), motor3ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M4INTERR), motor4ISR, RISING);

  //Using the potentiometers to set the starting positions of the actuators
  //act1.setPositionFromPotentiometer();
  //act2.setPositionFromPotentiometer();
  //act3.setPositionFromPotentiometer();
  //act4.setPositionFromPotentiometer();

  //If the potentiometers aren't installed, this can be used to manually set the positions
  act1.setLen(241);
  act2.setLen(260);

  //This is used to save the current position to be used for interpolation. 
  // This needs to be called once before any interpolation movement so we can know what
  // position the interpolation started from
  act1.setStartingPosition();
  act2.setStartingPosition();
  // act3.setStartingPosition();
  // act4.setStartingPosition();

  //Set the first goal of the torso to moving to the home position.
  setHomePositionGoal();


  printDebugToScreen("Setup Complete");
}



void loop()
{
  //* Reading the most recent command sent to the microcontroller
  /** after reading the command, we'll deal with it 
   * If it was a MOVE command, it gets added to the queue, if possible
   * If it was a GET command, send whatever information was requested
   * 
  */
  Command tempComm = readCommand();
  currentTime = millis();

  if(tempComm == NONE){//If there isn't a full command or not a full command yet
  //Do nothing
  } else if(tempComm == MOVE){//If there was just a MOVE command read
    printDebugToScreen("MOVE");
    if(addMoveGoal(commandDigits)){//Add it to the queue, if possible
      printActuatorGoals();
    } else {
      printDebugToScreen("MOVE Queue full");
      Serial.write("E: Queue Full ");// sending an error message to MATLAB 

    }
  } else if(tempComm == GET){//If there was just a GET command read
    printDebugToScreen("GET");
  }
  //At this point, the commands have just been read and the Move queue is as updated
  // as it's going to get for now

  //* Checking if we've reached the goal position, setting new goals for actuators
  /** At this point, we're going to check if all the actuators are at the current goal position
   *    If so, we'll do the following:
   *      - Send a C (CONFIRMATION) command to the MATLAB script to 
   *          say we've moved to the current goal position
   *      - Update the current goal to the next position
   * After that, we'll check if there is a goal at the current goal position.
   *    If so, we'll do the following:
   *      - Set the individual actuator goals based on the calculated 
   * 
  */






  if(digitalRead(50)==LOW){//If we pressed the button
    Serial.write("C123123123123");
    printDebugToScreen("Command Sent to MATLAB");
  }

}