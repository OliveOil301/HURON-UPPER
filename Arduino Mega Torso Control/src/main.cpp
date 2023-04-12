#include <Arduino.h>
#include "actuatorCon.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Screen stuff for debugging:
#define DEBUG_MODE true

#define MAX_MILLIMETERS_PER_SECOND 6
#define MOVEMENT_SPEED_SAFETY_FACTOR 1.0

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
    act1.incrementTicks();
  }
  else
  {
    act1.decrementTicks();
  }
}

void motor2ISR()
{
  if (digitalRead(M2READ))
  {
    act2.incrementTicks();
  }
  else
  {
    act2.decrementTicks();
  }
}

void motor3ISR()
{
  if (digitalRead(M3READ))
  {
    act3.incrementTicks();
  }
  else
  {
    act3.decrementTicks();
  }
}

void motor4ISR()
{
  if (digitalRead(M4READ))
  {
    act4.incrementTicks();
  }
  else
  {
    act4.decrementTicks();
  }
}


enum State{
  IDLE,
  COMMAND_MOVING,
  COMMAND_WAITING,
  COMMAND_GETTING
};
State currentState = IDLE;

unsigned long currentTime = 0;
unsigned long movementTime = 0; // The time it's going to take for the next movement
unsigned long waitStartTime = 0;

//* QUEUE CODE ----------------------
#define MAX_COMMAND_QUEUE_LENGTH 10
int commandQueue[MAX_COMMAND_QUEUE_LENGTH][5];//the queue of commands
// The command queue is structured as follows:
// |   num1   |   num2   |   num3   |   num4   |   Command Char   |
// This is structured to make movements as easy to understand as possible but 
// also allows other commands to sit in the queue, like D (DELAY) and G (GET) commands

int currentCommandIndex = 0; //The index of the above array for the current position goal.
int commandInsertIndex = 0; // This stores the index for the next command to be set to. This is used to keep track of the new goals in the system 


//Communication variables:
char validCommandCharacters[3] = {'M','G','W'};
//TODO: Add a list of valid command characters that cna be iterated through with a command
/** Command Start Characters:
 * 'M' - MOVE, move to the position indicated in the following digits
 * 'G' - GET, get data on the current position
 * 'W' - WAIT, wait for a specified number of milliseconds
*/



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
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.print(String(commandQueue[currentCommandIndex][1]));
  display.print(", ");
  display.print(String(commandQueue[currentCommandIndex][2]));
  display.print(", ");
  display.print(String(commandQueue[currentCommandIndex][3]));
  display.print(", ");
  display.print(String(commandQueue[currentCommandIndex][4]));
  display.display(); 

}

/** void setHomePositionGoal()
 * sets the first position goal to be the home position of the robot
*/
void setHomePositionGoal(){
  commandQueue[0][0] = 276;
  commandQueue[0][1] = 276;
  commandQueue[0][2] = 266;
  commandQueue[0][3] = 266;
}


/** bool addGoalToQueue(int commandChar, int* commandNumbers)
 * @param commandChar: the char of the command start to be saved with the numbers
 * @param commandNumbers: a list of integers representing the actions for a given command
 * @returns: a boolean representing whether the goal was added to the list. 
 *      If there are already 10 goals that have not yet been completed, the
 *      goal cannot be added to the list and false will be returned. 
 *      Otherwise, true is returned
 *      false will also be returned if there was an error with the command (position isn't valid, etc.)
*/
bool addGoalToQueue(int commandChar, int* commandNumbers){
  if((commandInsertIndex+1)%MAX_COMMAND_QUEUE_LENGTH == currentCommandIndex){
    //If we will overwrite the oldest goal, return false and disregard the movement goal
    return false;
  }
  //If we are good to add the goal to the array, we can continue

  switch(commandChar){
    case 'M':{
      //Parse the commandNumbers to get the actual actuator lengths
      int actuatorPositions[4] = {0, 0, 0, 0};
      for (int j = 0; j <= 9; j += 3)
      {
        actuatorPositions[j / 3] = commandNumbers[j] * 100 + commandNumbers[j + 1] * 10 + commandNumbers[j + 2];
      }

      for(int i = 0; i<4; i++){
        if(actuatorPositions[i]>328 || actuatorPositions[i]<239){
          return false;//If any of the thengths are out of bounds
        }
      }

      //add the new goal to the correct position in the array
      commandQueue[commandInsertIndex][0] = commandChar;//Saving the type of command
      commandQueue[commandInsertIndex][1] = actuatorPositions[0];
      commandQueue[commandInsertIndex][2] = actuatorPositions[1];
      commandQueue[commandInsertIndex][3] = actuatorPositions[2];
      commandQueue[commandInsertIndex][4] = actuatorPositions[3];
      //index the position while keepping it constrained from 0 to 9 with modulo
      commandInsertIndex = (commandInsertIndex+1)%MAX_COMMAND_QUEUE_LENGTH;
      return true;// Now return true since everything worked out correctly
      }break;
    case 'G':{
      return true;
      }break;
    case 'W':{
      unsigned long waitTime = 0;
      for (int i = 11; i>=0; i--){
        //Iterating through each of the characters in the command
        waitTime += commandNumbers[i]*pow(10,i);
      }
      commandQueue[commandInsertIndex][0] = commandChar;//Saving the type of command
      commandQueue[commandInsertIndex][1] = waitTime;
      commandQueue[commandInsertIndex][2] = 0;
      commandQueue[commandInsertIndex][3] = 0;
      commandQueue[commandInsertIndex][4] = 0;
      return true;
      }break;
  }
  return true;
}


bool isValidStartCharacter(char character){
    for (int i = 0; i < 3; i++){
      if(character == validCommandCharacters[i]){
        return true;//If our character is one of the ones in the list of valid ones
      }
    }
    return false;//If it's not one of the valid ones
}


/** bool readCommand()
 * @return true if a command was recieved, false otherwise
 * If a command was recieved, the command type is stored in the global variable "currentCommand"
 * If a move command was recieved, the actuatorGoalPosition list is updated with the requested position
*/
bool readCommand()
{
  if (Serial.available() < 13){
    //IF: there aren't enough characters in the port for a command to be read,
    //   - return NONE since there isn't a command or isn't one yet
    return false;
  } else {
    //IF: there *are* enough characters in the port for a command to be read,
    // printDebugToScreen("got enough chars");
    // delay(1000);
    char firstChar = Serial.read();

    if (isValidStartCharacter(firstChar)){
      //IF: the first character *is* a valid command start character,
      //   - Read the characters and convert them to 12 numbers instead of 12 number chars
      //   - Call addGoalToQueue() which will handle higher-level error checking on commands
      //   - return 
      // printDebugToScreen("valid char");
      // delay(1000);
      //Read the 12 command digits and convert them to the actual number (char '2' -> int 2)
      int commandDigits[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      for (int i = 0; i<12; i++){
        commandDigits[i] = Serial.read() - 48;
      }

      //RETURN the output of addGoalToQueue
      //   - True if the command was added to the queue, false for the following reasons:
      //     - False if a move command is out of the bounds of any actuator
      //     - False if the queue is full
      return addGoalToQueue(firstChar, commandDigits);
    } else {
      //IF: the first character is *not* a valid command start character,
      //   - clear the serial bus until a valid start character is found
      //   - then return false and the next loop can try to read the command
      clearSerialUntilCommand();
      return false;
    }
  }
}

/** bool newCommandsAvailable()
 * @return true if there is a new command in the queu available to complete
*/
bool newCommandsAvailable(){
  if(currentCommandIndex == commandInsertIndex){//If the next goal has not been filled yet
    return false;
  } else {
    return true;
  }
}


/** unsigned long getMovementTime()
 * @returns the fastest time we can interpolate the movement position
 *  This calculates the quickest time we can get all 4 actuators there from 
 *  the current and ending positions
*/
unsigned long getMovementTime(){
  int act1Delta = commandQueue[currentCommandIndex][1] - act1.getLen();
  int act2Delta = commandQueue[currentCommandIndex][2] - act2.getLen();
  int act3Delta = commandQueue[currentCommandIndex][3] - act3.getLen();
  int act4Delta = commandQueue[currentCommandIndex][4] - act4.getLen();
  unsigned long maxDelta = max(max(act1Delta,act2Delta),max(act3Delta,act4Delta));
  return maxDelta*((1/(float)MAX_MILLIMETERS_PER_SECOND)*1000)*MOVEMENT_SPEED_SAFETY_FACTOR;
  //return 12000;
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
  act1.setPositionFromPotentiometer();
  act2.setPositionFromPotentiometer();
  act3.setPositionFromPotentiometer();
  act4.setPositionFromPotentiometer();

  //If the potentiometers aren't installed, this can be used to manually set the positions
  // act1.setLen(241);
  // act2.setLen(260);



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
  bool commandRecieved = readCommand();
  currentTime = millis();
  
  if(commandRecieved){
    printDebugToScreen("got a command");
  }
  

  switch(currentState){
    case IDLE:{
      //If we are in this state, all of the commands in the queue have been completed, or we just finished a command
      //To get out, any of the following needs to be true
      //   - We are behind a command (one was added) 
      printDebugToScreen("In IDLE state");

      if (newCommandsAvailable()){
        //If we have a new command, find what it is and move to the corresponding state
        switch(commandQueue[currentCommandIndex][0]){

          case 'M'://We have a MOVE command up next:
            //save the current actuator positions
            movementTime = getMovementTime();
            // printDebugToScreen("Switching to Move");
            // delay(1000);
            // printDebugToScreen(String(movementTime));
            // delay(1000);
            //switch to the moving state
            
            currentState = COMMAND_MOVING;
            break;

          case 'W'://We have a WAIT command up next:
            waitStartTime = currentTime;
            currentState = COMMAND_WAITING;
            break;

          case 'G'://We have a GET command up next:
            currentState = COMMAND_GETTING;
            break;
        }
      }
      }break;
    case COMMAND_MOVING:{
      printActuatorGoals();
      delay(1000);
      //We have to move with a move command
      int act1Error = abs(act1.moveToPosition(commandQueue[currentCommandIndex][1], movementTime));
      int act2Error = abs(act2.moveToPosition(commandQueue[currentCommandIndex][2], movementTime));
      int act3Error = abs(act3.moveToPosition(commandQueue[currentCommandIndex][3], movementTime));
      int act4Error = abs(act4.moveToPosition(commandQueue[currentCommandIndex][4], movementTime));
      if(max(max(act1Error,act2Error),max(act3Error,act4Error))<=4){
        //If the maximum error is less than or equal to 4mm, we can say we're finished with the movement
        act1.stop();
        act2.stop();
        act3.stop();
        act4.stop();
        currentCommandIndex = (currentCommandIndex+1)%MAX_COMMAND_QUEUE_LENGTH;
        currentState = IDLE;
        printDebugToScreen("Switching to IDLE");
        delay(1000);
        printDebugToScreen("Act1 E:" + String(act1.currentEffort));
        delay(1000);
        printDebugToScreen("Act2 E:" + String(act2.currentEffort));
        delay(1000);
        printDebugToScreen("Act3 E:" + String(act3.currentEffort));
        delay(1000);
        printDebugToScreen("Act4 E:" + String(act4.currentEffort));
        delay(1000);
      }
      }break;
    case COMMAND_WAITING:{
      if(currentTime>=waitStartTime+commandQueue[currentCommandIndex][1]){
        //IF: It's time to continue
        currentCommandIndex = (currentCommandIndex+1)%MAX_COMMAND_QUEUE_LENGTH;
        currentState = IDLE;
      }
      //stuff
      }break;
    case COMMAND_GETTING:{
      //stuff
      }break;
    
  }


  // if(tempComm == false){//If there isn't a full command or not a full command yet
  // //Do nothing
  // } else if(tempComm == true){//If there was just a MOVE command read
  //   printDebugToScreen("MOVE");
  //   if(addGoalToQueue(commandDigits)){//Add it to the queue, if possible
  //     printActuatorGoals();
  //   } else {
  //     printDebugToScreen("MOVE Queue full");
  //     Serial.write("E: Queue Full ");// sending an error message to MATLAB 

  //   }
  // } else if(tempComm == GET){//If there was just a GET command read
  //   printDebugToScreen("GET");
  // }
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






  // if(digitalRead(50)==LOW){//If we pressed the button
  //   Serial.write("C123123123123");
  //   printDebugToScreen("Command Sent to MATLAB");
  // }

}