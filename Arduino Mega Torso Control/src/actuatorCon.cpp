#include "actuatorCon.h"

int isrRead;


/** actuatorCon(int interr, int read, int pwm, int dir1, int dir2, int pot, int smPotVal, int lrgPotVal)
 * @param interr the encoder pin that needs an interrupt attached
 * @param read the other encoder pin that doesn't need an interrupt but is read during the ISR
 * @param pwm the output pin that controls the speed of the actuator
 * @param dir1 output pin for controlling the actuator direction
 * @param dir2 output pin for controlling the actuator direction
 * @param pot the analog pin for reading the potentiometer that tracks the actuation rod position
 * @param smPotVal the smallest analog read value for the rod. Used for conversion to ticks
 * @param lrgPotVal the largest analog read value for the rod. Used for conversion to ticks
 * This constructor declares all the pins as appropriate input or outputs. The ISRs need to be made in the main funciton so they're not here
 * This also does not set the actuator position from the potentiometers. 
 *      The setPositionFromPotentiometer() methon must be called for each actuator in the setup function.
*/
actuatorCon::actuatorCon(int interr, int read, int pwm, int dir1, int dir2, int pot, int smPotVal, int lrgPotVal)
{
    this->interr = interr;
    this->read = read;
    this->pwm = pwm;
    this->dir1 = dir1;
    this->dir2 = dir2;
    this->potentiometer = pot;
    this->smallPotValue = smPotVal;
    this->largePotValue = lrgPotVal;


    pinMode(this->interr, INPUT);
    pinMode(this->read, INPUT);
    pinMode(this->pwm, OUTPUT);
    pinMode(this->dir1, OUTPUT);
    pinMode(this->dir2, OUTPUT);
    pinMode(this->potentiometer, INPUT);
}

void actuatorCon::getLen()
{
}

void actuatorCon::setLen()
{
}

/** * setTics(int s)
 * sets the tick count for this actuator.
 * @param ticks the number of ticks that the motor is already at
 * This is used on startup to set the actuator length so the actuators know their lengths
*/
void actuatorCon::setTicks(int ticks)
{
    motorTicks = ticks;
}

int actuatorCon::getTicks()
{
    return motorTicks;
}

/** void incrementTicks()
 * increases the tick count by one 
*/
void actuatorCon::incrementTicks(){
    this->motorTicks++;
}

/** void decrementTicks()
 * decreases the tick count by one
*/
void actuatorCon::decrementTicks(){
    this->motorTicks--;
}

/** void setPositionFromPotentiometer()
 * reads the potentiometer that tracks the actuator rod position and converts that analong measurement to ticks.
 * The conversion is dome by checking the analog read value at the smallest and largest position for each actuator.
 * Since the potentiometer may be attaches at different angles for each actuator, this is the most accurate method.
*/
void actuatorCon::setPositionFromPotentiometer(){
    this->motorTicks = map(analogRead(this->potentiometer), this->smallPotValue, this->largePotValue,MIN_TICKS,MAX_TICKS);
}

/** int moveToPosition(int startPosition, int endPosition, unsigned long startTime, unsigned long endTime)
 * sets the speed and direction of the actuator based on the following factors:
 *   - Current position
 *   - What percent complete the movement should be done (calculated from time vs time left)
 * @param startPosition the position the actuator started at. Used for interpolation
 * @param endPosition the position the actuator should end at
 * @param startTime the time the movement started
 * @param endTime the time the movement should end
 * @param currentTime the current time from millis() so it doesn't have to be called multiple times
 * 
*/
int actuatorCon::moveToPosition(int startPosition, int endPosition, unsigned long startTime, unsigned long endTime, unsigned long currentTime){
    //If we haven't run out of time:
    //  Set the goal position to the the correct proportion of the final position 
    //  based off the time that has passed. 
    //  50% of time passed = actuator should be 50% there


    //If we have run out of time:
    //At this point, we should just have a regualar 
    //  PID controller until we get close enough

}