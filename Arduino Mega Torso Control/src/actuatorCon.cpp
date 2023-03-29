#include "actuatorCon.h"

int isrRead;
int motorTics;
int smallPotValue;
int largePotValue;

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

    pinMode(this->interr, INPUT);
    pinMode(this->read, INPUT);
    pinMode(this->pwm, OUTPUT);
    pinMode(this->dir1, OUTPUT);
    pinMode(this->dir2, OUTPUT);
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
void actuatorCon::setTics(int ticks)
{
    motorTics = ticks;
}

int actuatorCon::getTics()
{
    return motorTics;
}

/** void incrementTicks()
 * increases the tick count by one 
*/
void actuatorCon::incrementTicks(){
    this->motorTics++;
}

/** void decrementTicks()
 * decreases the tick count by one
*/
void actuatorCon::decrementTicks(){
    this->motorTics--;
}

/** void setPositionFromPotentiometer()
 * reads the potentiometer that tracks the actuator rod position and converts that analong measurement to ticks.
 * The conversion is dome by checking the analog read value at the smallest and largest position for each actuator.
 * Since the potentiometer may be attaches at different angles for each actuator, this is the most accurate method.
*/
void actuatorCon::setPositionFromPotentiometer(){

}