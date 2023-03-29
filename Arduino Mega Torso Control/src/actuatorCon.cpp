#include "actuatorCon.h"

int isrRead;
int motorTics;

actuatorCon::actuatorCon(int interr, int read, int pwm, int dir1, int dir2)
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

void actuatorCon::incrementTicks(){
    this->motorTics++;
}

void actuatorCon::decrementTicks(){
    this->motorTics--;
}