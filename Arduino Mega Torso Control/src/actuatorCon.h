#ifndef ACTUATOR_CLASS_H
#define ACTUATOR_CLASS_H

#include <Arduino.h>

class actuatorCon
{
private:
    int interr;
    int read;
    int pwm;
    int dir1;
    int dir2;
    long motorTicks = 0;
    int potentiometer;
    int smallPotValue;
    int largePotValue;

    #define MAX_TICKS 41328 // This is the number of ticks that correspond to the maximum actuator position of 328mm
    #define MIN_TICKS 30115 // Number of ticks that corresponds to the minimum actuator length


public:
    actuatorCon(int interr, int read, int pwm, int dir1, int dir2, int pot, int smPotVal, int lrgPotVal);

    void setTicks(int ticks);
    int getTicks();
    void getLen();
    void setLen();
    void incrementTicks();
    void decrementTicks();
    void setPositionFromPotentiometer();
};

#endif