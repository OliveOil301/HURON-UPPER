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
    unsigned long motorTicks = 0;
    int potentiometer;
    int smallPotValue;
    int largePotValue;

    int lastPositionErrors[3] = {0, 0, 0};
    int positionErrorSum = 0;
    int derivativeError = 0;
    int interpolationStartLength = 239;

    #define MAX_TICKS 41328 // This is the number of ticks that correspond to the maximum actuator position of 328mm
    #define MIN_TICKS 30115 // Number of ticks that corresponds to the minimum actuator length
    #define P_VALUE 8
    #define I_VALUE 5
    #define D_VALUE 10
    #define TICKS_PER_ROTATION 16
    #define GEAR_RATIO 18.75
    #define SCREW_PITCH 1.25

    void recordPositionError(int error);

public:
    actuatorCon(int interr, int read, int pwm, int dir1, int dir2, int pot, int smPotVal, int lrgPotVal);

    void setTicks(int ticks);
    unsigned long getTicks();
    int getLen();
    void setLen(int millimeters);
    void incrementTicks();
    void decrementTicks();
    void setPositionFromPotentiometer();
    void recordInterpolationStartPos();
    int moveToPosition(int finalLength, unsigned long startTime, unsigned long endTime, unsigned long currentTime);
    void stop();
};

#endif