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

public:
    long motorTics = 0;
    actuatorCon(int interr, int read, int pwm, int dir1, int dir2);
    void getLen();
    void setLen();
};

#endif