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
    long motorTics = 0;

public:
    actuatorCon(int interr, int read, int pwm, int dir1, int dir2);

    void setTics(int s);
    int getTics();
    int getLen();
    void moveAct(int desired);
    void stopMotors();
};

#endif
