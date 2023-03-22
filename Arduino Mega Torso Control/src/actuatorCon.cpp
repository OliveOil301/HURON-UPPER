#include "actuatorCon.h"

int isrRead;
int motorTics;
float prevError = 0;
float Kp = 1;   
float Ki = 0;
float Kd = 0;
float sumError = 0;
int errorBound = 80;
bool runOnce = 0;
float origError;

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

int actuatorCon::getLen()
{
    float rot = this->getTics() / 16;
    float endRot = rot / 6.3;
    float len = endRot * 1.25;
    return len;
}

void actuatorCon::moveAct(int desiredLen)
{

    float error = this->getLen() - desiredLen;
    float currError = error;
    float derError = currError - prevError;
    sumError += error;

    if(sumError > errorBound) sumError = errorBound;
    if(sumError < -errorBound) sumError = -errorBound;

    if (error <= -2)
    {
        digitalWrite(this->dir1, LOW);
        digitalWrite(this->dir2, HIGH);
    }
    else if (error >= 2)
    {
        digitalWrite(this->dir1, HIGH);
        digitalWrite(this->dir2, LOW);
    }
    else 
    {
        digitalWrite(this->dir1, LOW);
        digitalWrite(this->dir2, LOW);
    }

    error = Kp * currError + Ki * sumError + Kd * derError;

    if (runOnce == 0){
        origError = error;
        runOnce = 1;
    }

    // Min Length = 243 mm
    // Max Length = 323 mm 
    // Difference of 80 mm

    int finalError = map(abs(error), 0, abs(origError), 180, 255);

    // Serial.println(origError);
    // Serial.println(finalError);

    analogWrite(this->pwm, finalError);

    prevError = error;
}

void actuatorCon::setTics(int s)
{
    this->motorTics = s;
}

int actuatorCon::getTics()
{
    return this->motorTics;
}
