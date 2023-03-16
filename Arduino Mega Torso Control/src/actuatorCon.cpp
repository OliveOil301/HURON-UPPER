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

int actuatorCon::getLen()
{
    float rot = this->getTics() / 16;
    float endRot = rot / 6.3;
    float len = endRot * 1.25;
    return len;
}

void actuatorCon::moveAct(int desiredLen)
{
    int error = this->getLen() - desiredLen;

    if (error <= 0)
    {
        Serial.println("ERROR 1");
        Serial.println(error);
        digitalWrite(this->dir1, HIGH);
        digitalWrite(this->dir2, LOW);
    }
    else
    {
        Serial.println("ERROR 2");
        Serial.println(error);
        digitalWrite(this->dir1, LOW);
        digitalWrite(this->dir2, HIGH);
    }

    // PROBLEM CHILD    vvv
    error = map(abs(error), 0, 80, 0, 255);
    // PROBLEM CHILD    ^^^ 
    
    Serial.println(error);
    analogWrite(this->pwm, error);
}

void actuatorCon::setTics(int s)
{
    this->motorTics = s;
}

int actuatorCon::getTics()
{
    return this->motorTics;
}
