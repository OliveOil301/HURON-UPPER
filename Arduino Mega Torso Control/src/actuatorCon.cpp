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

int actuatorCon::getLen()
{
    return (((this->motorTicks/TICKS_PER_ROTATION)/GEAR_RATIO)*SCREW_PITCH); 
}

void actuatorCon::setLen(int millimeters)
{
    this->motorTicks = (((unsigned long)millimeters)/SCREW_PITCH)*GEAR_RATIO*TICKS_PER_ROTATION;
}


/** * setTics(int s)
 * sets the tick count for this actuator.
 * @param ticks the number of ticks that the motor is already at
 * This can be used on startup to set the actuator length so the actuators know their lengths
*/
void actuatorCon::setTicks(int ticks)
{
    motorTicks = ticks;
}

unsigned long actuatorCon::getTicks()
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

/** void recordPositionError(int error)
 * stores a new error value for the integral part of the PID controller
*/
void actuatorCon::recordPositionError(int error){
    this->lastPositionErrors[2] = this->lastPositionErrors[1];
    this->lastPositionErrors[1] = this->lastPositionErrors[0];
    this->lastPositionErrors[0] = error;//The new error value
    positionErrorSum = this->lastPositionErrors[2] + this->lastPositionErrors[1] + this->lastPositionErrors[0];

    //Finding the derivative error
    //This should increase if the error gets higher/stays the same
    this->derivativeError = lastPositionErrors[2] -(lastPositionErrors[2]-lastPositionErrors[0]);
}


/** void setStartingPosition()
 * saves the starting position for interpolation movements as the current position
*/
void actuatorCon::recordInterpolationStartPos(){
    this->interpolationStartLength = getLen();
}

/** int moveToPosition(int startLength, int finalLength, unsigned long startTime, unsigned long endTime)
 * sets the speed and direction of the actuator based on the following factors:
 *   - Current position
 *   - What percent complete the movement should be done (calculated from time vs time left)
 * @param finalLength the position the actuator should end at
 * @param actuationTime the time it is going to take to complete the movement
 * @return the current position error, in millimeters
*/
int actuatorCon::moveToPosition(int finalLength, unsigned long actuationTime){
    unsigned long currentTime = millis();
    //The first thing we do is see if this is a new movement or one we've already started:
    if (this->lastInterpolationEndLength != finalLength){
        //IF: this is a new interpolation request,
        //   - Remember enerything that we need and make sure we don't re-write it until we move next time
        this->lastInterpolationEndLength = finalLength;
        this->interpolationStartLength = getLen();
        this->interpolationStartTime = currentTime;
        this->interpolationEndTime = currentTime + actuationTime;
    }
    
    float percentComplete = ((float)(currentTime-this->interpolationStartTime))/((float)(actuationTime));
    float effort = 0.01;
    int positionError = 0;

    if (percentComplete<1.00){//If we haven't yet run out of time,
        //  Set the goal position to the the correct proportion of the final position 
        //  based off the time that has passed. 
        //  50% of time passed = actuator should be 50% there
        int goalLength = this->interpolationStartLength + ((finalLength-this->interpolationStartLength)*percentComplete); 
        positionError = goalLength-getLen();
        recordPositionError(positionError);

        effort = P_VALUE*(float)positionError + I_VALUE*(float)this->positionErrorSum + D_VALUE*(float)derivativeError;
        
        // Serial.print("| goal,act: ");
        // Serial.print(goalLength);
        // Serial.print(", ");
        // Serial.print(getLen());
        // Serial.print("| der: ");
        // Serial.print(this->derivativeError);
        // Serial.print("| e: ");
        // Serial.print(effort);
        
        
    } else {//If we've run out of time
        //At this point, we should just have a regualar 
        //  PID controller until we get close enough
        positionError = finalLength-getLen();
        recordPositionError(positionError);
        effort = (P_VALUE*2)*positionError + (I_VALUE*2)*this->positionErrorSum + D_VALUE*(float)derivativeError;
        // Serial.print("o");
        // Serial.print("| goal,act: ");
        // Serial.print(finalLength);
        // Serial.print(", ");
        // Serial.print(getLen());
        // Serial.print("| der: ");
        // Serial.print(this->derivativeError);
        // Serial.print("| e: ");
        // Serial.print(effort);
    }


    //At this point, we're moving the motor according to the defined effort and direction values
    if (effort>=0){
        //If we want the ticks to go up, Dir1 = low, Dir2 = high
        digitalWrite(this->dir1, LOW);
        digitalWrite(this->dir2, HIGH);
    } else {
        //If we want the ticks to go down, Dir1 = high, Dir2 = low
        digitalWrite(this->dir1, HIGH);
        digitalWrite(this->dir2, LOW);
    }

    analogWrite(this->pwm, constrain(abs((int)effort), 0, 255));
    return finalLength-getLen();
}


/** void stop()
 * decreases the tick count by one
*/
void actuatorCon::stop(){
    digitalWrite(this->pwm, LOW);
}