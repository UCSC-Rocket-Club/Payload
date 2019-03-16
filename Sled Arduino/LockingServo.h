/*
  Servo that locks the rover into place during flight
*/
#ifndef LockingServo
#define LockingServo


#include "Arduino.h"
#include "Servo.h"

class LockingServo
{
  public:
    /* 
    The values for which servo is locked / unlocked (0-180˚)
 *  UNLOCKED_POS: position where rover can exit. this should be perpendicular to LOCKED_POS. 
 *  i.e. UNLOCKED_POS = 0, LOCKED_POS = 90
 *  as of now, locked pos is perpendicular to screws on the servo
 */
    LockingServo(int pwm, int lockedPos, int unlockedPos);

    /* 
    * Moves the servo to unlock or lock the rover once landing is determined
    */
    void unlockRover();

    //just stops the motor
    void lockRover();

  private:
    int pwmPin;
    int lockedPos;
    int unlockedPos;
    int delay = 1500;
    Servo servo;
};

#endif