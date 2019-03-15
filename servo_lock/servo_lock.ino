/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
/* 
 *  UNLOCKED_POS: position where rover can exit. this should be perpendicular to LOCKED_POS. 
 *  i.e. UNLOCKED_POS = 0, LOCKED_POS = 90
 *  as of now, locked pos is perpendicular to screws on the servo
 */
#define UNLOCKED_POS 10// Position of servo where rover can exit. This should be 
#define LOCKED_POS 100 // Position of servo where rover is locked. On the mini it's 95
// 
#define SERVO_PIN 9 // Gotta be PWM

Servo servo;  // create servo object to control a servo

// HELPER FUNCTIONS
void unlockRover() { 
  servo.write(UNLOCKED_POS);                  // sets the servo position according to the scaled value
  delay(1500);                           // waits for the servo to get there
}

void lockRover() { 
  servo.write(LOCKED_POS);                  // sets the servo position according to the scaled value
  delay(1500);                           // waits for the servo to get there
}

void setup() {
  servo.attach(SERVO_PIN);  // attaches the servo on pin SERVO_PIN to the servo object
  lockRover(); // make sure it's locked to begin with 
  // this also means if we quickly unplug and replug it can reset the servo to locked position
  delay(5000);
}


void loop() {
  unlockRover();
  lockRover();
}
