

#include <Servo.h>
#include "LandingDetection.h"
/* 
 *  UNLOCKED_POS: position where rover can exit. this should be perpendicular to LOCKED_POS. 
 *  i.e. UNLOCKED_POS = 0, LOCKED_POS = 90
 *  as of now, locked pos is perpendicular to screws on the servo
 */

// SERVO
#define UNLOCKED_POS 20// Position of servo where rover can exit. 
#define LOCKED_POS 115 // Position of servo where rover is locked. 
#define SERVO_PIN 9 // PWM pin on micro
#define TIME_UNLOCKED // The amount of time the rover is unlocked before we lock it again

// ACCELEROMETER SHIT
#define ZERO_MOTION_DETECTION_THRES 4
#define ZERO_MOTION_DETECTION_DUR 50


Servo servo;  // create servo object to control our locking servo
MPU6050 mpu;

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

  // FIRST MAKE SURE THE ROVER IS LOCKED
  servo.attach(SERVO_PIN);  // attaches the servo on pin SERVO_PIN to the servo object
  lockRover(); // make sure it's locked to begin with 
  delay(5000); // this also means if we quickly unplug and replug it can reset the servo to locked position

  // Set up IMU
  mpu = mpu_setup(mpu, ZERO_MOTION_DETECTION_THRES, ZERO_MOTION_DETECTION_DUR); // TODO : Check for error

  // Detect motion
  while(zeroMotionDetected(mpu) != 1)) {
    // Payload is in motion, wait for it to land
    delay(5000);
  }

  // Payload is no longer in motion, we can unlock it
  unlockRover();

  // 
  
  
  
  
}


void loop() {
  Serial.println(zeroMotionDetected(mpu));
  
}
