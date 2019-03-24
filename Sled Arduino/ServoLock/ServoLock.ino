

#include <Servo.h>


// SERVO SETTINGS
/* 
 *  UNLOCKED_POS: position where rover can exit. this should be perpendicular to LOCKED_POS. 
 *  i.e. UNLOCKED_POS = 0, LOCKED_POS = 90
 *  as of now, locked pos is perpendicular to screws on the servo
 */

#define UNLOCKED_POS 75// Position of servo where rover can exit. 
#define LOCKED_POS 162 // Position of servo where rover is locked. 
#define SERVO_PIN 9 // PWM pin on micro

// COMMUNICATION WITH ROVER MICROCONTROLLER
#define TX_PIN_TO_ROVER 14

#define DEV_MODE true

#define TIME_TO_WAIT 12000 //12 secs, ms // flight is 90 secs CHANGE_FOR_FLIGHT
#define TIME_UNLOCKED 8000 // 8 secs

Servo servo;  // create servo object to control our locking servo

// HELPER FUNCTIONS
void unlockRover() { 
  servo.write(UNLOCKED_POS);                  // sets the servo position according to the scaled value
  delay(2000);                           // waits for the servo to get there
}

void lockRover() { 
  servo.write(LOCKED_POS);                  // sets the servo position according to the scaled value
  delay(2000);                           // waits for the servo to get there
}

void signalRoverArduino() { 
  digitalWrite(TX_PIN_TO_ROVER, HIGH);;
}

void stopSignalRoverArduino() { 
  digitalWrite(TX_PIN_TO_ROVER, LOW);
}

void finish() {
  lockRover();
}

void setup() {

  // Init pins
  pinMode(TX_PIN_TO_ROVER, OUTPUT);
  digitalWrite(TX_PIN_TO_ROVER, LOW);

  // Init Servo: FIRST MAKE SURE THE ROVER IS LOCKED
  servo.attach(SERVO_PIN);  // attaches the servo on pin SERVO_PIN to the servo object
  lockRover(); // make sure it's locked to begin with 
  
  delay(TIME_TO_WAIT);
  
  unlockRover();

  //delay(TIME_UNLOCKED); // Wait after unlocking rover

  //lockRover();

  // Power on rover Arduino
  signalRoverArduino();

  delay(2000);

  //wrap up 
  lockRover();
  digitalWrite(TX_PIN_TO_ROVER, LOW);
  

}


void loop() {
  //Serial.println(zeroMotionDetected(mpu));
  
}
