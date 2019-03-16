/* ============================================
 *  LandingDetection.h
 *  Uses MPU 6050 data to determine whether the payload section/rocket has landed
 *  Accelerometer reading code taken from Jeff Rowberg 's I2CDev library
 * Documentation : https://www.i2cdevlib.com/docs/html/class_m_p_u6050.html
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//MPU6050 mpu(0x69); // <-- use for AD0 high
#define DEV_MODE true // enables serial print statements
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)


bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

MPU6050 mpu_setup(MPU6050 mpu, int zero_motion_detection_threshold, int zero_motion_detection_duration) {
	    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    #if DEV_MODE
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    #endif DEV_MODE
    mpu.initialize();

    // verify connection
    #if DEV_MODE
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    #endif DEV_MODE
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        
        #if DEV_MODE
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        #endif DEV_MODE
        dmpReady = true;


        packetSize = mpu.dmpGetFIFOPacketSize();
        mpu.setZeroMotionDetectionThreshold(zero_motion_detection_threshold);
        mpu.setZeroMotionDetectionDuration(zero_motion_detection_duration);
        
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        #if DEV_MODE
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        #endif DEV_MODE
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    return mpu;
}

boolean zeroMotionDetected(MPU6050 mpu) {
  // if programming failed, don't try to do anything
    if (!dmpReady) return;
    /*

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
    }

    // reset interrupt flag and get INT_STATUS byte
    */
    
    return mpu.getZeroMotionDetected();
}
