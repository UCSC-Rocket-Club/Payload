#include <SoftwareSerial.h>
#include <Print.h>
#define rxPin 5
#define txPin 6
int target = 3;
SoftwareSerial XBee(rxPin,txPin);
void setup() {
  // Initialize XBee Software Serial port. Make sure the baud
  // rate matches your XBee setting (9600 is default).
  // Serial.begin(9600);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  
  XBee.begin(9600);
  XBee.println("R"); // KEEP THIS 
  delay(2);
  XBee.println(F("Type 1, h, or H to write HIGH"));
  XBee.println(F("Type 0, l, or L to write LOW"));
  XBee.println(F("Type r or R to read from pin"));
}
void loop() {
  if (XBee.available()) {
    char c = XBee.read();
    XBee.println(c);
    switch(c) {
      case '1':
      case 'h':
      case 'H':
        XBee.println("Are you sure you want to detonate? y/n");
        while(!XBee.available()); // wait for a response
        c = XBee.read();
        switch(c) {
          case 'y':
            pinMode(target, OUTPUT);
            digitalWrite(target, HIGH);
            XBee.println();
            XBee.print("Setting pin ");
            XBee.print(target);
            XBee.println(" to HIGH for 5 seconds");
            delay(5000);
            digitalWrite(target, LOW);
            XBee.print("Setting pin ");
            XBee.print(target);
            XBee.println(" back to LOW");
            break;
          case 'n':
            XBee.println("Aborting write to HIGH");
            break;
        }
        break;
      case '0':
      case 'l':
      case 'L':
        pinMode(target, OUTPUT);
        digitalWrite(target, LOW);
        XBee.println();
        XBee.print("Setting pin ");
        XBee.print(target);
        XBee.println(" to LOW");
        break;
      case 'r':
      case 'R':
        pinMode(target, INPUT);
        XBee.println();
        XBee.print("Pin ");
        XBee.print(target);
        XBee.print(" = ");
        XBee.println(digitalRead(target));
    }
  }
}
