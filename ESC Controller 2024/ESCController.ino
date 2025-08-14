#include <Servo.h>
#include <Wire.h>

//#define SERIALDEBUG

#define I2CADDRESS 0x4c

const unsigned long timeout = 2000;
unsigned long lastReceiveTime = 0;
const uint8_t defaultPWM = 127;
const uint16_t thrusterLow = 1350;
const uint16_t thrusterHigh = 1650;

uint8_t motorValues[8] = { defaultPWM, defaultPWM, defaultPWM, defaultPWM, defaultPWM, defaultPWM, defaultPWM, defaultPWM };
bool newData = 0;
Servo ESC[8];
const byte ESCPins[] = { 3, 11, 10, 9, 6, 5, 2, 0 };
// Motors in order by I2C call 1, 2, 3, 4, 5, 6, 7, 8
// By Motors: M1 = P3, M2 = P11, M3 = P10, M4 = P9, M5 = P6, M6 = P5, M7 = P2, M8 = P0
// By Pins: P0 = M8, P2 = M7, P3 = M1, P5 = M6, P6 = M5, P9 = M4, P10 = M3, P11 = M2 

void setup() {
#ifdef SERIALDEBUG  // prints to USB when SERIALDEBUG is defined
  Serial.begin(9600);
#endif
  Wire.begin(I2CADDRESS);        // start listening for I2C commands
  Wire.onReceive(receiveEvent);  // register the I2C interrupt
  for (int i = 0; i < 8; i++) {  // register each pin as a servo object
    ESC[i].attach(ESCPins[i]);
  }
  lastReceiveTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastReceiveTime > timeout) {  // code to run after not recieving I2C commands for <timeout> time
    for (int i = 0; i < 8; i++) {
      ESC[i].writeMicroseconds(map(defaultPWM, 0, 255, thrusterLow, thrusterHigh));
    }
  } else if (newData) {  // code to run in order to assign commanded thrust level
    for (int i = 0; i < 8; i++) {
      ESC[i].writeMicroseconds(map(motorValues[i], 0, 255, thrusterLow, thrusterHigh));
    }
    newData = 0;
  }

  int ledToggle = !ledToggle;  // toggles the onboard LED every second main loop
  if (ledToggle) {
    digitalWrite(1, !digitalRead(1));
  }

  delay(10);  // limits how fast the main loop will run
}

void receiveEvent(int howMany) {  // code that runs every time we are given an I2C command
  while (Wire.available() && (howMany == 9)) {
    Wire.read();
    for (int i = 0; i < 8; i++) {
      motorValues[i] = Wire.read();  // reads the buffer 8 times and assigns those values to the servo value array
    }
#ifdef SERIALDEBUG  // prints to USB when SERIALDEBUG is defined
    Serial.println(motorValues[1]);
#endif
    newData = 1;
    lastReceiveTime = millis();  // for the timeout function
  }
}