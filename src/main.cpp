# include "Arduino.h"
# include "Wire.h"
# include "bmm150.h"
# include "icm20649.h"

/* LED Blink, Teensyduino Tutorial #1
   http://www.pjrc.com/teensy/tutorial.html

   This example code is in the public domain.
*/

// Teensy 2.0 has the LED on pin 11
// Teensy++ 2.0 has the LED on pin 6
// Teensy 3.x / Teensy LC have the LED on pin 13
const int ledPin = 13;
bool bmm_check = false;
bool imu_check = false;

BMM150 bmm = BMM150();
ICM20649 imu = ICM20649();

// the setup() method runs once, when the sketch starts

void setup() {
  // initialize the digital pin as an output.
  while (!Serial) ;

  pinMode(ledPin, OUTPUT);
  Serial.println("Test 1");

  Wire.begin();

  bmm_check = bmm.initialize();
  imu_check = imu.initialize();



}

// the loop() methor runs over and over again,
// as long as the board has power

void loop() {
    digitalWrite(ledPin, HIGH);   // set the LED on
    delay(200);                  // wait for a second
    digitalWrite(ledPin, LOW);    // set the LED off
    delay(200);                  // wait for a second
    Serial.println("BMM Check");
    Serial.println(bmm_check);

    Serial.println();

    Serial.println("IMU Check");
    Serial.println(imu_check);

    
    
    // cout << "x is equal to " << x;
}