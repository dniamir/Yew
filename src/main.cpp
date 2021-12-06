# include "Arduino.h"
# include "Wire.h"
# include "bmm150.h"
# include "icm20649.h"
# include "CDPA1616S.h"

// Teensy 2.0 has the LED on pin 11
// Teensy++ 2.0 has the LED on pin 6
// Teensy 3.x / Teensy LC have the LED on pin 13
const int ledPin = 13;
bool bmm_check = false;
bool imu_check = false;
bool led_on = false;

BMM150 bmm = BMM150();
ICM20649 imu = ICM20649();
CDPA1616S gps = CDPA1616S();

uint32_t gps_reading_ms = millis();
uint32_t led_off_ms = millis();
uint32_t led_on_ms = millis();
uint32_t mag_reading_ms = millis();
uint32_t imu_reading_ms = millis();

// the setup() method runs once, when the sketch starts

void setup() {
  Serial.begin(115200);

  while (!Serial);

  // initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);
  Serial.println("Test 1");

  Wire.begin();

  // Set the TX and RX pins to the alternate TX/RX 1 pins
  Serial1.setTX(26);
  Serial1.setRX(27);

  // Initialize BMM and IMUs
  bmm_check = bmm.initialize();
  imu_check = imu.initialize();

  // Initialize GPS
  gps.initialize();

}

// the loop() methor runs over and over again,
// as long as the board has power

void loop() {

  // Turn LED On
  if ((millis() - led_off_ms > 200) && !led_on){
    digitalWrite(ledPin, HIGH); 
    led_on_ms = millis();
    led_on = true;
  }

  // Turn LED Off
  if ((millis() - led_on_ms > 200) && led_on){
    digitalWrite(ledPin, LOW); 
    led_off_ms = millis();
    led_on = false;
  }                 

  // BMI Check
  if (millis() - mag_reading_ms > 1000){
    Serial.println("BMM Check");
    Serial.println(bmm_check);
    Serial.println();
    mag_reading_ms = millis();
  }

  // IMU Check
  if (millis() - imu_reading_ms > 1000){
    Serial.println("IMU Check");
    Serial.println(imu_check);
    Serial.println();
    imu_reading_ms = millis();
  }

  // GPS Check
  gps.readGPS();
  if (millis() - gps_reading_ms > 2000) {
    Serial.println("GPS Check");
    gps_reading_ms = millis(); // reset the timer
    gps.printBlock();
    Serial.println();
  }
}