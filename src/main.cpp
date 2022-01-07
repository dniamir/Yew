# include "Arduino.h"
# include "Wire.h"
# include "bmm150.h"
# include "icm20649.h"
# include "CDPA1616S.h"
# include "arduino_i2c.h"
# include "SD.h"
# include "logger.h"


// Teensy 3.x / Teensy LC have the LED on pin 13
const int ledPin = 13;
bool bmm_check = false;
bool imu_check = false;
bool led_on = false;

// Initialize SD writing
// const int chipSelect = BUILTIN_SDCARD;

// Initialize chips
ArduinoI2C arduino_i2c;
BMM150 bmm = BMM150(arduino_i2c);
ICM20649 imu = ICM20649(arduino_i2c);
CDPA1616S gps = CDPA1616S();

// Initialize Logger
// LOGGER data_log = LOGGER("DarienTest3.txt");
LOGGER data_log = LOGGER();

// Timing
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

  Wire.begin();

  // Set the TX and RX pins to the alternate TX/RX 1 pins
  Serial1.setTX(26);
  Serial1.setRX(27);

  // Initialize BMM and IMUs
  bmm_check = bmm.initialize();
  imu_check = imu.initialize();

  // Set BMM mode
  bmm.default_mode();

  // Initialize GPS
  gps.initialize();

  // Initialize LOG
  data_log.create_log();

  // Set IMU mode and FIFO
  imu.default_mode();
  imu.setup_fifo_6axis();


  // // Initialize SD card slot
  // Serial.print("Initializing SD card...");
  
  // // see if the card is present and can be initialized:
  // if (!SD.begin(chipSelect)) {
  //   Serial.println("Card failed, or not present");
  //   // don't do anything more:
  //   return;
  // }
  // Serial.println("card initialized.");

}

// the loop() methor runs over and over again,
// as long as the board has power

void loop() {

  // IMU Check
  if (millis() - imu_reading_ms > 200) {

    // Read IMU
    imu.read_fifo();
    imu_reading_ms = millis();

    // Write to SD Card
    int timestamp = imu.last_os_reading.reading_time_ms;

    // Save FIFO Data
    std::vector<int> timestamps = imu.last_fifo_reading.reading_time_ms;

    // Save raw accel, gyro, temp data [LSBs]
    data_log.write_to_log(timestamps, "AXR", imu.last_fifo_reading.ax_lsb);
    data_log.write_to_log(timestamps, "AYR", imu.last_fifo_reading.ay_lsb);
    data_log.write_to_log(timestamps, "AZR", imu.last_fifo_reading.az_lsb);
    data_log.write_to_log(timestamps, "GXR", imu.last_fifo_reading.gx_lsb);
    data_log.write_to_log(timestamps, "GYR", imu.last_fifo_reading.gy_lsb);
    data_log.write_to_log(timestamps, "GZR", imu.last_fifo_reading.gz_lsb);
    data_log.write_to_log(timestamps, "TMPR", imu.last_fifo_reading.temp_lsb);

    // Save processed accel, gyro, temp data [mgee, dps, degC]
    data_log.write_to_log(timestamps[0], "FC", (int)imu.last_fifo_reading.fifo_count);
    data_log.write_to_log(timestamps, "AXP", imu.last_fifo_reading.ax_mgee);
    data_log.write_to_log(timestamps, "AYP", imu.last_fifo_reading.ay_mgee);
    data_log.write_to_log(timestamps, "AZP", imu.last_fifo_reading.az_mgee);
    data_log.write_to_log(timestamps, "GXP", imu.last_fifo_reading.gx_dps);
    data_log.write_to_log(timestamps, "GYP", imu.last_fifo_reading.gy_dps);
    data_log.write_to_log(timestamps, "GZP", imu.last_fifo_reading.gz_dps);
    data_log.write_to_log(timestamps, "TMPP", imu.last_fifo_reading.temp_degc);

    // Serial.println(imu.last_fifo_reading.az_mgee[0]);
    // Serial.println(imu.last_os_reading.az_mgee);
    // Serial.println();

  // Turn LED On
  if ((millis() - led_off_ms > 200) && !led_on) {
    digitalWrite(ledPin, HIGH); 
    led_on_ms = millis();
    led_on = true;
    data_log.write_to_log("LED", "On");
  }

  // Turn LED Off
  if ((millis() - led_on_ms > 200) && led_on) {
    digitalWrite(ledPin, LOW); 
    led_off_ms = millis();
    led_on = false;
    data_log.write_to_log("LED", "Off");
  }                 

  // BMI Check
  if (millis() - mag_reading_ms > 10000) {

    // Read BMM
    bmm.read_mxyz();
    mag_reading_ms = millis();
    
    // Write data to log
    data_log.write_to_log(bmm.last_os_reading.reading_time_ms, "MXR", bmm.last_os_reading.mx_lsb);
    data_log.write_to_log(bmm.last_os_reading.reading_time_ms, "MYR", bmm.last_os_reading.my_lsb);
    data_log.write_to_log(bmm.last_os_reading.reading_time_ms, "MZR", bmm.last_os_reading.mz_lsb);

    data_log.write_to_log(bmm.last_os_reading.reading_time_ms, "MXP", bmm.last_os_reading.mx_ut);
    data_log.write_to_log(bmm.last_os_reading.reading_time_ms, "MYP", bmm.last_os_reading.my_ut);
    data_log.write_to_log(bmm.last_os_reading.reading_time_ms, "MZP", bmm.last_os_reading.mz_ut);

  }



  // GPS Check
  // gps.readGPS();
  // if (millis() - gps_reading_ms > 10000) {
  //   Serial.println("GPS Check");
  //   gps_reading_ms = millis(); // reset the timer
  //   gps.printBlock();
  //   Serial.println();
  // }

  }
}