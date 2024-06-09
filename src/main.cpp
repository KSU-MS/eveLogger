// Libs
#include "can.hpp"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>
#include <Wire.h>

// Some Global toggles for features
// #define HAS_DIS // Enable/Disable Display code
// #define HAS_GPS // Enable/Disable GPS data
#define HAS_NAV // Enable/Disable Vector Nav Boi

///
/// Global Variables
///

// pins for LEDs
int redLED = 36;
int blueLED = 37;

// Time calc vars
uint64_t global_ms_offset;
uint64_t last_sec_epoch;

// Timers
Metro timer_msg_RTC = Metro(1000); // Saving to disk
Metro timer_flush = Metro(50);     // Sending time CAN msg

// SD file bois
File logger;      // For saving to disk
String printname; // global thing for the filename

// Function defs
void read_can_message();                            // parse incoming msg
void write_to_SD(CAN_message_t msg, uint8_t bus);   // write can msg to disk
void sd_date_time(uint16_t *date, uint16_t *time);  // for sd lib things
String date_time(int time);                         // returns string of date
time_t get_t4_time() { return Teensy3Clock.get(); } // Fuck this cursed cast

// GPS module
#ifdef HAS_GPS
#include "gps.hpp"
Metro gps_timeout = Metro(1000); // GPS timeout if not present
void gps_can_msg();              // save global pos as CAN msg
#endif

// NAV module
#ifdef HAS_NAV
// #include "nav.hpp"
#include "vectornav.hpp"
vNav nd(Serial8);
Metro position_fix_send = 50;
Metro imu_data_send = 10;
void nav_can_msg(); // save nav data as CAN msg
#endif

// DIS module
#ifdef HAS_DIS
#include "dis.hpp"
#endif

void setup() {
  delay(1000); // Prevents wacky files when turning the car on and off rapidly

  // LED setup
  pinMode(blueLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(blueLED, HIGH);
  digitalWrite(redLED, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);

  // Wait for Serial to start
  Serial.begin(115200);

#ifdef HAS_GPS
  Serial.println("Init GPS");
  init_GPS();
  Serial.println("GPS set");
#endif

#ifdef HAS_NAV
  Serial.println("Init NAV");
  nd.init(EV);
  Serial.println("NAV set");

  // Setup can messages (defs are in can.hpp)
  vectornav_time.id = nav_time_id;
  vectornav_time.len = sizeof(nd.time);

  vectornav_attitude.id = nav_attitude_id;
  vectornav_attitude.len = sizeof(nd.attitude);

  vectornav_gyro.id = nav_gyro_id;
  vectornav_gyro.len = sizeof(nd.ang_rate);

  vectornav_position.id = nav_position_id;
  vectornav_position.len = sizeof(nd.lat_lon);

  vectornav_velocity.id = nav_velocity_id;
  vectornav_velocity.len = sizeof(nd.velocity);

  vectornav_accel.id = nav_accel_id;
  vectornav_accel.len = sizeof(nd.accel);
#endif

#ifdef HAS_DIS
  Serial.println("Init DIS");
  init_DIS();
  Serial.println("DIS set");
#endif

  init_CAN();

  Serial.println("Setting up time");
  setSyncProvider(get_t4_time);
  // COMMENT OUT THIS LINE AND PUSH ONCE RTC HAS BEEN SET!!!!
  // Teensy3Clock.set(1709674746); // set time (epoch) at powerup
  if (timeStatus() != timeSet) {
    Serial.println("RTC not set up, call Teensy3Clock.set(epoch)");
  } else {
    Serial.print("System date/time set to: ");
    Serial.println(Teensy3Clock.get());
  }
  last_sec_epoch = Teensy3Clock.get();

  // Set up SD card
  Serial.println("Initializing SD card...");
  SdFile::dateTimeCallback(sd_date_time); // Set date/time callback function
  if (!SD.begin(BUILTIN_SDCARD)) {        // Begin Arduino SD API (Teensy 3.5)
    Serial.println("SD card failed or not present");
  }

  // Make name of current time
  const char *filename = date_time(Teensy3Clock.get()).c_str();

  if (!SD.exists(filename)) { // Open file for writing
    logger = SD.open(filename, (uint8_t)O_WRITE | (uint8_t)O_CREAT);
  } else { // Print error if name is taken
    Serial.println("You generated a duplicate file name... Go check RTC.");
  }
  printname = filename;

  // Debug prints if it fails
  if (logger) { // Print on open
    Serial.print("Successfully opened SD file: ");
    Serial.println(filename);
    digitalWrite(blueLED, HIGH);
    digitalWrite(redLED, LOW);
  } else { // Print on fail
    Serial.println("Failed to open SD file");
    digitalWrite(blueLED, LOW);
    digitalWrite(redLED, HIGH);
  }

  // Print CSV heading to the logfile
  logger.println("time,msg.id,msg.len,data,bus");
  logger.flush();

  // Do te ting
  Serial.println("Log start");
}

void loop() {
  // Process and log incoming CAN messages
  read_can_message();

#ifdef HAS_GPS
  // Take GPS data and make CAN packet out of it
  gps_can_msg();
#endif

#ifdef HAS_NAV
  // Take NAV data and make CAN packet out of it
  nav_can_msg();
#endif

  // Flush data to SD card regardless of buffer size
  if (timer_flush.check()) {
    logger.flush();
    digitalToggle(blueLED);
  }

  // Print timestamp to serial & CAN
  if (timer_msg_RTC.check()) {
    Serial.println(Teensy3Clock.get());
    // msg_tx.id=0x3FF;
    // CAN.write(msg_tx);
  }
}

// While CAN packets are coming in, save the incoming msg and bus of origin
// currently very ugly, should look at re-writing this
void read_can_message() {
  if (BMS_CAN.read(msg)) {
    write_to_SD(msg, 0);
  }

  if (INV_CAN.read(msg)) {
    write_to_SD(msg, 1);
  }

  if (DAQ_CAN.read(msg)) {
    write_to_SD(msg, 2);
  }
}

// Build buffer "logger" till the timer ticks or buffer fills
void write_to_SD(CAN_message_t msg, uint8_t bus) {
  // Calculate Time
  uint64_t sec_epoch = Teensy3Clock.get();
  if (sec_epoch != last_sec_epoch) {
    global_ms_offset = millis() % 1000;
    last_sec_epoch = sec_epoch;
  }
  uint64_t current_time =
      sec_epoch * 1000 + (millis() - global_ms_offset) % 1000;

  // Log to SD
  logger.print(String(current_time) + ",");
  logger.print(String(msg.id, HEX) + ",");
  logger.print(String(msg.len) + ",");
  for (int i = 0; i < msg.len; i++) {
    if (msg.buf[i] < 16) {
      logger.print('0');
    }
    logger.print(msg.buf[i], HEX);
  }
  logger.print("," + String(bus));
  logger.println();
  digitalToggle(13); // Flip LED state for signs of life
}

#ifdef HAS_NAV
void nav_can_msg() {
  // If we have more than 4 new bytes, see if its a new line
  if (Serial8.available() > 4) {
    if (nd.check_sync_byte()) {
      // Get data
      nd.read_data();

      // Yeet data
      if (position_fix_send.check()) {
        memcpy(vectornav_time.buf, &nd.time, vectornav_time.len);
        memcpy(vectornav_position.buf, &nd.lat_lon, vectornav_position.len);

        DAQ_CAN.write(vectornav_time);
        DAQ_CAN.write(vectornav_position);

        write_to_SD(vectornav_time, 2);
        write_to_SD(vectornav_position, 2);
      }

      if (imu_data_send.check()) {
        memcpy(vectornav_attitude.buf, &nd.attitude, vectornav_attitude.len);
        memcpy(vectornav_gyro.buf, &nd.ang_rate, vectornav_gyro.len);
        memcpy(vectornav_velocity.buf, &nd.velocity, vectornav_velocity.len);
        memcpy(vectornav_accel.buf, &nd.accel, vectornav_accel.len);

        DAQ_CAN.write(vectornav_attitude);
        DAQ_CAN.write(vectornav_gyro);
        DAQ_CAN.write(vectornav_velocity);
        DAQ_CAN.write(vectornav_accel);

        write_to_SD(vectornav_attitude, 2);
        write_to_SD(vectornav_gyro, 2);
        write_to_SD(vectornav_velocity, 2);
        write_to_SD(vectornav_accel, 2);
      }
    }

// Update the display with INS state
// Look at page 139 of the docs if you want a better idea of wtf this means
#ifdef HAS_DIS
    // Update display
    if (displayUp.check()) {
      display.clearDisplay();
      draw_thing(printname, "top");
      if (nd.r_ins == 2) {
        draw_thing(String("NAV GOOD"), "bot");
      } else {
        draw_thing(String("NAV N/A"), "bot");
      }
    }
#endif
  }
}
#endif

// A function called once for fat32 file date stuff
void sd_date_time(uint16_t *date, uint16_t *time) {
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year(), month(), day());
  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour(), minute(), second());
}

// This function makes me sad
String date_time(int time) {
  String outString = "MDY_" + String(month(time)) + "-" + String(day(time)) +
                     "-" + String(year(time)) + "_HMS_" + String(hour(time)) +
                     "-" + String(minute(time)) + "-" + String(second(time)) +
                     ".CSV";

  return outString;
}
