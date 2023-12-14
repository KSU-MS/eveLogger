// Libs
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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
#define HAS_TEL // Enable/Disable XBee stuffs

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

// CAN Variables
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> fCAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> sCAN;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> tCAN;
static CAN_message_t msg_rx;
static CAN_message_t msg_tx;

// Function defs
void read_can_message();                            // parse incoming msg
void write_to_SD(CAN_message_t *msg);               // write can msg to disk
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
#include "nav.hpp"
Metro nav_timeout = Metro(1000); // NAV timeout if not present
void nav_can_msg();              // save nav data as CAN msg
#endif

// TEL module
#ifdef HAS_TEL
Metro tel_timeout = Metro(1000); // TEL timeout if not present
String serial2_out = "";         // a string to hold sending data
#endif

// DIS module
#ifdef HAS_DIS
void draw_thing(String msg, String pos); // Draw something idk
Metro displayUp = Metro(1000);           // Timer for updating display info
#define SCREEN_ADDRESS 0x3c              // See datasheet for Address

// in order of appearance, Width, Height, SPI pin, Reset pint
Adafruit_SSD1306 display(128, 64, &Wire, -1);
#endif

void setup() {
  delay(1000); // Prevents wacky files when turning the car on and off rapidly

  // LED test
  digitalWrite(blueLED, HIGH);
  digitalWrite(redLED, HIGH);

  // Wait for Serial to start
  Serial.begin(115200);
  // while (!Serial)
  //   ;

#ifdef HAS_GPS
  // Wait for GPS UART to start
  Serial.println("Init GPS");
  Serial8.begin(9600);
  while (!Serial8) {
    if (gps_timeout.check()) {
      Serial.println("No GPS unit");
      break;
    }
  }

  // page 12 of https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf
  // checksum generator https://nmeachecksum.eqth.net/
  // you can set a value from 0 (disable) to 5 (output once every 5 pos fixes)
  // 0  NMEA_SEN_GLL,  // GPGLL interval - Lat & long
  // 1  NMEA_SEN_RMC,  // GPRMC interval - Recommended Minimum Specific GNSS
  // 2  NMEA_SEN_VTG,  // GPVTG interval - Course over Ground and Ground Speed
  // 3  NMEA_SEN_GGA,  // GPGGA interval - GPS Fix Data
  // 4  NMEA_SEN_GSA,  // GPGSA interval - GNSS DOPS and Active Satellites
  // 5  NMEA_SEN_GSV,  // GPGSV interval - GNSS Satellites in View
  // 6-17           ,  // Reserved
  // 18 NMEA_SEN_MCHN, // PMTKCHN interval – GPS channel status
  Serial8.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  // Set update loop to 10hz
  Serial8.println("$PMTK220,100*2F");
  Serial.println("GPS set");
#endif

#ifdef HAS_NAV
  // Wait for NAV UART to start
  Serial.println("Init NAV");
  Serial8.begin(115200);
  while (!Serial8) {
    if (nav_timeout.check()) {
      Serial.println("No NAV unit");
      break;
    }
  }

  // Please just use the Vector Nav Control Center tool to generate the messages
  // I fucking hate calculating these by hand so much don't torture yourself
  Serial8.println("$VNWRG,75,1,40,01,11EA*614A");
  Serial.println("NAV set");
#endif

#ifdef HAS_TEL
  // Wait for TEL UART to start
  Serial.println("Init TEL");
  Serial2.begin(115200);
  while (!Serial2) {
    if (tel_timeout.check()) {
      Serial.println("No TEL unit");
      break;
    }
  }
#endif

#ifdef HAS_DIS
  // Wait for display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 type display not present"));
  }
  display.display(); // You must call .display() after draw command to apply
#endif

  // Start CAN thingies
  // FLEXCAN0_MCR &= 0xFFFDFFFF; // Enables CAN self-reception. Borked
  fCAN.begin();
  fCAN.setBaudRate(500000);
  sCAN.begin();
  sCAN.setBaudRate(500000);
  tCAN.begin();
  tCAN.setBaudRate(500000);

  Serial.println("Setting up time");
  setSyncProvider(get_t4_time);
  // COMMENT OUT THIS LINE AND PUSH ONCE RTC HAS BEEN SET!!!!
  // Teensy3Clock.set(1702575324); // set time (epoch) at powerup
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
  } else { // Print on fail
    Serial.println("Failed to open SD file");
    digitalWrite(redLED, HIGH);
  }

  // Print CSV heading to the logfile
  logger.println("time,msg.id,msg.len,data");
  logger.flush();

  // Do te ting
  digitalWrite(blueLED, LOW);
  digitalWrite(redLED, LOW);
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

  // Print timestamp to serial & CAN occasionally
  if (timer_msg_RTC.check()) {
    Serial.println(Teensy3Clock.get());
    // msg_tx.id=0x3FF;
    // CAN.write(msg_tx);
  }
}

// While CAN packets are coming in, save them
void read_can_message() {
  while (fCAN.read(msg_rx) || sCAN.read(msg_rx) || tCAN.read(msg_rx)) {
    write_to_SD(&msg_rx); // if this fills up this will take 8ms to write
  }
}

// Build buffer "logger" till the timer ticks or buffer fills
void write_to_SD(CAN_message_t *msg) {
  // Calculate Time
  uint64_t sec_epoch = Teensy3Clock.get();
  if (sec_epoch != last_sec_epoch) {
    global_ms_offset = millis() % 1000;
    last_sec_epoch = sec_epoch;
  }
  uint64_t current_time =
      sec_epoch * 1000 + (millis() - global_ms_offset) % 1000;

  // Log to SD
  logger.print(current_time);
  logger.print(",");
  logger.print(msg->id, HEX);
  logger.print(",");
  logger.print(msg->len);
  logger.print(",");
  for (int i = 0; i < msg->len; i++) {
    if (msg->buf[i] < 16) {
      logger.print("0");
    }
    logger.print(msg->buf[i], HEX);
  }
  logger.println();
  digitalToggle(13); // Flip LED state for signs of life

#ifdef HAS_TEL
  // Append shit to string
  serial2_out += String(current_time) + ",";
  serial2_out += String(msg->id, HEX) + ",";
  for (int i = 0; i < msg->len; i++) {
    if (msg->buf[i] < 16) {
      serial2_out += "0";
    }
    serial2_out += String(msg->buf[i], HEX);
  }

  // Send as 1 thicc boi
  Serial2.println(serial2_out);
#endif
}

#ifdef HAS_NAV
void nav_can_msg() {
  // Reset nav state
  nav_ready = false;

  // If we have more than 4 new bytes, see if its a new line
  if (Serial8.available() > 4)
    check_sync_byte();

  // If check_sync_byte() set is_nav_ready true, do shit with the data
  if (nav_ready) {
    // Calculate Time
    uint64_t sec_epoch = Teensy3Clock.get();
    if (sec_epoch != last_sec_epoch) {
      global_ms_offset = millis() % 1000;
      last_sec_epoch = sec_epoch;
    }
    uint64_t current_time =
        sec_epoch * 1000 + (millis() - global_ms_offset) % 1000;

    // Get NAV data
    navData nd = read_nav_data();

    logger.print(String(current_time) + ",");
    logger.print(String(pos_id) + ",");
    logger.print(String(strlen(nd.r_pos.c_str())) + ",");
    logger.println(nd.r_pos);

#ifdef HAS_TEL
    // Append shit to string
    serial2_out += String(current_time) + ",";
    serial2_out += String(pos_id) + ",";
    serial2_out += String(strlen(nd.r_pos.c_str())) + ",";
    serial2_out += String(nd.r_pos);

    // Send as 1 thicc boi
    Serial2.println(serial2_out);
#endif

// Update the display with INS state
// Look at page 139 of the docs if you want a better idea of wtf this means
#ifdef HAS_DIS
    // Update display
    if (displayUp.check()) {
      display.clearDisplay();
      draw_thing(printname, "top");
      if (nd.r_ins[0] != 2) {
        draw_thing(String("NAV GOOD"), "bot");
      } else {
        draw_thing(String("NAV N/A"), "bot");
      }
    }
#endif
  }
}
#endif

#ifdef HAS_GPS
void serialEvent2() {
  while (Serial8.available() && ready_serial8 == false) {
    char nextChar = char(Serial8.read()); // Cast UART data to char

    input_serial8 += nextChar; // Append nextChar to string

    // Break the while statement once the line ends
    if (nextChar == '\n') {
      ready_serial8 = true;
    }
  }
}

void gps_can_msg() {
  if (ready_serial8) {
#ifdef HAS_DIS
    // Update display
    if (displayUp.check()) {
      display.clearDisplay();
      draw_thing(printname, "top");
      draw_thing(parse_gga(input_serial8), "bot");
    }
#endif

    // Get GPS data
    String global_pos = (parse_rmc(input_serial8));

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
    logger.print(String(gps_id) + ",");
    logger.print(String(strlen(global_pos.c_str())) + ",");
    logger.println(global_pos);

#ifdef HAS_TEL
    // Append shit to string
    serial2_out += String(current_time) + ",";
    serial2_out += String(gps_id) + ",";
    serial2_out += String(strlen(global_pos.c_str())) + ",";
    serial2_out += String(global_pos);

    // Send as 1 thicc boi
    Serial2.println(serial2_out);
#endif

    // Reset vars
    input_serial8 = "";
    ready_serial8 = false;
  }
}

#endif

#ifdef HAS_DIS
// It take string in, and it puts shit onto display
void draw_thing(String msg, String pos) {
  display.setTextSize(1);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);                 // Use full 256 char 'Code Page 437' font

  if (pos == "top") {
    display.setCursor(0, 0);
    display.print(msg);
  } else if (pos == "bot") {
    display.setTextSize(2);
    display.setCursor(0, 32);
    display.print(msg);
  } else {
    return;
  }

  display.display();
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
