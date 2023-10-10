// Libs
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <SD.h>
#include <TimeLib.h>
#include <Wire.h>

// CAN Variables
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> fCAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> sCAN;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> tCAN;
static CAN_message_t msg_rx;
static CAN_message_t msg_tx;

// Global Variables
uint64_t global_ms_offset;       // Time calc things
uint64_t last_sec_epoch;         // Time calc things 2
Metro timerMsgRTC = Metro(1000); // Timer for saving to disk
Metro timerFlush = Metro(50);    // Timer for sending time can
File logger;                     // For saving to disk
String inputSerial8 = "";        // a string to hold incoming data
boolean IsReadySerial8 = false;  // whether the string is complete

// Define functions
void parse_can_message();                          // parse incoming msg
void write_to_SD(CAN_message_t *msg);              // write can msg to disk
void sd_date_time(uint16_t *date, uint16_t *time); // for sd lib things
String date_time(int time);                  // returns string of date/time
float degreeToDecimal(float num, byte sign); // GPS conversion function
String parseGll(String msg);                 // Parse GLL string function
String parseRmc(String msg);                 // Parse RMC string function

void setup() {
  delay(1000); // Prevents wacky files when turning the car on and off

  // Wait for Serial to start
  Serial.begin(9600);
  while (!Serial) {
  }

  // COMMENT OUT THIS LINE AND PUSH ONCE RTC HAS BEEN SET!!!!
  // Teensy3Clock.set(1660351622); // set time (epoch) at powerup
  if (timeStatus() != timeSet) {
    Serial.println("RTC not set up, call Teensy3Clock.set(epoch)");
  } else {
    Serial.println("System date/time set to: ");
    Serial.print(Teensy3Clock.get());
  }
  last_sec_epoch = Teensy3Clock.get();

  // Start CAN thingies
  // FLEXCAN0_MCR &= 0xFFFDFFFF; // Enables CAN self-reception. Borked
  fCAN.begin();
  fCAN.setBaudRate(500000);
  sCAN.begin();
  sCAN.setBaudRate(500000);
  tCAN.begin();
  tCAN.setBaudRate(500000);

  // Wait for GPS UART to start
  Serial.println("Init GPS");
  Serial8.begin(9600);
  while (!Serial8) {
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
  Serial8.println("$PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
  // Set update loop to 10hz
  Serial8.println("$PMTK220,100*2F");
  Serial.println("GPS set");

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
  }
  if (SD.exists(filename)) { // Print error if name is taken
    Serial.println("You generated a duplicate file name... Go check RTC.");
  }
  if (logger) { // Print on open
    Serial.print("Successfully opened SD file: ");
    Serial.println(filename);
  } else { // Print on fail
    Serial.println("Failed to open SD file");
  }

  // Print CSV heading to the logfile
  logger.println("time,msg.id,msg.len,data");
  logger.flush();
}

void loop() {
  // Process and log incoming CAN messages
  parse_can_message();

  // Take GPS data and make CAN packet out of it
  logger.println(parseGll(inputSerial8));

  // Flush data to SD card regardless of buffer size
  if (timerFlush.check()) {
    logger.flush();
  }

  // Print timestamp to serial & CAN occasionally
  if (timerMsgRTC.check()) {
    Serial.println(Teensy3Clock.get());
    // msg_tx.id=0x3FF;
    // CAN.write(msg_tx);
  }

  // Reset vars
  inputSerial8 = "";
  IsReadySerial8 = false;
}

// Write to SD card buffer
void parse_can_message() {
  while (fCAN.read(msg_rx) || sCAN.read(msg_rx) || tCAN.read(msg_rx)) {
    write_to_SD(&msg_rx); // if this fills up this will take 8ms to write
  }
}

// Build buffer logger till the timer ticks or buffer fills
// But this function does not appear to have any logic to flush at all...
// Could be built into the wacky class thing or something idk
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
}

// Run on new byte from UART 8
void serialEvent8() {
  while (Serial8.available() && IsReadySerial8 == false) {
    char nextChar = char(Serial8.read()); // Cast UART data to char

    inputSerial8 += nextChar; // Append nextChar to string

    // Break the while statement once the line ends
    if (nextChar == '\n') {
      IsReadySerial8 = true;
    }
  }
}

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

// Structure
// $GPGLL,(lat),(N/S),(long),(E/W),(UTC),(status),(mode)*(checksum)
// more at https://gpsd.gitlab.io/gpsd/NMEA.html
String parseGll(String msg) {

  // Check that the incoming string is GLL
  if (!strstr(msg.c_str(), "GLL")) {
    return "Ya shit wack, go check your GPS setup for GLL";
  }

  // Get length of str
  int len = strlen(msg.c_str());

  // Replace commas with end character '\0' to seperate into single strings
  for (int j = 0; j < len; j++) {
    if (msg[j] == ',' || msg[j] == '*') {
      msg[j] = '\0';
    }
  }

  // A lil working var
  int i = 0;

  // Go to string i and rip things
  // Raw lattitude in degrees
  i += strlen(&msg[i]) + 1;
  float lat_raw = atof(&msg[i]);

  // North or South char
  i += strlen(&msg[i]) + 1;
  char latNS = msg[i];

  // Raw longitude in degrees
  i += strlen(&msg[i]) + 1;
  float lon_raw = atof(&msg[i]);

  // East or West char
  i += strlen(&msg[i]) + 1;
  char lonEW = msg[i];

  // UTC time
  i += strlen(&msg[i]) + 1;
  float utc = atof(&msg[i]);

  // Is data valid (A) or not (V)
  i += strlen(&msg[i]) + 1;
  char valid = msg[i];

  // FAA mode
  i += strlen(&msg[i]) + 1;
  char mode = msg[i];

  // set output string to whatever
  String output = String(degreeToDecimal(lat_raw, latNS), 7) + "," +
                  String(degreeToDecimal(lon_raw, lonEW), 7) + ",";

  return output;
}

// Unfinished, RMC has a lot more data to parse
// Structure
// $GPRMC,time,status,lat,N/S,lon,E/W,Speed,degrees true,
// date,degrees,FAA mode,Nav status*checksum
// more at https://gpsd.gitlab.io/gpsd/NMEA.html
String parseRmc(String msg) {

  // Check that the incoming string is GLL
  if (!strstr(msg.c_str(), "RMC")) {
    return "Ya shit wack, go check your GPS setup for RMC";
  }

  // Get length of str
  int len = strlen(msg.c_str());

  // Replace commas with end character '\0' to seperate into single strings
  for (int j = 0; j < len; j++) {
    if (msg[j] == ',' || msg[j] == '*') {
      msg[j] = '\0';
    }
  }

  // A lil working var
  int i = 0;

  // Go to string i and rip things
  // UTC time
  i += strlen(&msg[i]) + 1;
  float utc = atof(&msg[i]);

  // Is data valid (A) or not (V)
  i += strlen(&msg[i]) + 1;
  char valid = msg[i];

  // Raw lattitude in degrees
  i += strlen(&msg[i]) + 1;
  float lat_raw = atof(&msg[i]);

  // North or South char
  i += strlen(&msg[i]) + 1;
  char latNS = msg[i];

  // Raw longitude in degrees
  i += strlen(&msg[i]) + 1;
  float lon_raw = atof(&msg[i]);

  // East or West char
  i += strlen(&msg[i]) + 1;
  char lonEW = msg[i];

  // spped
  i += strlen(&msg[i]) + 1;
  float speed = atof(&msg[i]);

  // Degrees true
  i += strlen(&msg[i]) + 1;
  float dtrue = atof(&msg[i]);

  // Date in ddmmyy
  i += strlen(&msg[i]) + 1;
  char date = msg[i];

  // Degrees magnetic
  i += strlen(&msg[i]) + 1;
  float magnetic = atof(&msg[i]);

  // East or West char
  i += strlen(&msg[i]) + 1;
  char lonEWdegree = msg[i];

  // FAA mode
  i += strlen(&msg[i]) + 1;
  char mode = msg[i];

  // A=autonomous, D=differential, E=Estimated,
  // M=Manual input mode N=not valid, S=Simulator, V = Valid
  i += strlen(&msg[i]) + 1;
  char status = msg[i];

  // set output string to whatever
  String output = String(degreeToDecimal(lat_raw, latNS), 7) + "," +
                  String(degreeToDecimal(lon_raw, lonEW), 7) + "," +
                  String(speed, 4);

  return output;
}

float degreeToDecimal(float num, byte sign) {
  // Want to convert DDMM.MMMM to a decimal number DD.DDDDD

  int intpart = (int)num;
  float decpart = num - intpart;

  int degree = (int)(intpart / 100);
  int mins = (int)(intpart % 100);

  if (sign == 'N' || sign == 'E') {
    // Return positive degree
    return (degree + (mins + decpart) / 60);
  } else {
    // Return negative degree
    return -(degree + (mins + decpart) / 60);
  }
}
