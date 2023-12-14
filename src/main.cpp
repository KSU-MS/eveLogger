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

#ifdef HAS_GPS
Metro gps_timeout = Metro(1000); // GPS timeout if not present
String gps_id = "";              // Whatever ID you want for your fake can msg
String input_serial8 = "";       // a string to hold incoming data
boolean ready_serial8 = true;    // whether the string is complete
#endif

#ifdef HAS_NAV
#include "nav.hpp"
Metro nav_timeout = Metro(1000);  // NAV timeout if not present
void nav_can_msg();               // save nav data as CAN msg
String read_nav_data(int option); // read nav data
void check_sync_byte(void);       // check for new msg
unsigned short calculate_imu_crc(byte data[], unsigned int length); // check msg
float degree_to_decimal(float num, byte sign); // GPS conversion function
#endif

int redLED = 36;  // Pin for red LED
int blueLED = 37; // Pin for blue LED

#ifdef HAS_DIS
Metro displayUp = Metro(1000); // Timer for updating display info
#define SCREEN_WIDTH 128       // OLED display width, in pixels
#define SCREEN_HEIGHT 64       // OLED display height, in pixels
#define OLED_RESET -1          // Reset pin # use -1 if unsure
#define SCREEN_ADDRESS 0x3c    // See datasheet for Address

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

///
/// Function defs
///

void read_can_message();                            // parse incoming msg
void write_to_SD(CAN_message_t *msg);               // write can msg to disk
void sd_date_time(uint16_t *date, uint16_t *time);  // for sd lib things
String date_time(int time);                         // returns string of date
time_t get_t4_time() { return Teensy3Clock.get(); } // Fuck this cursed cast

#ifdef HAS_GPS
void gps_can_msg();                            // save global pos as CAN msg
String parse_rmc(String msg);                  // Parse RMC string function
String parse_gga(String msg);                  // Prase GGA string im tired
float degree_to_decimal(float num, byte sign); // GPS conversion function
#endif

#ifdef HAS_NAV
#endif

#ifdef HAS_DIS
void draw_thing(String msg, String pos); // Draw something idk
#endif

#ifdef HAS_TEL
Metro tel_timeout = Metro(1000); // TEL timeout if not present
String serial2_out = "";         // a string to hold sending data
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
#endif

#ifdef HAS_DIS
  // Wait for display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 type display not present"));
  }
  display.display(); // You must call .display() after draw command to apply
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

  // Start CAN thingies
  // FLEXCAN0_MCR &= 0xFFFDFFFF; // Enables CAN self-reception. Borked
  fCAN.begin();
  fCAN.setBaudRate(500000);
  sCAN.begin();
  sCAN.setBaudRate(500000);
  tCAN.begin();
  tCAN.setBaudRate(500000);

#ifdef HAS_GPS
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
  // Please just use the Vector Nav Control Center tool to generate the messages
  // I fucking hate calculating these by hand so much don't torture yourself
  Serial8.println("$VNWRG,75,1,40,01,11EA*614A");
  Serial.println("NAV set");
#endif

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
// Read the NAV bytes
navData read_nav_data() {
  // Read the bytes into an array
  Serial8.readBytes(in, 87);

  // Grab the checksum
  checksum.b[0] = in[86];
  checksum.b[1] = in[85];

  // If the checksum is correct
  if (calculate_imu_crc(in, 85) == checksum.s) {
    // Calc time
    for (int i = 0; i < 8; i++) {
      ti.b[i] = in[3 + i];
    }

    // Calc Attitude and Rates
    for (int i = 0; i < 4; i++) {
      yaw.b[i] = in[11 + i];
      pit.b[i] = in[15 + i];
      rol.b[i] = in[19 + i];
      W_x.b[i] = in[23 + i];
      W_y.b[i] = in[27 + i];
      W_z.b[i] = in[31 + i];
    }

    // Calc Position
    for (int i = 0; i < 8; i++) {
      lat.b[i] = in[35 + i];
      lon.b[i] = in[43 + i];
      alt.b[i] = in[51 + i];
    }

    // Calc Velocity & Acceleration
    for (int i = 0; i < 4; i++) {
      v_n.b[i] = in[59 + i];
      v_e.b[i] = in[63 + i];
      v_d.b[i] = in[67 + i];
      a_x.b[i] = in[71 + i];
      a_y.b[i] = in[75 + i];
      a_z.b[i] = in[79 + i];
    }

    // "Calc" INS state
    for (int i = 0; i < 2; i++) {
      ins.b[i] = in[83 + i];
    }

    // Return values in readable format
    return navData{
        String(ti.f, 10),
        String(yaw.f, 10) + "+" + String(pit.f, 10) + "+" + String(rol.f, 10),
        String(W_x.f, 10) + "+" + String(W_y.f, 10) + "+" + String(W_z.f, 10),
        String(lat.f, 10) + "+" + String(lon.f, 10) + "+" + String(alt.f, 10),
        String(v_n.f, 10) + "+" + String(v_e.f, 10) + "+" + String(v_d.f, 10),
        String(a_x.f, 10) + "+" + String(a_y.f, 10) + "+" + String(a_z.f, 10),
        String(ins.f),
    };
  }

  // Return nothing if shits fucked
  return navData{};
}

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

// Check for the sync byte (0xFA)
void check_sync_byte(void) {
  for (int i = 0; i < 6; i++) {
    Serial8.readBytes(in, 1);
    if (in[0] == 0xFA) {
      nav_ready = true;
      break;
    }
  }
}

// Calculate the 16-bit CRC for the given ASCII or binary message.
unsigned short calculate_imu_crc(byte data[], unsigned int length) {
  unsigned int i;
  unsigned short crc = 0;
  for (i = 0; i < length; i++) {
    crc = (byte)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (byte)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
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

// Structure
// $GPRMC,time,status,lat,N/S,lon,E/W,Speed,degrees true,
// date,degrees,FAA mode,Nav status*checksum
// more at https://gpsd.gitlab.io/gpsd/NMEA.html
String parse_rmc(String msg) {

  // Check that the incoming string is RMC
  if (!strstr(msg.c_str(), "RMC")) {
    return "";
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
  float lat = atof(&msg[i]);

  // North or South char
  i += strlen(&msg[i]) + 1;
  char NS = msg[i];

  // Raw longitude in degrees
  i += strlen(&msg[i]) + 1;
  float lon = atof(&msg[i]);

  // East or West char
  i += strlen(&msg[i]) + 1;
  char EW = msg[i];

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
  char EWdegree = msg[i];

  // FAA mode
  i += strlen(&msg[i]) + 1;
  char mode = msg[i];

  // A=autonomous, D=differential, E=Estimated,
  // M=Manual input mode N=not valid, S=Simulator, V = Valid
  i += strlen(&msg[i]) + 1;
  char status = msg[i];

  // set output string to whatever
  String output = String(degree_to_decimal(lat, NS), 7) + "+" +
                  String(degree_to_decimal(lon, EW), 7) + "+" +
                  String(speed, 1);

  return output;
}

// Structure
// $GPGGA,UTC,Lat,N/S,Lon,E/W,GPS Quality,# of sats,
// Precision, Altitude,Units of Altitude,Geoidal separation,
// Unit of Geoidal separation,Age of differential,station ID*Checksum
String parse_gga(String msg) {

  // Check that the incoming string is GGA
  if (!strstr(msg.c_str(), "GGA")) {
    return "";
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

  // Lat
  i += strlen(&msg[i]) + 1;
  float lat = atof(&msg[i]);

  // N/S
  i += strlen(&msg[i]) + 1;
  char NS = msg[i];

  // Lon
  i += strlen(&msg[i]) + 1;
  float lon = atof(&msg[i]);

  // E/W
  i += strlen(&msg[i]) + 1;
  char EW = msg[i];

  // GPS quality
  // 0 - fix not available,
  // 1 - GPS fix,
  // 2 - Differential GPS fix(values above 2 are 2.3 features)
  // 3 = PPS fix
  // 4 = Real Time Kinematic
  // 5 = Float RTK
  // 6 = estimated(dead reckoning)
  // 7 = Manual input mode
  // 8 = Simulation mode
  i += strlen(&msg[i]) + 1;
  int quality = atof(&msg[i]);

  // Sats locked
  i += strlen(&msg[i]) + 1;
  int locked = atof(&msg[i]);

  // precision
  i += strlen(&msg[i]) + 1;
  float precision = atof(&msg[i]);

  // altitude
  i += strlen(&msg[i]) + 1;
  float altitude = atof(&msg[i]);

  // altitude unit
  i += strlen(&msg[i]) + 1;
  char altitudeChar = msg[i];

  // The vertical distance between the surface of the
  // Earth and the surface of a model of the Earth
  // Geoidal separation
  i += strlen(&msg[i]) + 1;
  float gSep = atof(&msg[i]);

  // Geoidal separation unit
  i += strlen(&msg[i]) + 1;
  char gSepChar = msg[i];

  // Age of differential GPS data in seconds
  i += strlen(&msg[i]) + 1;
  float age = atof(&msg[i]);

  // Station ID
  i += strlen(&msg[i]) + 1;
  int station = atof(&msg[i]);

  // set output string to whatever
  // String output = String(degree_to_decimal(lat, NS), 7) + "," +
  //                 String(degree_to_decimal(lon, EW), 7) + "," +
  //                 String(quality)
  //                 +
  //                 "," + String(locked) + "," + '\n';

  String output = "Q:" + String(quality) + "  #:" + String(locked);

  return output;
}

// Want to convert DDMM.MMMM to a decimal number DD.DDDDD? Slap it into this.
float degree_to_decimal(float num, byte sign) {

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
