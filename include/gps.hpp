#ifndef gps
#define gps

// All the GPS bullshit rolled up into a header file so that it isn't an eyesore
// when not in use, not that we should ever use it lmao

#include <Arduino.h>

// More functions
void init_GPS();
String parse_rmc(String msg);                  // Parse RMC string function
String parse_gga(String msg);                  // Prase GGA string im tired
float degree_to_decimal(float num, byte sign); // GPS conversion function

// Some global vars
String gps_id = "";           // Whatever ID you want for your fake can msg
String input_serial8 = "";    // a string to hold incoming data
boolean ready_serial8 = true; // whether the string is complete

void init_GPS() {
  // Wait for GPS UART to start
  Serial8.begin(9600);

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

// NOTE: These two functions probabaly need to be in the main.cpp, but hopefully
// we never have to use the adafruit GPS again sooo....
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
    send_packet(msg.id, msg.buf);
#endif

    // Reset vars
    input_serial8 = "";
    ready_serial8 = false;
  }
}

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

#endif
