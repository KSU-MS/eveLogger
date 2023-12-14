#ifndef gps
#define gps

// All the GPS bullshit rolled up into a header file so that it isn't an eyesore
// when not in use, not that we should ever use it lmao

#include <Arduino.h>

// More internal functions
String parse_rmc(String msg);                  // Parse RMC string function
String parse_gga(String msg);                  // Prase GGA string im tired
float degree_to_decimal(float num, byte sign); // GPS conversion function

// Some global vars
String gps_id = "";           // Whatever ID you want for your fake can msg
String input_serial8 = "";    // a string to hold incoming data
boolean ready_serial8 = true; // whether the string is complete

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
#endif
