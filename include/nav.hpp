#ifndef nav
#define nav

// Union functions for byte to usable fucking data conversions
// IMU sends data as bytes in reverse order, the union functions are used to
// convert this data into other data types to actually use it

#include <Arduino.h>

bool nav_ready = false; // check if the sync byte (0xFA) is detected
byte in[87];            // array to save data send from the IMU

// CAN IDs for the diffrent feilds
String ti_id = "0xNT";
String gyro_id = "0xNG";
String rate_id = "0xNR";
String pos_id = "0xNP";
String vel_id = "0xNV";
String accel_id = "0xNA";

class navData {
public:
  String r_ti;
  String r_gyro;
  String r_rate;
  String r_pos;
  String r_vel;
  String r_acl;
  String r_ins;
};

// GPS time data (cant use var time as it conflicts with alot of shit)
union {
  uint64_t f;
  byte b[8];
} ti;

// Attitude data (Gyro data stuffs)
union {
  float f;
  byte b[4];
} yaw;
union {
  float f;
  byte b[4];
} pit;
union {
  float f;
  byte b[4];
} rol;

// Angular rates
union {
  float f;
  byte b[4];
} W_x;
union {
  float f;
  byte b[4];
} W_y;
union {
  float f;
  byte b[4];
} W_z;

// Position data
union {
  double f;
  byte b[8];
} lat;
union {
  double f;
  byte b[8];
} lon;
union {
  double f;
  byte b[8];
} alt;

// Velocity data
union {
  float f;
  byte b[4];
} v_n;
union {
  float f;
  byte b[4];
} v_e;
union {
  float f;
  byte b[4];
} v_d;

// Acceleration data
union {
  float f;
  byte b[4];
} a_x;
union {
  float f;
  byte b[4];
} a_y;
union {
  float f;
  byte b[4];
} a_z;

// INS status data
union {
  uint16_t f;
  byte b[2];
} ins;

// Checksum
union {
  unsigned short s;
  byte b[2];
} checksum;

#endif
