#ifndef nav
#define nav

// Union functions for byte to usable fucking data conversions
// IMU sends data as bytes in reverse order, the union functions are used to
// convert this data into other data types to actually use it

#include <Arduino.h>

// Some internal functions
String read_nav_data(int option); // read nav data
void check_sync_byte(void);       // check for new msg
unsigned short calculate_imu_crc(byte data[], unsigned int length); // check msg

// Some global vars
bool nav_ready = false; // check if the sync byte (0xFA) is detected
byte in[87];            // array to save data send from the IMU

// CAN IDs for the diffrent feilds
String ti_id = "0xNT";
String gyro_id = "0xNG";
String rate_id = "0xNR";
String pos_id = "0xNP";
String vel_id = "0xNV";
String accel_id = "0xNA";

// Custom struct for reading shit from
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
