#ifndef nav
#define nav

// Union functions for byte to usable fucking data conversions
// IMU sends data as bytes in reverse order, the union functions are used to
// convert this data into other data types to actually use it

#include <Arduino.h>

// Custom struct for reading shit from
class vNav {
private:
  byte in[90];

public:
  // Holders for nav data
  uint64_t time;
  int32_t lat_lon[2];
  uint16_t ins_state;
  int16_t ang_rate[3];
  int16_t attitude[3];
  int16_t velocity[3];
  int16_t accel[3];

  // Some functions or something idk
  void init();
  void read_data();           // read nav data
  bool check_sync_byte(void); // check for new msg
  unsigned short calc_imu_crc(byte data[], unsigned int length); // check msg
};

// GPS time data
union {
  uint64_t v;
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
  double v;
  byte b[8];
} lat;
union {
  double v;
  byte b[8];
} lon;
union {
  double v;
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

void vNav::init() {
  // Wait for NAV UART to start
  Serial.println("Init NAV");
  Serial8.begin(115200);

  // Please just use the Vector Nav Control Center tool to generate the messages
  // I fucking hate calculating these by hand so much don't torture yourself
  Serial8.println("$VNWRG,75,1,40,01,11EA*614A");
  Serial.println("NAV set");
}

// Read the NAV bytes
void vNav::read_data() {
  // Read the bytes into an array
  Serial8.readBytes(in, 87);

  // Grab the checksum
  checksum.b[0] = in[86];
  checksum.b[1] = in[85];

  // If the checksum is correct
  if (calc_imu_crc(in, 85) == checksum.s) {
    // Get Time & Position
    for (int i = 0; i < 8; i++) {
      ti.b[i] = in[3 + i];
      lat.b[i] = in[35 + i];
      lon.b[i] = in[43 + i];
      alt.b[i] = in[51 + i];
    }
    time = ti.v;
    lat_lon[0] = int32_t(lat.v * 10000000);
    lat_lon[1] = int32_t(lon.v * 10000000);

    // Get Attitude, Rates, Velocity & Acceleration
    for (int i = 0; i < 4; i++) {
      yaw.b[i] = in[11 + i];
      pit.b[i] = in[15 + i];
      rol.b[i] = in[19 + i];
      W_x.b[i] = in[23 + i];
      W_y.b[i] = in[27 + i];
      W_z.b[i] = in[31 + i];
      v_n.b[i] = in[59 + i];
      v_e.b[i] = in[63 + i];
      v_d.b[i] = in[67 + i];
      a_x.b[i] = in[71 + i];
      a_y.b[i] = in[75 + i];
      a_z.b[i] = in[79 + i];
    }
    attitude[0] = int16_t(yaw.f * 100);
    attitude[1] = int16_t(rol.f * 100);
    attitude[2] = int16_t(pit.f * 100);
    ang_rate[0] = int16_t(W_x.f * 100);
    ang_rate[1] = int16_t(W_y.f * 100);
    ang_rate[2] = int16_t(W_z.f * 100);
    velocity[0] = int16_t(v_n.f * 100);
    velocity[1] = int16_t(v_e.f * 100);
    velocity[2] = int16_t(v_d.f * 100);
    accel[0] = int16_t(a_x.f * 100);
    accel[1] = int16_t(a_y.f * 100);
    accel[2] = int16_t(a_z.f * 100);

    // Get INS state
    for (int i = 0; i < 2; i++) {
      ins.b[i] = in[83 + i];
    }
    ins_state = ins.f;
  }
}

// Check for the sync byte (0xFA)
bool vNav::check_sync_byte(void) {
  for (int i = 0; i < 6; i++) {
    Serial8.readBytes(in, 1);
    if (in[0] == 0xFA) {
      return true;
      break;
    }
  }
  return false;
}

// Calculate the 16-bit CRC for the given ASCII or binary message.
unsigned short vNav::calc_imu_crc(byte data[], unsigned int length) {
  unsigned int i;
  unsigned short crc = 0;
  for (i = 0; i < length; i++) {
    crc = (byte)(crc >> 8) | (crc << 8); // Rotate crc left 8 bits
    crc ^= data[i];                      // XOR crc with data[i]
    crc ^= (byte)(crc & 0xff) >> 4;      // XOR crc with lower 4 bits of crc
    crc ^= crc << 12;                    // Rotate crc left 12 bits
    crc ^= (crc & 0x00ff) << 5; // XOR crc w lower 8 bits & shift left 5 bits
  }
  return crc;
}

#endif
