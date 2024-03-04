#ifndef nav
#define nav

// Union functions for byte to usable data conversions as IMU sends data as
// bytes in reverse order, the union functions are used to convert this data
// into other data types to actually use it

#include <Arduino.h>

// CAN IDs for the diffrent feilds
int ti_id = 0x1F4;
int gyro_id = 0x1F5;
int rate_id = 0x1F6;
int pos_id = 0x1F7;
int vel_id = 0x1F8;
int accel_id = 0x1F9;

// Custom struct for reading shit from
class navData {
public:
  bool nav_ready = false; // check if the sync byte (0xFA) is detected
  uint64_t r_ti;          // Time
  int16_t r_gyro[3];      // Attitude
  int16_t r_rate[3];      // Rate of Attitude
  int32_t r_pos[2];       // Lat lon
  int16_t r_vel[3];       // Velocity
  int16_t r_acl[3];       // Acceleration
  uint16_t r_ins;         // INS state

  void init();
  void read_data();
  void check_sync_byte();

private:
  unsigned short calculate_imu_crc(byte data[], unsigned int length);
  byte in[90]; // array to save data send from the IMU
};

// GPS time in unix epoch
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

// Start UART and config
void navData::init() {
  // Start NAV UART
  Serial8.begin(115200);

  // Please just use the Vector Nav Control Center tool to generate the messages
  // I fucking hate calculating these by hand so much don't torture yourself
  Serial8.println("$VNWRG,75,1,40,01,11EA*614A");
}

void navData::read_data() {
  // Read the bytes into an array
  Serial8.readBytes(in, 87);

  // Grab the checksum
  checksum.b[0] = in[86];
  checksum.b[1] = in[85];

  // Get Attitude, Rates, Velocity & Accel
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

  // If the checksum is correct
  if (calculate_imu_crc(in, 85) == checksum.s) {
    // Get Time & Position
    for (int i = 0; i < 8; i++) {
      ti.b[i] = in[3 + i];
      lat.b[i] = in[35 + i];
      lon.b[i] = in[43 + i];
      alt.b[i] = in[51 + i];
    }

    // Get INS state
    for (int i = 0; i < 2; i++) {
      ins.b[i] = in[83 + i];
    }

    // TODO: Fix this, this is ugly and it makes me sad
    r_ti = ti.v;

    r_gyro[0] = int16_t(yaw.f * 100);
    r_gyro[1] = int16_t(rol.f * 100);
    r_gyro[2] = int16_t(pit.f * 100);

    r_rate[0] = int16_t(W_x.f * 100);
    r_rate[1] = int16_t(W_y.f * 100);
    r_rate[2] = int16_t(W_z.f * 100);

    r_pos[0] = int32_t(lat.v * 10000000);
    r_pos[1] = int32_t(lon.v * 10000000);

    r_vel[0] = int16_t(v_n.f * 100);
    r_vel[1] = int16_t(v_e.f * 100);
    r_vel[2] = int16_t(v_d.f * 100);

    r_acl[0] = int16_t(a_x.f * 100);
    r_acl[1] = int16_t(a_y.f * 100);
    r_acl[2] = int16_t(a_z.f * 100);

    r_ins = ins.f;
  }

  // Don't update if shits fucked
}

// Check for the sync byte (0xFA)
void navData::check_sync_byte() {
  for (int i = 0; i < 6; i++) {
    Serial8.readBytes(in, 1);
    if (in[0] == 0xFA) {
      nav_ready = true;
      break;
    }
  }
}

// Calculate the 16-bit CRC for the given ASCII or binary message.
unsigned short navData::calculate_imu_crc(byte data[], unsigned int length) {
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
