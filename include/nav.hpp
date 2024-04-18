#ifndef nav
#define nav

// Union functions for byte to usable fucking data conversions
// IMU sends data as bytes in reverse order, the union functions are used to
// convert this data into other data types to actually use it

#include <Arduino.h>

// Custom struct for reading shit from
class vNav {
private:
  unsigned short calc_imu_crc(byte data[], uint16_t length); // check msg
  HardwareSerial &serial_port;                               // Target port
  byte in[90];                                               // Buffer to hold

  // Raw byte -> data guys
  union {
    uint64_t v;
    byte b[8];
  } raw_time;
  union {
    uint16_t v;
    byte b[2];
  } raw_INS;
  union {
    struct {
      float yaw;
      float pitch;
      float roll;
    };
    byte b[12];
  } raw_attitude;
  union {
    struct {
      float x;
      float y;
      float z;
    };
    byte b[12];
  } raw_ang_rate;
  union {
    struct {
      float north;
      float east;
      float down;
    };
    byte b[12];
  } raw_velocity;
  union {
    struct {
      float x;
      float y;
      float z;
    };
    byte b[12];
  } raw_accel;
  union {
    struct {
      double latitude;
      double longitude;
      double altitude;
    };
    byte b[24];
  } raw_position;
  union {
    uint16_t s;
    byte b[2];
  } checksum;

public:
  // Can ready values
  uint64_t time;
  uint16_t ins;
  int32_t lat_lon[2];
  int16_t attitude[3];
  int16_t ang_rate[3];
  int16_t velocity[3];
  int16_t accel[3];

  // Some functions or something idk
  vNav(HardwareSerial &target_port) : serial_port(target_port){};
  void init();
  void read_data();           // read nav data
  bool check_sync_byte(void); // check for new msg
};

void vNav::init() {
  // Wait for NAV UART to start
  Serial.println("Init NAV");
  serial_port.begin(230400);

  // Please just use the Vector Nav Control Center tool to generate the messages
  // I fucking hate calculating these by hand so much don't torture yourself
  // also if you change the feilds it fucks with the parser to be careful

  // Baud rate
  serial_port.println("$VNWRG,5,230400,1*7049");

  // Refrence frame offsets
  serial_port.println("$VNWRG,26,+0.000000,+0.000000,-1.000000,+0.000000,-1."
                      "000000,+0.000000,-1.000000,+0.000000,+0.000000*20A7");

  // GPS antenna offset
  serial_port.println("$VNWRG,57,+0.798,-0.127,-0.495*5546");

  // This sets what feilds you want and at what rate for the first bin output
  serial_port.println("$VNWRG,75,1,40,01,1042*23E6");

  // And this is for the secondary bin output
  serial_port.println("$VNWRG,76,1,2,01,01A8*0488");

  Serial.println("NAV set");
}

// Read the NAV bytes
void vNav::read_data() {
  // Read the bytes into an array
  serial_port.readBytes(in, 87);

  // Grab the checksum
  checksum.b[0] = in[86];
  checksum.b[1] = in[85];

  // If the checksum is correct
  if (calc_imu_crc(in, 85) == checksum.s) {
    // Get Time
    for (int i = 0; i < 8; i++) {
      raw_time.b[i] = in[3 + i];
    }
    time = raw_time.v;

    // Get INS state
    for (int i = 0; i < 2; i++) {
      raw_INS.b[i] = in[83 + i];
    }
    ins = raw_INS.v;

    // Get position
    for (int i = 0; i < 24; i++) {
      raw_position.b[i] = in[35 + i];
    }
    lat_lon[0] = int32_t(raw_position.latitude * 10000000);
    lat_lon[1] = int32_t(raw_position.longitude * 10000000);

    // Get attitude, rate of attitude, velocity & acceleration
    for (int i = 0; i < 12; i++) {
      raw_attitude.b[i] = in[11 + i];
      raw_ang_rate.b[i] = in[23 + i];
      raw_velocity.b[i] = in[59 + i];
      raw_accel.b[i] = in[71 + i];
    }
    attitude[0] = int16_t(raw_attitude.yaw * 100);
    attitude[1] = int16_t(raw_attitude.roll * 100);
    attitude[2] = int16_t(raw_attitude.pitch * 100);
    ang_rate[0] = int16_t(raw_ang_rate.x * 100);
    ang_rate[1] = int16_t(raw_ang_rate.y * 100);
    ang_rate[2] = int16_t(raw_ang_rate.z * 100);
    velocity[0] = int16_t(raw_velocity.north * 100);
    velocity[1] = int16_t(raw_velocity.east * 100);
    velocity[2] = int16_t(raw_velocity.down * 100);
    accel[0] = int16_t(raw_accel.x * 100);
    accel[1] = int16_t(raw_accel.y * 100);
    accel[2] = int16_t(raw_accel.z * 100);
  }
}

// Check for the sync byte (0xFA)
bool vNav::check_sync_byte(void) {
  for (int i = 0; i < 6; i++) {
    serial_port.readBytes(in, 1);
    if (in[0] == 0xFA) {
      return true;
    }
  }
  return false;
}

// Calculate the 16-bit CRC for the given ASCII or binary message.
uint16_t vNav::calc_imu_crc(byte data[], uint16_t length) {
  uint16_t crc = 0;
  for (uint16_t i = 0; i < length; i++) {
    crc = (byte)(crc >> 8) | (crc << 8); // Rotate crc left 8 bits
    crc ^= data[i];                      // XOR crc with data[i]
    crc ^= (byte)(crc & 0xff) >> 4;      // XOR crc with lower 4 bits of crc
    crc ^= crc << 12;                    // Rotate crc left 12 bits
    crc ^= (crc & 0x00ff) << 5; // XOR crc w lower 8 bits & shift left 5 bits
  }
  return crc;
}

#endif
