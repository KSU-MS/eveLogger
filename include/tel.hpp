#ifndef tel
#define tel

#include <Arduino.h>

void init_TEL() {
  // Start TEL UART
  Serial3.begin(115200);
}

void send_packet(uint16_t id, uint8_t *buf, size_t len) {
  // Start sending guy
  uint8_t id_[sizeof(id)];
  memcpy(id_, &id, sizeof(id));

  // Send ID
  Serial3.write(id_, sizeof(id_));

  // Send Buffer data
  for (unsigned int i = 0; i < len; i++) {
    Serial3.write(buf[i]);
  }

  // Break line
  Serial3.write('\n');
  Serial3.write(0xFF);
  Serial3.write('\n');
}

#endif
