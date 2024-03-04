#ifndef tel
#define tel

#include <Arduino.h>

void init_TEL() {
  // Start TEL UART
  Serial3.begin(115200);
}

void send_packet(uint32_t id, uint8_t *buf) {
  // Start sending guy
  Serial3.write(id);
  Serial3.write(',');
  for (int i = 0; i < sizeof(buf); i++) {
    Serial3.write(buf[i]);
  }

  // Break line
  Serial3.write('\n');
}

#endif
