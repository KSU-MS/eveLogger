// Libs
#include <Arduino.h>
#include <FlexCAN_T4.h>

// Global Variables
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> BMS_CAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> IMD_CAN;
CAN_message_t msg;

void setup() {
  // Init each controller
  BMS_CAN.begin();
  IMD_CAN.begin();

  // Set buad rate
  BMS_CAN.setBaudRate(500000);
  IMD_CAN.setBaudRate(250000);

  // Do te ting
  Serial.println("Log start");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  if (IMD_CAN.read(msg)) {
    BMS_CAN.write(msg);
  }

  digitalToggle(LED_BUILTIN);
}
