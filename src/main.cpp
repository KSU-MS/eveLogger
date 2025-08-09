// Libs
#include <Arduino.h>
#include <FlexCAN_T4.h>

// Global Variables
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN_1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN_2;
CAN_message_t msg;

void setup() {
  Serial.begin(9600);

  // Init each controller
  CAN_1.begin();
  CAN_2.begin();

  // Set buad rate
  CAN_1.setBaudRate(250000);
  CAN_2.setBaudRate(500000);

  // Do te ting
  Serial.println("Log start");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);

  if (CAN_1.read(msg)) {
    CAN_2.write(msg);
  }
}
