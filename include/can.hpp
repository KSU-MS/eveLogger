#ifndef can
#define can
// Imports
#include <FlexCAN_T4.h>
#include <KSUCAN.hpp>

// CAN Variables
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> BMS_CAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> INV_CAN;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> DAQ_CAN;
static CAN_message_t msg;

CAN_message_t vectornav_time, vectornav_position, vectornav_attitude,
    vectornav_gyro, vectornav_velocity, vectornav_accel;

CAN_message_t *vnav_msgs[] = {&vectornav_time,     &vectornav_position,
                              &vectornav_attitude, &vectornav_gyro,
                              &vectornav_velocity, &vectornav_accel};

// Basic init function
void init_CAN() {
  // Init each controller
  BMS_CAN.begin();
  INV_CAN.begin();
  DAQ_CAN.begin();

  // Set buad rate
  BMS_CAN.setBaudRate(500000);
  INV_CAN.setBaudRate(500000);
  DAQ_CAN.setBaudRate(1000000);
}
#endif // can
