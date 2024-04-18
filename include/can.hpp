#ifndef can
#define can
// Imports
#include <FlexCAN_T4.h>
#include <nav.hpp>

// CAN Variables
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> fCAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> sCAN;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> tCAN;
static CAN_message_t msg;

// CAN IDs for the diffrent feilds
const int nav_time_id = 0x1F4;
const int nav_attitude_id = 0x1F5;
const int nav_gyro_id = 0x1F6;
const int nav_position_id = 0x1F7;
const int nav_velocity_id = 0x1F8;
const int nav_accel_id = 0x1F9;

CAN_message_t vectornav_time, vectornav_attitude, vectornav_gyro,
    vectornav_position, vectornav_velocity, vectornav_accel;

CAN_message_t vnav_msgs[] = {vectornav_time,     vectornav_attitude,
                             vectornav_gyro,     vectornav_position,
                             vectornav_velocity, vectornav_accel};
#endif // can
