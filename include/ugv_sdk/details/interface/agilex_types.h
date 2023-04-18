/*
 * agilex_types.h
 *
 * Created on: Jul 09, 2021 21:57
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef AGILEX_TYPES_H
#define AGILEX_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
  CONST_OFF = 0x00,
  CONST_ON = 0x01,
  BREATH = 0x02,
  CUSTOM = 0x03
} AgxLightMode;

typedef struct {
  AgxLightMode mode;
  uint8_t custom_value;
} AgxLightOperation;

typedef enum {
  VEHICLE_STATE_NORMAL = 0x00,
  VEHICLE_STATE_ESTOP = 0x01,
  VEHICLE_STATE_EXCEPTION = 0x02
} AgxVehicleState;

typedef enum {
  //   CONTROL_MODE_STANDBY = 0x00,
  CONTROL_MODE_RC = 0x00,
  CONTROL_MODE_CAN = 0x01,
  CONTROL_MODE_UART = 0x02
} AgxControlMode;

typedef enum {
  //   CONTROL_MODE_STANDBY = 0x00,
  BRAKE_MODE_UNLOCK = 0x00,
  BRAKE_MODE_LOCK = 0x01
} AgxBrakeMode;

typedef enum {
  RC_SWITCH_UP = 0,
  RC_SWITCH_MIDDLE,
  RC_SWITCH_DOWN
} AgxRcSwitchState;

//Firmware update function
typedef enum{
  READY_TO_TURN_BOOT = 0x01,
  READY_TO_ERASE_FLASH = 0x02,
  FLASH_ERASE_SUCCESS = 0x03,
  FLASH_ERASE_FAILD = 0x04
} AgxPercyBootState;

typedef enum{
  FIRMWARE_DATA_RECEIVED = 0x01,
  DATA_1K_RECEIVED_SUCCESS = 0x02,
  DATA_1K_RECEIVED_FAILED = 0x03,
  DATA_1K_FLASHED = 0x04,
  DATA_1K_FLASH_FAILED = 0x05,
  FIRMWARE_TRANSFER_SUCCESS = 0x06
} AgxPercyFirmwareState;

typedef enum {
  //   CONTROL_MODE_STANDBY = 0x00,
  P_CONTROL_MODE_STANDY = 0x00,
  P_CONTROL_MODE_CAN = 0x01,
  P_CONTROL_MODE_APP = 0x02,
  P_CONTROL_MODE_REMOTE = 0x03
} AgxPercyControlMode;

typedef enum{
  AUTO_CONTROL = 0x00,
  CONTROL_ENABLE = 0x01
} AgxPercyLightctlMode;

typedef enum{
  TURN_OFF = 0x00,
  TURN_ON = 0x01
} AgxPercyLightMode;

typedef enum {
  //   CONTROL_MODE_STANDBY = 0x00,
  BRAKE_CTL_BY_VCU = 0x00,
  BRAKE_RELEASE = 0x01
} AgxPercyBrakeMode;

//power status
#define POWER_STATUS_E48V_MASK ((uint16_t)0x0001)
#define POWER_STATUS_E5V_MASK ((uint16_t)0x0002)
#define POWER_STATUS_F12V_MASK ((uint16_t)0x0004)
#define POWER_STATUS_J1_12V_MASK ((uint16_t)0x0100)
#define POWER_STATUS_J2_12V_MASK ((uint16_t)0x0200)
#define POWER_STATUS_SENSOR_12V_MASK ((uint16_t)0x0400)
#define POWER_STATUS_CAMERA_12V_MASK ((uint16_t)0x0800)
#define POWER_STATUS_5GROUTER_12V_MASK ((uint16_t)0x1000)
#define POWER_STATUS_SWITCHOARD_12V_MASK ((uint16_t)0x2000)
#define POWER_STATUS_USBHUB_12V_MASK ((uint16_t)0x4000)
#define POWER_STATUS_SICKSYSTEM_24V_MASK ((uint16_t)0x8000)

//error code 1
#define DRIVER_STATE_MASK ((uint8_t)0x01)
#define DRIVER_STATE_MOTOR_DRIVER1_POWER_MASK ((uint8_t)0x02)
#define DRIVER_STATE_MOTOR_DRIVER2_POWER_MASK ((uint8_t)0x04)
#define E_STOP_TRIGGER_MASK ((uint8_t)0x08)
#define DRIVER_STATE_SENSOR_MASK ((uint8_t)0x40)
//error code 2
#define PERCY_VOLTAGE_LOW_MASK ((uint8_t)0x01)
#define PERCY_VOLTAGE_LOW_WARMING_MASK ((uint8_t)0x02)
#define PERCY_REMOTER_COM_ERROR_MASK ((uint8_t)0x04)
#define PERCY_MOTOR1_DRIVER_CAN_ERROR_MASK ((uint8_t)0x08)
#define PERCY_MOTOR2_DRIVER_CAN_ERROR_MASK ((uint8_t)0x10)
#define PERCY_CAN_CONTROL_ERROR_MASK ((uint8_t)0x20)

#ifdef __cplusplus
}
#endif

#endif /* AGILEX_TYPES_H */
