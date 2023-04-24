/*
 * agilex_message.h
 *
 * Created on: Dec 10, 2020 11:47
 * Description:
 *  all values are using SI units (e.g. meter/second/radian)
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef AGILEX_MESSAGE_H
#define AGILEX_MESSAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "ugv_sdk/details/interface/agilex_types.h"

/***************** Firmware update messages *****************/
typedef struct {
  int32_t flash_size;
} UpdateStartMessage;

typedef struct {
  uint8_t flash_data[8];
} UpgradeProssessMessage;

typedef struct {
  AgxPercyBootState bootloader_state;
} UpdateStartBACKMessage;

typedef struct {
  AgxPercyFirmwareState flash_data[8];
} UpgradeProssessBACKMessage;

/***************** Control messages *****************/

typedef struct {
  float linear_velocity;
  float angular_velocity;
  float lateral_velocity;
  float steering_angle;
} MotionCommandMessage;

typedef struct {
  float linear_velocity;
  float angular_velocity;
} PercyMotionCommandMessage;

typedef struct {
  bool enable_cmd_ctrl;
  AgxLightOperation front_light;
  AgxLightOperation rear_light;
} LightCommandMessage;

typedef struct {
AgxPercyLightctlMode control_enable;
AgxPercyLightMode front_illumination_mode;
AgxPercyLightMode back_illumination_mode;
} PercyLightCmdMessage;

typedef uint8_t RGB_val[3];
typedef struct {
  RGB_val LEFT_LIGHT;
  RGB_val RIGHT_LIGHT;
} LightRGBMessage;

typedef struct {
  uint16_t power_status;
} PowerRailctlMessage;

/**************** pwoer rail messages *****************/
#define EXTERNAL_48V ((uint16_t)0x0001)
#define EXTERNAL_5V ((uint16_t)0x0002)
#define FAN_12V ((uint16_t)0x0004)

#define JETSON1_12V ((uint16_t)0x0100)
#define JETSON2_12V ((uint16_t)0x0200)
#define ULTRASONIC_SENSOR_12V ((uint16_t)0x0400)
#define CAMERA_12V ((uint16_t)0x0800)
#define ROUTER_5G_12V ((uint16_t)0x1000)
#define SWITCHBOARD_12V ((uint16_t)0x2000)
#define USB_HUB_12V ((uint16_t)0x4000)
#define SICK_SYSTEM_24V ((uint16_t)0x8000)

typedef struct {
  uint16_t power_status;
} PowerRailstatusMessage;

typedef struct {
  uint8_t power_statue;
} JetsonPoweroffMessage;

typedef struct {
  uint8_t power_statu[8];
} AMRShutdownMessage;

typedef struct {
  bool enable_braking;
} BrakingCommandMessage;

typedef struct {
  uint8_t motion_mode;
} MotionModeCommandMessage;

// V1-only messages
typedef struct {
  AgxControlMode control_mode;
  bool clear_all_error;
  float linear;
  float angular;
  float lateral;
} MotionCommandMessageV1;

typedef struct {
  bool set_neutral;
} ValueSetCommandMessageV1;

typedef ValueSetCommandMessageV1 ValueSetStateMessageV1;

/**************** Feedback messages *****************/

#define SYSTEM_ERROR_MOTOR_DRIVER_MASK ((uint16_t)0x0100)
#define SYSTEM_ERROR_HL_COMM_MASK ((uint16_t)0x0200)
#define SYSTEM_ERROR_BATTERY_FAULT_MASK ((uint16_t)0x0001)
#define SYSTEM_ERROR_BATTERY_WARNING_MASK ((uint16_t)0x0002)
#define SYSTEM_ERROR_RC_SIGNAL_LOSS_MASK ((uint16_t)0x0004)
#define SYSTEM_ERROR_MOTOR1_COMM_MASK ((uint16_t)0x0008)
#define SYSTEM_ERROR_MOTOR2_COMM_MASK ((uint16_t)0x0010)
#define SYSTEM_ERROR_MOTOR3_COMM_MASK ((uint16_t)0x0020)
#define SYSTEM_ERROR_MOTOR4_COMM_MASK ((uint16_t)0x0040)
#define SYSTEM_ERROR_STEER_ENCODER_MASK ((uint16_t)0x0080)

typedef struct {
  AgxVehicleState vehicle_state;
  AgxControlMode control_mode;
  float battery_voltage;
  uint16_t error_code;
} SystemStateMessage;

typedef struct {
  AgxVehicleState vehicle_state;
  AgxPercyControlMode control_mode;
  uint8_t revol_per[4];
  uint16_t error_code;
} PercySystemStateMessage;

typedef struct {
  float linear_velocity;
  float angular_velocity;  // only valid for differential drivering
  float lateral_velocity;
  float steering_angle;  // only valid for ackermann steering
} MotionStateMessage;

typedef struct {
  float linear_velocity;
  float angular_velocity;
} PercyMotionStateMessage;

typedef struct {
  uint8_t lightctl_enable;
  uint8_t front_illu_mode;
  uint8_t back_illu_mode;
  RGB_val LIGHT_STATUS[4];
} PercyLightStateMessage;

typedef LightCommandMessage LightStateMessage;

typedef struct {
  AgxRcSwitchState swa;
  AgxRcSwitchState swb;
  AgxRcSwitchState swc;
  AgxRcSwitchState swd;
  int8_t stick_right_v;
  int8_t stick_right_h;
  int8_t stick_left_v;
  int8_t stick_left_h;
  int8_t var_a;
} RcStateMessage;

typedef struct {
  uint8_t motor_id;
  int16_t rpm;
  float current;
  int32_t pulse_count;
} ActuatorHSStateMessage;

#define DRIVER_STATE_INPUT_VOLTAGE_LOW_MASK ((uint8_t)0x01)
#define DRIVER_STATE_MOTOR_OVERHEAT_MASK ((uint8_t)0x02)
#define DRIVER_STATE_DRIVER_OVERLOAD_MASK ((uint8_t)0x04)
#define DRIVER_STATE_DRIVER_OVERHEAT_MASK ((uint8_t)0x08)
#define DRIVER_STATE_SENSOR_FAULT_MASK ((uint8_t)0x10)
#define DRIVER_STATE_DRIVER_FAULT_MASK ((uint8_t)0x20)
#define DRIVER_STATE_DRIVER_ENABLED_MASK ((uint8_t)0x40)
#define DRIVER_STATE_DRIVER_RESET_MASK ((uint8_t)0x80)

typedef struct {
  uint8_t motor_id;
  float driver_voltage;
  uint8_t driver_error;
} ActuatorLSStateMessage;

// for ranger
typedef struct {
  uint8_t motion_mode;
  uint8_t mode_changing;
} MotionModeStateMessage;

// V1-only messages
typedef struct {
  uint8_t motor_id;
  float current;
  int16_t rpm;
  float driver_temp;
  float motor_temp;
} ActuatorStateMessageV1;

/***************** Sensor messages ******************/

typedef struct {
  float left_wheel;
  float right_wheel;
} OdometryMessage;

typedef struct {
  float accel_x;
  float accel_y;
  float accel_z;
} ImuAccelMessage;

typedef struct {
  float gyro_x;
  float gyro_y;
  float gyro_z;
} ImuGyroMessage;

typedef struct {
  float yaw;
  float pitch;
  float roll;
} ImuEulerMessage;

typedef struct {
  uint8_t trigger_state;
} SafetyBumperMessage;

typedef struct {
  uint8_t sensor_id;
  uint8_t distance[8];
} UltrasonicMessage;

typedef struct {
  uint8_t sensor_id;
  float relative_distance;
  float relative_angle;
  bool is_normal;
  int8_t channels[3];
} UwbMessage;

#define BUTTON_LONG_PRESS_3_SEC 0x01; //poweroff event
#define BUTTON_LONG_PRESS_10_SEC 0x02; //force shutdown event

typedef struct {
  uint8_t power_button_event;
} PowerBUttonEventFb;

typedef struct {
  uint8_t battery_soc;
  uint8_t battery_soh;
  float voltage;
  float current;
  float temperature;
} BmsBasicMessage;

#define BMS_PROT1_CHARGING_CURRENT_NONZERO_MASK ((uint8_t)0x01)
#define BMS_PROT1_CHARGING_OVERCURRENT_SET_MASK ((uint8_t)0x02)
#define BMS_PROT1_DISCHARGING_CURRENT_NONZERO_MASK ((uint8_t)0x10)
#define BMS_PROT1_DISCHARGING_OVERCURRENT_SET_MASK ((uint8_t)0x20)
#define BMS_PROT1_DISCHARGING_SHORTCIRCUIT_SET_MASK ((uint8_t)0x40)

#define BMS_PROT2_CORE_OPENCIRCUIT_SET_MASK ((uint8_t)0x01)
#define BMS_PROT2_TEMP_SENSOR_OPENCIRCUIT_SET_MASK ((uint8_t)0x02)
#define BMS_PROT2_CORE_OVERVOLTAGE_SET_MASK ((uint8_t)0x10)
#define BMS_PROT2_CORE_UNDERVOLTAGE_SET_MASK ((uint8_t)0x20)
#define BMS_PROT2_TOTAL_OVERVOLTAGE_SET_MASK ((uint8_t)0x40)
#define BMS_PROT2_TOTAL_UNDERVOLTAGE_SET_MASK ((uint8_t)0x80)

#define BMS_PROT3_CHARGING_OVERTEMP_SET_MASK ((uint8_t)0x04)
#define BMS_PROT3_DISCHARGING_OVERTEMP_SET_MASK ((uint8_t)0x08)
#define BMS_PROT3_CHARGING_UNDERTEMP_SET_MASK ((uint8_t)0x10)
#define BMS_PROT3_DISCHARGING_UNDERTEMP_SET_MASK ((uint8_t)0x20)
#define BMS_PROT3_CHARGING_TEMPDIFF_SET_MASK ((uint8_t)0x40)
#define BMS_PROT3_DISCHARGING_TEMPDIFF_SET_MASK ((uint8_t)0x80)

#define BMS_PROT4_CHARGING_MOS_STATE_SET_MASK ((uint8_t)0x01)
#define BMS_PROT4_DISCHARGING_MOS_STATE_SET_MASK ((uint8_t)0x02)
#define BMS_PROT4_CHARGING_MOS_FAILURE_SET_MASK ((uint8_t)0x04)
#define BMS_PROT4_DISCHARGING_MOS_FAILURE_SET_MASK ((uint8_t)0x08)
#define BMS_PROT4_WEAK_SIGNAL_SWITCH_OPEN_SET_MASK ((uint8_t)0x10)

typedef struct {
  uint8_t protection_code1;
  uint8_t protection_code2;
  uint8_t protection_code3;
  uint8_t protection_code4;
  uint8_t battery_max_teperature;
  uint8_t battery_min_teperature;
} BmsExtendedMessage;

typedef struct {
  uint32_t wheel_circumference;
  uint32_t wheel_track;
} MechanicalCaliMessage;

/************  Query/config messages ****************/

typedef struct {
  bool request;
} VersionRequestMessage;

typedef struct {
  uint16_t controller_hw_version;
  uint16_t motor_driver_hw_version;
  uint16_t controller_sw_version;
  uint16_t motor_driver_sw_version;
} VersionResponseMessage;

typedef struct {
  AgxControlMode mode;
} ControlModeConfigMessage;

typedef struct {
  AgxBrakeMode mode;
} BrakeModeConfigMessage;

typedef struct {
  bool set_as_neutral;
} SteerNeutralRequestMessage;

typedef struct {
  bool neutral_set_successful;
} SteerNeutralResponseMessage;

typedef enum {
  CLEAR_ALL_FAULT = 0x00,
  CLEAR_MOTOR1_FAULT = 0x01,
  CLEAR_MOTOR2_FAULT = 0x02,
  CLEAR_MOTOR3_FAULT = 0x03,
  CLEAR__MOTOR4_FAULT = 0x04
} FaultClearCode;

typedef struct {
  uint8_t error_clear_byte;
} StateResetConfigMessage;

typedef struct {
  uint8_t fixedcontent[8]; 
} RestoreFactorySetting;

typedef struct {
  uint8_t control_mode;
} PercyCANControl;

typedef struct {
  uint8_t control_mode;
} PercyClearError;

typedef struct {
  uint16_t max_acc;
  uint16_t max_dec;
} ACC_DEC_CONFIG;

typedef struct {
  uint8_t T_PWR_DELAY_1;
  uint8_t T_PWR_DELAY_2;
  uint8_t T_ON_TIMEOUT;
  uint8_t T_OFF_TIMEOUT;
  uint8_t T_ON;
  uint8_t T_OFF;
  uint8_t T_PB;
} PowerRailBootTimconfig;

typedef struct {
  uint8_t Fan1_revol_perc;
  uint8_t Fan2_revol_perc;
  uint8_t Fan3_revol_perc;
  uint8_t Fan4_revol_perc;
} FanFevolvingSpeedConfig;

typedef struct {
  uint8_t ChargeRail;
  uint8_t ChargeCurrent;
} ChargeConfig;

typedef struct {
  uint8_t FixContent;
} FirmwareVersionQ;

typedef struct {
  uint8_t version[80];
} FirmwareVersionConfig;

typedef struct {
  uint8_t fixed_content;
} AMRNodeConfig;

//////////////////////////////////////////////////////

typedef enum {
  AgxMsgUnkonwn = 0x00,
  // command
  AgxMsgMotionCommand,
  AgxMsgLightCommand,
  AgxMsgBrakingCommand,
  AgxMsgSetMotionModeCommand,
  // state feedback
  AgxMsgSystemState,
  AgxMsgMotionState,
  AgxMsgLightState,
  AgxMsgMotionModeState,
  AgxMsgRcState,
  // actuator feedback
  AgxMsgActuatorHSState,
  AgxMsgActuatorLSState,
  // sensor
  AgxMsgOdometry,
  AgxMsgImuAccel,
  AgxMsgImuGyro,
  AgxMsgImuEuler,
  AgxMsgSafetyBumper,
  AgxMsgUltrasonic,
  AgxMsgUwb,
  AgxMsgBmsBasic,
  AgxMsgBmsExtended,
  // query/config
  AgxMsgVersionRequest,
  AgxMsgVersionResponse,
  AgxMsgControlModeConfig,
  AgxMsgSteerNeutralRequest,
  AgxMsgSteerNeutralResponse,
  AgxMsgStateResetConfig,
  AgxMsgBrakeModeConfig,
  // V1-only messages
  AgxMsgMotionCommandV1,
  AgxMsgValueSetCommandV1,
  AgxMsgValueSetStateV1,
  AgxMsgActuatorStateV1
} MsgType;

typedef struct {
  MsgType type;
  union {
    // command
    MotionCommandMessage motion_command_msg;
    LightCommandMessage light_command_msg;
    BrakingCommandMessage braking_command_msg;
    MotionModeCommandMessage motion_mode_msg;
    // core state feedback
    SystemStateMessage system_state_msg;
    MotionStateMessage motion_state_msg;
    LightStateMessage light_state_msg;
    MotionModeStateMessage motion_mode_state_msg;
    RcStateMessage rc_state_msg;
    // actuator feedback
    ActuatorHSStateMessage actuator_hs_state_msg;
    ActuatorLSStateMessage actuator_ls_state_msg;
    // sensor
    OdometryMessage odometry_msg;
    ImuAccelMessage imu_accel_msg;
    ImuGyroMessage imu_gyro_msg;
    ImuEulerMessage imu_euler_msg;
    SafetyBumperMessage safety_bumper_msg;
    UltrasonicMessage ultrasonic_msg;
    UwbMessage uwb_msg;
    BmsBasicMessage bms_basic_msg;
    BmsExtendedMessage bms_extended_msg;
    // query/config
    VersionRequestMessage version_request_msg;
    VersionResponseMessage version_response_msg;
    ControlModeConfigMessage control_mode_config_msg;
    BrakeModeConfigMessage brake_mode_config_msg;
    SteerNeutralRequestMessage steer_neutral_request_msg;
    SteerNeutralResponseMessage steer_neutral_response_msg;
    StateResetConfigMessage state_reset_config_msg;
    // V1-only messages
    MotionCommandMessageV1 v1_motion_command_msg;
    ValueSetCommandMessageV1 v1_value_set_command_msg;
    ValueSetStateMessageV1 v1_value_set_state_msg;
    ActuatorStateMessageV1 v1_actuator_state_msg;
  } body;
} AgxMessage;

//add for percy robot
typedef enum {
  // Firmware Update Function
  Percy_UpgradeStartCommand,
  Percy_UpgradeProcessCommand,
  // command
  AgxPercyMsgMotionCommand,
  AgxPercyMsgLightCommand,
  AgxPercyMsgFrontRGBCommand,
  AgxPercyMsgBackRGBCommand,

  AgxMsgPercyBrakingCommand,
  AgxMsgPercyBrakingRes,

  AgxMsgPercyPowerRailCommand,
  AgxMsgPercyPowerRailRes,
  AgxMsgPercyJetsonPoweroffCommand,
  AgxMsgPercyJetsonPoweroffRes,
  AgxMsgPercyAMRShutDownCommand,
  AgxMsgPercyAMRShutDownRes,

  // state feedback
  AgxMsgPercySystemState,
  AgxMsgPercyMotionState,
  AgxMsgPercyLightState,
  AgxMsgPercyLightFrontRGBState,
  AgxMsgPercyLightBackRGBState,
  //RC states
  AgxMsgPercyRcState,
  // actuator feedback
  AgxMsgPercyActuatorHSState,
  AgxMsgPercyActuatorLSState,

  // sensor
  AgxMsgPercyOdometry,
  AgxMsgPercyBmsBasic,
  AgxMsgPercyPowerButton,
  AgxMsgPercyMechanicalCali,

  // query/config
  AgxMsgPercyFactory,
  AgxMsgPercyFactoryRes,

  AgxMsgPercyControlEnable,
  AgxMsgPercyClearErrorStates,
  AgxMsgPercyAccDecConfig,
  AgxMsgPercyAccDecRes,
  AgxMsgPercyPowerRailBoot,
  AgxMsgPercyPowerRailBootRes,
  AgxMsgPercyFanRevolingSpeed,
  AgxMsgPercyChargeConfig,
  AgxMsgPercyFirmwareVersion,
  AgxMsgPercyFirmwareConfig,
  AgxMsgPercyAMRNode,

} Percy_MsgType;

typedef struct {
  Percy_MsgType type;
  union {
    // Firmware Update Function
    UpdateStartMessage updatestart_command_msg;
    UpgradeProssessMessage upgrade_process_command_msg;
    // command
    PercyMotionCommandMessage motion_command_msg;
    PercyLightCmdMessage light_command_msg;
    LightRGBMessage front_light_RGB_msgs;
    LightRGBMessage back_light_RGB_msgs;
    BrakingCommandMessage braking_command_msg;
    PowerRailctlMessage pwr_rail_ctl_cmd_msg;
    JetsonPoweroffMessage jetson_pwr_off_cmd_msg;
    AMRShutdownMessage amr_shutdown_cmd_msg;

    // core state feedback
    PercySystemStateMessage system_state_msg;
    PercyMotionStateMessage motion_state_msg;
    PercyLightStateMessage light_state_msg;
    RcStateMessage rc_state_msg;
    // actuator feedback
    ActuatorHSStateMessage actuator_hs_state_msg;
    ActuatorLSStateMessage actuator_ls_state_msg;
    // sensor
    OdometryMessage odometry_msg;
    BmsBasicMessage bms_basic_msg;
    PowerBUttonEventFb PowerButtonEvfb_msg;
    MechanicalCaliMessage MechanicalCali_msg;

    // query/config
    RestoreFactorySetting restore_fac_set_msg;
    PercyCANControl can_control_msg;
    PercyClearError clear_error_msg;
    ACC_DEC_CONFIG acc_dec_conf_msg;
    PowerRailBootTimconfig power_rail_boot_msg;
    FanFevolvingSpeedConfig fan_speed_cmd_msg;
    ChargeConfig charge_config_msg;
    FirmwareVersionQ firm_ver_q_msg;
    FirmwareVersionConfig firm_ver_config_msg;
    AMRNodeConfig amr_node_config_msg;
  } body;
} Agx_Percy_Message;

#ifdef __cplusplus
}
#endif

#endif /* AGILEX_MESSAGE_H */
