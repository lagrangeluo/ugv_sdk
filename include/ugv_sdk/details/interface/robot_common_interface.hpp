/*
 * robot_interface.hpp
 *
 * Created on: Jul 08, 2021 11:48
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include <string>
#include <chrono>
#include <stdexcept>

#include "ugv_sdk/details/interface/agilex_message.h"
#include "ugv_sdk/details/interface/parser_interface.hpp"

#define AGX_MAX_ACTUATOR_NUM 8

namespace agilexrobotics {
using AgxMsgRefClock = std::chrono::steady_clock;
using AgxMsgTimeStamp = std::chrono::time_point<AgxMsgRefClock>;

struct CoreStateMsgGroup {
  AgxMsgTimeStamp time_stamp;

  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;
  MotionModeStateMessage motion_mode_state;
  RcStateMessage rc_state;
};

struct ActuatorStateMsgGroup {
  AgxMsgTimeStamp time_stamp;

  ActuatorHSStateMessage actuator_hs_state[AGX_MAX_ACTUATOR_NUM];  // v2 only
  ActuatorLSStateMessage actuator_ls_state[AGX_MAX_ACTUATOR_NUM];  // v2 only
  ActuatorStateMessageV1 actuator_state[AGX_MAX_ACTUATOR_NUM];     // v1 only
};

struct CommonSensorStateMsgGroup {};

class RobotCommonInterface {
 public:
  virtual ~RobotCommonInterface() = default;

  // functions to be implemented by class AgilexBase
  virtual void EnableCommandedMode() = 0;

  // functions to be implemented by each robot class
  virtual bool Connect(std::string can_name) = 0;

  virtual void ResetRobotState() = 0;

  virtual void DisableLightControl() {
    // do nothing if no light on robot
  }

  virtual ProtocolVersion GetParserProtocolVersion() = 0;

 protected:
  /****** functions not available/valid to all robots ******/
  // functions to be implemented by class AgilexBase
  virtual void SetMotionMode(uint8_t mode){};
  virtual void SetBrakedMode(AgxBrakeMode mode){};

  virtual CoreStateMsgGroup GetRobotCoreStateMsgGroup() {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
    return CoreStateMsgGroup{};
  };
  virtual ActuatorStateMsgGroup GetActuatorStateMsgGroup() {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
    return ActuatorStateMsgGroup{};
  };

  // any specific robot will use a specialized version of the two functions
  virtual void SendMotionCommand(double linear_vel, double angular_vel,
                                 double lateral_velocity,
                                 double steering_angle) {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
  };

  virtual void SendLightCommand(AgxLightMode front_mode,
                                uint8_t front_custom_value, AgxLightMode rear_mode,
                                uint8_t rear_custom_value) {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
  };
};

struct PercyCoreStateMsgGroup {
  AgxMsgTimeStamp time_stamp;

  PercySystemStateMessage system_state;
  PercyMotionStateMessage motion_state;
  PercyLightStateMessage light_state;
  //MotionModeStateMessage motion_mode_state;
  RcStateMessage rc_state;
};

struct PercyActuatorStateMsgGroup {
  AgxMsgTimeStamp time_stamp;

  ActuatorHSStateMessage actuator_hs_state[2];
  ActuatorLSStateMessage actuator_ls_state[2];
};

struct PercySensorStateMsgGroup {
  AgxMsgTimeStamp time_stamp;

  OdometryMessage odom_state;
  BmsBasicMessage bms_states;
  PowerBUttonEventFb button_state;
  MechanicalCaliMessage mechanical_state;
};

class PercyRobotCommonInterface {
 public:
  virtual ~PercyRobotCommonInterface() = default;

  // functions to be implemented by class AgilexBase
  virtual void EnableCommandedMode() = 0;

  // functions to be implemented by each robot class
  virtual bool Connect(std::string can_name) = 0;

  virtual void ResetRobotState() = 0;

  virtual void DisableLightControl() {
    // do nothing if no light on robot
  }

  virtual ProtocolVersion GetParserProtocolVersion() = 0;

 protected:
  // firmware update function
  virtual void FlashFirmwareStart(uint32_t flash_space){};
  virtual void FlashFirmwareFrame(uint8_t Flashdata){};

  // functions to be implemented by class AgilexBase
  virtual void SetMotionMode(uint8_t mode){};
  virtual void SetBrakedMode(AgxBrakeMode mode){};

  virtual PercyCoreStateMsgGroup GetRobotCoreStateMsgGroup() {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
    return PercyCoreStateMsgGroup{};
  };
  virtual PercyActuatorStateMsgGroup GetActuatorStateMsgGroup() {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
    return PercyActuatorStateMsgGroup{};
  };

  virtual PercySensorStateMsgGroup GetSensorStateMsgGroup() {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
    return PercySensorStateMsgGroup{};
  };
  // any specific robot will use a specialized version of the two functions
  virtual void SendMotionCommand(double linear_vel, double angular_vel,
                                 double lateral_velocity,
                                 double steering_angle) {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
  };

  virtual void SendLightCommand(PercyLightCmdMessage &light_msg) {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
  };

  virtual void SendLightRGB(RGB_val light_status[4]) {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
  };

};

}  // namespace agilexrobotics

#endif /* ROBOT_INTERFACE_HPP */
