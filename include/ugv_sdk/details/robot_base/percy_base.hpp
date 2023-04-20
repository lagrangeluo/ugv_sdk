/*
 * agilex_base.hpp
 *
 * Created on: Dec 22, 2020 17:14
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef AGILEX_PERCY_BASE_HPP
#define AGILEX_PERCY_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>

#include "ugv_sdk/details/async_port/async_can.hpp"
#include "ugv_sdk/details/interface/robot_common_interface.hpp"
#include "ugv_sdk/details/interface/parser_interface.hpp"

#include "ugv_sdk/details/protocol_percy/protocol_PERCY_parser.hpp"

namespace agilexrobotics {
template <typename ParserType>
class PercyBase : public PercyRobotCommonInterface {
 public:
  PercyBase() {
    static_assert(
            std::is_base_of<ParserInterface<ProtocolVersion::PERCY>,
                            ParserType>::value,

        "Incompatible parser for the PercyBase class, expecting one derived "
        "from ParserInterface!");
  };
  virtual ~PercyBase() { DisconnectPort(); }

  // do not allow copy or assignment
  PercyBase(const PercyBase &hunter) = delete;
  PercyBase &operator=(const PercyBase &hunter) = delete;

  bool Connect(std::string can_name) override {
    return ConnectPort(can_name,
                       std::bind(&PercyBase<ParserType>::ParseCANFrame, this,
                                 std::placeholders::_1));
  }

  // switch to commanded mode
  void EnableCommandedMode() {
    // construct message
    Agx_Percy_Message msg;
    msg.type = AgxMsgPercyControlEnable;
    msg.body.can_control_msg.control_mode = CONTROL_MODE_CAN;

    // encode msg to can frame and send to bus
    if (can_ != nullptr && can_->IsOpened()) {
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  // must be called at a frequency >= 50Hz
  void SendMotionCommand(double linear_vel, double angular_vel) {
    if (can_ != nullptr && can_->IsOpened()) {
      // motion control message
      Agx_Percy_Message msg;
      if (parser_.GetParserProtocolVersion() == ProtocolVersion::PERCY) {
        msg.type = AgxPercyMsgMotionCommand;
        msg.body.motion_command_msg.linear_velocity = linear_vel;
        msg.body.motion_command_msg.angular_velocity = angular_vel;
      } 
      // send to can bus
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  // light command
  void SendLightCommand(PercyLightCmdMessage &light_msg) {
    if (can_ != nullptr && can_->IsOpened()) {
      Agx_Percy_Message msg;
      msg.type = AgxPercyMsgLightCommand;

      msg.body.light_command_msg.control_enable = light_msg.control_enable;
      msg.body.light_command_msg.front_illumination_mode = light_msg.front_illumination_mode;
      msg.body.light_command_msg.back_illumination_mode = light_msg.back_illumination_mode;
      // send to can bus
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }
  // light RGB value
  void SendLightRGB(RGB_val light_status[4])
  {
    if (can_ != nullptr && can_->IsOpened()) {
      Agx_Percy_Message msg;
      msg.type = AgxPercyMsgFrontRGBCommand;
      msg.body.front_light_RGB_msgs.LEFT_LIGHT[0] = light_status[0][0];
      msg.body.front_light_RGB_msgs.LEFT_LIGHT[1] = light_status[0][1];
      msg.body.front_light_RGB_msgs.LEFT_LIGHT[2] = light_status[0][2];

      msg.body.front_light_RGB_msgs.RIGHT_LIGHT[0] = light_status[1][0];
      msg.body.front_light_RGB_msgs.RIGHT_LIGHT[1] = light_status[1][1];
      msg.body.front_light_RGB_msgs.RIGHT_LIGHT[2] = light_status[1][2];
      // send to can bus
      can_frame frame_1;
      if (parser_.EncodeMessage(&msg, &frame_1)) can_->SendFrame(frame_1);

      msg.type = AgxPercyMsgBackRGBCommand;
      msg.body.back_light_RGB_msgs.LEFT_LIGHT[0] = light_status[2][0];
      msg.body.back_light_RGB_msgs.LEFT_LIGHT[1] = light_status[2][1];
      msg.body.back_light_RGB_msgs.LEFT_LIGHT[2] = light_status[2][2];

      msg.body.back_light_RGB_msgs.RIGHT_LIGHT[0] = light_status[3][0];
      msg.body.back_light_RGB_msgs.RIGHT_LIGHT[1] = light_status[3][1];
      msg.body.back_light_RGB_msgs.RIGHT_LIGHT[2] = light_status[3][2];
      // send to can bus
      can_frame frame_2;
      if (parser_.EncodeMessage(&msg, &frame_2)) can_->SendFrame(frame_2);
    }
  }

  void ResetRobotState() override {}

  ProtocolVersion GetParserProtocolVersion() override {
    return parser_.GetParserProtocolVersion();
  }

  PercyCoreStateMsgGroup GetRobotCoreStateMsgGroup() override {
    std::lock_guard<std::mutex> guard(core_state_mtx_);
    return core_state_msgs_;
  }

  PercyActuatorStateMsgGroup GetActuatorStateMsgGroup() override {
    std::lock_guard<std::mutex> guard(actuator_state_mtx_);
    return actuator_state_msgs_;
  }

  PercySensorStateMsgGroup GetSensorStateMsgGroup() override {
    std::lock_guard<std::mutex> guard(sensor_state_mtx_);
    return sensor_state_msgs_;
  }

 protected:
  ParserType parser_;

  // divide feedback messages into smaller groups to avoid the
  // state mutex being locked for too often such that accessing
  // the data become difficult

  /* feedback group 1: core state */
  std::mutex core_state_mtx_;
  PercyCoreStateMsgGroup core_state_msgs_;

  /* feedback group 2: actuator state */
  std::mutex actuator_state_mtx_;
  PercyActuatorStateMsgGroup actuator_state_msgs_;

  /* feedback group 3: common sensor */
  std::mutex sensor_state_mtx_;
  PercySensorStateMsgGroup sensor_state_msgs_;

  // communication interface
  std::shared_ptr<AsyncCAN> can_;

  // connect to roboot from CAN or serial
  using CANFrameRxCallback = AsyncCAN::ReceiveCallback;
  bool ConnectPort(std::string dev_name, CANFrameRxCallback cb) {
    can_ = std::make_shared<AsyncCAN>(dev_name);
    can_->SetReceiveCallback(cb);
    return can_->Open();
  }

  void DisconnectPort() {
    if (can_ != nullptr && can_->IsOpened()) can_->Close();
  }

  void SetBrakeMode(AgxPercyBrakeMode mode) {
    // construct message
    Agx_Percy_Message msg;
    msg.type = AgxMsgBrakeModeConfig;
    msg.body.braking_command_msg.enable_braking = mode;

    // encode msg to can frame and send to bus
    if (can_ != nullptr && can_->IsOpened()) {
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  void ParseCANFrame(can_frame *rx_frame) {
    Agx_Percy_Message status_msg;
    if (parser_.DecodeMessage(rx_frame, &status_msg)) {
      UpdateRobotCoreState(status_msg);
      UpdateActuatorState(status_msg);
      UpdateSensorState(status_msg);
    }
  }

  void UpdateRobotCoreState(const Agx_Percy_Message &status_msg) {
    std::lock_guard<std::mutex> guard(core_state_mtx_);
    switch (status_msg.type) {
      case AgxMsgPercySystemState: {
        //   std::cout << "system status feedback received" << std::endl;
        core_state_msgs_.time_stamp = AgxMsgRefClock::now();
        core_state_msgs_.system_state = status_msg.body.system_state_msg;
        break;
      }
      case AgxMsgPercyMotionState: {
        // std::cout << "motion control feedback received" << std::endl;
        core_state_msgs_.time_stamp = AgxMsgRefClock::now();
        core_state_msgs_.motion_state = status_msg.body.motion_state_msg;
        break;
      }
      case AgxMsgPercyLightState: {
        // std::cout << "light control feedback received" << std::endl;
        core_state_msgs_.time_stamp = AgxMsgRefClock::now();
        core_state_msgs_.light_state = status_msg.body.light_state_msg;
        break;
      }
      case AgxMsgPercyLightFrontRGBState:{
        //TODO
      }
      case AgxMsgPercyLightBackRGBState:{
        //TODO
      }

      case AgxMsgPercyRcState: {
        // std::cout << "rc feedback received" << std::endl;
        core_state_msgs_.time_stamp = AgxMsgRefClock::now();
        core_state_msgs_.rc_state = status_msg.body.rc_state_msg;
        break;
      }
      default:
        break;
    }
  }

  void UpdateActuatorState(const Agx_Percy_Message &status_msg) {
    std::lock_guard<std::mutex> guard(actuator_state_mtx_);
    switch (status_msg.type) {
      case AgxMsgPercyActuatorHSState: {
        // std::cout << "actuator hs feedback received" << std::endl;
        actuator_state_msgs_.time_stamp = AgxMsgRefClock::now();
        actuator_state_msgs_
            .actuator_hs_state[status_msg.body.actuator_hs_state_msg.motor_id] =
            status_msg.body.actuator_hs_state_msg;
        break;
      }
      case AgxMsgPercyActuatorLSState: {
        // std::cout << "actuator ls feedback received" << std::endl;
        actuator_state_msgs_.time_stamp = AgxMsgRefClock::now();
        actuator_state_msgs_
            .actuator_ls_state[status_msg.body.actuator_ls_state_msg.motor_id] =
            status_msg.body.actuator_ls_state_msg;
        break;
      }
      default:
        break;
    }
  }
  
  void UpdateSensorState(const Agx_Percy_Message &status_msg) {
    std::lock_guard<std::mutex> guard(sensor_state_mtx_);
    switch (status_msg.type) {
      case AgxMsgPercyOdometry: {
        sensor_state_msgs_.time_stamp = AgxMsgRefClock::now();
        sensor_state_msgs_.odom_state = status_msg.body.odometry_msg;
        break;
      }
      case AgxMsgPercyBmsBasic: {
        sensor_state_msgs_.time_stamp = AgxMsgRefClock::now();
        sensor_state_msgs_.bms_states = status_msg.body.bms_basic_msg;
        break;
      }
      case AgxMsgPercyPowerButton: {
        sensor_state_msgs_.time_stamp = AgxMsgRefClock::now();
        sensor_state_msgs_.button_state = status_msg.body.PowerButtonEvfb_msg;
        break;
      }
      default:
        break;    
  }
  }

};

  using PercyRobot = PercyBase<Protocol_PERCY_Parser>;

}  // namespace agilexrobotics

#endif /* AGILEX_BASE_HPP */