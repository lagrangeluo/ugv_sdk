/*
 * agx_msg_parser.c
 *
 * Created on: Mar 24, 2023 18:00
 * Description:
 *
 * Copyright (c) 2023 AgileX Robotics Pte. Ltd.
 */

#include "agilex_protocol_percy.h"
#include "protocol_percy/agilex_msg_parser_percy.h"

#include "stdio.h"
#include "string.h"

bool DecodeCanFramePERCY(const struct can_frame *rx_frame, Agx_Percy_Message *msg) {
  msg->type = AgxMsgUnkonwn;

  switch (rx_frame->can_id) {
    /************* firmware update frame *************/
    case CAN_MSG_UPGRADE_START_ID: {
      //TODO
      msg->type = Percy_UpgradeStartCommand;
      // parse frame buffer to message
      M8_byte_Frame *frame = (M8_byte_Frame *)(rx_frame->data);
      msg->body.updatestart_command_msg.flash_size =
          (uint8_t)(frame->byte_0);
      break;
    }
    case CAN_MSG_UPGRADE_INSTRUCTIONS_0_ID:
    case CAN_MSG_UPGRADE_INSTRUCTIONS_F_ID: {
      //TODO
      msg->type = Percy_UpgradeProcessCommand;
      // parse frame buffer to message
      M8_byte_Frame *frame = (M8_byte_Frame *)(rx_frame->data);
      msg->body.upgrade_process_command_msg.flash_data[0] =
          (uint8_t)(frame->byte_0);
      break;
    }

    /***************** command frame *****************/
    case CAN_MSG_MOTION_COMMAND_ID: {
      msg->type = AgxPercyMsgMotionCommand;
      // parse frame buffer to message
      M8_byte_Frame *frame = (M8_byte_Frame *)(rx_frame->data);
      msg->body.motion_command_msg.linear_velocity =
          (int16_t)((uint16_t)(frame->byte_1) |
                    (uint16_t)(frame->byte_0) << 8) /
          1000.0;
      msg->body.motion_command_msg.angular_velocity =
          (int16_t)((uint16_t)(frame->byte_3) |
                    (uint16_t)(frame->byte_2) << 8) /
          1000.0;
      break;
    }
    case CAN_MSG_LIGHT_MODE_COMMAND_ID: {
      msg->type = AgxPercyMsgLightCommand;
      // parse frame buffer to message
      M8_byte_Frame *frame = (M8_byte_Frame *)(rx_frame->data);
      msg->body.light_command_msg.control_enable =
          (frame->byte_0 != 0) ? true : false;
      msg->body.light_command_msg.front_illumination_mode = frame->byte_1;
      msg->body.light_command_msg.back_illumination_mode = frame->byte_3;
      break;
    }
    case CAN_MSG_BRAKING_COMMAND_RES_ID: {
      msg->type = AgxPercyMsgLightCommand;
      // parse frame buffer to message
      M8_byte_Frame *frame = (M8_byte_Frame *)(rx_frame->data);
      msg->body.light_command_msg.control_enable =
          (frame->byte_0 != 0) ? true : false;
      msg->body.light_command_msg.front_illumination_mode = frame->byte_1;
      msg->body.light_command_msg.back_illumination_mode = frame->byte_3;
      break;
    }

    case CAN_MSG_BRAKING_COMMAND_ID: {
      msg->type = AgxMsgBrakingCommand;
      // TODO
      break;
    }
    /***************** feedback frame ****************/
    case CAN_MSG_SYSTEM_STATE_ID: {
      msg->type = AgxMsgPercySystemState;
      M8_byte_Frame *frame = (M8_byte_Frame *)(rx_frame->data);
      msg->body.system_state_msg.vehicle_state = frame->byte_0;
      msg->body.system_state_msg.control_mode = frame->byte_1;
      msg->body.system_state_msg.revol_per[0] = frame->byte_2;
      msg->body.system_state_msg.revol_per[1] = frame->byte_3;
      msg->body.system_state_msg.revol_per[2] = frame->byte_4;
      msg->body.system_state_msg.revol_per[3] = frame->byte_5;

      msg->body.system_state_msg.error_code =
          (uint16_t)(frame->byte_7) |
          (uint16_t)(frame->byte_6) << 8;
      break;
    }
    case CAN_MSG_MOTION_STATE_ID: {
      msg->type = AgxMsgPercyMotionState;
      M8_byte_Frame *frame = (M8_byte_Frame *)(rx_frame->data);
      msg->body.motion_state_msg.linear_velocity =
          (int16_t)((uint16_t)(frame->byte_1) |
                    (uint16_t)(frame->byte_0) << 8) /
          1000.0;
      msg->body.motion_state_msg.angular_velocity =
          (int16_t)((uint16_t)(frame->byte_3) |
                    (uint16_t)(frame->byte_2) << 8) /
          1000.0;
      break;
    }
    case CAN_MSG_LIGHT_STATE_ID: {
      msg->type = AgxMsgPercyLightState;
      M8_byte_Frame *frame = (M8_byte_Frame *)(rx_frame->data);
      msg->body.light_command_msg.control_enable =
          (frame->byte_0 != 0) ? true : false;
      msg->body.light_command_msg.front_illumination_mode = frame->byte_1;
      msg->body.light_command_msg.back_illumination_mode = frame->byte_3;
      break;
    }
    case CAN_MSG_LIGHT_FRONT_RGB_ID: {
      msg->type = AgxMsgPercyLightFrontRGBState;
      M8_byte_Frame *frame = (M8_byte_Frame *)(rx_frame->data);
      msg->body.front_light_RGB_msgs.LEFT_LIGHT[0] = frame->byte_1;
      msg->body.front_light_RGB_msgs.LEFT_LIGHT[1] = frame->byte_2;
      msg->body.front_light_RGB_msgs.LEFT_LIGHT[2] = frame->byte_3;
      msg->body.front_light_RGB_msgs.RIGHT_LIGHT[0] = frame->byte_4;
      msg->body.front_light_RGB_msgs.RIGHT_LIGHT[1] = frame->byte_5;
      msg->body.front_light_RGB_msgs.RIGHT_LIGHT[2] = frame->byte_6;
      break;
    }
    case CAN_MSG_LIGHT_BACK_RGB_ID: {
      msg->type = AgxMsgPercyLightBackRGBState;
      M8_byte_Frame *frame = (M8_byte_Frame *)(rx_frame->data);
      msg->body.front_light_RGB_msgs.LEFT_LIGHT[0] = frame->byte_1;
      msg->body.front_light_RGB_msgs.LEFT_LIGHT[1] = frame->byte_2;
      msg->body.front_light_RGB_msgs.LEFT_LIGHT[2] = frame->byte_3;
      msg->body.front_light_RGB_msgs.RIGHT_LIGHT[0] = frame->byte_4;
      msg->body.front_light_RGB_msgs.RIGHT_LIGHT[1] = frame->byte_5;
      msg->body.front_light_RGB_msgs.RIGHT_LIGHT[2] = frame->byte_6;
      break;
    }

    case CAN_MSG_RC_STATE_ID: {
      msg->type = AgxMsgPercyRcState;
      RcStateFrame *frame = (RcStateFrame *)(rx_frame->data);
      // switch a
      if ((frame->sws & RC_SWA_MASK) == RC_SWA_UP_MASK)
        msg->body.rc_state_msg.swa = RC_SWITCH_UP;
      else if ((frame->sws & RC_SWA_MASK) == RC_SWA_DOWN_MASK)
        msg->body.rc_state_msg.swa = RC_SWITCH_DOWN;
      // switch b
      if ((frame->sws & RC_SWB_MASK) == RC_SWB_UP_MASK)
        msg->body.rc_state_msg.swb = RC_SWITCH_UP;
      else if ((frame->sws & RC_SWB_MASK) == RC_SWB_MIDDLE_MASK)
        msg->body.rc_state_msg.swb = RC_SWITCH_MIDDLE;
      else if ((frame->sws & RC_SWB_MASK) == RC_SWB_DOWN_MASK)
        msg->body.rc_state_msg.swb = RC_SWITCH_DOWN;
      // switch c
      if ((frame->sws & RC_SWC_MASK) == RC_SWC_UP_MASK)
        msg->body.rc_state_msg.swc = RC_SWITCH_UP;
      else if ((frame->sws & RC_SWC_MASK) == RC_SWC_MIDDLE_MASK)
        msg->body.rc_state_msg.swc = RC_SWITCH_MIDDLE;
      else if ((frame->sws & RC_SWC_MASK) == RC_SWC_DOWN_MASK)
        msg->body.rc_state_msg.swc = RC_SWITCH_DOWN;
      // switch d
      if ((frame->sws & RC_SWD_MASK) == RC_SWD_UP_MASK)
        msg->body.rc_state_msg.swd = RC_SWITCH_UP;
      else if ((frame->sws & RC_SWD_MASK) == RC_SWD_DOWN_MASK)
        msg->body.rc_state_msg.swd = RC_SWITCH_DOWN;
      msg->body.rc_state_msg.stick_right_v = frame->stick_right_v;
      msg->body.rc_state_msg.stick_right_h = frame->stick_right_h;
      msg->body.rc_state_msg.stick_left_v = frame->stick_left_v;
      msg->body.rc_state_msg.stick_left_h = frame->stick_left_h;
      msg->body.rc_state_msg.var_a = frame->var_a;
      break;
    }
    case CAN_MSG_ACTUATOR1_HS_STATE_ID:
    case CAN_MSG_ACTUATOR2_HS_STATE_ID: {
      msg->type = AgxMsgPercyActuatorHSState;
      ActuatorHSStateFrame *frame = (ActuatorHSStateFrame *)(rx_frame->data);
      msg->body.actuator_hs_state_msg.motor_id =
          rx_frame->can_id - CAN_MSG_ACTUATOR1_HS_STATE_ID;
      msg->body.actuator_hs_state_msg.rpm =
          (int16_t)((uint16_t)(frame->rpm.low_byte) |
                    (uint16_t)(frame->rpm.high_byte) << 8);
      msg->body.actuator_hs_state_msg.current =
          ((uint16_t)(frame->current.low_byte) |
           (uint16_t)(frame->current.high_byte) << 8) *
          0.1;
      msg->body.actuator_hs_state_msg.pulse_count =
          (int32_t)((uint32_t)(frame->pulse_count.lsb) |
                    (uint32_t)(frame->pulse_count.low_byte) << 8 |
                    (uint32_t)(frame->pulse_count.high_byte) << 16 |
                    (uint32_t)(frame->pulse_count.msb) << 24);
      break;
    }
    case CAN_MSG_ACTUATOR1_LS_STATE_ID:
    case CAN_MSG_ACTUATOR2_LS_STATE_ID:{
      msg->type = AgxMsgPercyActuatorLSState;
      M8_byte_Frame *frame = (M8_byte_Frame *)(rx_frame->data);
      msg->body.actuator_hs_state_msg.motor_id =
          rx_frame->can_id - CAN_MSG_ACTUATOR1_LS_STATE_ID;
      msg->body.actuator_ls_state_msg.driver_voltage =
          ((uint16_t)(frame->byte_1) |
           (uint16_t)(frame->byte_0) << 8) *
          0.1;
      msg->body.actuator_ls_state_msg.driver_state = frame->byte_6;
      break;
    }
    /****************** sensor frame *****************/
    case CAN_MSG_ODOMETRY_ID: {
      msg->type = AgxMsgPercyOdometry;
      OdometryFrame *frame = (OdometryFrame *)(rx_frame->data);
      msg->body.odometry_msg.left_wheel =
          (int32_t)((uint32_t)(frame->left_wheel.lsb) |
                    (uint32_t)(frame->left_wheel.low_byte) << 8 |
                    (uint32_t)(frame->left_wheel.high_byte) << 16 |
                    (uint32_t)(frame->left_wheel.msb) << 24);
      msg->body.odometry_msg.right_wheel =
          (int32_t)((uint32_t)(frame->right_wheel.lsb) |
                    (uint32_t)(frame->right_wheel.low_byte) << 8 |
                    (uint32_t)(frame->right_wheel.high_byte) << 16 |
                    (uint32_t)(frame->right_wheel.msb) << 24);
      break;
    }
    case CAN_MSG_BMS_BASIC_ID: {
      msg->type = AgxMsgPercyBmsBasic;
      BmsBasicFrame *frame = (BmsBasicFrame *)(rx_frame->data);
      msg->body.bms_basic_msg.battery_soc = frame->battery_soc;
      msg->body.bms_basic_msg.battery_soh = frame->battery_soh;
      msg->body.bms_basic_msg.voltage =
          ((uint16_t)(frame->voltage.low_byte) |
           (uint16_t)(frame->voltage.high_byte) << 8) /
          10.0f;
      msg->body.bms_basic_msg.current =
          (int16_t)((uint16_t)(frame->current.low_byte) |
                    (uint16_t)(frame->current.high_byte) << 8) /
          10.0f;
      msg->body.bms_basic_msg.temperature =
          (int16_t)(((uint16_t)(frame->temperature.low_byte) |
                     (uint16_t)(frame->temperature.high_byte) << 8)) /
          10.0f;
      break;
    }
    case CAN_MSG_POWER_BUTTON_EVENT_ID: {
      msg->type = AgxMsgPercyPowerButton;
      M8_byte_Frame *frame = (M8_byte_Frame *)(rx_frame->data);
      msg->body.PowerButtonEvfb_msg.power_button_event = frame->byte_0;
      break;
    }

    /*************** query/config frame **************/
    //TODO
    default:
      break;
  }

  return true;
}

bool EncodeCanFramePERCY(const Agx_Percy_Message *msg, struct can_frame *tx_frame) {
  bool ret = true;
  switch (msg->type) {
    /***************** Firmware Update frame *****************/
    case Percy_UpgradeStartCommand: {
      tx_frame->can_id = CAN_MSG_UPGRADE_START_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
      frame.byte_0 = (uint8_t)(msg->body.updatestart_command_msg.flash_size >> 24);
      frame.byte_1 = (uint8_t)((msg->body.updatestart_command_msg.flash_size >> 16) & 0x00ff);
      frame.byte_2 = (uint8_t)((msg->body.updatestart_command_msg.flash_size >> 8) & 0x00ff);
      frame.byte_3 = (uint8_t)(msg->body.updatestart_command_msg.flash_size & 0x00ff);
      frame.byte_4 = 0;
      frame.byte_5 = 0;
      frame.byte_6 = 0;
      frame.byte_7 = 0;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case Percy_UpgradeProcessCommand: {
      tx_frame->can_id = CAN_MSG_UPGRADE_INSTRUCTIONS_0_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
      frame.byte_0 = (uint8_t)(msg->body.updatestart_command_msg.flash_size >> 24);
      frame.byte_1 = (uint8_t)((msg->body.updatestart_command_msg.flash_size >> 16) & 0x00ff);
      frame.byte_2 = (uint8_t)((msg->body.updatestart_command_msg.flash_size >> 8) & 0x00ff);
      frame.byte_3 = (uint8_t)(msg->body.updatestart_command_msg.flash_size & 0x00ff);
      frame.byte_4 = 0;
      frame.byte_5 = 0;
      frame.byte_6 = 0;
      frame.byte_7 = 0;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }

    /***************** command frame *****************/
    case AgxPercyMsgMotionCommand: {
      tx_frame->can_id = CAN_MSG_MOTION_COMMAND_ID;
      tx_frame->can_dlc = 8;
      MotionCommandFrame frame;
      int16_t linear_cmd =
          (int16_t)(msg->body.motion_command_msg.linear_velocity * 1000);
      int16_t angular_cmd =
          (int16_t)(msg->body.motion_command_msg.angular_velocity * 1000);
      frame.linear_velocity.high_byte = (uint8_t)(linear_cmd >> 8);
      frame.linear_velocity.low_byte = (uint8_t)(linear_cmd & 0x00ff);
      frame.angular_velocity.high_byte = (uint8_t)(angular_cmd >> 8);
      frame.angular_velocity.low_byte = (uint8_t)(angular_cmd & 0x00ff);
      frame.reserved.high_byte = 0;
      frame.reserved.low_byte = 0;
      frame.reserved.msb = 0;
      frame.reserved.lsb = 0;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxPercyMsgLightCommand: {
      static uint8_t count = 0;
      tx_frame->can_id = CAN_MSG_LIGHT_MODE_COMMAND_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = msg->body.light_command_msg.control_enable;
        frame.byte_1 = msg->body.light_command_msg.front_illumination_mode;
        frame.byte_2 = 0;
        frame.byte_3 = msg->body.light_command_msg.back_illumination_mode;
        frame.byte_4 = 0;
        frame.byte_5 = 0;
        frame.byte_6 = 0;
        frame.byte_7 = 0;
      
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxPercyMsgFrontRGBCommand: {
      static uint8_t count = 0;
      tx_frame->can_id = CAN_MSG_LIGHT_FRONTRGB_COMMAND_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = 0;
        frame.byte_1 = msg->body.front_light_RGB_msgs.LEFT_LIGHT[0];
        frame.byte_2 = msg->body.front_light_RGB_msgs.LEFT_LIGHT[1];
        frame.byte_3 = msg->body.front_light_RGB_msgs.LEFT_LIGHT[2];
        frame.byte_4 = msg->body.front_light_RGB_msgs.RIGHT_LIGHT[0];
        frame.byte_5 = msg->body.front_light_RGB_msgs.RIGHT_LIGHT[1];
        frame.byte_6 = msg->body.front_light_RGB_msgs.RIGHT_LIGHT[2];
        frame.byte_7 = 0;
      
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxPercyMsgBackRGBCommand: {
      static uint8_t count = 0;
      tx_frame->can_id = CAN_MSG_LIGHT_BACKRGB_COMMAND_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = 0;
        frame.byte_1 = msg->body.back_light_RGB_msgs.LEFT_LIGHT[0];
        frame.byte_2 = msg->body.back_light_RGB_msgs.LEFT_LIGHT[1];
        frame.byte_3 = msg->body.back_light_RGB_msgs.LEFT_LIGHT[2];
        frame.byte_4 = msg->body.back_light_RGB_msgs.RIGHT_LIGHT[0];
        frame.byte_5 = msg->body.back_light_RGB_msgs.RIGHT_LIGHT[1];
        frame.byte_6 = msg->body.back_light_RGB_msgs.RIGHT_LIGHT[2];
        frame.byte_7 = 0;
      
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }

    case AgxMsgPercyBrakingCommand: {
      tx_frame->can_id = CAN_MSG_BRAKING_COMMAND_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = msg->body.braking_command_msg.enable_braking;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyPowerRailCommand: {
      tx_frame->can_id = CAN_MSG_POWER_RAIL_CONTROL_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = (uint8_t)(msg->body.pwr_rail_ctl_cmd_msg.power_status >> 8);
        frame.byte_1 = (uint8_t)(msg->body.pwr_rail_ctl_cmd_msg.power_status & 0x00ff);

      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyJetsonPoweroffCommand: {
      tx_frame->can_id = CAN_MSG_JETSON_POWER_OFF_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = msg->body.jetson_pwr_off_cmd_msg.power_statue;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyAMRShutDownCommand: {
      tx_frame->can_id = CAN_MSG_AMR_SHUT_DOWN_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = msg->body.amr_shutdown_cmd_msg.power_statu[0];
        frame.byte_1 = msg->body.amr_shutdown_cmd_msg.power_statu[1];
        frame.byte_2 = msg->body.amr_shutdown_cmd_msg.power_statu[2];
        frame.byte_3 = msg->body.amr_shutdown_cmd_msg.power_statu[3];
        frame.byte_4 = msg->body.amr_shutdown_cmd_msg.power_statu[4];
        frame.byte_5 = msg->body.amr_shutdown_cmd_msg.power_statu[5];
        frame.byte_6 = msg->body.amr_shutdown_cmd_msg.power_statu[6];
        frame.byte_7 = msg->body.amr_shutdown_cmd_msg.power_statu[7];

      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }

    /***************** feedback frame ****************/
    /*case AgxMsgPercySystemState: {
      tx_frame->can_id = CAN_MSG_SYSTEM_STATE_ID;
      tx_frame->can_dlc = 8;
      SystemStateFrame frame;
      frame.vehicle_state = msg->body.system_state_msg.vehicle_state;
      frame.control_mode = msg->body.system_state_msg.control_mode;
      uint16_t battery =
          (uint16_t)(msg->body.system_state_msg.battery_voltage * 10);
      frame.battery_voltage.high_byte = (uint8_t)(battery >> 8);
      frame.battery_voltage.low_byte = (uint8_t)(battery & 0x00ff);
      frame.error_code.high_byte =
          (uint8_t)(msg->body.system_state_msg.error_code >> 8);
      frame.error_code.low_byte =
          (uint8_t)(msg->body.system_state_msg.error_code & 0x00ff);
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyMotionState: {
      tx_frame->can_id = CAN_MSG_MOTION_STATE_ID;
      tx_frame->can_dlc = 8;
      MotionStateFrame frame;
      int16_t linear =
          (int16_t)(msg->body.motion_state_msg.linear_velocity * 1000);
      int16_t angular =
          (int16_t)(msg->body.motion_state_msg.angular_velocity * 1000);
      int16_t lateral =
          (int16_t)(msg->body.motion_state_msg.lateral_velocity * 1000);
      int16_t steering =
          (int16_t)(msg->body.motion_state_msg.steering_angle * 1000);
      frame.linear_velocity.high_byte = (uint8_t)(linear >> 8);
      frame.linear_velocity.low_byte = (uint8_t)(linear & 0x00ff);
      frame.angular_velocity.high_byte = (uint8_t)(angular >> 8);
      frame.angular_velocity.low_byte = (uint8_t)(angular & 0x00ff);
      frame.lateral_velocity.high_byte = (uint8_t)(lateral >> 8);
      frame.lateral_velocity.low_byte = (uint8_t)(lateral & 0x00ff);
      frame.steering_angle.high_byte = (uint8_t)(steering >> 8);
      frame.steering_angle.low_byte = (uint8_t)(steering & 0x00ff);
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyLightState: {
      tx_frame->can_id = CAN_MSG_LIGHT_STATE_ID;
      tx_frame->can_dlc = 8;
      LightStateFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgRcState: {
      tx_frame->can_id = CAN_MSG_RC_STATE_ID;
      tx_frame->can_dlc = 8;
      RcStateFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgActuatorHSState: {
      tx_frame->can_id = msg->body.actuator_hs_state_msg.motor_id +
                         CAN_MSG_ACTUATOR1_HS_STATE_ID;
      tx_frame->can_dlc = 8;
      ActuatorHSStateFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgActuatorLSState: {
      tx_frame->can_id = msg->body.actuator_ls_state_msg.motor_id +
                         CAN_MSG_ACTUATOR1_LS_STATE_ID;
      tx_frame->can_dlc = 8;
      ActuatorLSStateFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }*/
    /****************** sensor frame *****************/
    /*case AgxMsgOdometry: {
      tx_frame->can_id = CAN_MSG_ODOMETRY_ID;
      tx_frame->can_dlc = 8;
      OdometryFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgImuAccel: {
      tx_frame->can_id = CAN_MSG_IMU_ACCEL_ID;
      tx_frame->can_dlc = 8;
      ImuAccelFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgImuGyro: {
      tx_frame->can_id = CAN_MSG_IMU_GYRO_ID;
      tx_frame->can_dlc = 8;
      ImuGyroFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgImuEuler: {
      tx_frame->can_id = CAN_MSG_IMU_EULER_ID;
      tx_frame->can_dlc = 8;
      ImuEulerFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgSafetyBumper: {
      tx_frame->can_id = CAN_MSG_SAFETY_BUMPER_ID;
      tx_frame->can_dlc = 8;
      SafetyBumperFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgUltrasonic: {
      tx_frame->can_id =
          msg->body.ultrasonic_msg.sensor_id + CAN_MSG_ULTRASONIC_1_ID;
      tx_frame->can_dlc = 8;
      UltrasonicFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgUwb: {
      tx_frame->can_id = msg->body.uwb_msg.sensor_id + CAN_MSG_UWB_1_ID;
      tx_frame->can_dlc = 8;
      UwbFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgBmsBasic: {
      tx_frame->can_id = CAN_MSG_BMS_BASIC_ID;
      tx_frame->can_dlc = 8;
      BmsBasicFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgBmsExtended: {
      tx_frame->can_id = CAN_MSG_BMS_EXTENDED_ID;
      tx_frame->can_dlc = 8;
      BmsExtendedFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }*/
    /*************** query/config frame **************/
    case AgxMsgPercyFactory: {
      tx_frame->can_id = CAN_MSG_RESTORY_FACTORY_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = msg->body.restore_fac_set_msg.fixedcontent[0];
        frame.byte_1 = msg->body.restore_fac_set_msg.fixedcontent[1];
        frame.byte_2 = msg->body.restore_fac_set_msg.fixedcontent[2];
        frame.byte_3 = msg->body.restore_fac_set_msg.fixedcontent[3];
        frame.byte_4 = msg->body.restore_fac_set_msg.fixedcontent[4];
        frame.byte_5 = msg->body.restore_fac_set_msg.fixedcontent[5];
        frame.byte_6 = msg->body.restore_fac_set_msg.fixedcontent[6];
        frame.byte_7 = msg->body.restore_fac_set_msg.fixedcontent[7];

      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyControlEnable: {
      tx_frame->can_id = CAN_MSG_CAN_CONTROL_ENABLE_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = msg->body.can_control_msg.control_mode;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyClearErrorStates: {
      tx_frame->can_id = CAN_MSG_CLEAR_ERROR_STATES_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = msg->body.clear_error_msg.control_mode;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyAccDecConfig: {
      tx_frame->can_id = CAN_MSG_ACC_DEC_CONFIG_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = (uint8_t)(msg->body.acc_dec_conf_msg.max_acc >> 8);
        frame.byte_1 = (uint8_t)(msg->body.acc_dec_conf_msg.max_acc & 0x00ff);
        frame.byte_2 = (uint8_t)(msg->body.acc_dec_conf_msg.max_dec >> 8);
        frame.byte_3 = (uint8_t)(msg->body.acc_dec_conf_msg.max_dec & 0x00ff);

      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyPowerRailBoot: {
      tx_frame->can_id = CAN_MSG_ACC_DEC_CONFIG_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = msg->body.power_rail_boot_msg.T_PWR_DELAY_1;
        frame.byte_1 = msg->body.power_rail_boot_msg.T_PWR_DELAY_2;
        frame.byte_2 = msg->body.power_rail_boot_msg.T_ON_TIMEOUT;
        frame.byte_3 = msg->body.power_rail_boot_msg.T_OFF_TIMEOUT;
        frame.byte_4 = msg->body.power_rail_boot_msg.T_ON;
        frame.byte_5 = msg->body.power_rail_boot_msg.T_OFF;
        frame.byte_6 = msg->body.power_rail_boot_msg.T_PB;

      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyFanRevolingSpeed: {
      tx_frame->can_id = CAN_MSG_FAN_RE_SPD_CONFIG_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = msg->body.fan_speed_cmd_msg.Fan1_revol_perc;
        frame.byte_1 = msg->body.fan_speed_cmd_msg.Fan2_revol_perc;
        frame.byte_2 = msg->body.fan_speed_cmd_msg.Fan3_revol_perc;
        frame.byte_3 = msg->body.fan_speed_cmd_msg.Fan4_revol_perc;

      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyChargeConfig: {
      tx_frame->can_id = CAN_MSG_CHARGE_CONFIG_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = msg->body.charge_config_msg.ChargeRail;
        frame.byte_1 = msg->body.charge_config_msg.ChargeCurrent;

      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyFirmwareVersion: {
      tx_frame->can_id = CAN_MSG_FIRMWARE_VER_QUERY_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = msg->body.firm_ver_q_msg.FixContent;

      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyFirmwareConfig: {
      tx_frame->can_id = CAN_MSG_FIRMWARE_VER_CONFIG_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = msg->body.firm_ver_config_msg.version[0];
        frame.byte_1 = msg->body.firm_ver_config_msg.version[1];
        frame.byte_2 = msg->body.firm_ver_config_msg.version[2];
        frame.byte_3 = msg->body.firm_ver_config_msg.version[3];
        frame.byte_4 = msg->body.firm_ver_config_msg.version[4];
        frame.byte_5 = msg->body.firm_ver_config_msg.version[5];
        frame.byte_6 = msg->body.firm_ver_config_msg.version[6];
        frame.byte_7 = msg->body.firm_ver_config_msg.version[7];

      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgPercyAMRNode: {
      tx_frame->can_id = CAN_MSG_AMR_NODE_ID_CONFIG_ID;
      tx_frame->can_dlc = 8;
      M8_byte_Frame frame;
        frame.byte_0 = msg->body.amr_node_config_msg.fixed_content;

      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }

    default: {
      ret = false;
      break;
    }
  }
  return ret;
}

uint8_t CalcCanFrameChecksumPERCY(uint16_t id, uint8_t *data, uint8_t dlc) {
  uint8_t checksum = 0x00;
  checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
  for (int i = 0; i < (dlc - 1); ++i) checksum += data[i];
  return checksum;
}
