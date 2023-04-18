/* 
 * agilex_msg_parser.h
 * 
 * Created on: Mar 24, 2023 18:00
 * Description:
 *
 * Copyright (c) 2023 AgileX Robotics Pte. Ltd.
 */

#ifndef AGILEX_MSG_PARSER_PERCY_H
#define AGILEX_MSG_PARSER_PERCY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __linux__
#include <linux/can.h>
#else
struct can_frame {
  uint32_t can_id;
  uint8_t can_dlc;
  uint8_t data[8] __attribute__((aligned(8)));
};
#endif

#include "ugv_sdk/details/interface/agilex_message.h"

bool DecodeCanFramePERCY(const struct can_frame *rx_frame, Agx_Percy_Message *msg);
bool EncodeCanFramePERCY(const Agx_Percy_Message *msg, struct can_frame *tx_frame);
uint8_t CalcCanFrameChecksumPERCY(uint16_t id, uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif /* AGILEX_MSG_PARSER_PERCY_H */
