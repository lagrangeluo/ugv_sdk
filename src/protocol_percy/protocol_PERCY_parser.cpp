/*
 * protocol_v2_parser.cpp
 *
 * Created on: Jul 08, 2021 14:53
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "ugv_sdk/details/protocol_percy/protocol_PERCY_parser.hpp"
#include "protocol_percy/agilex_msg_parser_percy.h"

namespace agilexrobotics {
bool Protocol_PERCY_Parser::DecodeMessage(const struct can_frame *rx_frame,
                                     Agx_Percy_Message *msg) {
  return DecodeCanFramePERCY(rx_frame, msg);
}

bool Protocol_PERCY_Parser::EncodeMessage(const Agx_Percy_Message *msg,
                                     struct can_frame *tx_frame) {
  return EncodeCanFramePERCY(msg, tx_frame);
}

uint8_t Protocol_PERCY_Parser::CalculateChecksum(uint16_t id, uint8_t *data,
                                            uint8_t dlc) {
  return CalcCanFrameChecksumPERCY(id, data, dlc);
}
}  // namespace agilexrobotics