/*
 * protocol_v2_parser.hpp
 *
 * Created on: Jul 08, 2021 14:51
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef PROTOCOL_V2_PARSER_PERCY_HPP
#define PROTOCOL_V2_PARSER_PERCY_HPP

#include "ugv_sdk/details/interface/parser_interface.hpp"

namespace agilexrobotics {
class Protocol_PERCY_Parser : public ParserInterface<ProtocolVersion::PERCY> {
 public:
  bool DecodeMessage(const struct can_frame *rx_frame,
                     Agx_Percy_Message *msg);
  bool EncodeMessage(const Agx_Percy_Message *msg,
                     struct can_frame *tx_frame);

// override the pure virtual method but we dont use them
  bool DecodeMessage(const struct can_frame *rx_frame,
                     AgxMessage *msg) override { return 0; };
  bool EncodeMessage(const AgxMessage *msg,
                     struct can_frame *tx_frame) override { return 0; };

  uint8_t CalculateChecksum(uint16_t id, uint8_t *data, uint8_t dlc) override;
};
}  // namespace agilexrobotics

#endif /* PROTOCOL_V2_PARSER_PERCY_HPP */
