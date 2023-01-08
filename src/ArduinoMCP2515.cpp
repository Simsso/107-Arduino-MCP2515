/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-MCP2515/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <107-Arduino-MCP2515.h>

#include <algorithm>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace MCP2515;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static CanBitRateConfig const BIT_RATE_CONFIG_ARRAY[] =
{
  BitRate_125kBPS_16MHz,
  BitRate_250kBPS_16MHz,
  BitRate_500kBPS_16MHz,
  BitRate_1000kBPS_16MHz,
  BitRate_125kBPS_8MHz,
  BitRate_250kBPS_8MHz,
  BitRate_500kBPS_8MHz,
  BitRate_1000kBPS_8MHz
};

/**************************************************************************************
 * INLINE FUNCTIONS
 **************************************************************************************/

inline bool isBitSet(uint8_t const reg_val, uint8_t const bit_pos)
{
  return ((reg_val & (1<<bit_pos)) == (1<<bit_pos));
}

inline bool isBitClr(uint8_t const reg_val, uint8_t const bit_pos)
{
  return !isBitSet(reg_val, bit_pos);
}

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ArduinoMCP2515::ArduinoMCP2515(SpiSelectFunc select,
                               SpiDeselectFunc deselect,
                               SpiTransferFunc transfer,
                               MicroSecondFunc micros,
                               OnReceiveBufferFullFunc on_rx_buf_full,
                               OnTransmitBufferEmptyFunc on_tx_buf_empty)
: _io{select, deselect, transfer}
, _cfg{_io}
, _ctrl{_io}
, _micros{micros}
, _on_rx_buf_full{on_rx_buf_full}
, _on_tx_buf_empty{on_tx_buf_empty}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ArduinoMCP2515::begin()
{
  _ctrl.reset();
  _cfg.disableFilter_RxB0();
  _cfg.disableFilter_RxB1();
  _cfg.enableRollover_RxB0();

  if (_on_tx_buf_empty)
  {
    _cfg.enableIntFlag(CANINTE::TX0IE);
    _cfg.enableIntFlag(CANINTE::TX1IE);
    _cfg.enableIntFlag(CANINTE::TX2IE);
  }

  if (_on_rx_buf_full)
  {
    _cfg.enableIntFlag(CANINTE::RX0IE);
    _cfg.enableIntFlag(CANINTE::RX1IE);
  }
}

void ArduinoMCP2515::setBitRate(CanBitRate const bit_rate)
{
  _cfg.setBitRateConfig(BIT_RATE_CONFIG_ARRAY[static_cast<size_t>(bit_rate)]);
}

#if LIBCANARD
bool ArduinoMCP2515::transmit(CanardFrame const & frame)
{
  return transmitCANFrame(CAN_EFF_BITMASK | frame.extended_can_id,
                          reinterpret_cast<uint8_t const *>(frame.payload),
                          static_cast<uint8_t const>(frame.payload_size));
}
#else
bool ArduinoMCP2515::transmit(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  return transmitCANFrame(id, data, len);
}
#endif

void ArduinoMCP2515::onExternalEventHandler()
{
  uint8_t const status = _ctrl.status();

  if(isBitSet(status, bp(STATUS::RX0IF))) onReceiveBuffer_0_Full();
  if(isBitSet(status, bp(STATUS::RX1IF))) onReceiveBuffer_1_Full();
  if(isBitSet(status, bp(STATUS::TX0IF))) onTransmitBuffer_0_Empty();
  if(isBitSet(status, bp(STATUS::TX1IF))) onTransmitBuffer_1_Empty();
  if(isBitSet(status, bp(STATUS::TX2IF))) onTransmitBuffer_2_Empty();
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

bool ArduinoMCP2515::transmitCANFrame(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  if (isBitClr(_io.readRegister(Register::TXB0CTRL), bp(TXBnCTRL::TXREQ)))
  {
    _ctrl.transmit(TxB::TxB0, id, data, len);
    return true;
  }

#if LIBCANARD
  /* Only use a single transmit buffer in order to prevent unintentional
   * priority inversion while transmitting OpenCyphal/CAN frames.
   */
  return false;
#endif

  if (isBitClr(_io.readRegister(Register::TXB1CTRL), bp(TXBnCTRL::TXREQ)))
  {
    _ctrl.transmit(TxB::TxB1, id, data, len);
    return true;
  }

  if (isBitClr(_io.readRegister(Register::TXB2CTRL), bp(TXBnCTRL::TXREQ)))
  {
    _ctrl.transmit(TxB::TxB2, id, data, len);
    return true;
  }

  return false;
}

void ArduinoMCP2515::onReceiveBuffer_0_Full()
{
  uint32_t id = 0;
  uint8_t data[8] = {0}, len = 0;
  unsigned long const rx_timestamp_us = _micros();

  _ctrl.receive(RxB::RxB0, id, data, len);
  _ctrl.clearIntFlag(CANINTF::RX0IF);
  onReceiveBuffer_n_Full(rx_timestamp_us, id, data, len);
}

void ArduinoMCP2515::onReceiveBuffer_1_Full()
{
  uint32_t id = 0;
  uint8_t data[8] = {0}, len = 0;
  unsigned long const rx_timestamp_us = _micros();

  _ctrl.receive(RxB::RxB1, id, data, len);
  _ctrl.clearIntFlag(CANINTF::RX1IF);
  onReceiveBuffer_n_Full(rx_timestamp_us, id, data, len);
}

void ArduinoMCP2515::onTransmitBuffer_0_Empty()
{
  if (_on_tx_buf_empty)
    _on_tx_buf_empty(this);
  _ctrl.clearIntFlag(CANINTF::TX0IF);
}

void ArduinoMCP2515::onTransmitBuffer_1_Empty()
{
  if (_on_tx_buf_empty)
    _on_tx_buf_empty(this);
  _ctrl.clearIntFlag(CANINTF::TX1IF);
}

void ArduinoMCP2515::onTransmitBuffer_2_Empty()
{
  if (_on_tx_buf_empty)
    _on_tx_buf_empty(this);
  _ctrl.clearIntFlag(CANINTF::TX2IF);
}

void ArduinoMCP2515::onReceiveBuffer_n_Full(unsigned long const timestamp_us, uint32_t const id, uint8_t const * data, uint8_t const len) const
{
  if (_on_rx_buf_full)
  {
#if LIBCANARD
    CanardFrame const frame
    {
#if (CANARD_VERSION_MAJOR == 1)
      timestamp_us,                        /* timestamp_usec  */
#endif
      id & CAN_ADR_BITMASK,                /* extended_can_id limited to 29 bit */
      len,                                 /* payload_size    */
      reinterpret_cast<const void *>(data) /* payload         */
    };
    _on_rx_buf_full(frame);
#else
    _on_rx_buf_full(timestamp_us, id, data, len);
#endif
  }
}

void prepareId(uint8_t *buffer, const uint32_t id)
{
  // When using standard identifiers (11 bits), the remaining eighteen 
  // bits of the mask and filters are matched to the first two data bytes,
  // with the two most significant bits of those eighteen bits left unused.
  // (11 + 2 + 16 = 29)
  uint16_t canid = (uint16_t)(id & 0x0FFFF);

  buffer[0] = (uint8_t) (canid >> 3);
  buffer[1] = (uint8_t) ((canid & 0x07 ) << 5);
  buffer[2] = 0;
  buffer[3] = 0;
}

void ArduinoMCP2515::setFilterMask(const uint8_t mask_id, const uint32_t ulData)
{
  // Call this function in config mode. It only supports standard IDs (not EXT ones).
  // Parameter 'mask_id' is either 0 or 1 (which mask to use).
  // Parameter 'ulData' is the mask itself.
  // Only the least significant 11 bits of ulData are used.
  
  uint8_t tbufdata[4];
  prepareId(tbufdata, ulData);

  Register reg;
  switch (mask_id) {
      case 0: 
        reg = MCP2515::Register::RXM0SIDH;
        _io.modifyRegister(MCP2515::Register::RXB0CTRL,
          RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
          RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT);
        break;
      case 1:
        reg = MCP2515::Register::RXM1SIDH;
        _io.modifyRegister(MCP2515::Register::RXB1CTRL,
          RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
          RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT);
        break;
      default: return;
  }

  _io.writeRegister(reg, tbufdata, 4);
}

void ArduinoMCP2515::setFilter(const uint8_t filter_num, const uint32_t ulData)
{
  // Call this function in config mode. It only supports standard IDs (not EXT ones).
  // Parameter 'filter_num' is in [0,6).
  Register reg;
  switch (filter_num) {
    case 0: reg = MCP2515::Register::RXF0SIDH; break;
    case 1: reg = MCP2515::Register::RXF1SIDH; break;
    case 2: reg = MCP2515::Register::RXF2SIDH; break;
    case 3: reg = MCP2515::Register::RXF3SIDH; break;
    case 4: reg = MCP2515::Register::RXF4SIDH; break;
    case 5: reg = MCP2515::Register::RXF5SIDH; break;
    default: return;
  }

  uint8_t tbufdata[4];
  prepareId(tbufdata, ulData);
  _io.writeRegister(reg, tbufdata, 4);
}
