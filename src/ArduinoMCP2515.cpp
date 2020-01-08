/**
 * @brief   Arduino library for controlling the MCP2515 in order to receive/transmit CAN frames.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 * @license LGPL 3.0
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ArduinoMCP2515.h>

#include <sstream>
#include <iomanip>
#include <algorithm>

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ArduinoMCP2515::ArduinoMCP2515(int const cs_pin,
                               int const int_pin,
                               OnCanFrameReceiveFunc on_can_frame_rx)
: _io{cs_pin}
, _event{int_pin}
, _on_can_frame_rx{on_can_frame_rx}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ArduinoMCP2515::begin()
{
  _io.begin();
  _event.begin();
}

bool ArduinoMCP2515::transmit(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  return false; /* TODO */
}

/**************************************************************************************
 * PUBLIC FUNCTION DEFINITION
 **************************************************************************************/

std::string toStr(uint32_t const id, uint8_t const * data, uint8_t const len)
{
  std::stringstream ss;
  ss << std::hex
     << "ID: " << std::setw(4) << std::setfill(' ') << id << " DATA: ";
  std::for_each(data,
                data + len,
                [&ss](uint8_t const data)
                {
                  ss << std::setw(2) << std::setfill('0') << data << " ";
                });
  return ss.str();
}