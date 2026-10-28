/*
  MCP2515.h - Library for Microchip MCP2515 CAN Controller
  
  Author: David Harding
  Maintainer: RechargeCar Inc (http://rechargecar.com)
  Further Modification: Collin Kidder
  
  Created: 11/08/2010
  
  For further information see:
  
  http://ww1.microchip.com/downloads/en/DeviceDoc/21801e.pdf
  http://en.wikipedia.org/wiki/CAN_bus


  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __CH32_CAN__
#define __CH32_CAN__

#include "Arduino.h"
#include <can_common.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//#define DEBUG_SETUP
#define BI_NUM_FILTERS 28

#define BI_RX_BUFFER_SIZE	3
#define BI_TX_BUFFER_SIZE  3
// Frames the receive queue holds while the main loop is busy (20 B each). Every
// frame lost here breaks an ISO-TP transfer; the loop can stall for >10 ms.
#ifndef CAN_RX_QUEUE_BUFFER_SIZE
#define CAN_RX_QUEUE_BUFFER_SIZE  32
#endif
// Frames handled per main loop turn (CAN_RX_ON_MAIN_LOOP)
#define CAN_RX_FRAMES_PER_LOOP CAN_RX_QUEUE_BUFFER_SIZE
// Frames buffered per transmit ring (see ch32v203_can_builtin.cpp)
#define CAN_TX_QUEUE_BUFFER_SIZE  12
// The default ring (flow control, plain sendFrame) is deeper
#define CAN_TX_DEFAULT_RING_DEPTH 24
#define CAN_TX_RING_COUNT 3
// A frame that sits in a mailbox this long is aborted
#define CAN_TX_MAILBOX_TIMEOUT_MS 200
#define CAN_MAX_RETRANSMIT_COUNT 128

typedef struct {
    uint16_t brp : 10;
    uint8_t tseg_1 : 4,
            tseg_2 : 3,
            sjw    : 2;
    // Note: bit length are here for compile-time verification
} CANTimingConfig_t;

class CH32CAN;

class CH32CAN : public CAN_COMMON
{
public:
  CH32CAN();

  //block of functions which must be overriden from CAN_COMMON to implement functionality for this hardware
  int _setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended);
  int _setFilter(uint32_t id, uint32_t mask, bool extended);
  uint32_t init(uint32_t ul_baudrate);
  uint32_t beginAutoSpeed();
  uint32_t set_baudrate(uint32_t ul_baudrate);
  void setListenOnlyMode(bool state);
  void setNoACKMode(bool state);
  void enable();
  void disable();
  bool sendFrame(CAN_FRAME const &txFrame);

  // Ordered transmit streams. Every handle owns one hardware mailbox fed by a
  // strict FIFO, so its frames can neither be reordered by the mailbox
  // scheduler nor by other traffic. Handle 0 is the default stream shared by
  // everything else (and by streams that did not get a mailbox of their own).
  CanTxHandle acquireTxHandle();
  void releaseTxHandle(CanTxHandle handle);
  CanTxStatus trySendFrame(CAN_FRAME const &txFrame, CanTxHandle handle);
  void flushTxHandle(CanTxHandle handle);
  bool takeTxFault(CanTxHandle handle);
  bool rx_avail();
  void setRXBufferSize(int newSize);
  void setTXBufferSize(int newSize);
  uint16_t available(); //like rx_avail but returns the number of waiting frames
  uint32_t get_rx_buff(CAN_FRAME &msg);
  
  void watchForList(uint8_t mailbox, uint32_t id1, bool rtr1, uint32_t id2, bool rtr2);
  void watchForList(uint8_t mailbox, uint16_t id1, bool rtr1, uint16_t id2, bool rtr2, uint16_t id3, bool rtr3, uint16_t id4, bool rtr4);
  void watchForMask(uint8_t mailbox, uint32_t id, uint32_t mask, bool rtr = false, bool rtrMask = false, bool onlyExtendedId = false);
  void watchForMask(uint8_t mailbox, uint16_t id1, uint16_t mask1, bool rtr1, bool rtrMask1,
                    uint16_t id2, uint16_t mask2, bool rtr2, bool rtrMask2);

  friend void CAN_Rx_handler(void *pvParameters);
  
  #if defined(CAN_RX_ON_MAIN_LOOP)
  void loop();
  #endif

protected:
  bool readyForTraffic;
  int cyclesSinceTraffic;

private:
  bool filterIsConfigured[BI_NUM_FILTERS];
  int rxBufferSize;
  int txBufferSize;
  bool listenOnlyMode;
  CANTimingConfig_t currentTimingConfig;
  uint32_t lastRecoveryTick = 0;
  uint32_t recoveryBackoffMs = 0;
  uint32_t lastErrorLogTick = 0;
  uint32_t suppressedErrorCount = 0;
  
  bool processFrame(CAN_FRAME &frame, uint8_t filter_id);
  bool configureController();
  void checkBusHealth();
  void serviceTxTimeouts();
  bool recoverController();
};

#endif
