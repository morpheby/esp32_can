/*
  CH32_CAN.cpp - Library for CH32V203 built-in CAN module
    Original library for ESP32 reworked to support CH32v20x chips with Arduino and FreeRTOS
  
  Author: Ilya Mikhaltsou
  
  Created: 12.01.2025
*/

#include "Arduino.h"
#include "ch32vxxx/ch32vxxx_isr.h"
#include "ch32v203_can_builtin.h"
#include "ch32yyxx_can.h"
#include "spdlog/logger.h"
#include "wiring_time.h"
#include <memory>
#include <new>

#if __has_include("spdlog/spdlog.h")
#include "spdlog/spdlog.h"

static std::shared_ptr<spdlog::logger> logger = nullptr;
#endif

typedef struct
{
    CANTimingConfig_t cfg;
    uint32_t speed;
} VALID_TIMING;

// FIXME: Hardcoded value
#if CH32_SYSCLK_FREQ_HZ == 144'000'000
#define CAN_CLOCK_FREQ (CH32_SYSCLK_FREQ_HZ)/2
#elif CH32_SYSCLK_FREQ_HZ == 120'000'000
#define CAN_CLOCK_FREQ (CH32_SYSCLK_FREQ_HZ)/2
#endif

#define CAN_BRP_FROM_RESOLUTION_HZ(freq) ((CAN_CLOCK_FREQ)/(freq))

#define CAN_TIMING_CONFIG(quanta_hz, ts1, ts2, sjw_v) {.brp = CAN_BRP_FROM_RESOLUTION_HZ((quanta_hz)), .tseg_1 = (ts1) - 1, .tseg_2 = (ts2) - 1, .sjw = (sjw_v) - 1}

#define CAN_TIMING_CONFIG_NULL      CAN_TIMING_CONFIG(CAN_CLOCK_FREQ, 1, 1, 1)

#if CH32_SYSCLK_FREQ_HZ == 144'000'000

#if CAN_BRP_FROM_RESOLUTION_HZ(16000) < 1024
// NOTE: Configuration not tested and not even calculated. Consider using bitcalc or other software to adjust
// #define CAN_TIMING_CONFIG_1KBITS        CAN_TIMING_CONFIG(16000, 13, 2, 1)
#endif

#if CAN_BRP_FROM_RESOLUTION_HZ(80000) < 1024
// NOTE: Configuration not tested and not even calculated. Consider using bitcalc or other software to adjust
// #define TWAI_TIMING_CONFIG_5KBITS       CAN_TIMING_CONFIG(80000, 13, 2, 1)
#endif

#if CAN_BRP_FROM_RESOLUTION_HZ(160000) < 1024
#define CAN_TIMING_CONFIG_10KBITS       CAN_TIMING_CONFIG(160'000, 13, 2, 1)
#endif

#define CAN_TIMING_CONFIG_20KBITS       CAN_TIMING_CONFIG(320'000, 13, 2, 1)
#define CAN_TIMING_CONFIG_50KBITS       CAN_TIMING_CONFIG(800'000, 13, 2, 1)
#define CAN_TIMING_CONFIG_100KBITS      CAN_TIMING_CONFIG(1'600'000, 13, 2, 1)
#define CAN_TIMING_CONFIG_125KBITS      CAN_TIMING_CONFIG(2'000'000, 13, 2, 1)
#define CAN_TIMING_CONFIG_250KBITS      CAN_TIMING_CONFIG(4'000'000, 13, 2, 1)
#define CAN_TIMING_CONFIG_500KBITS      CAN_TIMING_CONFIG(8'000'000, 12, 3, 2)
#define CAN_TIMING_CONFIG_800KBITS      CAN_TIMING_CONFIG(16'000'000, 14, 5, 3)
#define CAN_TIMING_CONFIG_1MBITS        CAN_TIMING_CONFIG(16'000'000, 11, 4, 3)

#elif CH32_SYSCLK_FREQ_HZ == 120'000'000

#if CAN_BRP_FROM_RESOLUTION_HZ(16000) < 1024
// NOTE: Configuration not tested and not even calculated. Consider using bitcalc or other software to adjust
// #define CAN_TIMING_CONFIG_1KBITS        CAN_TIMING_CONFIG(16000, 13, 2, 1)
#endif

#if CAN_BRP_FROM_RESOLUTION_HZ(80000) < 1024
// NOTE: Configuration not tested and not even calculated. Consider using bitcalc or other software to adjust
// #define TWAI_TIMING_CONFIG_5KBITS       CAN_TIMING_CONFIG(80000, 13, 2, 1)
#endif

#if CAN_BRP_FROM_RESOLUTION_HZ(160000) < 1024
#define CAN_TIMING_CONFIG_10KBITS       CAN_TIMING_CONFIG(160'000, 13, 2, 1)
#endif

#define CAN_TIMING_CONFIG_20KBITS       CAN_TIMING_CONFIG(300'800, 12, 2, 1)
#define CAN_TIMING_CONFIG_50KBITS       CAN_TIMING_CONFIG(800'000, 13, 2, 1)
#define CAN_TIMING_CONFIG_100KBITS      CAN_TIMING_CONFIG(1'500'000, 12, 2, 1)
#define CAN_TIMING_CONFIG_125KBITS      CAN_TIMING_CONFIG(2'000'000, 13, 2, 1)
#define CAN_TIMING_CONFIG_250KBITS      CAN_TIMING_CONFIG(4'000'000, 13, 2, 1)
#define CAN_TIMING_CONFIG_500KBITS      CAN_TIMING_CONFIG(5'000'000, 7, 2, 2)
#define CAN_TIMING_CONFIG_800KBITS      CAN_TIMING_CONFIG(12'000'000, 10, 4, 3)
#define CAN_TIMING_CONFIG_1MBITS        CAN_TIMING_CONFIG(12'000'000, 8, 3, 3)

#endif

static QueueHandle_t rx_queue;
static volatile bool canNeedsBusReset = false;
// Tick of the last frame successfully received or transmitted
static volatile TickType_t canLastTrafficTick = 0;

// Controller is considered stuck if it is error passive (or has a pending
// mailbox) and no frame got through for this long
#define CAN_STALL_TIMEOUT_MS        1000
#define CAN_RECOVERY_BACKOFF_MIN_MS 1000
#define CAN_RECOVERY_BACKOFF_MAX_MS 8000
#define CAN_ERROR_LOG_INTERVAL_MS   1000

static void canCountRxFrameFromISR();
static inline void canMarkTrafficFromISR() {
    canLastTrafficTick = xTaskGetTickCountFromISR();
}

// Transmit path.
//
// Each of the three hardware mailboxes is fed from its own software FIFO
// ("ring") and holds at most one frame at a time. A ring is therefore strictly
// ordered, no matter what the hardware scheduler or the other rings do. Ring 0
// is shared by everything that does not ask for an ordered stream; rings 1 and
// 2 are handed out as transmit handles (see acquireTxHandle()).
//
// Frames are loaded into the mailbox either by the TX interrupt (the mailbox
// became empty) or by the producer itself (the mailbox was already empty).
// Both do it with interrupts masked around "take from the ring + fill the
// mailbox", so frames of one ring can not overtake each other.
struct TxEntry {
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
    uint8_t flags;
};
static constexpr uint8_t TX_FLAG_EXTENDED = 1;
static constexpr uint8_t TX_FLAG_RTR = 2;
static constexpr uint32_t TX_REQUEST = 1; // TXRQ in TXMIR

static QueueHandle_t tx_ring[CAN_TX_RING_COUNT];
static volatile uint8_t txHandlesInUse = 0;   // bit per ring (ring 0 is always shared)
static volatile uint8_t txFaultMask = 0;      // bit per ring: frames were lost
static volatile TickType_t txMailboxLoadedTick[CAN_TX_RING_COUNT];
static volatile uint32_t txAbortCount = 0;

// Diagnostics: frames lost inside the driver (read them with a debugger or from the log)
struct CanDriverStats {
    volatile uint32_t rxFifoOverruns;   // hardware FIFO overflowed: a received frame was lost
    volatile uint32_t rxQueueDrops;     // software queue full: a received frame was dropped in the ISR
    volatile uint32_t txRingFull;       // trySendFrame() reported Busy
    volatile uint32_t txSendFrameFails; // blocking sendFrame() could not queue the frame
    volatile uint32_t rxQueueHighWater; // most frames seen waiting in the receive queue
    volatile uint32_t maxLoopGapMs;     // longest time between two CH32CAN::loop() calls
    volatile uint32_t rxFrames;         // frames taken from the hardware FIFOs
    volatile uint32_t rxBurstMax;       // most frames received within one 16 ms window
    volatile uint32_t lastDropQueueLen; // frames waiting when the last frame was dropped
    volatile uint32_t lastDropLoopAge;  // ticks since CH32CAN::loop() last ran when it happened
    volatile uint32_t lastLoopTick;
};
CanDriverStats canDriverStats;

static void canCountRxFrameFromISR() {
    static uint32_t windowStart = 0, inWindow = 0;
    const uint32_t now = xTaskGetTickCountFromISR();
    canDriverStats.rxFrames = canDriverStats.rxFrames + 1;
    if (now - windowStart >= pdMS_TO_TICKS(16)) {
        windowStart = now;
        inWindow = 0;
    }
    if (++inWindow > canDriverStats.rxBurstMax) {
        canDriverStats.rxBurstMax = inWindow;
    }
}

static inline bool txMailboxEmpty(uint8_t mb) {
    return CAN1->TSTATR & (CAN_TSTATR_TME0 << mb);
}

// Write-one-to-clear flags live at the same offsets for all mailboxes, 8 bits apart
static inline uint32_t txMailboxFlag(uint32_t mailbox0Flag, uint8_t mb) {
    return mailbox0Flag << (8 * mb);
}

static void txLoadMailbox(uint8_t mb, const TxEntry &entry) {
    auto &box = CAN1->sTxMailBox[mb];
    uint32_t identifier = (entry.flags & TX_FLAG_EXTENDED)
        ? ((entry.id << 3) | CAN_Id_Extended)
        : (entry.id << 21);
    if (entry.flags & TX_FLAG_RTR) {
        identifier |= CAN_RTR_Remote;
    }

    box.TXMIR = identifier;
    box.TXMDTR = (box.TXMDTR & 0xFFFFFFF0) | (entry.length & 0x0F);
    box.TXMDLR = ((uint32_t)entry.data[3] << 24) | ((uint32_t)entry.data[2] << 16) |
                 ((uint32_t)entry.data[1] << 8) | entry.data[0];
    box.TXMDHR = ((uint32_t)entry.data[7] << 24) | ((uint32_t)entry.data[6] << 16) |
                 ((uint32_t)entry.data[5] << 8) | entry.data[4];
    txMailboxLoadedTick[mb] = xTaskGetTickCountFromISR();
    box.TXMIR = identifier | TX_REQUEST;
}

// Fills the mailbox from its ring if it is empty. Interrupts must be masked
// (or the caller is the TX interrupt).
static void txFeedMailbox(uint8_t mb, bool fromIsr) {
    TxEntry entry;
    if (!txMailboxEmpty(mb)) {
        return;
    }
    const bool received = fromIsr ? xQueueReceiveFromISR(tx_ring[mb], &entry, nullptr) == pdTRUE
                                  : xQueueReceive(tx_ring[mb], &entry, 0) == pdTRUE;
    if (received) {
        txLoadMailbox(mb, entry);
    }
}

static void txKick(uint8_t mb) {
    taskENTER_CRITICAL();
    if (tx_ring[mb]) {
        txFeedMailbox(mb, false);
    }
    taskEXIT_CRITICAL();
}

static TxEntry frameToEntry(const CAN_FRAME *frame) {
    TxEntry entry{};
    entry.id = frame->id;
    entry.length = frame->length > 8 ? 8 : frame->length;
    entry.flags = (frame->extended ? TX_FLAG_EXTENDED : 0) | (frame->rtr ? TX_FLAG_RTR : 0);
    if (!frame->rtr) {
        memcpy(entry.data, frame->data.bytes, entry.length);
    }
    return entry;
}

// Receive queue entry: half the size of a CAN_FRAME, so twice as many frames fit
struct RxEntry {
    uint32_t id;
    uint8_t data[8];
    uint16_t timestamp;
    uint8_t length;
    uint8_t flags;
    uint8_t fid;
};

static CAN_FRAME entryToFrame(const RxEntry &entry) {
    CAN_FRAME frame;
    frame.id = entry.id;
    frame.extended = entry.flags & TX_FLAG_EXTENDED;
    frame.rtr = (entry.flags & TX_FLAG_RTR) ? 1 : 0;
    frame.timestamp = entry.timestamp;
    frame.length = entry.length;
    frame.fid = entry.fid;
    if (!frame.rtr) {
        memcpy(frame.data.bytes, entry.data, entry.length);
    }
    return frame;
}

#if !defined(CAN_RX_ON_MAIN_LOOP)
#if configSUPPORT_STATIC_ALLOCATION
static StaticTask_t CAN_Rx_task;
static StackType_t CAN_Rx_task_stack[128];
#endif
static TaskHandle_t CAN_Rx_handler_task = NULL;
#endif

// static CanTxMsg mailboxMessages[3];
// static uint16_t mailboxRetransmitCounter[3] = {0, 0, 0};

//because of the way the TWAI library works, it's just easier to store the valid timings here and anything not found here
//is just plain not supported. If you need a different speed then add it here. Be sure to leave the zero record at the end
//as it serves as a terminator
const VALID_TIMING valid_timings[] = 
{
    {CAN_TIMING_CONFIG_1MBITS,   1000000},
    {CAN_TIMING_CONFIG_500KBITS,  500000},
    {CAN_TIMING_CONFIG_250KBITS,  250000},
    {CAN_TIMING_CONFIG_125KBITS,  125000},
    {CAN_TIMING_CONFIG_800KBITS,  800000},
    {CAN_TIMING_CONFIG_100KBITS,  100000},
    {CAN_TIMING_CONFIG_50KBITS,    50000},
    {CAN_TIMING_CONFIG_20KBITS,    20000},
    {CAN_TIMING_CONFIG_20KBITS,        0} //this is a terminator record. When the code sees an entry with 0 speed it stops searching
};

#if !(USE_TINYUSB)

// CAN doesn't work with USB, unless you have remap available. CH32V203K8T6 doesn't, so can't really test this.

_ISR_DEF(USB_LP_CAN1_RX0_IRQHandler) {
    // CAN FIFO0
    static CanRxMsg msg;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (rx_queue) {
        while (CAN_MessagePending(CAN1, CAN_FIFO0)) {
            uint16_t timestamp = (CAN1->sFIFOMailBox[0].RXMDTR & CAN_RXMDT0R_TIME) >> 16;
            CAN_Receive(CAN1, CAN_FIFO0, &msg);
            canMarkTrafficFromISR();
            canCountRxFrameFromISR();
            if (xQueueIsQueueFullFromISR(rx_queue)) {
                canDriverStats.rxQueueDrops = canDriverStats.rxQueueDrops + 1;
                canDriverStats.lastDropQueueLen = uxQueueMessagesWaitingFromISR(rx_queue);
                canDriverStats.lastDropLoopAge = xTaskGetTickCountFromISR() - canDriverStats.lastLoopTick;
                break;
            }

            RxEntry frame;
            frame.id = (msg.IDE == CAN_Id_Extended) ? msg.ExtId : msg.StdId;
            frame.timestamp = timestamp;
            frame.length = std::min(msg.DLC, (uint8_t) 8);
            frame.flags = ((msg.IDE == CAN_Id_Extended) ? TX_FLAG_EXTENDED : 0) | ((msg.RTR == CAN_RTR_Remote) ? TX_FLAG_RTR : 0);
            uint8_t filterGrpNum = 0;
            int filterNum = msg.FMI;
            for (filterGrpNum = 0; filterGrpNum < 28; ++filterGrpNum) {
                uint16_t mlt = 1;
                uint32_t filter_number_bit_pos = ((uint32_t)1) << filterGrpNum;
                if (CAN1->FAFIFOR & filter_number_bit_pos) continue;
                if (!(CAN1->FSCFGR & filter_number_bit_pos)) mlt *= 2;
                if (CAN1->FMCFGR & filter_number_bit_pos) mlt *= 2;
                filterNum -= mlt;
                if (filterNum < 0) break;
            }
            frame.fid = filterGrpNum;
            if (!(frame.flags & TX_FLAG_RTR)) {
                memcpy(frame.data, msg.Data, frame.length);
            }
            
            xQueueSendFromISR(rx_queue, &frame, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (CAN1->RFIFO0 & CAN_RFIFO0_FOVR0) {
        canDriverStats.rxFifoOverruns = canDriverStats.rxFifoOverruns + 1;
    }
    CAN_ClearFlag(CAN1, CAN_FLAG_FF0 | CAN_FLAG_FOV0);
    CAN_ClearITPendingBit(CAN1,  CAN_IT_FMP0 | CAN_IT_FF0 | CAN_IT_FOV0);
}

_ISR_DEF(CAN1_RX1_IRQHandler) {
    // CAN FIFO1
    static CanRxMsg msg;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (rx_queue) {
        while (CAN_MessagePending(CAN1, CAN_FIFO1)) {
            uint16_t timestamp = (CAN1->sFIFOMailBox[1].RXMDTR & CAN_RXMDT1R_TIME) >> 16;
            CAN_Receive(CAN1, CAN_FIFO1, &msg);
            canMarkTrafficFromISR();
            canCountRxFrameFromISR();
            if (xQueueIsQueueFullFromISR(rx_queue)) {
                canDriverStats.rxQueueDrops = canDriverStats.rxQueueDrops + 1;
                canDriverStats.lastDropQueueLen = uxQueueMessagesWaitingFromISR(rx_queue);
                canDriverStats.lastDropLoopAge = xTaskGetTickCountFromISR() - canDriverStats.lastLoopTick;
                break;
            }

            RxEntry frame;
            frame.id = (msg.IDE == CAN_Id_Extended) ? msg.ExtId : msg.StdId;
            frame.timestamp = timestamp;
            frame.length = std::min(msg.DLC, (uint8_t) 8);
            frame.flags = ((msg.IDE == CAN_Id_Extended) ? TX_FLAG_EXTENDED : 0) | ((msg.RTR == CAN_RTR_Remote) ? TX_FLAG_RTR : 0);
            uint8_t filterGrpNum = 0;
            int filterNum = msg.FMI;
            for (filterGrpNum = 0; filterGrpNum < 28; ++filterGrpNum) {
                uint16_t mlt = 1;
                uint32_t filter_number_bit_pos = ((uint32_t)1) << filterGrpNum;
                if (!(CAN1->FAFIFOR & filter_number_bit_pos)) continue;
                if (!(CAN1->FSCFGR & filter_number_bit_pos)) mlt *= 2;
                if (CAN1->FMCFGR & filter_number_bit_pos) mlt *= 2;
                filterNum -= mlt;
                if (filterNum < 0) break;
            }
            frame.fid = filterGrpNum;
            if (!(frame.flags & TX_FLAG_RTR)) {
                memcpy(frame.data, msg.Data, frame.length);
            }
            
            xQueueSendFromISR(rx_queue, &frame, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    if (CAN1->RFIFO1 & CAN_RFIFO1_FOVR1) {
        canDriverStats.rxFifoOverruns = canDriverStats.rxFifoOverruns + 1;
    }
    CAN_ClearFlag(CAN1, CAN_FLAG_FF1 | CAN_FLAG_FOV1);
    CAN_ClearITPendingBit(CAN1,  CAN_IT_FMP1 | CAN_IT_FF1 | CAN_IT_FOV1);
}

_ISR_DEF(USB_HP_CAN1_TX_IRQHandler) {
    // CAN TX: a mailbox became empty (sent, aborted or failed)
    if (CAN1->TSTATR & (CAN_TSTATR_TXOK0 | CAN_TSTATR_TXOK1 | CAN_TSTATR_TXOK2)) {
        canMarkTrafficFromISR();
    }

    for (uint8_t mb = 0; mb < CAN_TX_RING_COUNT; ++mb) {
        // Acknowledge before looking at the mailbox: a completion after this
        // point raises the interrupt again instead of being lost
        const uint32_t completed = txMailboxFlag(CAN_TSTATR_RQCP0, mb);
        if (CAN1->TSTATR & completed) {
            CAN1->TSTATR = completed;
        }
        if (tx_ring[mb]) {
            txFeedMailbox(mb, true);
        }
    }
}

_ISR_DEF(CAN1_SCE_IRQHandler) {
    // CAN Error
    if (CAN_GetFlagStatus(CAN1, CAN_FLAG_BOF) == SET) {
        canNeedsBusReset = true;
    }
    // Error passive is handled by the stall detection in the RX loop
    if (CAN_GetFlagStatus(CAN1, CAN_FLAG_SLAK) == SET) {
        canNeedsBusReset = true;
    }

    CAN_ClearFlag(CAN1, CAN_FLAG_EWG | CAN_FLAG_EPV | CAN_FLAG_BOF | CAN_FLAG_LEC | CAN_FLAG_SLAK);
    CAN_ClearITPendingBit(CAN1,  CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR | CAN_IT_WKU | CAN_IT_SLK);
}

#endif

CH32CAN::CH32CAN() : CAN_COMMON(BI_NUM_FILTERS) 
{
    rxBufferSize = CAN_RX_QUEUE_BUFFER_SIZE;
    txBufferSize = CAN_TX_QUEUE_BUFFER_SIZE;

    readyForTraffic = false;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    cyclesSinceTraffic = 0;
}

#if defined(CAN_RX_ON_MAIN_LOOP)
void CH32CAN::loop() {
    const TickType_t xDelay = 0;
    RxEntry entry;

    static uint32_t lastLoopMs = 0;
    const uint32_t nowMs = millis();
    if (lastLoopMs && nowMs - lastLoopMs > canDriverStats.maxLoopGapMs) {
        canDriverStats.maxLoopGapMs = nowMs - lastLoopMs;
    }
    lastLoopMs = nowMs;
    canDriverStats.lastLoopTick = xTaskGetTickCount();
    const uint32_t waiting = rx_queue ? uxQueueMessagesWaiting(rx_queue) : 0;
    if (waiting > canDriverStats.rxQueueHighWater) {
        canDriverStats.rxQueueHighWater = waiting;
    }

    auto espCan = this;

#else
void CAN_Rx_handler(void *pvParameters)
{
    CH32CAN* espCan = (CH32CAN*)pvParameters;
    const TickType_t xDelay = 200 / portTICK_PERIOD_MS;
    RxEntry entry;

    for(;;)
    {
#endif
        if (xQueueReceive(rx_queue, &entry, xDelay) != pdTRUE) {
            espCan->cyclesSinceTraffic++;
            espCan->checkBusHealth();
#if defined(CAN_RX_ON_MAIN_LOOP)
            return;
#else
            continue;
#endif
        }
        {
            CAN_FRAME frame = entryToFrame(entry);
            espCan->processFrame(frame, frame.fid);
        }
#if defined(CAN_RX_ON_MAIN_LOOP)
    // Drain a burst in one go: one frame per main loop turn lets the receive
    // queue overflow (and drop frames) while the loop is busy
    for (int more = 1; more < CAN_RX_FRAMES_PER_LOOP && xQueueReceive(rx_queue, &entry, 0) == pdTRUE; ++more) {
        CAN_FRAME frame = entryToFrame(entry);
        espCan->processFrame(frame, frame.fid);
    }
    return;
}
#else
    }
}
#endif

CanTxHandle CH32CAN::acquireTxHandle()
{
    CanTxHandle handle = CAN_TX_HANDLE_DEFAULT;

    taskENTER_CRITICAL();
    for (uint8_t ring = 1; ring < CAN_TX_RING_COUNT; ++ring) {
        if (!(txHandlesInUse & (1 << ring))) {
            txHandlesInUse |= 1 << ring;
            txFaultMask &= ~(1 << ring);
            handle = ring;
            break;
        }
    }
    taskEXIT_CRITICAL();

    // Out of dedicated mailboxes: share the default one. Frames of different
    // streams are then interleaved, but every stream stays in order.
    return handle;
}

void CH32CAN::releaseTxHandle(CanTxHandle handle)
{
    if (handle == CAN_TX_HANDLE_DEFAULT || handle >= CAN_TX_RING_COUNT) {
        return;
    }
    flushTxHandle(handle);
    taskENTER_CRITICAL();
    txHandlesInUse &= ~(1 << handle);
    txFaultMask &= ~(1 << handle);
    taskEXIT_CRITICAL();
}

CanTxStatus CH32CAN::trySendFrame(CAN_FRAME const &txFrame, CanTxHandle handle)
{
    if (handle >= CAN_TX_RING_COUNT) {
        handle = CAN_TX_HANDLE_DEFAULT;
    }
    if (!readyForTraffic || !tx_ring[handle]) {
        return CanTxStatus::Failed;
    }
    // Never continue a stream after a frame of it was lost
    if (txFaultMask & (1 << handle)) {
        return CanTxStatus::Failed;
    }

    const TxEntry entry = frameToEntry(&txFrame);
    if (xQueueSend(tx_ring[handle], &entry, 0) != pdTRUE) {
        canDriverStats.txRingFull = canDriverStats.txRingFull + 1;
        return CanTxStatus::Busy;
    }
    txKick(handle);
    return CanTxStatus::Ok;
}

void CH32CAN::flushTxHandle(CanTxHandle handle)
{
    // The default ring is shared with unrelated traffic
    if (handle == CAN_TX_HANDLE_DEFAULT || handle >= CAN_TX_RING_COUNT || !tx_ring[handle]) {
        return;
    }
    taskENTER_CRITICAL();
    xQueueReset(tx_ring[handle]);
    if (!txMailboxEmpty(handle)) {
        // Write only the abort bit: the other flags are write-one-to-clear
        CAN1->TSTATR = txMailboxFlag(CAN_TSTATR_ABRQ0, handle);
    }
    taskEXIT_CRITICAL();
}

bool CH32CAN::takeTxFault(CanTxHandle handle)
{
    if (handle == CAN_TX_HANDLE_DEFAULT || handle >= CAN_TX_RING_COUNT) {
        return false;
    }
    taskENTER_CRITICAL();
    const bool faulted = txFaultMask & (1 << handle);
    txFaultMask &= ~(1 << handle);
    taskEXIT_CRITICAL();
    return faulted;
}

void CH32CAN::serviceTxTimeouts()
{
    const TickType_t now = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(CAN_TX_MAILBOX_TIMEOUT_MS);
    uint8_t aborted = 0;

    // A frame that merely waits for arbitration on a busy bus is not stuck:
    // aborting it would break a healthy transfer. Only a bus that carries no
    // frames at all (nobody acknowledges, the node can not synchronise) makes
    // a pending mailbox a failure; a frame that waits very long anyway is
    // given up as well.
    const bool busSilent = now - canLastTrafficTick >= timeout;

    for (uint8_t mb = 0; mb < CAN_TX_RING_COUNT; ++mb) {
        taskENTER_CRITICAL();
        const TickType_t pending = now - txMailboxLoadedTick[mb];
        if (!txMailboxEmpty(mb) && ((busSilent && pending >= timeout) || pending >= 4 * timeout)) {
            // Auto-retransmission retries a frame nobody acknowledges forever
            // and would keep the mailbox (and the stream behind it) stuck.
            CAN1->TSTATR = txMailboxFlag(CAN_TSTATR_ABRQ0, mb);
            txMailboxLoadedTick[mb] = now;
            if (mb != CAN_TX_HANDLE_DEFAULT && tx_ring[mb]) {
                // A gap in the stream: whatever follows is stale
                txFaultMask |= 1 << mb;
                xQueueReset(tx_ring[mb]);
            }
            aborted |= 1 << mb;
            txAbortCount = txAbortCount + 1;
        }
        taskEXIT_CRITICAL();
    }

    #ifdef SPDLOG_DEBUG
    if (aborted) {
        // Short on purpose: the ISO-TP log sink keeps only 64 characters per entry, prefix included
        logger->warn(SPDLOG_FMT_STRING("CAN txto{:x} E{:08x}"), (int)aborted, (unsigned)CAN1->ERRSR);
    }
    #endif
}

void CH32CAN::setRXBufferSize(int newSize)
{
    rxBufferSize = newSize;
}

void CH32CAN::setTXBufferSize(int newSize)
{
    txBufferSize = newSize;
}

void CH32CAN::watchForList(uint8_t mailbox, uint32_t id1, bool rtr1, uint32_t id2, bool rtr2) {
    uint32_t fullFilterId1 = (id1 << 3) | (rtr1 ? 0x2 : 0) | 0x4;
    uint32_t fullFilterId2 = (id2 << 3) | (rtr2 ? 0x2 : 0) | 0x4;
    CAN_FilterInitTypeDef filter {
        .CAN_FilterIdHigh = (uint16_t) (fullFilterId1 >> 16),
        .CAN_FilterIdLow  = (uint16_t) (fullFilterId1 & 0x0000FFFF),
        .CAN_FilterMaskIdHigh = (uint16_t) (fullFilterId2 >> 16),
        .CAN_FilterMaskIdLow  = (uint16_t) (fullFilterId2 & 0x0000FFFF),
        .CAN_FilterFIFOAssignment = ((mailbox & 1) ? CAN_Filter_FIFO1 : CAN_Filter_FIFO0), // Split between mailboxes to get more FIFO size
        .CAN_FilterNumber = mailbox,
        .CAN_FilterMode = CAN_FilterMode_IdList,
        .CAN_FilterScale = CAN_FilterScale_32bit,
        .CAN_FilterActivation = ENABLE,
    };
    CAN_FilterInit(&filter);
    filterIsConfigured[mailbox] = true;
}

void CH32CAN::watchForList(uint8_t mailbox, uint16_t id1, bool rtr1, uint16_t id2, bool rtr2, uint16_t id3, bool rtr3, uint16_t id4, bool rtr4) {
    uint16_t fullFilterId1 = (id1 << 5) | (rtr1 ? 0x10 : 0);
    uint16_t fullFilterId2 = (id2 << 5) | (rtr2 ? 0x10 : 0);
    uint16_t fullFilterId3 = (id3 << 5) | (rtr3 ? 0x10 : 0);
    uint16_t fullFilterId4 = (id4 << 5) | (rtr4 ? 0x10 : 0);
    CAN_FilterInitTypeDef filter {
        .CAN_FilterIdHigh = fullFilterId3,
        .CAN_FilterIdLow  = fullFilterId1,
        .CAN_FilterMaskIdHigh = fullFilterId4,
        .CAN_FilterMaskIdLow  = fullFilterId2,
        .CAN_FilterFIFOAssignment = ((mailbox & 1) ? CAN_Filter_FIFO1 : CAN_Filter_FIFO0), // Split between mailboxes to get more FIFO size
        .CAN_FilterNumber = mailbox,
        .CAN_FilterMode = CAN_FilterMode_IdList,
        .CAN_FilterScale = CAN_FilterScale_16bit,
        .CAN_FilterActivation = ENABLE,
    };
    CAN_FilterInit(&filter);
    filterIsConfigured[mailbox] = true;
}

void CH32CAN::watchForMask(uint8_t mailbox, uint32_t id, uint32_t mask, bool rtr, bool rtrMask, bool onlyExtendedId) {
    uint32_t fullFilterId   = (id   << 3) | (rtr     ? 0x2 : 0) | 0x4;
    uint32_t fullFilterMask = (mask << 3) | (rtrMask ? 0x2 : 0) | (onlyExtendedId ? 0x4 : 0);
    CAN_FilterInitTypeDef filter {
        .CAN_FilterIdHigh = (uint16_t) (fullFilterId >> 16),
        .CAN_FilterIdLow  = (uint16_t) (fullFilterId & 0x0000FFFF),
        .CAN_FilterMaskIdHigh = (uint16_t) (fullFilterMask >> 16),
        .CAN_FilterMaskIdLow  = (uint16_t) (fullFilterMask & 0x0000FFFF),
        .CAN_FilterFIFOAssignment = ((mailbox & 1) ? CAN_Filter_FIFO1 : CAN_Filter_FIFO0), // Split between mailboxes to get more FIFO size
        .CAN_FilterNumber = mailbox,
        .CAN_FilterMode = CAN_FilterMode_IdMask,
        .CAN_FilterScale = CAN_FilterScale_32bit,
        .CAN_FilterActivation = ENABLE,
    };
    CAN_FilterInit(&filter);
    filterIsConfigured[mailbox] = true;
}

void CH32CAN::watchForMask(uint8_t mailbox, uint16_t id1, uint16_t mask1, bool rtr1, bool rtrMask1,
                           uint16_t id2, uint16_t mask2, bool rtr2, bool rtrMask2) {
    uint16_t fullFilterId1   = (id1   << 5) | (rtr1     ? 0x10 : 0);
    uint16_t fullFilterMask1 = (mask1 << 5) | (rtrMask1 ? 0x10 : 0) | 0x8;
    uint16_t fullFilterId2   = (id2   << 5) | (rtr2     ? 0x10 : 0);
    uint16_t fullFilterMask2 = (mask2 << 5) | (rtrMask2 ? 0x10 : 0) | 0x8;
    CAN_FilterInitTypeDef filter {
        .CAN_FilterIdHigh = fullFilterId2,
        .CAN_FilterIdLow  = fullFilterId1,
        .CAN_FilterMaskIdHigh = fullFilterMask2,
        .CAN_FilterMaskIdLow  = fullFilterMask1,
        .CAN_FilterFIFOAssignment = ((mailbox & 1) ? CAN_Filter_FIFO1 : CAN_Filter_FIFO0), // Split between mailboxes to get more FIFO size
        .CAN_FilterNumber = mailbox,
        .CAN_FilterMode = CAN_FilterMode_IdMask,
        .CAN_FilterScale = CAN_FilterScale_16bit,
        .CAN_FilterActivation = ENABLE,
    };
    CAN_FilterInit(&filter);
    filterIsConfigured[mailbox] = true;
}

int CH32CAN::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended)
{
    if (mailbox >= BI_NUM_FILTERS) {
        return -1;
    }

    watchForMask(mailbox, id, mask);
    return mailbox;
}

int CH32CAN::_setFilter(uint32_t id, uint32_t mask, bool extended)
{
    int i;
    for (i = 0; i < BI_NUM_FILTERS; ++i) {
        if (!filterIsConfigured[i]) {
            if (_setFilterSpecific(i, id, mask, extended) >= 0)
                return i;
        }
    }
    
    #ifdef SPDLOG_DEBUG
    logger->debug(SPDLOG_FMT_STRING("Filter {:x} was not set"), (int)id);
    #endif

    return -1;
}

uint32_t CH32CAN::init(uint32_t ul_baudrate)
{
    #ifdef SPDLOG_DEBUG
    logger = spdlog::get("CAN");
    if(logger == nullptr)
        logger = spdlog::default_logger();
    #endif

    GPIO_InitTypeDef      GPIO_InitStructure = {0};

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Enable 5V pull-up (required for 5V CAN Transceivers that have no internal pull-up)
    // EXTEN->EXTEN_CTR = (EXTEN->EXTEN_CTR & (~EXTEN_USBD_LS) | EXTEN_USBD_PU_EN);

    NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN1_RX1_IRQn);
    NVIC_EnableIRQ(CAN1_SCE_IRQn);
    
    set_baudrate(ul_baudrate);

    readyForTraffic = true;
    return ul_baudrate;
}

uint32_t CH32CAN::beginAutoSpeed()
{
    CANTimingConfig_t oldMode = currentTimingConfig;

    readyForTraffic = false;
    
    bool oldListenMode = listenOnlyMode;
    setListenOnlyMode(true);

    int idx = 0;
    while (valid_timings[idx].speed != 0)
    {
        disable();
        currentTimingConfig = valid_timings[idx].cfg;

        #ifdef SPDLOG_DEBUG
        logger->info(SPDLOG_FMT_STRING("Autospeed, trying {}"), (int) valid_timings[idx].speed);
        #endif

        enable();
        delay(600); //wait a while
        if (cyclesSinceTraffic < 2) //only would happen if there had been traffic
        {
            setListenOnlyMode(oldListenMode);
            #ifdef SPDLOG_DEBUG
            logger->info("Autospeed success");
            #endif
            return valid_timings[idx].speed;
        }
        else
        {
            currentTimingConfig = oldMode;
            #ifdef SPDLOG_DEBUG
            logger->info("Autospeed failed");
            #endif
        }
        idx++;
    }

    #ifdef SPDLOG_DEBUG
    logger->info("Could not complete autospeed");
    #endif

    disable();
    return 0;
}

uint32_t CH32CAN::set_baudrate(uint32_t ul_baudrate)
{
    disable();

    //now try to find a valid timing to use
    int idx = 0;
    while (valid_timings[idx].speed != 0)
    {
        if (valid_timings[idx].speed == ul_baudrate)
        {
            currentTimingConfig = valid_timings[idx].cfg;

            enable();

            return ul_baudrate;
        }
        idx++;
    }

    #ifdef SPDLOG_DEBUG
    logger->error("Invalid bit timing specified");
    #endif
    return 0;
}

void CH32CAN::setListenOnlyMode(bool state)
{
    disable();
    listenOnlyMode = state;
    enable();
}

void CH32CAN::setNoACKMode(bool state)
{
}

void CH32CAN::enable()
{
    rx_queue = xQueueCreate(rxBufferSize, sizeof(RxEntry));
    for (uint8_t ring = 0; ring < CAN_TX_RING_COUNT; ++ring) {
        // Ring 0 also carries flow control frames, which can not be retried by anybody
        tx_ring[ring] = xQueueCreate(ring == CAN_TX_HANDLE_DEFAULT ? CAN_TX_DEFAULT_RING_DEPTH : txBufferSize, sizeof(TxEntry));
        txMailboxLoadedTick[ring] = xTaskGetTickCount();
    }
    txFaultMask = 0;
    
#if !defined(CAN_RX_ON_MAIN_LOOP)
#if configSUPPORT_STATIC_ALLOCATION
    CAN_Rx_handler_task = xTaskCreateStatic(CAN_Rx_handler, "CAN_RX", sizeof(CAN_Rx_task_stack) / sizeof(StackType_t),
        this, configMAX_PRIORITIES - 1, CAN_Rx_task_stack, &CAN_Rx_task);
#else
    xTaskCreate(CAN_Rx_handler, "CAN_RX", 256, this, configMAX_PRIORITIES - 1, &CAN_Rx_handler_task);
#endif
#endif
    
    if (!configureController()) {
        #ifdef SPDLOG_DEBUG
        logger->info("Failed to setup CAN");
        #endif
        return;
    }
    CAN_SlaveStartBank(28);         // all 28 banks belong to CAN1
    CAN1->FCTLR |= 0x1;             // enter filter init mode
    CAN1->FWR = 0;                  // reset all filters
    CAN1->FCTLR &= ~(uint32_t)0x1;  // exit filter init mode
    for (size_t i = 0; i < BI_NUM_FILTERS; ++i) {
        filterIsConfigured[i] = false;
    }

    canNeedsBusReset = false;
    canLastTrafficTick = xTaskGetTickCount();
    lastRecoveryTick = canLastTrafficTick;
    recoveryBackoffMs = CAN_RECOVERY_BACKOFF_MIN_MS;
    
    readyForTraffic = true;
}

bool CH32CAN::configureController()
{
    CAN_InitTypeDef CAN_InitStructure = {
        .CAN_Prescaler = currentTimingConfig.brp,
        .CAN_Mode = CAN_Mode_Normal,
        .CAN_SJW = currentTimingConfig.sjw,
        .CAN_BS1 = currentTimingConfig.tseg_1,
        .CAN_BS2 = currentTimingConfig.tseg_2,
        .CAN_TTCM = DISABLE,    // Don't use TTCAN
        .CAN_ABOM = DISABLE,    // Manually reset bus after error
        .CAN_AWUM = DISABLE,    // Manually wake-up from sleep
        // .CAN_NART = ENABLE,     // Don't resend messages automatically
        .CAN_NART = DISABLE,     // Resend messages automatically (lost arbitration is retried by hardware); mailbox timeouts are handled in serviceTxTimeouts()
        .CAN_RFLM = ENABLE,     // When the receiving FIFO overflows, don't receive new messages
        .CAN_TXFP = ENABLE,    // Mailboxes are sent in the order they were requested (not by identifier), so streams on different mailboxes share the bus fairly
    };

    // CAN_Init returns failure also when leaving init mode times out (no 11
    // recessive bits on a noisy bus); the controller is configured then and
    // joins the bus on its own. Only a missing BRP means init never happened.
    bool ok = CAN_Init(CAN1, &CAN_InitStructure) == CAN_InitStatus_Success;
    if (!ok && (CAN1->BTIMR & 0x3FF) != (uint32_t)(currentTimingConfig.brp - 1)) {
        return false;
    }

    CAN_DBGFreeze(CAN1, DISABLE);
    CAN_WakeUp(CAN1);

    CAN_ITConfig(CAN1, CAN_IT_BOF | CAN_IT_SLK | CAN_IT_EPV | CAN_IT_ERR | CAN_IT_FMP0 | CAN_IT_FMP1 | CAN_IT_FF0 | CAN_IT_FF1 | CAN_IT_TME, ENABLE);
    if (debuggingMode)
    {
        CAN_ITConfig(CAN1, CAN_IT_LEC | CAN_IT_ERR | CAN_IT_FOV0 | CAN_IT_FOV1, ENABLE);
    }
    return true;
}

void CH32CAN::checkBusHealth()
{
    if (!readyForTraffic) {
        return;
    }

    serviceTxTimeouts();

    // Sample traffic first: an ISR can advance it between these two reads.

    const TickType_t lastTrafficTick = canLastTrafficTick;
    TickType_t now = xTaskGetTickCount();
    uint32_t errsr = CAN1->ERRSR;
    uint8_t lastError = (errsr & CAN_ERRSR_LEC) >> 4;

    if (lastError != CAN_ErrorCode_NoErr)
    {
        // LEC is sticky until the next successful frame; clear it so that
        // only new errors are reported
        CAN1->ERRSR &= ~CAN_ERRSR_LEC;
        ++suppressedErrorCount;
        if (now - lastErrorLogTick >= pdMS_TO_TICKS(CAN_ERROR_LOG_INTERVAL_MS)) {
            #ifdef SPDLOG_DEBUG
            logger->warn(SPDLOG_FMT_STRING("CAN err{:x} x{} T{} R{}"),
                (int)lastError, suppressedErrorCount, (int)((errsr >> 16) & 0xFF), (int)((errsr >> 24) & 0xFF));
            #endif
            lastErrorLogTick = now;
            suppressedErrorCount = 0;
        }
    }

    TickType_t sinceTraffic = now - lastTrafficTick;
    if (sinceTraffic < pdMS_TO_TICKS(CAN_STALL_TIMEOUT_MS)) {
        recoveryBackoffMs = CAN_RECOVERY_BACKOFF_MIN_MS;
    }

    const uint32_t allEmpty = CAN_TSTATR_TME0 | CAN_TSTATR_TME1 | CAN_TSTATR_TME2;
    bool busOff = errsr & CAN_ERRSR_BOFF;
    bool errorPassive = errsr & CAN_ERRSR_EPVF;
    bool txPending = (CAN1->TSTATR & allEmpty) != allEmpty;
    bool stalled = (errorPassive || txPending) && sinceTraffic >= pdMS_TO_TICKS(CAN_STALL_TIMEOUT_MS);

    if (!(canNeedsBusReset || busOff || stalled)) {
        return;
    }
    if (now - lastRecoveryTick < pdMS_TO_TICKS(recoveryBackoffMs)) {
        return;
    }

    #ifdef SPDLOG_DEBUG
    logger->warn(SPDLOG_FMT_STRING("CAN stuck E{:08x} T{:08x}"), errsr, CAN1->TSTATR);
    #endif

    if (!recoverController()) {
        #ifdef SPDLOG_DEBUG
        logger->error("CAN controller reset failed");
        #endif
    }

    lastRecoveryTick = xTaskGetTickCount();
    recoveryBackoffMs = std::min<uint32_t>(recoveryBackoffMs * 2, CAN_RECOVERY_BACKOFF_MAX_MS);
}

bool CH32CAN::recoverController()
{
    // Full peripheral reset: an INRQ cycle or a wake-up does not reliably bring
    // the controller out of a stuck error state. Filters live in the same
    // peripheral, so preserve them across the reset. The backup is taken from
    // the heap only for the duration of the reset (task stacks are too small).
    std::unique_ptr<CAN_FilterRegister_TypeDef[]> savedBanks(
        new (std::nothrow) CAN_FilterRegister_TypeDef[BI_NUM_FILTERS]);

    NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
    NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_DisableIRQ(CAN1_RX1_IRQn);
    NVIC_DisableIRQ(CAN1_SCE_IRQn);
    vTaskSuspendAll();

    // Frames sitting in the mailboxes are lost with the reset. That leaves a
    // gap in the ordered streams that used them: whatever follows is stale.
    for (uint8_t mb = 1; mb < CAN_TX_RING_COUNT; ++mb) {
        if (tx_ring[mb] && !txMailboxEmpty(mb)) {
            txFaultMask |= 1 << mb;
            xQueueReset(tx_ring[mb]);
        }
    }

    bool ok;
    const bool fullReset = savedBanks != nullptr;
    if (fullReset) {
        uint32_t fctlr = CAN1->FCTLR;
        uint32_t fmcfgr = CAN1->FMCFGR;
        uint32_t fscfgr = CAN1->FSCFGR;
        uint32_t fafifor = CAN1->FAFIFOR;
        uint32_t fwr = CAN1->FWR;
        for (size_t i = 0; i < BI_NUM_FILTERS; ++i) {
            savedBanks[i].FR1 = CAN1->sFilterRegister[i].FR1;
            savedBanks[i].FR2 = CAN1->sFilterRegister[i].FR2;
        }

        // Pending mailboxes are lost with the reset
        CAN_DeInit(CAN1);
        ok = configureController();

        CAN1->FCTLR = fctlr | 0x1;      // enter filter init mode (keeps slave start bank)
        CAN1->FWR = 0;
        CAN1->FMCFGR = fmcfgr;
        CAN1->FSCFGR = fscfgr;
        CAN1->FAFIFOR = fafifor;
        for (size_t i = 0; i < BI_NUM_FILTERS; ++i) {
            CAN1->sFilterRegister[i].FR1 = savedBanks[i].FR1;
            CAN1->sFilterRegister[i].FR2 = savedBanks[i].FR2;
        }
        CAN1->FWR = fwr;
        CAN1->FCTLR &= ~(uint32_t)0x1;  // exit filter init mode
    } else {
        // No memory for the filter backup: fall back to re-initialising the
        // controller without a peripheral reset, which leaves filters intact
        for (uint8_t mb = 0; mb < 3; ++mb) {
            CAN_CancelTransmit(CAN1, mb);
        }
        ok = configureController();
    }

    canNeedsBusReset = false;
    canLastTrafficTick = xTaskGetTickCount();

    xTaskResumeAll();
    NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN1_RX1_IRQn);
    NVIC_EnableIRQ(CAN1_SCE_IRQn);

    savedBanks.reset();

    #ifdef SPDLOG_DEBUG
    if (!fullReset) {
        logger->warn("CAN reset without peripheral reset: no memory for filter backup");
    }
    #endif

    // Mailboxes are empty now; restart the transmit rings
    for (uint8_t ring = 0; ring < CAN_TX_RING_COUNT; ++ring) {
        txKick(ring);
    }

    return ok;
}

void CH32CAN::disable()
{
    readyForTraffic = false;

    CAN_DeInit(CAN1);

#if !defined(CAN_RX_ON_MAIN_LOOP)
    if (CAN_Rx_handler_task != NULL)
    {
        vTaskDelete(CAN_Rx_handler_task);
        CAN_Rx_handler_task = NULL;
    }
#endif

    if (rx_queue) {
        vQueueDelete(rx_queue);
        rx_queue = NULL;
    }
    for (uint8_t ring = 0; ring < CAN_TX_RING_COUNT; ++ring) {
        if (tx_ring[ring]) {
            QueueHandle_t queue = tx_ring[ring];
            tx_ring[ring] = NULL;
            vQueueDelete(queue);
        }
    }
}

bool CH32CAN::processFrame(CAN_FRAME &frame, uint8_t filter_id)
{
    CANListener *thisListener;

    cyclesSinceTraffic = 0; //reset counter to show that we are receiving traffic

    if (!readyForTraffic) {
        return false;
    }

    //frame is accepted, lets see if it matches a mailbox callback
    if (cbCANFrame[filter_id])
    {
        (*cbCANFrame[filter_id])(&frame);
    }
    
    for (int listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++)
    {
        thisListener = listener[listenerPos];
        if (thisListener != NULL)
        {
            if (thisListener->isCallbackActive(filter_id)) 
            {
                thisListener->gotFrame(&frame, filter_id);
                return true;
            }
            else if (thisListener->isCallbackActive(numFilters)) //global catch-all 
            {
                thisListener->gotFrame(&frame, 0xFF);
                return true;
            }
        }
    }
    
    if (cbGeneral)
    {
        (*cbGeneral)(&frame);
    }

    #ifdef SPDLOG_DEBUG
    SPDLOG_LOGGER_TRACE(logger, SPDLOG_FMT_STRING("Frame processed at filter {:x}"), (int) filter_id);
    #endif

    return true;
}

bool CH32CAN::sendFrame(CAN_FRAME const &txFrame)
{
    if (!readyForTraffic || !tx_ring[CAN_TX_HANDLE_DEFAULT]) return false;
    const TxEntry entry = frameToEntry(&txFrame);
    if (xQueueSend(tx_ring[CAN_TX_HANDLE_DEFAULT], &entry, pdMS_TO_TICKS(4)) != pdTRUE) {
        canDriverStats.txSendFrameFails = canDriverStats.txSendFrameFails + 1;
        return false;
    }
    txKick(CAN_TX_HANDLE_DEFAULT);
    return true;

}

bool CH32CAN::rx_avail()
{
    if (!rx_queue) return false;
    return uxQueueMessagesWaiting(rx_queue) > 0?true:false;
}

uint16_t CH32CAN::available()
{
    if (!rx_queue) return 0;
    return uxQueueMessagesWaiting(rx_queue);
}

uint32_t CH32CAN::get_rx_buff(CAN_FRAME &msg)
{
    RxEntry entry;
    //receive next CAN frame from queue
    if(xQueueReceive(rx_queue,&entry, portMAX_DELAY) == pdTRUE)
    {
        msg = entryToFrame(entry); //do a copy in the case that the receive worked
        return true;
    }
    return false; //otherwise we leave the msg variable alone and just return false
}
