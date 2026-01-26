/*
  CH32_CAN.cpp - Library for CH32V203 built-in CAN module
    Original library for ESP32 reworked to support CH32v20x chips with Arduino and FreeRTOS
  
  Author: Ilya Mikhaltsou
  
  Created: 12.01.2025
*/

#include "Arduino.h"
#include "ch32v203_can_builtin.h"
#include "ch32v20x_can.h"

#if __has_include("spdlog/spdlog.h")
#include "spdlog/spdlog.h"
#define SPDLOG_DEBUG
#endif

typedef struct
{
    CANTimingConfig_t cfg;
    uint32_t speed;
} VALID_TIMING;

// FIXME: Hardcoded value
#define CAN_CLOCK_FREQ (SYSCLK_FREQ_144MHz_HSI)/2

#define CAN_BRP_FROM_RESOLUTION_HZ(freq) ((CAN_CLOCK_FREQ)/(freq))

#define CAN_TIMING_CONFIG(quanta_hz, ts1, ts2, sjw_v) {.brp = CAN_BRP_FROM_RESOLUTION_HZ((quanta_hz)), .tseg_1 = (ts1) - 1, .tseg_2 = (ts2) - 1, .sjw = (sjw_v) - 1}

#define CAN_TIMING_CONFIG_NULL      CAN_TIMING_CONFIG(CAN_CLOCK_FREQ, 1, 1, 1)

#if CAN_BRP_FROM_RESOLUTION_HZ(16000) < 1024
// NOTE: Configuration not tested and not even calculated. Consider using bitcalc or other software to adjust
// #define CAN_TIMING_CONFIG_1KBITS        CAN_TIMING_CONFIG(16000, 13, 2, 1)
#endif

#if CAN_BRP_FROM_RESOLUTION_HZ(80000) < 1024
// NOTE: Configuration not tested and not even calculated. Consider using bitcalc or other software to adjust
// #define TWAI_TIMING_CONFIG_5KBITS       CAN_TIMING_CONFIG(80000, 13, 2, 1)
#endif

#if CAN_BRP_FROM_RESOLUTION_HZ(160000) < 1024
#define CAN_TIMING_CONFIG_10KBITS       CAN_TIMING_CONFIG(160000, 13, 2, 1)
#endif

#define CAN_TIMING_CONFIG_20KBITS       CAN_TIMING_CONFIG(320000, 13, 2, 1)
#define CAN_TIMING_CONFIG_50KBITS       CAN_TIMING_CONFIG(800000, 13, 2, 1)
#define CAN_TIMING_CONFIG_100KBITS      CAN_TIMING_CONFIG(1600000, 13, 2, 1)
#define CAN_TIMING_CONFIG_125KBITS      CAN_TIMING_CONFIG(2000000, 13, 2, 1)
#define CAN_TIMING_CONFIG_250KBITS      CAN_TIMING_CONFIG(4000000, 13, 2, 1)
#define CAN_TIMING_CONFIG_500KBITS      CAN_TIMING_CONFIG(8000000, 12, 3, 2)
#define CAN_TIMING_CONFIG_800KBITS      CAN_TIMING_CONFIG(16000000, 14, 5, 3)
#define CAN_TIMING_CONFIG_1MBITS        CAN_TIMING_CONFIG(16000000, 11, 4, 3)

static QueueHandle_t rx_queue;
static QueueHandle_t tx_queue;
static bool canNeedsBusReset = false;


#if configSUPPORT_STATIC_ALLOCATION
#if !defined(CAN_RX_ON_MAIN_LOOP)
static StaticTask_t CAN_Rx_task;
static StackType_t CAN_Rx_task_stack[128];
#endif
static StaticTask_t CAN_Tx_task;
static StackType_t CAN_Tx_task_stack[128];
#endif

#if !defined(CAN_RX_ON_MAIN_LOOP)
static TaskHandle_t CAN_Rx_handler_task = NULL;
#endif
static TaskHandle_t CAN_Tx_handler_task = NULL;

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

static void frameToMsg(CanTxMsg *msg, const CAN_FRAME *frame) {
    msg->IDE = frame->extended ? CAN_Id_Extended : CAN_Id_Standard;
    msg->StdId = frame->id;
    msg->ExtId = frame->id;
    msg->DLC = frame->length;
    msg->RTR = frame->rtr ? CAN_RTR_Remote : CAN_RTR_Data;
    if (!frame->rtr) {
        memcpy(msg->Data, frame->data.bytes, frame->length);
    }
}

#if !(USE_TINYUSB)

// CAN doesn't work with USB, unless you have remap available. CH32V203K8T6 doesn't, so can't really test this.

extern "C" {

ISR void USB_LP_CAN1_RX0_IRQHandler(void) {
    // CAN FIFO0
    static CanRxMsg msg;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (rx_queue) {
        while (CAN_MessagePending(CAN1, CAN_FIFO0) && !xQueueIsQueueFullFromISR(rx_queue)) {
            uint16_t timestamp = (CAN1->sFIFOMailBox[0].RXMDTR & CAN_RXMDT0R_TIME) >> 16;
            CAN_Receive(CAN1, CAN_FIFO0, &msg);

            CAN_FRAME frame;
            frame.id = (msg.IDE == CAN_Id_Extended) ? msg.ExtId : msg.StdId;
            frame.extended = msg.IDE == CAN_Id_Extended;
            frame.timestamp = timestamp;
            frame.length = msg.DLC;
            frame.rtr = msg.RTR == CAN_RTR_Remote;
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
            if (!frame.rtr) {
                memcpy(frame.data.bytes, msg.Data, frame.length);
            }
            
            xQueueSendFromISR(rx_queue, &frame, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken != pdFALSE) { taskYIELD (); }
        }
    }

    CAN_ClearFlag(CAN1, CAN_FLAG_FF0 | CAN_FLAG_FOV0);
    CAN_ClearITPendingBit(CAN1,  CAN_IT_FMP0 | CAN_IT_FF0 | CAN_IT_FOV0);
}

ISR void CAN1_RX1_IRQHandler(void) {
    // CAN FIFO1
    static CanRxMsg msg;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (rx_queue) {
        while (CAN_MessagePending(CAN1, CAN_FIFO1) && !xQueueIsQueueFullFromISR(rx_queue)) {
            uint16_t timestamp = (CAN1->sFIFOMailBox[1].RXMDTR & CAN_RXMDT1R_TIME) >> 16;
            CAN_Receive(CAN1, CAN_FIFO1, &msg);

            CAN_FRAME frame;
            frame.id = (msg.IDE == CAN_Id_Extended) ? msg.ExtId : msg.StdId;
            frame.extended = msg.IDE == CAN_Id_Extended;
            frame.timestamp = timestamp;
            frame.length = msg.DLC;
            frame.rtr = msg.RTR == CAN_RTR_Remote;
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
            if (!frame.rtr) {
                memcpy(frame.data.bytes, msg.Data, frame.length);
            }
            
            xQueueSendFromISR(rx_queue, &frame, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken != pdFALSE) { taskYIELD (); }
        }
    }

    CAN_ClearFlag(CAN1, CAN_FLAG_FF1 | CAN_FLAG_FOV1);
    CAN_ClearITPendingBit(CAN1,  CAN_IT_FMP1 | CAN_IT_FF1 | CAN_IT_FOV1);
}

ISR void USB_HP_CAN1_TX_IRQHandler(void) {
    // CAN TX
    // Find an empty mailbox or a mailbox with a failure
    // CanTxMsg mailboxMessages_new[3];
    // uint16_t mailboxRetransmitCounter_new[3] = {0, 0, 0};
    CanTxMsg mailboxMessage;

    CAN_FRAME frame;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (tx_queue) {
        for (int i = 0, j = 0; i < 3; ++i) {
            switch (CAN_TransmitStatus(CAN1, i)) {
            case CAN_TxStatus_Pending:
                // This one is full, ignore it
                // mailboxMessages_new[j] = mailboxMessages[i];
                // mailboxRetransmitCounter_new[j] = mailboxRetransmitCounter[i];
                break;
                
            case CAN_TxStatus_Failed:
            //     // This queue has an error. Check what error it is and either discard or retransmit
            //     if(mailboxRetransmitCounter[i] < CAN_MAX_RETRANSMIT_COUNT) {
            //         uint8_t mb = CAN_TxStatus_NoMailBox;
            //         do {
            //             mb = CAN_Transmit(CAN1, mailboxMessages + i);
            //         } while (mb == CAN_TxStatus_NoMailBox);
            //         mailboxMessages_new[mb] = mailboxMessages[i];
            //         mailboxRetransmitCounter_new[mb] = mailboxRetransmitCounter[i];
            //         Serial.printf("Retransmit: %u\n", mailboxRetransmitCounter_new[mb]);
            //         if (mb < i) {
            //             i = mb;
            //         }
            //         break;
            //     }
            // check why we get tons of dropped messages
                // Serial.println("Drop CAN message");
                // fallthrough
            case CAN_TxStatus_Ok:
                // This one has finished, get a new message from the queue and send it
                if (xQueueReceiveFromISR(tx_queue, &frame, &xHigherPriorityTaskWoken) == pdTRUE) {
                    frameToMsg(&mailboxMessage, &frame);
                    uint8_t mb = CAN_TxStatus_NoMailBox;
                    do {
                        mb = CAN_Transmit(CAN1, &mailboxMessage);
                    } while (mb == CAN_TxStatus_NoMailBox);
                    // mailboxMessages_new[mb] = mailboxMessages[i];
                    // if (mb < i) {
                    //     i = mb;
                    // }
                } else {
                    vTaskNotifyGiveFromISR(CAN_Tx_handler_task, &xHigherPriorityTaskWoken);
                }
                
                if (xHigherPriorityTaskWoken != pdFALSE) { taskYIELD (); }

                break;
            }
        }
        // memcpy(mailboxMessages, mailboxMessages_new, 3 * sizeof(CanTxMsg));
        // memcpy(mailboxRetransmitCounter, mailboxRetransmitCounter_new, 3 * sizeof(uint16_t));
    }
    
    CAN_ClearFlag(CAN1, CAN_FLAG_RQCP0 | CAN_FLAG_RQCP1 | CAN_FLAG_RQCP2);
    CAN_ClearITPendingBit(CAN1,  CAN_IT_TME);
}

ISR void CAN1_SCE_IRQHandler(void) {
    // CAN Error
    if (CAN_GetFlagStatus(CAN1, CAN_FLAG_BOF) == SET) {
        canNeedsBusReset = true;
    }
    if (CAN_GetFlagStatus(CAN1, CAN_FLAG_EPV) == SET) {
        canNeedsBusReset = true;
    }
    if (CAN_GetFlagStatus(CAN1, CAN_FLAG_SLAK) == SET) {
        canNeedsBusReset = true;
    }

    CAN_ClearFlag(CAN1, CAN_FLAG_EWG | CAN_FLAG_EPV | CAN_FLAG_BOF | CAN_FLAG_LEC | CAN_FLAG_SLAK);
    CAN_ClearITPendingBit(CAN1,  CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR | CAN_IT_WKU | CAN_IT_SLK);
}

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
    CAN_FRAME frame;

    auto espCan = this;

#else
void CAN_Rx_handler(void *pvParameters)
{
    CH32CAN* espCan = (CH32CAN*)pvParameters;
    const TickType_t xDelay = 200 / portTICK_PERIOD_MS;
    CAN_FRAME frame;

    for(;;)
    {
#endif
        if (xQueueReceive(rx_queue, &frame, xDelay) != pdTRUE) {
            espCan->cyclesSinceTraffic++;
            
            uint8_t lastError = CAN_GetLastErrorCode(CAN1);

            if (lastError != CAN_ErrorCode_NoErr)
            {
                #ifdef SPDLOG_DEBUG
                log->warn(FMT_STRING("CAN error detected on bus: {:x}"), (int)lastError);
                #endif
            }
            if (canNeedsBusReset) {
                espCan->cyclesSinceTraffic = 0;
                espCan->readyForTraffic = false;
                canNeedsBusReset = false;

                #ifdef SPDLOG_DEBUG
                log->warn("CAN bus will be reset");
                #endif

                // Clear bus flags
                CAN_ClearFlag(CAN1, CAN_FLAG_EWG | CAN_FLAG_EPV | CAN_FLAG_BOF | CAN_FLAG_LEC);
                
                CAN_InitTypeDef CAN_InitStructure = {
                    .CAN_Prescaler = espCan->currentTimingConfig.brp,
                    .CAN_Mode = CAN_Mode_Normal,
                    .CAN_SJW = espCan->currentTimingConfig.sjw,
                    .CAN_BS1 = espCan->currentTimingConfig.tseg_1,
                    .CAN_BS2 = espCan->currentTimingConfig.tseg_2,
                    .CAN_TTCM = DISABLE,    // Don't use TTCAN
                    .CAN_ABOM = DISABLE,    // Manually reset bus after error
                    .CAN_AWUM = DISABLE,    // Manually wake-up from sleep
                    // .CAN_NART = ENABLE,     // Don't resend messages automatically
                    .CAN_NART = DISABLE,     // Resend messages automatically
                    .CAN_RFLM = ENABLE,     // When the receiving FIFO overflows, don't receive new messages
                    .CAN_TXFP = ENABLE,    // Send priority is determined by the message identifier
                };

                if (CAN_Init(CAN1, &CAN_InitStructure) != CAN_InitStatus_Success) {
                    #ifdef SPDLOG_DEBUG
                    log->error("Failed to setup CAN");
                    #endif
                    return;
                }
                CAN_DBGFreeze(CAN1, DISABLE);
                CAN_WakeUp(CAN1);
                espCan->readyForTraffic = true;
            }
#if defined(CAN_RX_ON_MAIN_LOOP)
            return;
#else
            continue;
#endif
        }
        espCan->processFrame(frame, frame.fid);
#if defined(CAN_RX_ON_MAIN_LOOP)
    return;
}
#else
    }
}
#endif

void CAN_Tx_handler(void *pvParameters)
{
    // CH32CAN* espCan = (CH32CAN*)pvParameters;
    CAN_FRAME frame;
    
    for(;;)
    {
        if ((CAN1->TSTATR & (CAN_TSTATR_TME0 | CAN_TSTATR_TME1 | CAN_TSTATR_TME2)) == (CAN_TSTATR_TME0 | CAN_TSTATR_TME1 | CAN_TSTATR_TME2)) {
            if (xQueueReceive(tx_queue, &frame, portMAX_DELAY) != pdTRUE) {
                continue;
            }

            // Jump-start queue
            CanTxMsg mailboxMessage;
            frameToMsg(&mailboxMessage, &frame);

            // We are 100% certain this is going to be the first mailbox,
            // and to prevent races we want to set it before queue has started
            // mailboxMessages[0] = mailboxMessage;
            // mailboxRetransmitCounter[0] = 0;
            CAN_Transmit(CAN1, &mailboxMessage);
        } else {
            ulTaskNotifyTake(1, pdMS_TO_TICKS(100));
        }
    }
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
    log->debug(FMT_STRING("Filter {:x} was not set"), (int)id);
    #endif

    return -1;
}

uint32_t CH32CAN::init(uint32_t ul_baudrate)
{


    #ifdef SPDLOG_DEBUG
    auto log = spdlog::get("CAN");
    if(log == nullptr)
        log = spdlog::default_logger();
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
        log->info(FMT_STRING("Autospeed, trying {}"), (int) valid_timings[idx].speed);
        #endif

        enable();
        delay(600); //wait a while
        if (cyclesSinceTraffic < 2) //only would happen if there had been traffic
        {
            setListenOnlyMode(oldListenMode);
            #ifdef SPDLOG_DEBUG
            log->info("Autospeed success");
            #endif
            return valid_timings[idx].speed;
        }
        else
        {
            currentTimingConfig = oldMode;
            #ifdef SPDLOG_DEBUG
            log->info("Autospeed failed");
            #endif
        }
        idx++;
    }

    #ifdef SPDLOG_DEBUG
    log->info("Could not complete autospeed");
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
    log->error("Invalid bit timing specified");
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
        .CAN_NART = DISABLE,     // Resend messages automatically
        .CAN_RFLM = ENABLE,     // When the receiving FIFO overflows, don't receive new messages
        .CAN_TXFP = ENABLE,    // Send priority is determined by the message identifier
    };

    rx_queue = xQueueCreate(rxBufferSize, sizeof(CAN_FRAME));
    tx_queue = xQueueCreate(txBufferSize, sizeof(CAN_FRAME));
    
#if configSUPPORT_STATIC_ALLOCATION
    CAN_Tx_handler_task = xTaskCreateStatic(CAN_Tx_handler, "CAN_TX", sizeof(CAN_Tx_task_stack) / sizeof(StackType_t),
        this, configMAX_PRIORITIES - 1, CAN_Tx_task_stack, &CAN_Tx_task);

#if !defined(CAN_RX_ON_MAIN_LOOP)
    CAN_Rx_handler_task = xTaskCreateStatic(CAN_Rx_handler, "CAN_RX", sizeof(CAN_Tx_task_stack) / sizeof(StackType_t),
        this, configMAX_PRIORITIES - 1, CAN_Rx_task_stack, &CAN_Rx_task);
#endif
#else
    xTaskCreate(CAN_Tx_handler, "CAN_TX", 128, this, configMAX_PRIORITIES - 1, &CAN_Tx_handler_task);
#if !defined(CAN_RX_ON_MAIN_LOOP)
    xTaskCreate(CAN_Rx_handler, "CAN_RX", 256, this, configMAX_PRIORITIES - 1, &CAN_Rx_handler_task);
#endif
#endif
    
    if (CAN_Init(CAN1, &CAN_InitStructure) != CAN_InitStatus_Success) {
        #ifdef SPDLOG_DEBUG
        log->info("Failed to setup CAN");
        #endif
        return;
    }
    CAN_DBGFreeze(CAN1, DISABLE);
    CAN_WakeUp(CAN1);
    
    CAN_ITConfig(CAN1, CAN_IT_BOF | CAN_IT_SLK | CAN_IT_EPV | CAN_IT_ERR | CAN_IT_FMP0 | CAN_IT_FMP1 | CAN_IT_FF0 | CAN_IT_FF1 | CAN_IT_TME, ENABLE);
    if (debuggingMode)
    {
        CAN_ITConfig(CAN1, CAN_IT_LEC | CAN_IT_ERR | CAN_IT_FOV0 | CAN_IT_FOV1, ENABLE);
    }
    
    readyForTraffic = true;
}

void CH32CAN::disable()
{
    readyForTraffic = false;

    CAN_DeInit(CAN1);

    if (CAN_Tx_handler_task != NULL)
    {
        vTaskDelete(CAN_Tx_handler_task);
        CAN_Tx_handler_task = NULL;
    }
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
    if (tx_queue) {
        vQueueDelete(tx_queue);
        tx_queue = NULL;
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
    SPDLOG_LOGGER_TRACE(log, FMT_STRING("Frame processed at filter {:x}"), (int) filter_id);
    #endif

    return true;
}

bool CH32CAN::sendFrame(CAN_FRAME const &txFrame)
{
    if (!readyForTraffic) return false;
    return xQueueSend(tx_queue, &txFrame, pdMS_TO_TICKS(4)) == pdTRUE;
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
    CAN_FRAME frame;
    //receive next CAN frame from queue
    if(xQueueReceive(rx_queue,&frame, portMAX_DELAY) == pdTRUE)
    {
        msg = frame; //do a copy in the case that the receive worked
        return true;
    }
    return false; //otherwise we leave the msg variable alone and just return false
}

