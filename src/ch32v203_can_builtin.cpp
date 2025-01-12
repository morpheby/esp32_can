/*
  CH32_CAN.cpp - Library for CH32V203 built-in CAN module
    Original library for ESP32 reworked to support CH32v20x chips with Arduino and FreeRTOS
  
  Author: Ilya Mikhaltsou
  
  Created: 12.01.2025
*/

#include "Arduino.h"
#include "ch32v203_can_builtin.h"
#include "ch32v20x_can.h"

typedef struct
{
    CANTimingConfig_t cfg;
    uint32_t speed;
} VALID_TIMING;

// FIXME: Hardcoded value
#define CAN_CLOCK_FREQ SYSCLK_FREQ_144MHz_HSI

#define CAN_BRP_FROM_RESOLUTION_HZ(freq) ((CAN_CLOCK_FREQ)/(freq) - 1)

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

static TaskHandle_t CAN_Rx_handler_task = NULL;
static TaskHandle_t CAN_Tx_handler_task = NULL;

static CanTxMsg mailboxMessages[3];
static uint16_t mailboxRetransmitCounter[3] = {0, 0, 0};

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
        memcpy(&msg->Data, &frame->data, frame->length);
    }
}

#if !(USE_TINYUSB)

// CAN doesn't work with USB, unless you have remap available. CH32V203K8T6 doesn't, so can't really test this.

__attribute__((interrupt)) void
USB_LP_CAN1_RX0_IRQHandler(void) {
    // CAN FIFO0
    static CanRxMsg msg;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    while (CAN_MessagePending(CAN1, CAN_FIFO0) && !xQueueIsQueueFullFromISR(rx_queue)) {
        uint16_t timestamp = CAN1->sFIFOMailBox->RXMDTR & CAN_RXMDT0R_TIME;
        CAN_Receive(CAN1, CAN_FIFO0, &msg);

        CAN_FRAME frame;
        frame.id = msg.StdId | msg.ExtId;
        frame.extended = msg.IDE == CAN_Id_Extended;
        frame.timestamp = timestamp;
        frame.length = msg.DLC;
        frame.rtr = msg.RTR == CAN_RTR_Remote;
        if (!frame.rtr) {
            memcpy(&frame.data, msg.Data, frame.length);
        }
        
        xQueueSendFromISR(rx_queue, &frame, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken != pdFALSE) { taskYIELD (); }
    }

    CAN_ClearFlag(CAN1, CAN_FLAG_FMP0 | CAN_FLAG_FF0 | CAN_FLAG_FOV0);
    CAN_ClearITPendingBit(CAN1,  CAN_IT_FMP0 | CAN_IT_FF0 | CAN_IT_FOV0);
}

__attribute__((interrupt)) void
CAN1_RX1_IRQHandler(void) {
    // CAN FIFO1
    static CanRxMsg msg;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    while (CAN_MessagePending(CAN1, CAN_FIFO1) && !xQueueIsQueueFullFromISR(rx_queue)) {
        uint16_t timestamp = CAN1->sFIFOMailBox->RXMDTR & CAN_RXMDT1R_TIME;
        CAN_Receive(CAN1, CAN_FIFO1, &msg);

        CAN_FRAME frame;
        frame.id = msg.StdId | msg.ExtId;
        frame.extended = msg.IDE == CAN_Id_Extended;
        frame.timestamp = timestamp;
        frame.length = msg.DLC;
        frame.rtr = msg.RTR == CAN_RTR_Remote;
        if (!frame.rtr) {
            memcpy(frame.data.bytes, msg.Data, frame.length);
        }
        
        xQueueSendFromISR(rx_queue, &frame, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken != pdFALSE) { taskYIELD (); }
    }

    CAN_ClearFlag(CAN1, CAN_FLAG_FMP1 | CAN_FLAG_FF1 | CAN_FLAG_FOV1);
    CAN_ClearITPendingBit(CAN1,  CAN_IT_FMP1 | CAN_IT_FF1 | CAN_IT_FOV1);
}

__attribute__((interrupt)) void
USB_HP_CAN1_TX_IRQHandler(void) {
    // CAN TX
    // Find an empty mailbox or a mailbox with a failure
    CanTxMsg mailboxMessages_new[3];
    uint16_t mailboxRetransmitCounter_new[3] = {0, 0, 0};

    CAN_FRAME frame;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    for (int i = 0; i < 3; ++i) {
        switch (CAN_TransmitStatus(CAN1, i)) {
        case CAN_TxStatus_Pending:
            // This one is full, ignore it
            mailboxMessages_new[i] = mailboxMessages[i];
            mailboxRetransmitCounter_new[i] = mailboxRetransmitCounter[i];
            continue;
            
        case CAN_TxStatus_Failed:
            // This queue has an error. Check what error it is and either discard or retransmit
            if(++mailboxRetransmitCounter[i] <= CAN_MAX_RETRANSMIT_COUNT) {
                uint8_t mb = CAN_TxStatus_NoMailBox;
                do {
                    mb = CAN_Transmit(CAN1, mailboxMessages + i);
                } while (mb == CAN_TxStatus_NoMailBox);
                mailboxMessages_new[mb] = mailboxMessages[i];
                mailboxRetransmitCounter_new[mb] = mailboxRetransmitCounter[i];
                continue;
            }
            // fallthrough
        case CAN_TxStatus_Ok:
            // This one has finished, get a new message from the queue and send it
            if (xQueueReceiveFromISR(tx_queue, &frame, &xHigherPriorityTaskWoken) == pdTRUE) {
                frameToMsg(mailboxMessages + i, &frame);
                uint8_t mb = CAN_TxStatus_NoMailBox;
                do {
                    mb = CAN_Transmit(CAN1, mailboxMessages + i);
                } while (mb == CAN_TxStatus_NoMailBox);
                mailboxMessages_new[mb] = mailboxMessages[i];
            } else {
                vTaskNotifyGiveFromISR(CAN_Tx_handler_task, &xHigherPriorityTaskWoken);
            }
            
            if (xHigherPriorityTaskWoken != pdFALSE) { taskYIELD (); }

            break;
        }
    }
    memcpy(mailboxMessages, mailboxMessages_new, 3 * sizeof(CanTxMsg));
    memcpy(mailboxRetransmitCounter, mailboxRetransmitCounter_new, 3 * sizeof(uint16_t));
    
    CAN_ClearFlag(CAN1, CAN_FLAG_RQCP0 | CAN_FLAG_RQCP1 | CAN_FLAG_RQCP2);
    CAN_ClearITPendingBit(CAN1,  CAN_IT_TME);
}

__attribute__((interrupt)) void
CAN1_SCE_IRQHandler(void) {
    // CAN Error
    if (CAN_GetFlagStatus(CAN1, CAN_FLAG_BOF) == SET) {
        canNeedsBusReset = true;
    }

    CAN_ClearFlag(CAN1, CAN_FLAG_EWG | CAN_FLAG_EPV | CAN_FLAG_BOF | CAN_FLAG_LEC);
    CAN_ClearITPendingBit(CAN1,  CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR);
}
#endif

CH32CAN::CH32CAN() : CAN_COMMON(BI_NUM_FILTERS) 
{
    rxBufferSize = CAN_RX_QUEUE_BUFFER_SIZE;

    readyForTraffic = false;
    cyclesSinceTraffic = 0;
}

void CAN_Rx_handler(void *pvParameters)
{
    CH32CAN* espCan = (CH32CAN*)pvParameters;
    const TickType_t xDelay = 200 / portTICK_PERIOD_MS;
    CAN_FRAME frame;

    for(;;)
    {
        if (xQueueReceive(rx_queue, &frame, xDelay) != pdTRUE) {
            espCan->cyclesSinceTraffic++;
            
            uint8_t lastError = CAN_GetLastErrorCode(CAN1);

            if (lastError != CAN_ErrorCode_NoErr)
            {
                printf("Errors detected on bus: %hu", lastError);
            } else if (canNeedsBusReset) {
                espCan->cyclesSinceTraffic = 0;
                espCan->readyForTraffic = false;
                // Reinitialize bus
                // TODO:
                // if (twai_initiate_recovery() != ESP_OK)
                {
                    printf("Could not initiate bus recovery!\n");
                }
            }
            continue;
        }
        espCan->processFrame(frame, frame.fid >> 2);
    }
}
void CAN_Tx_handler(void *pvParameters)
{
    // CH32CAN* espCan = (CH32CAN*)pvParameters;
    CAN_FRAME frame;
    
    for(;;)
    {   
        if ((CAN1->TSTATR & (CAN_TSTATR_TME0 | CAN_TSTATR_TME1 | CAN_TSTATR_TME2)) == (CAN_TSTATR_TME0 | CAN_TSTATR_TME1 | CAN_TSTATR_TME2)) {    
            if (xQueueReceive(rx_queue, &frame, 0) != pdTRUE) {
                continue;
            }

            // Jump-start queue
            CanTxMsg mailboxMessage;
            frameToMsg(&mailboxMessage, &frame);

            // We are 100% certain this is going to be the first mailbox,
            // and to prevent races we want to set it before queue has started
            mailboxMessages[0] = mailboxMessage;
            mailboxRetransmitCounter[0] = 0;
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

int CH32CAN::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended)
{
    if (mailbox >= BI_NUM_FILTERS)
        return -1;

    bool isListFilter = (extended && mask >= 0x1FFFFFFF) || (!extended && mask >= 0x7FF);

    // Detect possible configurations
    // int numberOfFiltersConfiguredInSlot = (int) filterIsConfigured[mailbox ^ 1] + filterIsConfigured[mailbox ^ 2] + filterIsConfigured[mailbox ^ 3];
    bool noFiltersAreMaskTypeInSlot = filterIsListMode[mailbox ^ 1] || !filterIsConfigured[mailbox ^ 1] || 
                                      filterIsListMode[mailbox ^ 2] || !filterIsConfigured[mailbox ^ 2] ||
                                      filterIsListMode[mailbox ^ 3] || !filterIsConfigured[mailbox ^ 3];
    bool noFiltersAre32BitTypeInSlot = filterIs16bit[mailbox ^ 1] || !filterIsConfigured[mailbox ^ 1] || 
                                       filterIs16bit[mailbox ^ 2] || !filterIsConfigured[mailbox ^ 2] ||
                                       filterIs16bit[mailbox ^ 3] || !filterIsConfigured[mailbox ^ 3];

    CAN_FilterInitTypeDef filter {
        .CAN_FilterIdHigh = (uint16_t) ((CAN1->sFilterRegister[mailbox >> 2].FR1) >> 16),
        .CAN_FilterIdLow  = (uint16_t) ((CAN1->sFilterRegister[mailbox >> 2].FR1) & 0x0000FFFF),
        .CAN_FilterMaskIdHigh = (uint16_t) ((CAN1->sFilterRegister[mailbox >> 2].FR2) >> 16),
        .CAN_FilterMaskIdLow  = (uint16_t) ((CAN1->sFilterRegister[mailbox >> 2].FR2) & 0x0000FFFF),
        .CAN_FilterFIFOAssignment = ((mailbox & 4) ? CAN_Filter_FIFO1 : CAN_Filter_FIFO0), // Split between mailboxes to get more FIFO size
        .CAN_FilterNumber = (uint8_t) (mailbox >> 2),
        .CAN_FilterMode = CAN_FilterMode_IdList,
        .CAN_FilterScale = CAN_FilterScale_16bit,
        .CAN_FilterActivation = ENABLE,
    };

    if (noFiltersAreMaskTypeInSlot && isListFilter) {
        // List mode
        if (!extended && noFiltersAre32BitTypeInSlot) {
            // Adjacent filters are 16 bit or free, and we need to put a 16 bit filter in list mode

            switch (mailbox & 3) {
            case 0:
                filter.CAN_FilterIdLow = id;
                break;
            case 1:
                filter.CAN_FilterIdHigh = id;
                break;
            case 2:
                filter.CAN_FilterMaskIdLow = id;
                break;
            case 3:
                filter.CAN_FilterMaskIdHigh = id;
            }
            filter.CAN_FilterScale = CAN_FilterScale_16bit;
            filter.CAN_FilterMode = CAN_FilterMode_IdList;

            filterIsConfigured[mailbox] = true;
            filterIsListMode[mailbox] = true;
            filterIs16bit[mailbox] = true;

            CAN_FilterInit(&filter);
            return mailbox;
        } else if (!filterIs16bit[mailbox ^ 1] && !filterIs16bit[mailbox ^ 2] && !filterIs16bit[mailbox ^ 3]) {
            // We need to put a 32 bit filter in list mode. Even if it is 16 bit

            if (filterIsConfigured[mailbox ^ 1] && !filterIsConfigured[mailbox]) {
                // We are overwriting something we shouldn't
                return -1;
            }
            
            if (mailbox & 2) {
                filter.CAN_FilterIdLow = id & 0x0000FFFF;
                filter.CAN_FilterIdHigh = id >> 16;
            } else {
                filter.CAN_FilterMaskIdLow = id & 0x0000FFFF;
                filter.CAN_FilterMaskIdHigh = id >> 16;
            }
            filter.CAN_FilterScale = CAN_FilterScale_32bit;
            filter.CAN_FilterMode = CAN_FilterMode_IdList;
            
            filterIsConfigured[mailbox] = true;
            filterIsConfigured[mailbox ^ 1] = true;
            filterIsListMode[mailbox] = true;
            filterIsListMode[mailbox^1] = true;
            filterIs16bit[mailbox] = false;
            filterIs16bit[mailbox ^ 1] = false;

            CAN_FilterInit(&filter);
            return mailbox;
        }
    } else if (!filterIsListMode[mailbox ^ 1] && !filterIsListMode[mailbox ^ 2] && !filterIsListMode[mailbox ^ 3]) {
        // Mask mode. Add current one as if it is in mask mode (full mask)
        if (!extended && noFiltersAre32BitTypeInSlot) {
            // Adjacent filters are 16 bit or free, and we need to put a 16 bit filter in mask mode
            
            if ((filterIsConfigured[mailbox ^ 2] || filterIsConfigured[mailbox ^ 1]) && !filterIsConfigured[mailbox]) {
                // We are overwriting something we shouldn't
                return -1;
            }
            
            if (mailbox & 2) {
                filter.CAN_FilterIdLow = id;
                filter.CAN_FilterMaskIdLow = mask;
            } else {
                filter.CAN_FilterIdHigh = id;
                filter.CAN_FilterMaskIdHigh = mask;
            }

            filter.CAN_FilterScale = CAN_FilterScale_16bit;
            filter.CAN_FilterMode = CAN_FilterMode_IdMask;
            
            filterIsConfigured[mailbox] = true;
            filterIsConfigured[mailbox ^ 1] = true;
            filterIsListMode[mailbox] = false;
            filterIsListMode[mailbox^1] = false;
            filterIs16bit[mailbox] = false;
            filterIs16bit[mailbox ^ 1] = false;

            CAN_FilterInit(&filter);
            return mailbox;
        } else if (!filterIs16bit[mailbox ^ 1] && !filterIs16bit[mailbox ^ 2] && !filterIs16bit[mailbox ^ 3]) {
            // The whole slot is ours, so just fill it
            
            if ((filterIsConfigured[mailbox ^ 3] || filterIsConfigured[mailbox ^ 2] || filterIsConfigured[mailbox ^ 1]) && !filterIsConfigured[mailbox]) {
                // We are overwriting something we shouldn't
                return -1;
            }
            
            filter.CAN_FilterIdLow = id & 0x0000FFFF;
            filter.CAN_FilterIdHigh = id >> 16;
            filter.CAN_FilterMaskIdLow = mask & 0x0000FFFF;
            filter.CAN_FilterMaskIdHigh = mask >> 16;

            filterIsConfigured[mailbox] = true;
            filterIsConfigured[mailbox ^ 1] = true;
            filterIsConfigured[mailbox ^ 2] = true;
            filterIsConfigured[mailbox ^ 3] = true;
            filterIsListMode[mailbox] = isListFilter;
            filterIsListMode[mailbox^1] = isListFilter;
            filterIsListMode[mailbox^2] = isListFilter;
            filterIsListMode[mailbox^3] = isListFilter;
            filterIs16bit[mailbox] = false;
            filterIs16bit[mailbox ^ 1] = false;
            filterIs16bit[mailbox ^ 2] = false;
            filterIs16bit[mailbox ^ 3] = false;

            CAN_FilterInit(&filter);
            return mailbox;
        }
    } else if (!isListFilter && !extended && filterIsListMode[mailbox ^ 2] && filterIs16bit[mailbox ^ 2] &&
               !filterIsConfigured[mailbox ^ 1] && !filterIsConfigured[mailbox ^ 3]) {
        // Special case where we can promote one remaining filter to mask
        if (mailbox & 2) {
            filter.CAN_FilterIdLow = id;
            filter.CAN_FilterMaskIdLow = mask;
            filter.CAN_FilterMaskIdHigh = 0x7FF;
        } else {
            filter.CAN_FilterIdHigh = id;
            filter.CAN_FilterMaskIdHigh = mask;
            filter.CAN_FilterMaskIdLow = 0x7FF;
        }

        filter.CAN_FilterScale = CAN_FilterScale_16bit;
        filter.CAN_FilterMode = CAN_FilterMode_IdMask;
        
        filterIsConfigured[mailbox] = true;
        filterIsConfigured[mailbox ^ 1] = true;
        filterIsConfigured[mailbox ^ 3] = true;
        filterIsListMode[mailbox] = false;
        filterIsListMode[mailbox^1] = false;
        filterIsListMode[mailbox^3] = false;
        filterIs16bit[mailbox] = true;
        filterIs16bit[mailbox ^ 1] = true;
        filterIs16bit[mailbox ^ 3] = true;
        
        CAN_FilterInit(&filter);
        return mailbox;
    }

    // Not a supported variant
    return -1;
}

int CH32CAN::_setFilter(uint32_t id, uint32_t mask, bool extended)
{
    int i;
    for (i = BI_NUM_FILTERS - 1; i >= 0 && !filterIsConfigured[i]; --i);
    while (++i < BI_NUM_FILTERS && _setFilterSpecific(i, id, mask, extended) < 0);

    if (i == BI_NUM_FILTERS) {
        if (debuggingMode) Serial.println("Could not set filter!");

        return -1;
    }

    return i;
}

uint32_t CH32CAN::init(uint32_t ul_baudrate)
{
    GPIO_InitTypeDef      GPIO_InitStructure = {0};
    // CAN_InitTypeDef       CAN_InitStructure = {0};
    // CAN_FilterInitTypeDef CAN_FilterInitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Enable 5V pull-up (required for 5V CAN Transceivers that have no internal pull-up)
    // EXTEN->EXTEN_CTR = (EXTEN->EXTEN_CTR & (~EXTEN_USBD_LS) | EXTEN_USBD_PU_EN);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    set_baudrate(ul_baudrate);
    
    CAN_ITConfig(CAN1, CAN_IT_BOF | CAN_IT_FMP0 | CAN_IT_FMP1 | CAN_IT_FF0 | CAN_IT_FF1 | CAN_IT_TME, ENABLE);
    if (debuggingMode)
    {
        CAN_ITConfig(CAN1, CAN_IT_EPV | CAN_IT_LEC | CAN_IT_ERR | CAN_IT_FOV0 | CAN_IT_FOV1, ENABLE);
    }
    
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
        Serial.print("Trying Speed ");
        Serial.print(valid_timings[idx].speed);
        enable();
        delay(600); //wait a while
        if (cyclesSinceTraffic < 2) //only would happen if there had been traffic
        {
            setListenOnlyMode(oldListenMode);
            Serial.println(" SUCCESS!");
            return valid_timings[idx].speed;
        }
        else
        {
            currentTimingConfig = oldMode;
            Serial.println(" FAILED.");
        }
        idx++;
    }
    Serial.println("None of the tested CAN speeds worked!");
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
    printf("Could not find a valid bit timing! You will need to add your desired speed to the library!\n");
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
    CAN_DBGFreeze(CAN1, state ? ENABLE : DISABLE);
}

void CH32CAN::enable()
{
    
    CAN_InitTypeDef CAN_InitStructure = {
        .CAN_Prescaler = currentTimingConfig.brp,
        .CAN_Mode = CAN_OperatingMode_Normal,
        .CAN_SJW = currentTimingConfig.sjw,
        .CAN_BS1 = currentTimingConfig.tseg_1,
        .CAN_BS2 = currentTimingConfig.tseg_2,
        .CAN_TTCM = DISABLE,    // Don't use TTCAN
        .CAN_ABOM = DISABLE,    // Manually reset bus after error
        .CAN_AWUM = DISABLE,    // Manually wake-up from sleep
        .CAN_NART = ENABLE,     // Don't resend messages automatically
        .CAN_RFLM = ENABLE,     // When the receiving FIFO overflows, don't receive new messages
        .CAN_TXFP = DISABLE,    // Send priority is determined by the message identifier
    };
    
    if (CAN_Init(CAN1, &CAN_InitStructure) != CAN_InitStatus_Success) {
        printf("Failed to setup CAN\n");
        return;
    }

    rx_queue = xQueueCreate(rxBufferSize, sizeof(CAN_FRAME));
    tx_queue = xQueueCreate(rxBufferSize, sizeof(CAN_FRAME));

    xTaskCreate(CAN_Tx_handler, "CAN_TX", 256, this, configMAX_PRIORITIES - 1, &CAN_Tx_handler_task);
    xTaskCreate(CAN_Rx_handler, "CAN_RX", 512, this, configMAX_PRIORITIES - 1, &CAN_Rx_handler_task);

    readyForTraffic = true;
}

void CH32CAN::disable()
{
    readyForTraffic = false;

    CAN_DeInit(CAN1);

    for (auto task : {CAN_Tx_handler_task, CAN_Rx_handler_task}) {
        if (task != NULL)
        {
            vTaskDelete(task);
            task = NULL;
        }
    }

    for (auto queue : {rx_queue, tx_queue}) {
        if (queue) {
            vQueueDelete(queue);
        }
    }
}

bool CH32CAN::processFrame(CAN_FRAME &frame, uint8_t filter_id)
{
    CANListener *thisListener;

    cyclesSinceTraffic = 0; //reset counter to show that we are receiving traffic

    //frame is accepted, lets see if it matches a mailbox callback
    if (cbCANFrame[filter_id])
    {
        (*cbCANFrame[filter_id])(&frame);
    }
    else if (cbGeneral)
    {
        (*cbGeneral)(&frame);
    }
    else
    {
        for (int listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++)
        {
            thisListener = listener[listenerPos];
            if (thisListener != NULL)
            {
                if (thisListener->isCallbackActive(filter_id)) 
                {
                    thisListener->gotFrame(&frame, filter_id);
                }
                else if (thisListener->isCallbackActive(numFilters)) //global catch-all 
                {
                    thisListener->gotFrame(&frame, 0xFF);
                    return true;
                }
            }
        }
    }

    if (debuggingMode) Serial.write('_');
    return true;
}

bool CH32CAN::sendFrame(CAN_FRAME& txFrame)
{
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
    if(xQueueReceive(rx_queue,&frame, 0) == pdTRUE)
    {
        msg = frame; //do a copy in the case that the receive worked
        return true;
    }
    return false; //otherwise we leave the msg variable alone and just return false
}

