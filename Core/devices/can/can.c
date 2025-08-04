#include "can.h"

can_message_t can_message_from_sep = {0};

extern FDCAN_HandleTypeDef hfdcan1;

FDCAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[12];
int indx = 0;
/// S

int can_init(void){
    FDCAN_FilterTypeDef can_filter_sep = {0};
    can_filter_sep.IdType = FDCAN_STANDARD_ID;
    can_filter_sep.FilterIndex = 0;
    can_filter_sep.FilterType = FDCAN_FILTER_MASK;
    can_filter_sep.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    can_filter_sep.FilterID1 = 0x100;
    can_filter_sep.FilterID2 = 0x7FF;
    int comres = HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_sep);
    can_filter_sep.IdType = FDCAN_STANDARD_ID;
    can_filter_sep.FilterIndex = 1;
    can_filter_sep.FilterType = FDCAN_FILTER_MASK;
    can_filter_sep.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    can_filter_sep.FilterID1 = 0x000;
    can_filter_sep.FilterID2 = 0x000;
    comres = HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_sep);
    comres = HAL_FDCAN_Start(&hfdcan1);
    comres = HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    return comres;
}

void can_send(uint16_t id, uint8_t *data, uint8_t length){
    if (length > 8)
    {
        length = 8;
    }
    FDCAN_TxHeaderTypeDef   TxHeader;
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = length;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        
    /* Reception Error */
    Error_Handler();
    }
    add_buffer_can(RxData, RxHeader.DataLength);

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
  }
}

int add_buffer_can(uint8_t *data, uint16_t size){
    if(can_message_from_sep.length + size > sizeof(can_message_from_sep.data)){
        return -1;
    }
    memcpy(can_message_from_sep.data + can_message_from_sep.length, data, size);
    can_message_from_sep.length += size;
    return 0;
}
