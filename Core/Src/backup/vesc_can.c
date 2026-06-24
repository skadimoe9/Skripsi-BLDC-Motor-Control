/*
 * vesc_can.c
 *
 *  Created on: Apr 24, 2026
 *      Author: skadi
 */

#include "vesc_can.h"
#include "vesc_uart.h"
#include <stdio.h>

extern CAN_HandleTypeDef hcan1;

static uint8_t can_seq = 0;

void can_test_init(void)
{
    CAN_FilterTypeDef canfilter;

    canfilter.FilterBank = 0;
    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;
    canfilter.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfilter.FilterActivation = ENABLE;
    canfilter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &canfilter) != HAL_OK) {
        uart_print("CAN filter error\r\n");
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        uart_print("CAN start error\r\n");
        Error_Handler();
    }

    uart_print("CAN start OK\r\n");
}

void can_test_send_dummy(void)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8];
    uint16_t t = (uint16_t)(HAL_GetTick() & 0xFFFF);

    txData[0] = can_seq;
    txData[1] = (uint8_t)(~can_seq);
    txData[2] = (uint8_t)(t & 0xFF);
    txData[3] = (uint8_t)((t >> 8) & 0xFF);
    txData[4] = 0x55;
    txData[5] = 0xAA;
    txData[6] = (uint8_t)(0xF0 | (can_seq & 0x0F));
    txData[7] = txData[0] ^ txData[1] ^ txData[2] ^ txData[3] ^ txData[4] ^ txData[5] ^ txData[6];

    txHeader.StdId = 0x123;
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) == HAL_OK) {
    	char uart_msg[128];
        snprintf(uart_msg, sizeof(uart_msg),
                 "CAN TX seq=%u data0=%02X data7=%02X\r\n",
                 can_seq, txData[0], txData[7]);
        uart_print(uart_msg);
    } else {
        uart_print("CAN TX FAIL\r\n");
    }

    can_seq++;
}

void can_test_poll_rx(void)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        	char uart_msg[128];
            snprintf(uart_msg, sizeof(uart_msg),
                     "CAN RX ID=0x%03lX DLC=%u D0=%02X D1=%02X\r\n",
                     (uint32_t)rxHeader.StdId, (unsigned int)rxHeader.DLC, rxData[0], rxData[1]);
            uart_print(uart_msg);
        }
    }
}
