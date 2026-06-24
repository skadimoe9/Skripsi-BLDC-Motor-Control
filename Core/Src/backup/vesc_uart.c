/*
 * vesc_uart.c
 *
 *  Created on: Apr 23, 2026
 *      Author: skadimoe9/fadhli
 */

#include "vesc_uart.h"

extern UART_HandleTypeDef huart6;

void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)s, strlen(s), 100);
}

void process_received_line(const char *line) // Kirim ke PC via UART buat respon
{
    if (strcmp(line, "PING") == 0)
    {
        uart_print("ACK,PING\r\n");
    }
    else if (strcmp(line, "START") == 0)
    {
        uart_print("ACK,START\r\n");
    }
    else if (strcmp(line, "STOP") == 0)
    {
        uart_print("ACK,STOP\r\n");
    }
    else if (strncmp(line, "SET_RPM,", 8) == 0)
    {
        int rpm = atoi(&line[8]);

        char msg[64];
        snprintf(msg, sizeof(msg), "ACK,SET_RPM,%d\r\n", rpm);
        uart_print(msg);
    }
    else
    {
        uart_print("ERR,Ngetik_Apaan_Cok\r\n");
    }
}

void telemetry_send(float vin, float current, uint8_t hall, uint8_t sector)
{
    char msg[96];

    snprintf(msg, sizeof(msg),
             "TEL,%lu,%lu,%u,%u\r\n",
             (unsigned long)vin, (unsigned long)current, hall, sector);

    uart_print(msg);
}
