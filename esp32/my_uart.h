/*
 * uart.h
 *
 *  Created on: 6 янв. 2024 г.
 *      Author: Intel
 */

#ifndef MAIN_MY_UART_H_
#define MAIN_MY_UART_H_

#include "main.h"
#include <stdbool.h>
#include <unistd.h>
#include <driver/uart.h>



/*
#define TXD_PIN 			CONFIG_TX_GPIO_PIN
#define RXD_PIN 			CONFIG_RX_GPIO_PIN

#define RX_BUF_SIZE 	1024

#define LED_GPIO 			CONFIG_LED_GPIO*/

void uart_init(void);
int uart_sendData(const char* logName, uint8_t* data, uint8_t lendata);

#endif /* MAIN_MY_UART_H_ */
