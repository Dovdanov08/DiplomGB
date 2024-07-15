/*
 * main.h
 *
 *  Created on: 3 янв. 2024 г.
 *      Author: Intel
 */

#ifndef MAIN_MAIN_H_
#define MAIN_MAIN_H_

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "mqtt.h"
#include "spiffs_config.h"
#include <sdkconfig.h>
#include "my_uart.h"
#include "wifi.h"
#include "math.h"
//-----------UART-------///
#define TXD_PIN 			CONFIG_TX_GPIO_PIN
#define RXD_PIN 			CONFIG_RX_GPIO_PIN
#define RX_BUF_SIZE 		256
//------------------------//

#define LED_GPIO 			CONFIG_LED_GPIO

//-------------Commands_for_STM32------//
#define send_temper		0xAF
#define send_ADC			0xFA
//-------------------------------------//
#endif /* MAIN_MAIN_H_ */
