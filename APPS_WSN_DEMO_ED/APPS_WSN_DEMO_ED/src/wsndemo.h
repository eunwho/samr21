/**
* \file  wsndemo.h
*
* \brief WSNDemo application interface
*
* Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries. 
*
* \asf_license_start
*
* \page License
*
* Subject to your compliance with these terms, you may use Microchip
* software and any derivatives exclusively with Microchip products. 
* It is your responsibility to comply with third party license terms applicable 
* to your use of third party software (including open source software) that 
* may accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, 
* WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, 
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, 
* AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE 
* LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL 
* LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE 
* SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY 
* RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, 
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*
* \asf_license_stop
*
*/
/*
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/

#ifndef WSNDEMO_H
#define WSNDEMO_H

#include "board.h"
#include "sysTimer.h"

void wsndemo_init(void);
void wsndemo_task(void);

#ifndef LED_COUNT
#define LED_COUNT 0
#endif

#if LED_COUNT > 2
#define LED_NETWORK       LED0_GPIO
//#define LED_DATA          LED1_GPIO
#define LED_DATA          PIN_PA28		// by jsk

#define LED_BLINK         LED2_GPIO
#define LED_IDENTIFY      LED0_GPIO
#elif LED_COUNT == 2
#define LED_NETWORK       LED0_GPIO
#define LED_DATA          LED1_GPIO
#define LED_BLINK         LED1_GPIO
#define LED_IDENTIFY      LED0_GPIO
#elif LED_COUNT == 1
#define LED_NETWORK       LED0_GPIO
#define LED_DATA          LED0_GPIO
#define LED_BLINK         LED0_GPIO
#define LED_IDENTIFY      LED0_GPIO
#endif

#ifdef LED0_ACTIVE_LEVEL
#define LED_NETWORK_GPIO       LED0_GPIO
#define LED_DATA_GPIO          LED0_GPIO
#define LED_BLINK_GPIO         LED0_GPIO
#define LED_IDENTIFY_GPIO      LED0_GPIO
#define LED_IDENTIFY_ACTIVE_LEVEL  LED0_ACTIVE_LEVEL
#define LED_IDENTIFY_INACTIVE_LEVEL  LED0_ACTIVE_LEVEL
#define LED_NETWORK_ACTIVE_LEVEL  LED0_INACTIVE_LEVEL
#define LED_NETWORK_INACTIVE_LEVEL  LED0_INACTIVE_LEVEL
#define LED_DATA_ACTIVE_LEVEL  LED0_ACTIVE_LEVEL
#define LED_DATA_INACTIVE_LEVEL  LED0_INACTIVE_LEVEL
#define LED_BLINK_ACTIVE_LEVEL  LED0_ACTIVE_LEVEL
#define LED_BLINK_INACTIVE_LEVEL  LED0_INACTIVE_LEVEL
#endif

#define APP_SCAN_DURATION 10
// #define APP_CAPTION_SIZE  (sizeof(APP_CAPTION) - 1 + SHORT_ADDRESS_CAPTION_SIZE)
#define APP_CAPTION_SIZE  10	// by jsk


/*- Types ------------------------------------------------------------------*/
COMPILER_PACK_SET(1)
typedef struct  AppMessage_t {
	uint8_t commandId;
	uint8_t nodeType;
	uint64_t extAddr;
	uint16_t shortAddr;
	uint32_t softVersion;
	uint32_t channelMask;
	uint16_t panId;
	uint8_t workingChannel;
	uint16_t nextHopAddr;
	uint8_t lqi;
	int8_t rssi;

	struct {
		uint8_t type;
		uint8_t size;
		int32_t battery;
		int32_t temperature;
		int32_t light;
	} sensors;

	struct {
		uint8_t type;
		uint8_t size;
		char text[APP_CAPTION_SIZE];
	} caption;
} AppMessage_t;

typedef enum AppState_t {
	APP_STATE_INITIAL,
	APP_STATE_START_NETWORK,
	APP_STATE_CONNECT_NETWORK,
	APP_STATE_CONNECTING_NETWORK,
	APP_STATE_IN_NETWORK,
	APP_STATE_SEND,
	APP_STATE_WAIT_CONF,
	APP_STATE_SENDING_DONE,
	APP_STATE_WAIT_SEND_TIMER,
	APP_STATE_WAIT_COMMAND_TIMER,
	APP_STATE_PREPARE_TO_SLEEP,
	APP_STATE_SLEEP,
	APP_STATE_WAKEUP,
} AppState_t;
COMPILER_PACK_RESET()
/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
extern SYS_Timer_t appDataSendingTimer;

#endif /* WSNDEMO_H */
