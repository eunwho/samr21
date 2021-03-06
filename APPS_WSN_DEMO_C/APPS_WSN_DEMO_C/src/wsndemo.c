/**
* \file  wsndemo.c
*
* \brief WSNDemo application implementation
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "phy.h"
#include "sysTimer.h"
#include "commands.h"
#include "miwi_api.h"

#if defined(PAN_COORDINATOR)
#include "sio2host.h"
#endif

#if SAMD || SAMR21 || SAML21 || SAMR30
#include "system.h"
#else
#include "sysclk.h"
#if (LED_COUNT > 0)
#include "led.h"
#endif
#endif

#include "asf.h"
#include "wsndemo.h"
/*****************************************************************************
*****************************************************************************/
#include "define.h"
#include "extern.h"

#define APP_SCAN_DURATION 10
#define APP_CAPTION_SIZE  (sizeof(APP_CAPTION) - 1 + SHORT_ADDRESS_CAPTION_SIZE)

/*- Types ------------------------------------------------------------------*/
COMPILER_PACK_SET(1)
typedef struct  AppMessage_t {
	uint8_t commandId;				// [0]
	uint8_t nodeType;				// [1]
	uint64_t extAddr;				// [2,3,4,5,6,7,8,9]
	uint16_t shortAddr;				// [10,11]
	uint32_t softVersion;			// [12,13,14,15]
	uint32_t channelMask;			// [16,17,18,19]
	uint16_t panId;					// [20,21]
	uint8_t workingChannel;			// [22]
	uint16_t nextHopAddr;			// [23,24]
	uint8_t lqi;					// [25]
	int8_t rssi;					// [26]

	struct {
		uint8_t type;				// [27]
		uint8_t size;				// [28]
		int32_t battery;			// [29,30,31,32]
		int32_t temperature;		// [33,34,35,36]
		int32_t light;				// [37,38,39,40]
	} sensors;

	struct {
		uint8_t type;				// [41]
		uint8_t size;				// [42]
		char text[APP_CAPTION_SIZE];// [43~]	
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

#if defined(COORDINATOR) || defined (ENDDEVICE)
static SYS_Timer_t appNetworkStatusTimer;
static bool appNetworkStatus;
#endif

#if defined(PAN_COORDINATOR)
static uint8_t rx_data[APP_RX_BUF_SIZE];
#endif

static AppMessage_t appMsg;
SYS_Timer_t appDataSendingTimer;
#ifndef PAN_COORDINATOR
static uint8_t wsnmsghandle;
#endif

void UartBytesReceived(uint16_t bytes, uint8_t *byte );
static void Connection_Confirm(miwi_status_t status);
#ifndef PAN_COORDINATOR
void searchConfim(uint8_t foundScanResults, void* ScanResults);
void appLinkFailureCallback(void);
#else
#if defined(MIWI_MESH_TOPOLOGY_SIMULATION_MODE)
static void appBroadcastDataConf(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer);
#endif
#endif
/*- Implementations --------------------------------------------------------*/

#if defined(PAN_COORDINATOR)

/*****************************************************************************
*****************************************************************************/
	 
void UartBytesReceived(uint16_t bytes, uint8_t *byte )
{
	for (uint16_t i = 0; i < bytes; i++) {
		APP_CommandsByteReceived(byte[i]);
	}
}

static void appUartSendMessage(uint8_t *data, uint8_t size)
{
	uint8_t cs = 0;

	sio2host_putchar(0x10);
	sio2host_putchar(0x02);

	for (uint8_t i = 0; i < size; i++) {
		if (data[i] == 0x10) {
			sio2host_putchar(0x10);
			cs += 0x10;
		}

		sio2host_putchar(data[i]);
		cs += data[i];
	}

	sio2host_putchar(0x10);
	sio2host_putchar(0x03);
	cs += 0x10 + 0x02 + 0x10 + 0x03;

	sio2host_putchar(cs);
}

#endif

/*****************************************************************************
*****************************************************************************/
void appDataInd(RECEIVED_MESH_MESSAGE *ind);

uint8_t bitFlag[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};

void appDataInd(RECEIVED_MESH_MESSAGE *ind)
{	
	int addrId, byteNo, bitNo, setValue,tmp;
	char * zbeeId;
	char * chTemp;
	
	AppMessage_t *msg = (AppMessage_t *)ind->payload;
	chTemp = (char *)ind->payload;

#if (LED_COUNT > 0)
	LED_Toggle(LED_DATA);
#endif
	msg->lqi = ind->packetLQI;
	msg->rssi = ind->packetRSSI;
	int32_t light;
	
	//usart_write_buffer_wait(&usart_instance, chTemp + 43, 14); //"sun001 -0x0001"
	/*
	zbeeId = (msg->caption).text;
	*/

	//sprintf(chTemp,"caption.text = %s, SensValue = %d \r\n",zbeeId,setValue);
	//snprintf(chTemp,40,"caption.text = %s,\r\n",zbeeId);
	// snprintf(chTemp,100,"%s \r\n",);
	//usart_write_buffer_wait(&usart_instance, chTemp, sizeof(chTemp));

	if(*(chTemp +43) =='S'){
		addrId = (*(chTemp+47)-'0') * 10 + (*(chTemp+48)-'0');
		if( (addrId > 0) && (addrId < 81)){ 
			byteNo = (addrId-1) / 8;
			bitNo = (addrId-1) % 8;
			// setValue = (((msg->sensors).light) > 1200 ) ? bitFlag[bitNo] : 0x00;
			setValue = bitFlag[bitNo];
		
			if( sensStateTable[byteNo] & setValue){
				sensStateTable[byteNo] = (sensStateTable[byteNo] & (~setValue));
			} else {
				sensStateTable[byteNo] = (sensStateTable[byteNo] | setValue);				
			}				
		}
	}
	appUartSendMessage(ind->payload, ind->payloadSize); //jsk
}

/*****************************************************************************
*****************************************************************************/
static void appDataSendingTimerHandler(SYS_Timer_t *timer)
{
	if (APP_STATE_WAIT_SEND_TIMER == appState) {
		appState = APP_STATE_SEND;
	} else {
		SYS_TimerStart(&appDataSendingTimer);
	}

	(void)timer;
}

#if defined(COORDINATOR) || defined (ENDDEVICE)

/*****************************************************************************
*****************************************************************************/
static void appNetworkStatusTimerHandler(SYS_Timer_t *timer)
{
#if (LED_COUNT > 0)
	LED_Toggle(LED_NETWORK);
#endif
	(void)timer;
}
#endif

/*****************************************************************************
*****************************************************************************/
#if defined(COORDINATOR) || defined (ENDDEVICE)
static void appDataConf(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
#if (LED_COUNT > 0)
	LED_Off(LED_DATA);
#endif

	if (SUCCESS == status) {
		if (!appNetworkStatus) {
#if (LED_COUNT > 0)
			LED_On(LED_NETWORK);
#endif
			SYS_TimerStop(&appNetworkStatusTimer);
			appNetworkStatus = true;
		}
	} else {
		if (appNetworkStatus) {
#if (LED_COUNT > 0)
			LED_Off(LED_NETWORK);
#endif
			SYS_TimerStart(&appNetworkStatusTimer);
			appNetworkStatus = false;
		}
	}
	if (APP_STATE_WAIT_CONF == appState)
	{
		appState = APP_STATE_SENDING_DONE;
	}
}

#endif

/*****************************************************************************
*****************************************************************************/
static void appSendData(void)
{
    uint16_t shortAddressLocal = 0xFFFF;
    uint16_t shortAddressPanId = 0xFFFF;
#ifndef PAN_COORDINATOR
    uint16_t dstAddr = 0; /* PAN Coordinator Address */
#endif

	appMsg.sensors.battery     = rand() & 0xffff;
	appMsg.sensors.temperature = rand() & 0x7f;
	appMsg.sensors.light       = rand() & 0xff;

	/* Get Short address */
	MiApp_Get(SHORT_ADDRESS, (uint8_t *)&shortAddressLocal);
        appMsg.shortAddr = shortAddressLocal;
	appMsg.extAddr   = appMsg.shortAddr;
#ifndef PAN_COORDINATOR
    /* Get Next Hop Short address to reach PAN Coordinator*/
	appMsg.nextHopAddr = MiApp_MeshGetNextHopAddr(PAN_COORDINATOR_SHORT_ADDRESS);
#endif

	/* Get current working channel */
	MiApp_Get(CHANNEL, (uint8_t *)&appMsg.workingChannel);

	/* Get current working PanId */
	MiApp_Get(PANID, (uint8_t *)&shortAddressPanId);
        appMsg.panId = shortAddressPanId;
#if defined(PAN_COORDINATOR)
	sprintf(&(appMsg.caption.text[APP_CAPTION_SIZE - SHORT_ADDRESS_CAPTION_SIZE]), "-0x%04X", shortAddressLocal);
	appUartSendMessage((uint8_t *)&appMsg, sizeof(appMsg));		// debug jsk
	SYS_TimerStart(&appDataSendingTimer);
	appState = APP_STATE_WAIT_SEND_TIMER;
#else
#if (LED_COUNT > 0)
	LED_On(LED_DATA);
#endif

	appMsg.caption.type         = 32;
#if defined(COORDINATOR)
	if (shortAddressLocal & RXON_ENDEVICE_ADDRESS_MASK)
	{
		appMsg.caption.size         = APP_CAPTION_ED_SIZE;
	    memcpy(appMsg.caption.text, APP_CAPTION_ED, APP_CAPTION_ED_SIZE);
		sprintf(&(appMsg.caption.text[APP_CAPTION_ED_SIZE - SHORT_ADDRESS_CAPTION_SIZE]), "-0x%04X", shortAddressLocal);
	}
	else
#endif
	{
	    appMsg.caption.size         = APP_CAPTION_SIZE;
	    memcpy(appMsg.caption.text, APP_CAPTION, APP_CAPTION_SIZE);
		sprintf(&(appMsg.caption.text[APP_CAPTION_SIZE - SHORT_ADDRESS_CAPTION_SIZE]), "-0x%04X", shortAddressLocal);
	}

	if (MiApp_SendData(2, (uint8_t *)&dstAddr, sizeof(appMsg), (uint8_t *)&appMsg, wsnmsghandle, true, appDataConf))
	{
		++wsnmsghandle;
		appState = APP_STATE_WAIT_CONF;
	}
	else
	{
		appState = APP_STATE_SENDING_DONE;
	}
#endif
}

/*************************************************************************//**
*****************************************************************************/
static void appInit(void)
{
	appMsg.commandId            = APP_COMMAND_ID_NETWORK_INFO;
	appMsg.nodeType             = APP_NODE_TYPE;
	appMsg.extAddr              = 0;
	appMsg.shortAddr            = 0;
	appMsg.softVersion          = 0x01100000;
	appMsg.channelMask          = CHANNEL_MAP;
	appMsg.nextHopAddr          = 0;
	appMsg.lqi                  = 0;
	appMsg.rssi                 = 0;

	appMsg.sensors.type        = 1;
	appMsg.sensors.size        = sizeof(int32_t) * 3;
	appMsg.sensors.battery     = 0;
	appMsg.sensors.temperature = 0;
	appMsg.sensors.light       = 0;

	appMsg.caption.type         = 32;
	appMsg.caption.size         = APP_CAPTION_SIZE;
	memcpy(appMsg.caption.text, APP_CAPTION, APP_CAPTION_SIZE);

	appDataSendingTimer.interval = APP_SENDING_INTERVAL;
	appDataSendingTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appDataSendingTimer.handler = appDataSendingTimerHandler;

#if defined(COORDINATOR) || defined (ENDDEVICE)
	appNetworkStatus = false;
	appNetworkStatusTimer.interval = APP_NWKSTATUS_INTERVAL;
	appNetworkStatusTimer.mode = SYS_TIMER_PERIODIC_MODE;
	appNetworkStatusTimer.handler = appNetworkStatusTimerHandler;
	SYS_TimerStart(&appNetworkStatusTimer);
#else
#if (LED_COUNT > 0)
	LED_On(LED_NETWORK);
#endif
#endif

	APP_CommandsInit();
	MiApp_SubscribeDataIndicationCallback(appDataInd);
#ifndef PAN_COORDINATOR
	MiApp_SubscribeLinkFailureCallback(appLinkFailureCallback);
#endif
#if defined(PAN_COORDINATOR)
    appState = APP_STATE_START_NETWORK;
#else
	appState = APP_STATE_CONNECT_NETWORK;
#endif
}

/*************************************************************************//**
*****************************************************************************/
static void APP_TaskHandler(void)
{
	switch (appState) {
	case APP_STATE_INITIAL:
	{
		appInit();
	}
	break;
#if defined(PAN_COORDINATOR)
	case APP_STATE_START_NETWORK:
	{
		MiApp_StartConnection(START_CONN_DIRECT, APP_SCAN_DURATION, CHANNEL_MAP, Connection_Confirm);
		appState = APP_STATE_SEND;
	}
	break;
#else
	case APP_STATE_CONNECT_NETWORK:
	{
		MiApp_SearchConnection(APP_SCAN_DURATION, CHANNEL_MAP, searchConfim);
		appState = APP_STATE_CONNECTING_NETWORK;
	}
	break;
#endif

	case APP_STATE_SEND:
	{
		appSendData();
	}
	break;

	case APP_STATE_SENDING_DONE:
	{
		SYS_TimerStart(&appDataSendingTimer);
		appState = APP_STATE_WAIT_SEND_TIMER;
	}
	break;
	default:
		break;
	}

#if defined(PAN_COORDINATOR)
	uint16_t bytes;
	if ((bytes = sio2host_rx(rx_data, APP_RX_BUF_SIZE)) > 0) {
		UartBytesReceived(bytes, (uint8_t *)&rx_data);
	}
#if defined(MIWI_MESH_TOPOLOGY_SIMULATION_MODE)

	/* This section of code tries to simulate and restore the topology
	*  for testing purposes. When the SW0 is pressed for less than 3secs,
	*  the PAN coordinator sends out a application data to all the coordinators
	*  to set its route entry respectively to generate a line topology.	
	*  After testing when this button is pressed for more than 3secs, it sends
	*  out a topology reset command, which will bring back the network back to
	*  mesh state.
	*/

	/* Read the button level */
	if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE)
	{
		uint8_t commandId;
		uint16_t dstAddr = MESH_BROADCAST_TO_COORDINATORS;
		uint8_t count = 0;
		
		/* Wait for button debounce time */
		delay_ms(50);
		/* Check whether button is in default state */
		while(port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE)
		{
			delay_ms(500);
			count += 1;
		}
		if (count > 5)
		{
			commandId = APP_COMMAND_ID_TOPOLOGY_SIMULATION_RESET;
			MiApp_SendData(2, (uint8_t *)&dstAddr, 1, &commandId, 1, false, appBroadcastDataConf);
		}
		else
		{
			commandId = APP_COMMAND_ID_SIMULATE_LINE_TOPOLOGY;
			MiApp_SendData(2, (uint8_t *)&dstAddr, 1, &commandId, 2, false, appBroadcastDataConf);
		}
	}
#endif
#endif
}

/*****************************************************************************
*****************************************************************************/

/**
 * Init function of the WSNDemo application
 */
void wsndemo_init(void)
{
	MiApp_ProtocolInit(&defaultParamsRomOrRam, &defaultParamsRamOnly);

#if defined(PAN_COORDINATOR)
	sio2host_init();
#endif
}

#ifndef PAN_COORDINATOR
/**
 * Search confirmation
 */
void searchConfim(uint8_t foundScanResults, void* ScanResults)
{
	searchConf_t* searchConfRes = (searchConf_t *)ScanResults;
	if (foundScanResults)
	{
		for (uint8_t loopindex = 0; loopindex < foundScanResults; loopindex++)
		{
			if (searchConfRes->beaconList[loopindex].connectionPermit)
			{
				MiApp_EstablishConnection(searchConfRes->beaconList[loopindex].logicalChannel,
				SHORT_ADDR_LEN, (uint8_t*)&searchConfRes->beaconList[loopindex].shortAddress, CAPABILITY_INFO, Connection_Confirm);
				return;
			}
		}
		/* Initiate the search again since no connection permit found to join */
		appState = APP_STATE_CONNECT_NETWORK;
	}
	else
	{
		/* Initiate the search again since no beacon */
		appState = APP_STATE_CONNECT_NETWORK;
	}
}
#endif

/**
 * Connection confirmation
 */
static void Connection_Confirm(miwi_status_t status)
{
	if (SUCCESS == status)
	{
        appState = APP_STATE_SEND;
	}
	else
	{
#if defined(PAN_COORDINATOR)
		appState = APP_STATE_START_NETWORK;
#else
        appState = APP_STATE_CONNECT_NETWORK;
#endif
	}
}

/**
 * Task of the WSNDemo application
 * This task should be called in a while(1)
 */
void wsndemo_task(void)
{
	MeshTasks();
	APP_TaskHandler();
}

#ifndef PAN_COORDINATOR
void appLinkFailureCallback(void)
{
	/* On link failure initiate search to establish connection */
	appState = APP_STATE_CONNECT_NETWORK;
	SYS_TimerStop(&appDataSendingTimer);
}
#endif

#if defined(PAN_COORDINATOR) && defined(MIWI_MESH_TOPOLOGY_SIMULATION_MODE)
static void appBroadcastDataConf(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
		
}
#endif
