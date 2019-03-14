/**
* \file  wsndemo.c
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

#include "system.h"
#include "asf.h"
#include "wsndemo.h"


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
static void appDataInd(RECEIVED_MESH_MESSAGE *ind)
{
	AppMessage_t *msg = (AppMessage_t *)ind->payload;
#if (LED_COUNT > 0)
	LED_Toggle(LED_DATA);
#endif
	msg->lqi = ind->packetLQI;
	msg->rssi = ind->packetRSSI;
#if defined(PAN_COORDINATOR)
	appUartSendMessage(ind->payload, ind->payloadSize);
#else
    appCmdDataInd(ind);
#endif
}

/*****************************************************************************
*****************************************************************************/
volatile static AppState_t appStateTemp;
static void appDataSendingTimerHandler(SYS_Timer_t *timer)
{
	LED_Toggle(LED_NETWORK); //by jsk

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
	(void)timer;
}
#endif

/*****************************************************************************
*****************************************************************************/
#if defined(COORDINATOR) || defined (ENDDEVICE)
static void appDataConf(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
	if (SUCCESS == status) {
		if (!appNetworkStatus) {
			SYS_TimerStop(&appNetworkStatusTimer);
			appNetworkStatus = true;
		}
	} else {
		if (appNetworkStatus) {
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

    uint16_t dstAddr = 0; /* PAN Coordinator Address */

//	appMsg.sensors.battery     = rand() & 0xffff;
//	appMsg.sensors.temperature = rand() & 0x7f;
//	appMsg.sensors.light       = rand() & 0xff;
	appMsg.sensors.battery     = 11;
	appMsg.sensors.temperature = 22;
	appMsg.sensors.light       = 33;

	/* Get Short address */
	MiApp_Get(SHORT_ADDRESS, (uint8_t *)&shortAddressLocal);
        appMsg.shortAddr = shortAddressLocal;
	appMsg.extAddr   = appMsg.shortAddr;

    /* Get Next Hop Short address to reach PAN Coordinator*/
	appMsg.nextHopAddr = MiApp_MeshGetNextHopAddr(PAN_COORDINATOR_SHORT_ADDRESS);

	/* Get current working channel */
	MiApp_Get(CHANNEL, (uint8_t *)&appMsg.workingChannel);

	/* Get current working PanId */
	MiApp_Get(PANID, (uint8_t *)&shortAddressPanId);
        appMsg.panId = shortAddressPanId;

	appMsg.caption.type         = 32;

    appMsg.caption.size         = APP_CAPTION_SIZE;
    memcpy(appMsg.caption.text, APP_CAPTION, APP_CAPTION_SIZE);
	sprintf(&(appMsg.caption.text[APP_CAPTION_SIZE - SHORT_ADDRESS_CAPTION_SIZE]), "-0x%04X", shortAddressLocal);

	if (MiApp_SendData(2, (uint8_t *)&dstAddr, sizeof(appMsg), (uint8_t *)&appMsg, wsnmsghandle, true, appDataConf))
	{
		++wsnmsghandle;
		appState = APP_STATE_WAIT_CONF;
	} else {
		appState = APP_STATE_SENDING_DONE;
	}
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
#endif

	APP_CommandsInit();
/*
	appCmdIdentifyDurationTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appCmdIdentifyDurationTimer.handler = appCmdIdentifyDurationTimerHandler;

	appCmdIdentifyPeriodTimer.mode = SYS_TIMER_PERIODIC_MODE;
	appCmdIdentifyPeriodTimer.handler = appCmdIdentifyPeriodTimerHandler;
*/
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
	switch(appState)
	{
		case APP_STATE_INITIAL:
			appInit();
			break;	
		case APP_STATE_CONNECT_NETWORK:
			MiApp_SearchConnection(APP_SCAN_DURATION, CHANNEL_MAP, searchConfim);
			appState = APP_STATE_CONNECTING_NETWORK;
			break;

		case APP_STATE_SEND:
			//LED_Toggle(PIN_PA27);
			// LED_Toggle(PIN_PA28);
			// delay_ms(200);
			appSendData();
			// system_sleep();
			break;
		case APP_STATE_SENDING_DONE:
			LED_Toggle(PIN_PA28);
			delay_ms(200);
			SYS_TimerStart(&appDataSendingTimer);
			appState = APP_STATE_WAIT_SEND_TIMER;
			break;		

		default:break;
	}
}

// Init function of the WSNDemo application
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
	
	if(foundScanResults)
	{
		for(uint8_t loopindex = 0; loopindex < foundScanResults; loopindex++)
		{
			if(searchConfRes->beaconList[loopindex].connectionPermit)
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
/*
	appSendData();
	LED_Off(PIN_PA28);
	if(	appState == APP_STATE_SENDING_DONE )	system_sleep();	
*/
}

void appLinkFailureCallback(void)
{
	/* On link failure initiate search to establish connection */
	appState = APP_STATE_CONNECT_NETWORK;
	SYS_TimerStop(&appDataSendingTimer);
}
