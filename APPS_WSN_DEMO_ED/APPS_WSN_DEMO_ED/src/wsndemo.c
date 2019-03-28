/**
* \file  wsndemo.c
*/
#include "header.h"
#include "extern.h"


static AppState_t appState = APP_STATE_INITIAL;
static SYS_Timer_t appNetworkStatusTimer;

static bool appNetworkStatus;

static AppMessage_t appMsg;
SYS_Timer_t appDataSendingTimer;

static uint8_t wsnmsghandle;
static void Connection_Confirm(miwi_status_t status);

void rtc_match_callback(void){
	
	// port_pin_toggle_output_level(LED_0_PIN);
	//	LED_Toggle(LED_NETWORK); //by jsk

	if (APP_STATE_WAIT_SEND_TIMER == appState) appState = APP_STATE_SEND;
	else 	appState= APP_STATE_INITIAL;

	alarm.mask = RTC_CALENDAR_ALARM_MASK_SEC;
	alarm.time.second += SLEEP_SECOND;
	alarm.time.second = alarm.time.second % 60;
	rtc_calendar_set_alarm(&rtc_instance, &alarm, RTC_CALENDAR_ALARM_0);
}

static void appDataInd(RECEIVED_MESH_MESSAGE *ind){
	AppMessage_t *msg = (AppMessage_t *)ind->payload;
	LED_Toggle(LED_DATA);
	msg->lqi = ind->packetLQI;	msg->rssi = ind->packetRSSI;
    appCmdDataInd(ind);
}


static void appDataSendingTimerHandler(SYS_Timer_t *timer){
	if (APP_STATE_WAIT_SEND_TIMER == appState) appState = APP_STATE_SEND;
	else	SYS_TimerStart(&appDataSendingTimer);
	(void)timer;
}


static void appNetworkStatusTimerHandler(SYS_Timer_t *timer)
{
#if (LED_COUNT > 0 )
	LED_Toggle(LED_NETWORK);
#endif
	(void)timer;
}

static void appDataConf(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
#if (LED_COUNT > 0 )
	LED_Off(LED_DATA);
#endif
	
	if (SUCCESS == status) {
		if (!appNetworkStatus) {
#if (LED_COUNT > 0 )
			LED_On(LED_NETWORK);
#endif
			SYS_TimerStop(&appNetworkStatusTimer);
			appNetworkStatus = true;
		}
	} else {
		if (appNetworkStatus) {
#if (LED_COUNT > 0 )
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


static void appSendData(void)
{
    uint16_t shortAddressLocal = 0xFFFF;
    uint16_t shortAddressPanId = 0xFFFF;

    uint16_t dstAddr = 0; /* PAN Coordinator Address */

	//adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN6);
	adc_start_conversion(&adc_instance);
	delay_us(100);
	adc_read(&adc_instance,&adcResult);
	appMsg.sensors.light       = adcResult;

	adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_TEMP);
	delay_us(10);
	adc_start_conversion(&adc_instance);
	delay_us(10);
	adc_read(&adc_instance,&adcResult);
	appMsg.sensors.temperature = adcResult;

	adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_SCALEDIOVCC);
	adc_start_conversion(&adc_instance);
	delay_us(10);
	adc_read(&adc_instance,&adcResult);
	appMsg.sensors.battery     = adcResult;
	
	/* Get Short address */
	MiApp_Get(SHORT_ADDRESS, (uint8_t *)&shortAddressLocal);
	//	shortAddressLocal = ADDR_LOCAL;
		appMsg.shortAddr = shortAddressLocal;
	appMsg.extAddr   = appMsg.shortAddr;
    /* Get Next Hop Short address to reach PAN Coordinator*/
	appMsg.nextHopAddr = MiApp_MeshGetNextHopAddr(PAN_COORDINATOR_SHORT_ADDRESS);
	/* Get current working channel */
	MiApp_Get(CHANNEL, (uint8_t *)&appMsg.workingChannel);
	/* Get current working PanId */
	MiApp_Get(PANID, (uint8_t *)&shortAddressPanId);
        appMsg.panId = shortAddressPanId;
#if (LED_COUNT > 0 )
	LED_On(LED_DATA);
#endif
	appMsg.caption.type         = 32;
    appMsg.caption.size         = APP_CAPTION_SIZE;
    memcpy(appMsg.caption.text, APP_CAPTION, APP_CAPTION_SIZE);
	sprintf(appMsg.caption.text, "SUN%03d -0x%04x", MAC_ADDR, shortAddressLocal);
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

//  changing sleep mode
	appDataSendingTimer.interval = APP_SENDING_INTERVAL;
	appDataSendingTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appDataSendingTimer.handler = appDataSendingTimerHandler;

	appNetworkStatus = false;
	appNetworkStatusTimer.interval = APP_NWKSTATUS_INTERVAL;
	appNetworkStatusTimer.mode = SYS_TIMER_PERIODIC_MODE;
	appNetworkStatusTimer.handler = appNetworkStatusTimerHandler;
	SYS_TimerStart(&appNetworkStatusTimer);

#if (LED_COUNT > 0 )
	LED_On(LED_NETWORK);
#endif

	APP_CommandsInit();
	MiApp_SubscribeDataIndicationCallback(appDataInd);

	appState = APP_STATE_CONNECT_NETWORK;
}

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
			system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
			appSendData();
			break;
		case APP_STATE_SENDING_DONE:
			appState = APP_STATE_WAIT_SEND_TIMER;
			//rtc_calendar_set_time(&rtc_instance, &rtc_time);
			SYS_TimerStop(&appDataSendingTimer);
			system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
			system_sleep();
			break;		
		default:break;
	}
}

// Init function of the WSNDemo application
void wsndemo_init(void)
{
	MiApp_ProtocolInit(&defaultParamsRomOrRam, &defaultParamsRamOnly);
}

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

/**
 * Connection confirmation
 */
static void Connection_Confirm(miwi_status_t status)
{
	if (SUCCESS == status){		
		appState = APP_STATE_SEND;
		// rtc_calendar_set_time(&rtc_instance, &rtc_time);	// jsk
		// system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);		// jsk
	} 
	else        appState = APP_STATE_CONNECT_NETWORK;
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

void appLinkFailureCallback(void)
{
	/* On link failure initiate search to establish connection */
	appState = APP_STATE_CONNECT_NETWORK;
	SYS_TimerStop(&appDataSendingTimer);
}
