//
//	file  commands.c

#include <stdlib.h>
#include <string.h>
#include "asf.h"
#include "config.h"
#include "miwi_api.h"
#include "sysTimer.h"
#include "commands.h"
#include "wsndemo.h"

#if !(SAMD || SAMR21||SAML21||SAMR30)
	#if (LED_COUNT > 0)
		#include "led.h"
	#endif
#endif

/*- Definitions ------------------------------------------------------------*/
#define APP_CMD_UART_BUFFER_SIZE    16
#define APP_CMD_PENDING_TABLE_SIZE  5
#define APP_CMD_INVALID_ADDR        0xffff

/*- Types ------------------------------------------------------------------*/
typedef enum {
	APP_CMD_UART_STATE_IDLE,
	APP_CMD_UART_STATE_SYNC,
	APP_CMD_UART_STATE_DATA,
	APP_CMD_UART_STATE_MARK,
	APP_CMD_UART_STATE_CSUM,
} AppCmdUartState_t;

COMPILER_PACK_SET(1)

typedef struct {
	uint8_t commandId;
	uint64_t dstAddr;
} AppCmdUartHeader_t;

typedef struct {
	uint8_t commandId;
	uint64_t dstAddr;
	uint16_t duration;
	uint16_t period;
} AppCmdUartIdentify_t;

typedef struct {
	uint8_t id;
} AppCmdHeader_t;

typedef struct {
	uint8_t id;
	uint16_t duration;
	uint16_t period;
} AppCmdIdentify_t;

COMPILER_PACK_RESET()
/*- Prototypes -------------------------------------------------------------*/
static void appCmdUartProcess(uint8_t *data, uint8_t size);
static void appCmdBuffer(uint16_t addr, uint8_t *data, uint8_t size);
static void appCmdDataRequest(uint16_t addr, uint8_t size, uint8_t* payload);
static void appCmdDataConf(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer);
static bool appCmdHandle(uint8_t *data, uint8_t size);
static void appCmdIdentifyDurationTimerHandler(SYS_Timer_t *timer);
static void appCmdIdentifyPeriodTimerHandler(SYS_Timer_t *timer);

/*- Variables --------------------------------------------------------------*/
static AppCmdUartState_t appCmdUartState = APP_CMD_UART_STATE_IDLE;
static uint8_t appCmdUartPtr;
static uint8_t appCmdUartBuf[APP_CMD_UART_BUFFER_SIZE];
static uint8_t appCmdUartCsum;
static SYS_Timer_t appCmdIdentifyDurationTimer;
static SYS_Timer_t appCmdIdentifyPeriodTimer;

/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*****************************************************************************/
void APP_CommandsInit(void)
{
	appCmdIdentifyDurationTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appCmdIdentifyDurationTimer.handler = appCmdIdentifyDurationTimerHandler;

	appCmdIdentifyPeriodTimer.mode = SYS_TIMER_PERIODIC_MODE;
	appCmdIdentifyPeriodTimer.handler = appCmdIdentifyPeriodTimerHandler;
}

/*************************************************************************//**
*****************************************************************************/
void APP_CommandsByteReceived(uint8_t byte)
{
	switch (appCmdUartState) {
	case APP_CMD_UART_STATE_IDLE:
		if (0x10 == byte) {
			appCmdUartPtr = 0;
			appCmdUartCsum = byte;
			appCmdUartState = APP_CMD_UART_STATE_SYNC;
		}
		break;

	case APP_CMD_UART_STATE_SYNC:
	{
		appCmdUartCsum += byte;

		if (0x02 == byte) {
			appCmdUartState = APP_CMD_UART_STATE_DATA;
		} else {
			appCmdUartState = APP_CMD_UART_STATE_IDLE;
		}
	}
	break;

	case APP_CMD_UART_STATE_DATA:
	{
		appCmdUartCsum += byte;

		if (0x10 == byte) {
			appCmdUartState = APP_CMD_UART_STATE_MARK;
		} else {
			appCmdUartBuf[appCmdUartPtr++] = byte;
		}

		if (appCmdUartPtr == APP_CMD_UART_BUFFER_SIZE) {
			appCmdUartState = APP_CMD_UART_STATE_IDLE;
		}
	}
	break;

	case APP_CMD_UART_STATE_MARK:
	{
		appCmdUartCsum += byte;

		if (0x10 == byte) {
			appCmdUartBuf[appCmdUartPtr++] = byte;

			if (appCmdUartPtr == APP_CMD_UART_BUFFER_SIZE) {
				appCmdUartState = APP_CMD_UART_STATE_IDLE;
			} else {
				appCmdUartState = APP_CMD_UART_STATE_DATA;
			}
		} else if (0x03 == byte) {
			appCmdUartState = APP_CMD_UART_STATE_CSUM;
		} else {
			appCmdUartState = APP_CMD_UART_STATE_IDLE;
		}
	}
	break;

	case APP_CMD_UART_STATE_CSUM:
	{
		if (byte == appCmdUartCsum) {
			appCmdUartProcess(appCmdUartBuf, appCmdUartPtr);
		}

		appCmdUartState = APP_CMD_UART_STATE_IDLE;
	}
	break;

	default:
		break;
	}
}

/*************************************************************************//**
*****************************************************************************/
static void appCmdUartProcess(uint8_t *data, uint8_t size)
{
	AppCmdUartHeader_t *header = (AppCmdUartHeader_t *)data;

	if (size < sizeof(AppCmdUartHeader_t)) {
		return;
	}

	if (APP_COMMAND_ID_IDENTIFY == header->commandId) {
		AppCmdUartIdentify_t *uartCmd = (AppCmdUartIdentify_t *)data;
		AppCmdIdentify_t cmd;

		cmd.id = APP_COMMAND_ID_IDENTIFY;
		cmd.duration = uartCmd->duration;
		cmd.period = uartCmd->period;

		appCmdBuffer(header->dstAddr, (uint8_t *)&cmd,
				sizeof(AppCmdIdentify_t));
	}
}

/*************************************************************************//**
*****************************************************************************/
static void appCmdBuffer(uint16_t addr, uint8_t *data, uint8_t size)
{
	if (0 == addr)
	{
		appCmdHandle(data, size);
	}
	else
	{
		appCmdDataRequest(addr, size, data);
	}
}

uint8_t wsnmsghandle;
/*************************************************************************//**
*****************************************************************************/
static void appCmdDataRequest(uint16_t addr, uint8_t size, uint8_t* payload)
{
	MiApp_SendData(SHORT_ADDR_LEN, (uint8_t*)&addr, size, payload, wsnmsghandle, true, appCmdDataConf);
}

/*************************************************************************//**
*****************************************************************************/
static void appCmdDataConf(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
}

/*************************************************************************//**
*****************************************************************************/
void appCmdDataInd(RECEIVED_MESH_MESSAGE *ind)
{
	appCmdHandle(ind->payload, ind->payloadSize);
}

/*************************************************************************//**
*****************************************************************************/
static bool appCmdHandle(uint8_t *data, uint8_t size)
{
	AppCmdHeader_t *header = (AppCmdHeader_t *)data;

	if (size < sizeof(AppCmdHeader_t)) {
		return false;
	}

	if (APP_COMMAND_ID_IDENTIFY == header->id) {
		AppCmdIdentify_t *req = (AppCmdIdentify_t *)data;

		if (sizeof(AppCmdIdentify_t) != size) {
			return false;
		}

		SYS_TimerStop(&appCmdIdentifyDurationTimer);
		SYS_TimerStop(&appCmdIdentifyPeriodTimer);

		appCmdIdentifyDurationTimer.interval = req->duration;
		appCmdIdentifyDurationTimer.mode = SYS_TIMER_INTERVAL_MODE;
		appCmdIdentifyDurationTimer.handler
			= appCmdIdentifyDurationTimerHandler;
		SYS_TimerStart(&appCmdIdentifyDurationTimer);

		appCmdIdentifyPeriodTimer.interval = req->period;
		appCmdIdentifyPeriodTimer.mode = SYS_TIMER_PERIODIC_MODE;
		appCmdIdentifyPeriodTimer.handler
			= appCmdIdentifyPeriodTimerHandler;
		SYS_TimerStart(&appCmdIdentifyPeriodTimer);
/*
#if (LED_COUNT > 0)
		LED_On(LED_IDENTIFY);
#endif
*/		return true;
	}
#if defined(COORDINATOR) && defined(MIWI_MESH_TOPOLOGY_SIMULATION_MODE)

	/* Based on the command from PANC, the coordinators will form a line topology.
	*  This is achieved by configuring the routing to test mode and setting
	*  route parameters from application.
	*  For eg.: If the short address of a device is 0x0300, we set the next hop as below...
	*              -----------------------------------------------
	*             |   Dest. Addr. |  Next Hop Addr.	|  Hop Count  |
	*              -----------------------------------------------
	*             |     0x0000    |     0x0200      |      3      |
	*             |     0x0100    |     0x0200      |      2      |
	*             |     0x0200    |     0x0200      |      1      |
	*             |     0x0300    |     0x0300      |      0      |
	*             |     0x0400    |     0x0400      |      1      |
	*             |     0x0500    |     0x0400      |      2      |
	*             |     0x0600    |     0x0400      |      3      |
	*             |     0x0700    |     0x0400      |      4      |
	*             |   .so on      |       ..        |      ..     |
	*              -----------------------------------------------
	* Note: The App sending interval has been increased to 60secs as the number of packets
	* to be routed will be very high near the PANC and gradually decreases as address of 
	* the device increases.
	* So the timeout in the WSNMonitor has to be adjusted at least to 240 secs 
	* to view the topology. 
	*/
	else if (APP_COMMAND_ID_TOPOLOGY_SIMULATION_RESET == header->id) {
		bool routeTest = false;
		MiApp_Set(ROUTE_TEST_MODE, (uint8_t *)&routeTest);
		SYS_TimerStop(&appDataSendingTimer);
		appDataSendingTimer.interval = APP_SENDING_INTERVAL;
		SYS_TimerStart(&appDataSendingTimer);
	}
	else if (APP_COMMAND_ID_SIMULATE_LINE_TOPOLOGY == header->id) {
		uint16_t shortAddr = 0xFFFF;
		uint8_t coordIndex;
		uint8_t shortAddrIndex;
		bool routeTest = true;

		RouteEntry_t routeEntry;
		routeEntry.coordNextHopLQI = 3;
		routeEntry.coordScore = 3;

		SYS_TimerStop(&appDataSendingTimer);
		appDataSendingTimer.interval = APP_SENDING_INTERVAL_IN_SIMULATION;
		SYS_TimerStart(&appDataSendingTimer);

		MiApp_Set(ROUTE_TEST_MODE, (uint8_t *)&routeTest);

		MiApp_Get(SHORT_ADDRESS, (uint8_t *)&shortAddr);

		if (shortAddr & COORDINATOR_ADDRESS_MASK)
		{
			shortAddrIndex = shortAddr >> 8;
			for (coordIndex = 0; coordIndex < NUM_OF_COORDINATORS; coordIndex++)
			{
				if (coordIndex < shortAddrIndex)
				{
					routeEntry.coordNextHop = shortAddrIndex - 1;
					routeEntry.coordHopCount = shortAddrIndex - coordIndex;

				}
				else if(coordIndex == shortAddrIndex)
				{
					routeEntry.coordNextHop = 0;
					routeEntry.coordHopCount = 0;
				}
				else
				{
					routeEntry.coordNextHop = shortAddrIndex + 1;
					routeEntry.coordHopCount = coordIndex - shortAddrIndex;
				}
				MiApp_MeshSetRouteEntry(coordIndex, &routeEntry);
			}
		}
	}

#endif
	return false;
}

/*************************************************************************//**
*****************************************************************************/
static void appCmdIdentifyDurationTimerHandler(SYS_Timer_t *timer)
{
#if (LED_COUNT > 0)
	LED_Off(LED_IDENTIFY);
#endif
	SYS_TimerStop(&appCmdIdentifyPeriodTimer);
	(void)timer;
}

/*************************************************************************//**
*****************************************************************************/
static void appCmdIdentifyPeriodTimerHandler(SYS_Timer_t *timer)
{
#if (LED_COUNT > 0)
	LED_Toggle(LED_IDENTIFY);
#endif
	(void)timer;
}
