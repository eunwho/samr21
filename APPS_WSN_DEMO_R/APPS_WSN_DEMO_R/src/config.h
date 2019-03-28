/*
	file  config.h
*/

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "miwi_config.h"
#include  "miwi_config_mesh.h"
/*****************************************************************************
*****************************************************************************/
#define APP_SENDING_INTERVAL    10000

#define APP_SENDING_INTERVAL_IN_SIMULATION 60000
#define APP_NWKSTATUS_INTERVAL  1000

#define APP_RX_BUF_SIZE         200

#define PAN_COORDINATOR_SHORT_ADDRESS   0x0000

/* To display the short address like -0x1234 */
#define SHORT_ADDRESS_CAPTION_SIZE     7

#define ID_ROUTER	     1
#define APP_CAPTION     "ROU"
#define APP_CAPTION_ED_SIZE  (sizeof(APP_CAPTION_ED) - 1 + SHORT_ADDRESS_CAPTION_SIZE)
#define APP_CAPTION_ED  "RD"
#define APP_NODE_TYPE   1
#define APP_COORDINATOR 0
#define APP_ROUTER      1
#define APP_ENDDEVICE   0

#endif /* _CONFIG_H_ */
