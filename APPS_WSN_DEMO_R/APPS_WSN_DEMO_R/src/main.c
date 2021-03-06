/**
* \file  main.c
*
* \brief Main file for WSN Demo Example on MiWi Mesh.
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

/**
* \mainpage
* \section preface Preface
* This is the reference manual for the WSN Demo Application
* The WSNDemo application implements a typical wireless sensor network scenario,
* in which one central node collects the data from a network of sensors and
* passes this data over a serial connection for further processing.
* In the case of the WSNDemo this processing is performed by the WSNMonitor PC
* application. The MiWi™ Quick Start Guide  provides a detailed description
* of the WSNDemo application scenario, and instructions on how to use WSNMonitor.
* <P>• Device types (PAN Coordinator, Coordinator and End Device) and its address in 
* MiWi™ Mesh network is displayed on the nodes.</P>
* <P>• The value of the extended address field is set equal to the value of the
* short address field.</P>
* <P>• For all frames, the LQI and RSSI fields are filled in by the coordinator
* with the values of LQI and RSSI from the received frame. This means that nodes
* that are not connected to the coordinator directly will have the same values
* as the last node on the route to the coordinator.</P>
* <P>• Sensor data values are generated randomly on all platforms.</P>
* <P>• Sending data to the nodes can be triggered when the light button on the 
* node is clicked. This also blinks the LED in node.
* </P>
*/


/************************ HEADERS ****************************************/
#include "asf.h"
#include "sio2host.h"
#include "wsndemo.h"
#include "miwi_api.h"
#include "config.h"

#if ((BOARD == SAMR30_XPLAINED_PRO) || (BOARD == SAMR21_XPLAINED_PRO))
#include "edbg-eui.h"
#endif

/************************** DEFINITIONS **********************************/
#if (BOARD == SAMR21ZLL_EK)
#define NVM_UID_ADDRESS   ((volatile uint16_t *)(0x00804008U))
#endif

/************************** PROTOTYPES **********************************/
void ReadMacAddress(void);

int main ( void )
{
	irq_initialize_vectors();

	system_init();
	delay_init();

	cpu_irq_enable();	

	sio2host_init();
	ReadMacAddress();
	wsndemo_init();
	
    while(1)
    {
		wsndemo_task();
    }

}

void ReadMacAddress(void){
	myLongAddress[0] = 16;
	myLongAddress[1] = 53;
	myLongAddress[2] = 0;
	myLongAddress[3] = 32;
	myLongAddress[4] = 89;
	myLongAddress[5] = 37;
	myLongAddress[6] = 128;
	myLongAddress[7] = 130 - ID_ROUTER;
}

