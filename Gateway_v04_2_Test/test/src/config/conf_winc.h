/**
 *
 * \file
 *
 * \brief WINC1500 configuration.
 *
 * Copyright (c) 2016-2017 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef CONF_WINC_H_INCLUDED
#define CONF_WINC_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"

/* This configuration files comes with empty settings! */
/* Default settings for SAMR21 Xplained Pro with WINC on EXT1 are */
/* available in comments for reference. */
// #warning Please modify conf_winc.h for WINC module settings!

/*
   ---------------------------------
   ---------- PIN settings ---------
   ---------------------------------
*/
// 게이트웨이-v04 세팅용 //
#define CONF_WINC_PIN_RESET				PIN_PA13
#define CONF_WINC_PIN_CHIP_ENABLE		PIN_PA14
#define CONF_WINC_PIN_WAKE				PIN_PA22

//SAMR21 X-PRO 세팅용
//#define CONF_WINC_PIN_RESET			PIN_PA13
//#define CONF_WINC_PIN_CHIP_ENABLE		PIN_PA22
//#define CONF_WINC_PIN_WAKE			PIN_PA23

/*
   ---------------------------------
   ---------- SPI settings ---------
   ---------------------------------
*/

#define CONF_WINC_USE_SPI				(1)

// 게이트웨이 세팅용 //
#define CONF_WINC_SPI_MODULE			SERCOM3
#define CONF_WINC_SPI_SERCOM_MUX		SPI_SIGNAL_MUX_SETTING_E
#define CONF_WINC_SPI_PINMUX_PAD0		PINMUX_PA16D_SERCOM3_PAD0 /* in */
#define CONF_WINC_SPI_PINMUX_PAD1		PINMUX_UNUSED /* cs driven from software */
#define CONF_WINC_SPI_PINMUX_PAD2		PINMUX_PA18D_SERCOM3_PAD2 /* out */
#define CONF_WINC_SPI_PINMUX_PAD3		PINMUX_PA19D_SERCOM3_PAD3 /* sck */
#define CONF_WINC_SPI_CS_PIN			PIN_PA17

#define CONF_WINC_SPI_MISO			PIN_PA16
#define CONF_WINC_SPI_MOSI			PIN_PA18
#define CONF_WINC_SPI_SCK			PIN_PA19
#define CONF_WINC_SPI_SS			PIN_PA17

#define CONF_WINC_SPI_INT_PIN	PIN_PA15A_EIC_EXTINT15
#define CONF_WINC_SPI_INT_MUX	MUX_PA15A_EIC_EXTINT15
#define CONF_WINC_SPI_INT_EIC	(15)

// SAMR21 X-PRO 세팅용 //
/*
#define CONF_WINC_SPI_MODULE			SERCOM5
#define CONF_WINC_SPI_SERCOM_MUX		SPI_SIGNAL_MUX_SETTING_E
#define CONF_WINC_SPI_PINMUX_PAD0		PINMUX_PB02D_SERCOM5_PAD0	// in
#define CONF_WINC_SPI_PINMUX_PAD1		PINMUX_UNUSED				// cs driven from software
#define CONF_WINC_SPI_PINMUX_PAD2		PINMUX_PB22D_SERCOM5_PAD2	// out
#define CONF_WINC_SPI_PINMUX_PAD3		PINMUX_PB23D_SERCOM5_PAD3	// sck
#define CONF_WINC_SPI_CS_PIN			PIN_PB03

#define CONF_WINC_SPI_MISO				PIN_PB02
#define CONF_WINC_SPI_MOSI				PIN_PB22
#define CONF_WINC_SPI_SCK				PIN_PB23
#define CONF_WINC_SPI_SS				PIN_PB03

// SPI interrupt pin. //
#define CONF_WINC_SPI_INT_PIN			PIN_PA22A_EIC_EXTINT6
#define CONF_WINC_SPI_INT_MUX			PINMUX_PA22A_EIC_EXTINT6
#define CONF_WINC_SPI_INT_EIC			(6)
*/
/** SPI clock. */
#define CONF_WINC_SPI_CLOCK				(12000000)

/*
   ---------------------------------
   --------- Debug Options ---------
   ---------------------------------
*/

#define CONF_WINC_DEBUG					(1)
#define CONF_WINC_PRINTF				printf

#ifdef __cplusplus
}
#endif

#endif /* CONF_WINC_H_INCLUDED */
