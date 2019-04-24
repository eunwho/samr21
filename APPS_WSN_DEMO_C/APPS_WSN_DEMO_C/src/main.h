/**
 * \file
 *
 * \brief MAIN configuration.
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
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
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
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

#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

// wifi setting
#define MAIN_WLAN_SSID                    "sunhst_2.4GHz" /**< Destination SSID */
#define MAIN_WLAN_AUTH                    M2M_WIFI_SEC_WPA_PSK /**< Security manner */
//#define MAIN_WLAN_PSK                     "05131103" /**< Password for Destination SSID */
#define MAIN_WLAN_PSK                     "12345678" /**< Password for Destination SSID */

//#define MAIN_WIFI_M2M_PRODUCT_NAME        "WINC1500-TEST"
#define MAIN_WIFI_M2M_PRODUCT_NAME        "tt"
#define GATEWAY_FULL				      "Socket MAX\n"
//#define MAIN_WIFI_M2M_SERVER_IP           0x79FEA2F9	// 121.254.162.249
//#define MAIN_WIFI_M2M_SERVER_IP           //0xc0a80164 //0xFFFFFFFF /* 255.255.255.255 */
#define MAIN_WIFI_M2M_SERVER_IP				0xC0A80032
#define MAIN_WIFI_M2M_SERVER_PORT         (502)
#define MAIN_WIFI_M2M_REPORT_INTERVAL     (1000)

#define MAIN_WIFI_M2M_BUFFER_SIZE          1460

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_INCLUDED */
