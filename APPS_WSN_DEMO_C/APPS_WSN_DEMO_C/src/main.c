/**
* \file  main.c
*/

#include "header.h"
#include "extern.h"
#include "global.h"

/*
#include "asf.h"
#include "main.h"
#include "sio2host.h"
#include "wsndemo.h"
#include "miwi_api.h"

#include "common/include/nm_common.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "define.h"
#include "global.h"
*/

void configure_rtc_count(void);

struct rtc_module rtc_instance;

void configure_rtc_count(void)
{
	struct rtc_count_config config_rtc_count;
	rtc_count_get_config_defaults(&config_rtc_count);
	config_rtc_count.prescaler           = RTC_COUNT_PRESCALER_DIV_1;
	config_rtc_count.mode                = RTC_COUNT_MODE_32BIT;
	#ifdef FEATURE_RTC_CONTINUOUSLY_UPDATED
	config_rtc_count.continuously_update = true;
	#endif
	rtc_count_init(&rtc_instance, RTC, &config_rtc_count);
	rtc_count_enable(&rtc_instance);
}


volatile unsigned int clock_count_timer = 0;

/* Receive buffer definition. */
static uint8_t gau8SocketTestBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];

/* Socket for client */
static SOCKET tcp_client_socket = -1;
/* Socket for server */
static SOCKET tcp_server_socket = -1;
/* Wi-Fi connection state */
static uint8_t wifi_connected;
volatile unsigned int tc_sol_check;
unsigned int ib;

//! [module_inst]
struct tc_module tc_instance;
struct tc_config config_tc;
//! [module_inst]

_Bool CoilCheckData(unsigned char arrPoint,int number);
char buff[6]={0,};
char ch_buff[5]={0,};
char sol_valve[200]={'2',};

char F_condi[12]={'2',};
char R_condi[12]={'2',};

_Bool F_sol_valve[100];
_Bool R_sol_valve[100];

bool test_flag =true;
int Cm_arr_count = 0;
int sen_num=0;
unsigned char slaveDef[12] = {0x00,0x00,0x00,0x00,0x00,0x06,0x01,0x01,0x00,0x00,0x00,0xFF};
unsigned char slaveDef2[14];
unsigned char CoilDataArray[32];

//int set_485 = -1 ;

unsigned int senser_arrive_count[128];
_Bool senser_arrive_bool[100];

_Bool zigbee_setdata = false;


volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 TCP client example --"STRING_EOL \
"-- "BOARD_NAME " --"STRING_EOL	\
"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

typedef struct s_msg_wifi_product {
	uint8_t name[9];
} t_msg_wifi_product;

/*
static t_msg_wifi_product msg_wifi_product = {
	.name = MAIN_WIFI_M2M_PRODUCT_NAME,
};
*/

uint8_t bufDout[]={0x02,0x52,'A','0','0','0','0','0','0','0','0',0x03,0x00};
volatile uint16_t temp;

void configure_tc(void);
void configure_tc_callbacks(void);

static void wifi_cb(uint8_t u8MsgType, void *pvMsg);
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg);

void initGpio(void);
void tc_isr(struct tc_module *const module_inst);

void usart_read_callback(struct usart_module *const usart_module)
{
	usart_write_buffer_job(&usart_instance,
	(uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);
}

void usart_write_callback(struct usart_module *const usart_module)
{
	port_pin_toggle_output_level(LED_0_PIN);
}

void readMacAddress(void);
void configure_usart(void);
void configure_usart_callbacks(void);
void config_rs485_TX_EN(void);

// for 485
void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = 9600;
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_usart.pinmux_pad2 = PINMUX_PB22D_SERCOM5_PAD2;		// Tx
	config_usart.pinmux_pad3 = PINMUX_PB23D_SERCOM5_PAD3;		// Rx

	while (usart_init(&usart_instance, SERCOM5, &config_usart) != STATUS_OK) {
	}

	usart_enable(&usart_instance);
}

void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_instance, usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);

	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}

void initGpio(void)
{
	PORT->Group[0].DIRSET.bit.DIRSET=PORT_PA27;	// RS3485 ENABLE PIN CONTROL PORT
}

uint8_t stTest[] = "Hellow World!\r\n";

uint32_t getElapRtc( uint32_t rtc_tag);

uint32_t getElapRtc( uint32_t rtc_tag){
	uint32_t temp32,ret32;
	temp32 = rtc_count_get_count( & rtc_instance);
	if( temp32 < rtc_tag){
		ret32 = (uint32_t)(0xFFFFFFFF) - rtc_tag + temp32;
	} else {
		ret32 = temp32 - rtc_tag;
	}
	return ret32;
}

uint32_t rtcCount, rtcCountTag, rtcTagSocket;
int main ( void )
{	
	int i;
	tstrWifiInitParam param;
	int8_t ret;
	struct sockaddr_in addr;

	irq_initialize_vectors();
	system_init();
	delay_init();
	cpu_irq_enable();	

	initGpio();
//	configure_tc();
//	configure_tc_callbacks();
//	configure_rtc_count( );

	config_i2c_GLCD_Select();
	configure_i2c_master();
	port_pin_set_output_level(PIN_PA23, false); // GLCD /EN high
	delay_us(10);
	gLcdInit();

	gLcdShow2(gLcdRunPic);
	delay_ms(1000);
	gLcdShow2(gLcdStopPic);
	delay_ms(1000);

	sio2host_init();
	printf("sio2host_init Ok");

//--- socket to PLC	
	nm_bsp_init();		// WINC1500 핀 설정 및 초기화

	// Initialize socket address structure. 
	addr.sin_family = AF_INET;
	addr.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT);
	addr.sin_addr.s_addr = _htonl(MAIN_WIFI_M2M_SERVER_IP); // IP

	// Initialize Wi-Fi parameters structure.
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));	// param 초기화
	param.pfAppWifiCb = wifi_cb;	
	ret = m2m_wifi_init(&param);	
	
	if(M2M_SUCCESS != ret){
		//		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while(1);
	}

	socketInit();
	registerSocketCallback(socket_cb, NULL);

	m2m_wifi_connect((char *)MAIN_WLAN_SSID,sizeof(MAIN_WLAN_SSID),
		MAIN_WLAN_AUTH,(char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);	

	system_interrupt_enable_global();	//! [enable_global_interrupts]
	readMacAddress();
	wsndemo_init();
	configure_usart();

	while (true) {
		wsndemo_task();

		m2m_wifi_handle_events(NULL);	// Handle pending events from network controller. //
		if(wifi_connected == M2M_WIFI_CONNECTED){
			if(tcp_client_socket < 0){
				if((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0){
					continue; 
				}
				ret = connect(tcp_client_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
				if(ret < 0) {
					close(tcp_client_socket); tcp_client_socket = -1;
				}
			} else {
				rtcCount = getElapRtc(rtcCountTag);
				if( rtcCount > 500 ){
					rtcCountTag = rtc_count_get_count( & rtc_instance);
					if(test_flag){
						send(tcp_client_socket, &slaveDef, sizeof(slaveDef), 0);	// origin
						test_flag=false;
					} else {
						for( i = 0 ; i < 4 ; i++ )	write_plc[13 + i] = sensStateTable[i];
						send(tcp_client_socket, &write_plc, sizeof(write_plc), 0);
						// txd_en;	usart_write_buffer_wait(&usart_instance, bufDout0, sizeof(bufDout));	rxd_en;
						test_flag=true;
					}
					//delay_ms(500);
				}
			}			
			if(tcp_server_socket < 0) {
				if((tcp_server_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0){
					continue;
				}
				bind(tcp_server_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
			}
		}
	}
	return 0;
}

void readMacAddress(void){
	myLongAddress[0] = 16;
	myLongAddress[1] = 53;
	myLongAddress[2] = 0;
	myLongAddress[3] = 32;
	myLongAddress[4] = 89;
	myLongAddress[5] = 37;
	myLongAddress[6] = 128;
	myLongAddress[7] = 255;
}

_Bool CoilCheckData(unsigned char arrPoint,int number)
{
	volatile _Bool state = false;
	volatile unsigned char flag = 0;
	flag |= arrPoint;
	
	switch(number)
	{
		case 1:	state = flag & 1	? true : false;	break;
		case 2: state = flag & 2	? true : false;	break;
		case 3:	state = flag & 4	? true : false;	break;
		case 4:	state = flag & 8	? true : false;	break;
		case 5:	state = flag & 16	? true : false;	break;
		case 6:	state = flag & 32	? true : false;	break;
		case 7:	state = flag & 64	? true : false;	break;
		case 8:	state = flag & 128	? true : false;	break;
	}
	
	return state;
}

void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	int i;
	switch(u8Msg) 
	{
		case SOCKET_MSG_BIND:
		{
			tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;			
			if(pstrBind && pstrBind->status == 0) {
				listen(tcp_server_socket, 0);				
			} else {
				close(tcp_server_socket);
				tcp_server_socket = -1;
			}
		}
		break;	

		case SOCKET_MSG_LISTEN:
		{
			tstrSocketListenMsg *pstrListen = (tstrSocketListenMsg *)pvMsg;
			if(pstrListen && pstrListen->status == 0){
				accept(tcp_server_socket, NULL, NULL);
			} else {
				close(tcp_server_socket);
				tcp_server_socket = -1;
			}
		}
		break;
		
		case SOCKET_MSG_ACCEPT:
		{				
			tstrSocketAcceptMsg *pstrAccept = (tstrSocketAcceptMsg *)pvMsg;
			
			if(pstrAccept) {
				accept(tcp_server_socket, NULL, NULL);
				tcp_client_socket = pstrAccept->sock;
				recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 1000);		
			} else {
				close(tcp_server_socket);
				tcp_server_socket = -1;				
			}
		}
		
		case SOCKET_MSG_CONNECT:
		{
			tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
			
			if(pstrConnect && pstrConnect->s8Error >= 0){
				// slaveDef[]
				send(tcp_client_socket, &slaveDef, sizeof(slaveDef), 0);
			} else {
				close(tcp_client_socket);
				tcp_client_socket = -1;
			}
		}
		break;

		case SOCKET_MSG_SEND:
			recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
			break;
		
		case SOCKET_MSG_RECV: 
		{
			tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
			if(pstrRecv && pstrRecv->s16BufferSize > 0)
			{
				switch(pstrRecv->pu8Buffer[7]) { // read coil
					case 1:	
					for( i = 0 ; i < 4 ; i++){	
						bufDout[ 3+i*2 ] = pstrRecv->pu8Buffer[9+i] & 0x0F ;
						(bufDout[3+i*2 ] > 9) ? ( bufDout[3+i*2 + 1 ] += 'A') : (bufDout[3+i*2] += '0');
						bufDout[3+i*2+1] = (( pstrRecv->pu8Buffer[9+i] >> 4 ) & 0x0F );
						(bufDout[3+i*2+1] > 9) ? ( bufDout[3+i*2] += 'A') : (bufDout[3+i*2+1] += '0');						
					}
//--- digital out proc
					rtcCount = getElapRtc(rtcCountTag);
					//printf("rxd from Plc = %s,count=%lu \r\n",bufDout,rtcCount);
					txd_en;	usart_write_buffer_wait(&usart_instance, bufDout, sizeof(bufDout));	rxd_en;
					break;	
				}
			} else {
				close(tcp_client_socket);
				tcp_client_socket = -1;
			}
		}
		break;			
		default:break;	
	}	
}

void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch(u8MsgType){
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if(pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			// printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if(pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			// printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
			wifi_connected = 0;
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), 
			MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
		}
		break;
	}
	case M2M_WIFI_REQ_DHCP_CONF:
		wifi_connected = 1;
		break;
	default:
		break;
	}
}

// [callback_funcs]
void tc_isr(struct tc_module *const module_inst)
{
	if(zigbee_setdata){
		clock_count_timer++;
		if(clock_count_timer>=200) {				
			clock_count_timer=0;
			buff[0]=' ';
			buff[1]=' ';
			buff[2]=' ';
			buff[3]=' ';			
			buff[4]=' ';
		}
	}
}

void configure_tc(void)
{	
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size    = TC_COUNTER_SIZE_16BIT;
	config_tc.wave_generation = TC_WAVE_GENERATION_NORMAL_PWM;
	config_tc.counter_16_bit.compare_capture_channel[0] = 0xFFFF;
	// tc_init(&tc_instance, PWM_MODULE, &config_tc);
	tc_enable(&tc_instance);	
}

void configure_tc_callbacks(void)
{
	//! [setup_register_callback]	
	tc_register_callback(&tc_instance,tc_isr,TC_CALLBACK_CC_CHANNEL0);
	//! [setup_register_callback]

	//! [setup_enable_callback]
	tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
	//! [setup_enable_callback]
}

// end of main.c