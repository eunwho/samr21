/**
* \file  main.c
*/
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
unsigned char slaveDef[12];

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

uint8_t bufDout[]={0x02,0x52,'W','P','A','0','0','F','F','F','F',0x03,0x00};
volatile uint16_t temp;

void configure_tc(void);
void configure_tc_callbacks(void);

static void wifi_cb(uint8_t u8MsgType, void *pvMsg);
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg);

void usart_read_callback(struct usart_module *const usart_module);
void usart_write_callback(struct usart_module *const usart_module);
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

void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = 9600;
	
//	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
//	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
//	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
//	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
//	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;

	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	// config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	// config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
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

int main ( void )
{	
	tstrWifiInitParam param;
	int8_t ret;
	struct sockaddr_in addr;

	irq_initialize_vectors();
	system_init();
	delay_init();
	cpu_irq_enable();	

	initGpio();
	configure_tc();
//	configure_tc_callbacks();

	sio2host_init();
	readMacAddress();
	wsndemo_init();
/*	
	nm_bsp_init();		// WINC1500 핀 설정 및 초기화

	// Initialize socket address structure. 
	addr.sin_family = AF_INET;
	addr.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT);
	addr.sin_addr.s_addr = _htonl(MAIN_WIFI_M2M_SERVER_IP); // IP

	// Initialize Wi-Fi parameters structure.
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));	// param 초기화
	param.pfAppWifiCb = wifi_cb;	ret = m2m_wifi_init(&param);	
	
	if(M2M_SUCCESS != ret){
		//		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while(1);
	}
	// Initialize socket module 
	socketInit();
	registerSocketCallback(socket_cb, NULL);
	// Connect to router. 
	m2m_wifi_connect((char *)MAIN_WLAN_SSID,sizeof(MAIN_WLAN_SSID),
		MAIN_WLAN_AUTH,(char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);	
	system_interrupt_enable_global();	//! [enable_global_interrupts]
*/
	configure_usart();
	
	while (true) {
		wsndemo_task();
/*
		m2m_wifi_handle_events(NULL);	// Handle pending events from network controller. //

		if(wifi_connected == M2M_WIFI_CONNECTED){
			if(tcp_client_socket < 0){
				if((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0){
					continue; //txd_en; printf("main: failed to create TCP client socket error!\r\n");rxd_en;
				}
				ret = connect(tcp_client_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
				if(ret < 0) {
					close(tcp_client_socket); tcp_client_socket = -1;
				}
			}
			
			if(tcp_server_socket < 0) {
				if((tcp_server_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0){
					//txd_en;  printf("main: failed to create TCP server socket error!\r\n"); rxd_en;
					continue;
				}
				bind(tcp_server_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
			} else {
			//txd_en; printf("Wifi disconnected"); rxd_en;
			}
		}
*/	}

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
	myLongAddress[7] = 127;
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
	switch(u8Msg) 
	{	/* Socket BIND*/
		case SOCKET_MSG_BIND:
		{
			tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;			
			if(pstrBind && pstrBind->status == 0) {
				// printf("socket_cb: bind success!\r\n");
				listen(tcp_server_socket, 0);				
			} else {
//				printf("socket_cb: bind error!\r\n");
				close(tcp_server_socket);
				tcp_server_socket = -1;
			}
		}
		break;	
		case SOCKET_MSG_LISTEN:
		{
			tstrSocketListenMsg *pstrListen = (tstrSocketListenMsg *)pvMsg;
			if(pstrListen && pstrListen->status == 0) 
			{
//				printf("socket_cb: listen success!\r\n");
				accept(tcp_server_socket, NULL, NULL);
			} 
			else 
			{
//				printf("socket_cb: listen error!\r\n");
				close(tcp_server_socket);
				tcp_server_socket = -1;
			}
		}
		break;
		
		/* Connect accept */
		case SOCKET_MSG_ACCEPT:
		{				
			tstrSocketAcceptMsg *pstrAccept = (tstrSocketAcceptMsg *)pvMsg;
			
			if(pstrAccept) 
			{
				//printf("socket_cb: accept success!\r\n");
				accept(tcp_server_socket, NULL, NULL);
				tcp_client_socket = pstrAccept->sock;
				recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 1000);		
			} 
			else 
			{
				//txd_en; printf("socket_cb: accept error!\r\n");	rxd_en;
				close(tcp_server_socket);
				tcp_server_socket = -1;				
			}
		}
		
		/* Socket connected */
		case SOCKET_MSG_CONNECT:
		{
			tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
			
			if(pstrConnect && pstrConnect->s8Error >= 0) 
			{
				//READ				
				slaveDef[0]  = 0x00; // transaction id
				slaveDef[1]  = 0x00; // transaction id
				slaveDef[2]  = 0x00; // protocol id
				slaveDef[3]  = 0x00; // protocol id
				slaveDef[4]  = 0x00; // length
				slaveDef[5]  = 0x06; // length 4-5  //unit id부터 바이트 길이를 표시
				slaveDef[6]  = 0x01; // unit id
				slaveDef[7]  = 0x01; // function
				slaveDef[8]  = 0x00; // Data 8byte -> 1 word
				slaveDef[9]  = 0x00; // ..
				slaveDef[10] = 0x00; // ..
				slaveDef[11] = 0xFF; // ..
				//txd_en;
				//printf("CON\r\n");
				//rxd_en;
				send(tcp_client_socket, &slaveDef, sizeof(slaveDef), 0);
			} 
			else 
			{
				//txd_en;printf("socket_cb: connect error!\r\n");rxd_en;
				close(tcp_client_socket);
				tcp_client_socket = -1;
			}
		}
		break;

		/* Message send */
		case SOCKET_MSG_SEND:
		{
			//printf("socket_cb: send success!\r\n");
			//printf("TCP Server Test Complete!\r\n");
			//printf("close socket\n");
			//close(tcp_client_socket); tcp_client_socket =-1;
			/*txd_en;
			printf("SEND \r\n");
			rxd_en;*/
			recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
		}
		break;
		
		/* Message receive */
		case SOCKET_MSG_RECV: {
			tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
			if(pstrRecv && pstrRecv->s16BufferSize > 0)
			{
				switch(pstrRecv->pu8Buffer[7]) { // read coil
					case 1:	
					delay_us(10);			
					// pu8Buffer[9]번부터 8bit씩 순서대로 PLC 코일출력 상태 값을 저장한다.
					// pu8Buffer[9]의 경우 하위비트부터 1번째~8번째의 코일출력 값을 가진다.
					// 9번째로 넘어가면 pu8Buffer[10]에 할당된다
					// process only 4 byte 0f 10byte 
					
					for(int i = 0 ; i < 4 ; i++){	
						bufDout[3+i*2] = (( pstrRecv->pu8Buffer[9+i] >> 4 ) & 0x0F );
						(bufDout[3+i*2] > 9) ? ( bufDout[3+i*2] += 'A') : (bufDout[3+i*2] += '0');

						bufDout[ 3+i*2+1 ] = pstrRecv->pu8Buffer[9+i] & 0x0F ;
						(bufDout[3+i*2+1 ] > 9) ? ( bufDout[3+i*2 + 1 ] += 'A') : (bufDout[3+i*2 + 1] += '0');
					}
					txd_en;printf("%s",bufDout);rxd_en;
					break;
				}
				
				//솔밸브 1개당 1비트 2개의 상태를 가짐 감지됨 or 비감지 PLC로 보내질 센서값
				//int sensorState[sizeof(sol_valve)];				
				//	200개 해야지 센서 100개
				//갯수만큼 늘려줘야함. 1개 기준 14개
				//
				//unsigned char write_plc[14];
				write_plc[17];
//				int wpl=0;
				//1개당 전진 후진을 정하고있음.
				//8비트라서 2개 씩 처리 해야함 값 1byte당 4개의 솔밸브를 처리할수 있음
				unsigned char wr_value= 0x00 ;
				//txd_en;
				//printf(" [%d%d%d%d%d%d] \r\n",sol_valve[0],sol_valve[1],sol_valve[2],sol_valve[3],sol_valve[4],sol_valve[5]);
				//rxd_en;
				int byteup=0;
				//초기화
//				write_plc[13]  = 0x00;
//				write_plc[14]  = 0x00;
//				write_plc[15]  = 0x00;
//				write_plc[16]  = 0x00;
				
				for(int sen_len=0; sen_len<128; sen_len++)
				{					
					switch((sen_len%8))
					{
						case 0: if(sol_valve[sen_len]=='0'){ wr_value += 0x00; }else if(sol_valve[sen_len]=='1'){ wr_value += 0x01;	}else{ wr_value += 0x00; }break; // 0 8 16 ....
						case 1: if(sol_valve[sen_len]=='0'){ wr_value += 0x00; }else if(sol_valve[sen_len]=='1'){ wr_value += 0x02;	}else{ wr_value += 0x00; }break; // 1 9 17
						case 2: if(sol_valve[sen_len]=='0'){ wr_value += 0x00; }else if(sol_valve[sen_len]=='1'){ wr_value += 0x04;	}else{ wr_value += 0x00; }break;
						case 3: if(sol_valve[sen_len]=='0'){ wr_value += 0x00; }else if(sol_valve[sen_len]=='1'){ wr_value += 0x08;	}else{ wr_value += 0x00; }break;
						case 4: if(sol_valve[sen_len]=='0'){ wr_value += 0x00; }else if(sol_valve[sen_len]=='1'){ wr_value += 0x10;	}else{ wr_value += 0x00; }break;
						case 5: if(sol_valve[sen_len]=='0'){ wr_value += 0x00; }else if(sol_valve[sen_len]=='1'){ wr_value += 0x20;	}else{ wr_value += 0x00; }break;
						case 6: if(sol_valve[sen_len]=='0'){ wr_value += 0x00; }else if(sol_valve[sen_len]=='1'){ wr_value += 0x40;	}else{ wr_value += 0x00; }break;
						case 7: if(sol_valve[sen_len]=='0'){ wr_value += 0x00; }else if(sol_valve[sen_len]=='1'){ wr_value += 0x80;	}else{ wr_value += 0x00; }break;						
					}					
					if((sen_len%8)==7 && sen_len > 0){						
						if(byteup<4)	write_plc[13+byteup] = wr_value;						
						byteup++;
						wr_value=0x00;
					}else{
						//byteup++;
						//wr_value=0x00;
					}
				}

				slaveDef[0]  = 0x00; // transaction id
				slaveDef[1]  = 0x00; // transaction id
				slaveDef[2]  = 0x00; // protocol id
				slaveDef[3]  = 0x00; // protocol id
				slaveDef[4]  = 0x00; // length
				slaveDef[5]  = 0x06; // length 4-5
				slaveDef[6]  = 0x01; // unit id
				slaveDef[7]  = 0x01; // function
				slaveDef[8]  = 0x00; // Data 8byte -> 1 word
				slaveDef[9]  = 0xA0; // ..
				slaveDef[10] = 0x00; // ..
				slaveDef[11] = 0xFF; // ..
				
				if(test_flag){
					send(tcp_client_socket, &slaveDef, sizeof(slaveDef), 0);	
					test_flag=false;
				} else {
					write_plc[0]   = 0x00; // 1   ] transaction id
					write_plc[1]   = 0x00; // 2   ] transaction id
					write_plc[2]   = 0x00; // 3   ] protocol id
					write_plc[3]   = 0x00; // 4   ] protocol id
					write_plc[4]   = 0x00; // 5   ] length
					write_plc[5]   = 0x0B; // 6	  ] length
					write_plc[6]   = 0x01; // 7-01] unit id
					write_plc[7]   = 0x0F; // 7-02] function
					write_plc[8]   = 0x00; // 7-03] Starting Addr
					write_plc[9]   = 0x00; // 7-04] Starting Addr   //0x0000 0번부터 입력 
					write_plc[10]  = 0x00; // 7-05] Quantity of Outputs
					write_plc[11]  = 0x20; // 7-06] Quantity of Outputs  //0x0020 32개 제어
					write_plc[12]  = 0x04; // 7-07] byte Count
					//write_plc[13]  = 0xFF; // 7-08] 8 bit
					//write_plc[14]  = 0xFF; // 7-09] 8 bit
					//send(tcp_client_socket, &slaveDef, sizeof(slaveDef), 0);
					send(tcp_client_socket, &write_plc, sizeof(write_plc), 0);					
					test_flag=true;					
					delay_ms(75);
				}
			} else {
				//txd_en;printf("socket_cb: recv error! %d \r\n",pstrRecv->s16BufferSize);rxd_en;				
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