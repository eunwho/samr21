#include "main.h"
#include "wireless_api.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "conf_quick_start_callback.h"

#define  txd_en PORT->Group[0].OUTSET.reg=PORT_PA27
#define  rxd_en PORT->Group[0].OUTCLR.reg=PORT_PA27

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

static void tc_isr(struct tc_module *const module_inst);
static void configure_tc(void);
static void configure_tc_callbacks(void);

static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg);
static void wifi_cb(uint8_t u8MsgType, void *pvMsg);

static void app_task(void);
static void WirelessTask(void);
static void init_port(void);


void configure_usart(void);
void configure_usart_callbacks(void);

void usart_write_callback(struct usart_module *const usart_module);
void usart_read_callback(struct usart_module *const usart_module);

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


//! [rx_buffer_var]
#define MAX_RX_BUFFER_LENGTH   5

//! [module_inst]
struct usart_module usart_instance;
//! [module_inst]

volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];

//! [setup]
void configure_usart(void)
{
	//! [setup_config]
	struct usart_config config_usart;
	//! [setup_config]
	//! [setup_config_defaults]
	usart_get_config_defaults(&config_usart);
	//! [setup_config_defaults]

	//! [setup_change_config]
	config_usart.baudrate    = 9600;
	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	//! [setup_change_config]

	//! [setup_set_config]
	while (usart_init(&usart_instance,
	EDBG_CDC_MODULE, &config_usart) != STATUS_OK) {
	}
	//! [setup_set_config]

	//! [setup_enable]
	usart_enable(&usart_instance);
	//! [setup_enable]
}

void usart_write_callback(struct usart_module *const usart_module)
{
	port_pin_toggle_output_level(LED_0_PIN);
}

//! [callback_funcs]
void usart_read_callback(struct usart_module *const usart_module)
{
	
	usart_write_buffer_job(&usart_instance,
	(uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);
}


void configure_usart_callbacks(void)
{
	//! [setup_register_callbacks]
	usart_register_callback(&usart_instance,
	usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance,
	usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	//! [setup_register_callbacks]

	//! [setup_enable_callbacks]
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
	//! [setup_enable_callbacks]
}
//! [setup]

uint8_t writeDout[]={0x02,0x52,'A','A','A','A','A','A','A','A','A',0x03,0x00};
int debug1;

int main(void)
{				
	tstrWifiInitParam param;
	int8_t ret;
	struct sockaddr_in addr;
	
	/* Initialize the board. */
	system_init();	// MCU 설정및 초기화 부분	
	
	/* Initialize GPIO. */
	init_port();
	
	/* Initialize the UART console. */
	modules_init();		// UART 설정및 초기화
	
	//! [TC_setup_init]
	configure_tc();
	configure_tc_callbacks();
	//! [TC_setup_init]
	
	/* Zigbee Initialize */
	wireless_init();

	/* Initialize the BSP. */
	nm_bsp_init();		// WINC1500 핀 설정 및 초기화

	/* Initialize socket address structure. */
	addr.sin_family = AF_INET;
	addr.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT);
	addr.sin_addr.s_addr = _htonl(MAIN_WIFI_M2M_SERVER_IP); // IP

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));	// param 초기화

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	
	if(M2M_SUCCESS != ret){
//		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while(1);
	}

	/* Initialize socket module */
	socketInit();
	registerSocketCallback(socket_cb, NULL);

	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID,sizeof(MAIN_WLAN_SSID),
			MAIN_WLAN_AUTH,(char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
	
	system_interrupt_enable_global();	//! [enable_global_interrupts]
	
	configure_usart();
	configure_usart_callbacks();

	while(1)
	{			
		m2m_wifi_handle_events(NULL);	// Handle pending events from network controller. //
		if(wifi_connected == M2M_WIFI_CONNECTED) 
		{
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
			}		
		} else {
			//txd_en; printf("Wifi disconnected"); rxd_en;
		}
		
		WirelessTask();
		
		usart_read_buffer_job(&usart_instance, (uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);		
		
 		sen_num = ((buff[1]-'0')*100)+((buff[2]-'0')*10)+(buff[3]-'0');
		
		if(buff[4]=='0'){
			switch(sen_num){
	 //시작주소부터 0번째에 상태값 입력 // 센서로부터 수신이 되었다는 것을 체크 //
				case 1: sol_valve[0] = '0';	break;
				case 2: sol_valve[1] = '0';	break;
				case 3: sol_valve[2] = '0';	break;
				case 4: sol_valve[3] = '0';	break;
				case 5: sol_valve[4] = '0';	break;
				case 6: sol_valve[5] = '0';	break;
				case 7: sol_valve[6] = '0';	break;
				case 8: sol_valve[7] = '0';	break;
				case 9: sol_valve[8] = '0';	break;
				case 10: sol_valve[9] = '0';	break;
				case 11: sol_valve[10] = '0';	break;
				case 12: sol_valve[11] = '0';	break;
				case 13: sol_valve[12] = '0';	break;
				case 14: sol_valve[13] = '0';	break;
			}
		}
		else if(buff[4]=='1'){
			switch(sen_num){
				case 1: sol_valve[0] = '1';	break;
				case 2: sol_valve[1] = '1';	break;
				case 3: sol_valve[2] = '1';	break;
				case 4: sol_valve[3] = '1';	break;
				case 5: sol_valve[4] = '1';	break;
				case 6: sol_valve[5] = '1';	break;
				case 7: sol_valve[6] = '1';	break;
				case 8: sol_valve[7] = '1';	break;
				case 9: sol_valve[8] = '1';	break;
				case 10: sol_valve[9] = '1';	break;
				case 11: sol_valve[10] = '1';	break;
				case 12: sol_valve[11] = '1';	break;
				case 13: sol_valve[12] = '1';	break;
				case 14: sol_valve[13] = '1';	break;
			}
		}
	}
	return 0;
}

//PLC coil read의 데이터를 체크

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

	
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
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
						writeDout[3+i*2] = (( pstrRecv->pu8Buffer[9+i] >> 4 ) & 0x0F );
						(writeDout[3+i*2] > 9) ? ( writeDout[3+i*2] += 'A') : (writeDout[3+i*2] += '0');

						writeDout[ 3+i*2+1 ] = pstrRecv->pu8Buffer[9+i] & 0x0F ;
						(writeDout[3+i*2+1 ] > 9) ? ( writeDout[3+i*2 + 1 ] += 'A') : (writeDout[3+i*2 + 1] += '0');
					}
					txd_en;printf("%s",writeDout);rxd_en;
					break;
				}
				
				//솔밸브 1개당 1비트 2개의 상태를 가짐 감지됨 or 비감지 PLC로 보내질 센서값
				//int sensorState[sizeof(sol_valve)];				
				//	200개 해야지 센서 100개
				//갯수만큼 늘려줘야함. 1개 기준 14개
				//
				//unsigned char write_plc[14];
				unsigned char write_plc[17];
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

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch(u8MsgType) 
	{
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
	//! [setup_config_defaults]
	tc_get_config_defaults(&config_tc);
	//! [setup_config_defaults]

	//! [setup_change_config]
	config_tc.counter_size    = TC_COUNTER_SIZE_16BIT;
	config_tc.wave_generation = TC_WAVE_GENERATION_NORMAL_PWM;
	config_tc.counter_16_bit.compare_capture_channel[0] = 0xFFFF;
	//! [setup_change_config]

	//! [setup_change_config_pwm]
	config_tc.pwm_channel[0].enabled = true;
	config_tc.pwm_channel[0].pin_out = PWM_OUT_PIN;
	config_tc.pwm_channel[0].pin_mux = PWM_OUT_MUX;
	//! [setup_change_config_pwm]

	//! [setup_set_config]
	tc_init(&tc_instance, PWM_MODULE, &config_tc);
	//! [setup_set_config]

	//! [setup_enable]
	tc_enable(&tc_instance);
	//! [setup_enable]
	
	
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
//! [setup]
//=======================================================================================//

void init_port(void)
{
	PORT->Group[0].DIRSET.bit.DIRSET=PORT_PA27;	// RS3485 ENABLE PIN CONTROL PORT	
	rxd_en;
}


/*
 * \brief Tasks Handled By the Stack and application are Performed here,
 *  this function is handled in a loop to perform tasks continuously 
 */
void WirelessTask(void)
{
	/* These methods are called to perform the default tasks of the MAC Stack */
	pal_task();
	tal_task();

	/* Custom (User) tasks are performed here. */
	app_task();
}

/*
 * \brief Application task
 */
 void app_task(void)
{
//	usr_wireless_app_task();
	init_data_reception();
}

/*
 * \brief This method (callback) is called when a frame has been transmitted by the transceiver
 * \param status  Status of frame transmission i.e MAC_SUCCESS,MAC_NO_ACK,CHANNEL_ACCESS_FAILURE etc
 * \param frame pointer to the transmitted frame
 */
void tal_tx_frame_done_cb(retval_t status, frame_info_t *frame)
{
	/*Perform application tasks when a frame is transmitted here*/
	usr_frame_transmitted_cb(status, frame);
}

/*
 * \brief This method (callback) is called when a frame is received by the transceiver
 * \param frame pointer to the received frame
 */
void tal_rx_frame_cb(frame_info_t *frame)
{
	/*Perform application tasks when a frame is received here*/
	usr_frame_received_cb(frame);
	/*frame->mpdu[frame]
	&(frame->mpdu[FRAME_OVERHEAD-FCS_LEN+1])
	*/
	//
	
	// Free-up the buffer which was used for reception once the frame is extracted.
	bmm_buffer_free(frame->buffer_header);
}