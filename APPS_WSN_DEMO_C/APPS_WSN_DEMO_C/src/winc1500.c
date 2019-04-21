/*
 * winc1500.c
 */
#include "header.h"
#include "extern.h"

//#define  txd_en PORT->Group[0].OUTSET.reg=PORT_PA27
//#define  rxd_en PORT->Group[0].OUTCLR.reg=PORT_PA27

unsigned int clock_count_timer = 0;

/* Receive buffer definition. */
uint8_t gau8SocketTestBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];

SOCKET tcp_client_socket = -1;		// Socket for client */
SOCKET tcp_server_socket = -1;		// Socket for server */

uint8_t wifi_connected;
unsigned int tc_sol_check;
unsigned int ib;

//! [module_inst]
struct tc_module tc_instance;
struct tc_config config_tc;
//! [module_inst]


char buff[6]={0,};
char ch_buff[5]={0,};
char sol_valve[200]={'2',};

char F_condi[12]={'2',};
char R_condi[12]={'2',};
	
_Bool F_sol_valve[100];
_Bool R_sol_valve[100];

bool test_flag =true;
int Cm_arr_count = 0;
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

void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	switch(u8Msg) 
	{	/* Socket BIND*/
		case SOCKET_MSG_BIND:
		{
			tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;
			
			if(pstrBind && pstrBind->status == 0) 
			{
//				printf("socket_cb: bind success!\r\n");
				listen(tcp_server_socket, 0);				
			} 
			else 
			{
//				printf("socket_cb: bind error!\r\n");
				close(tcp_server_socket);
				tcp_server_socket = -1;
			}
		}
		break;
		
		/* Socket listen */
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
		case SOCKET_MSG_RECV:
		{
			tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
			if(pstrRecv && pstrRecv->s16BufferSize > 0)
			{
				// size
	//			unsigned int sr = 8;
				// coil
	//			unsigned int lr = 9;		
				switch(pstrRecv->pu8Buffer[7])
				{
					case 1:				
					for(int i=0;i<9;i++)
					{						
						RS485_TXD_EN();	
						printf("s%d%03d\r\n",i+1,pstrRecv->pu8Buffer[9+i]);	
						RS485_RXD_EN();
					}	
//					zigbee_setdata = false;
					//txd_en;printf("s%d%003d \r\n",1, pstrRecv->pu8Buffer[9]);rxd_en;					
					//txd_en; printf("do");rxd_en;
					break;
				}
				
				//솔밸브 1개당 1비트 2개의 상태를 가짐 감지됨 or 비감지 PLC로 보내질 센서값
				//int sensorState[sizeof(sol_valve)];				
				//
				
				/*
					200개 해야지 센서 100개
				*/
				
				
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
				
				//count
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
					
					if((sen_len%8)==7 && sen_len > 0)
					{						
						if(byteup<4)
						{
							write_plc[13+byteup] = wr_value;						
						}
						byteup++;
						wr_value=0x00;
					}else{
						//byteup++;
						//wr_value=0x00;
					}
				}
				/*
				txd_en;
				printf("\r\nWRITE\r\n");
				printf("13]%02X\r\n",write_plc[13]);
				printf("14]%02X\r\n",write_plc[14]);
				printf("15]%02X\r\n",write_plc[15]);
				rxd_en;*/
				// 0 0 0 0 0 6 1 1 0 0 0 A
				// 0 0 0 0 0 6 1 5 0 5 0 0
				// 0 0 0 0 0 6 0 1 0 3 0 0
				// 0 0 0 0 0 3 1 133 2
				// 0 0 0 0 0 3 1 129 1
				
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
				
				if(test_flag)
				{
					send(tcp_client_socket, &slaveDef, sizeof(slaveDef), 0);	
					test_flag=false;
				}
				else
				{
					
					
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
				
				
				//delay_ms(500);
				//close(tcp_client_socket);
				//tcp_client_socket = -1;
//				sol_valve[3] = '1'; PLC와의 통신상태를 체크하여 모니터에 표시함
			}
			else
			{
				//txd_en;printf("socket_cb: recv error! %d \r\n",pstrRecv->s16BufferSize);rxd_en;				
//				sol_valve[3] ='0';	PLC와의 통신상태를 체크하여 모니터에 표시함
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
	switch(u8MsgType) 
	{
		case M2M_WIFI_RESP_CON_STATE_CHANGED:
		{
			tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
			if(pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) 
			{
//				printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
				m2m_wifi_request_dhcp_client();
			} 
			else if(pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) 
			{
//				printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
				wifi_connected = 0;
				m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), 
				MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
			}

			break;
		}

		case M2M_WIFI_REQ_DHCP_CONF:
		{
//			uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
			wifi_connected = 1;
//			printf("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
//					pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
			break;
		}

		default:break;
	}
}


// [callback_funcs]
void tc_isr(struct tc_module *const module_inst)
{
	
	// 2ms x 50 = 100ms
	
	if(zigbee_setdata){
		// 2ms x 50 = 100ms
		
		clock_count_timer++;
		if(clock_count_timer>=200)
		{				
			clock_count_timer=0;
//			sol_valve[0] = '0';
			buff[0]=' ';
			buff[1]=' ';
			buff[2]=' ';
			buff[3]=' ';			
			buff[4]=' ';
//			set_485 = 0;
		}
	}
	
	//센서값 수신체크
/*	for(int sen_tc=0; sen_tc<128; sen_tc++)
	{
		//senser_arrive_count[128];
		if(senser_arrive_bool[sen_tc]==true)
		{
			senser_arrive_count[sen_tc]=0;
			senser_arrive_bool[sen_tc] = false;
		}
		else
		{
			senser_arrive_count[sen_tc]++;
			if(senser_arrive_count[sen_tc]>=3000)	// 2ms * 3000 = 6s
			{
				//printf("%d SLerror-3\r\n",tc_sol_check);
				//lserr_flag=true;
				sol_valve[sen_tc] = '0';
				//F_condi[sen_tc]='2';
				senser_arrive_count[sen_tc] = false;
				//txd_en; printf("%d\r\n", sen_tc); rxd_en;
			}
		}
	}	*/
}

//int a=0;

//! [setup]
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
//	config_tc.pwm_channel[0].pin_out = PWM_OUT_PIN;
//	config_tc.pwm_channel[0].pin_mux = PWM_OUT_MUX;
	//! [setup_change_config_pwm]

	//! [setup_set_config]
//	tc_init(&tc_instance, PWM_MODULE, &config_tc);
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

void WirelessTask(void)
{
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

void wireless_init(void)
{
	/*Initialize IRQ*/
	irq_initialize_vectors();

//	system_init();
	system_board_init();

    delay_init();

	/*Initialize the Stack SW Timer*/
	//sw_timer_init();
	
	/*Initialize the TAL Layer*/
	if(tal_init()!= MAC_SUCCESS)
	{
		//Error in Initializing the TAL Layer
		app_alert();	
	}
//	cpu_irq_enable();
	
	uint16_t src_addr = CCPU_ENDIAN_TO_LE16(SRC_ADDR);
	uint16_t pan_id = CCPU_ENDIAN_TO_LE16(SRC_PAN_ID);
	uint8_t channel = CHANNEL_TRANSMIT_RECEIVE;
	uint8_t channel_page = CHANNEL_PAGE_TRANSMIT_RECEIVE;
	
	/* Set Default address. */
	tal_pib_set(macShortAddress, (pib_value_t *)&src_addr);

	/* Set PAN ID. */
	tal_pib_set(macPANId, (pib_value_t *)&pan_id);

	/* Set channel. */ /* Channel 11  is set as default in tal_init() */
	tal_pib_set(phyCurrentChannel, (pib_value_t *)&channel);
	
	/* Set Channel Page */ /* Channel Page 0  is set as default in tal_init() */
	tal_pib_set(phyCurrentPage,(pib_value_t *)&channel_page);
	
	//TODO (Project Wizard) - Change the Transmit Power of the transceiver if required
	/* The Following Lines of Code are used to set the transmit power */	
	/* 	int8_t tx_pwr_dbm = 3;	 // transmit power = 3dBm
	
	uint8_t temp_var;
	
	temp_var = CONV_DBM_TO_phyTransmitPower(tx_pwr_dbm);

	tal_pib_set(phyTransmitPower, (pib_value_t *)&temp_var);	 */
/*
	#if (ANTENNA_DIVERSITY == 1)
    if(ENABLE_ANTENNA_DIVERSITY)
    {
	    tal_ant_div_config(ANT_DIVERSITY_ENABLE,ANTENNA_DEFAULT);
    }
    else
    {
	    tal_ant_div_config(ANT_DIVERSITY_DISABLE,ANT_SELECT); 
    }
    #endif
*/

//	init_data_reception();
}

/*
 * \brief Function to transmit frames as per 802.15.4 std.
 *
 * \param dst_addr_mode     destination address mode - can be 16 or 64 bit
 * \param dst_addr          destination address
 * \param src_addr_mode     source address mode - can be 16 or 64 bit
 * \param msdu_handle       msdu handle for the upper layers to track packets
 * \param payload           data payload pointer
 * \param payload_length    data length
 * \param ack_req           specifies ack requested for frame if set to 1
 * \param csma_mode         specifies the csma mode for transmission
 * \param frame_retry       specifies whether frame retry is required or not
 *
 * \return MAC_SUCCESS      if the TAL has accepted the data for frame
 * transmission
 *         TAL_BUSY         if the TAL is busy servicing the previous tx request
 */
retval_t transmit_frame(uint8_t dst_addr_mode,
		uint8_t* dst_addr,
		uint8_t src_addr_mode,
		uint8_t msdu_handle,
		uint8_t *payload,
		uint8_t payload_length,
		bool ack_req,csma_mode_t csma_mode ,bool frame_retry)
{
	frame_info_t *tx_frame_info;
	uint8_t i;
	uint8_t frame_length;
	uint8_t *frame_ptr;
	uint8_t *temp_frame_ptr;
	uint16_t temp_value,fcf = 0;
	/* Get length of current frame. */

	tx_frame_info = (frame_info_t *)storage_buffer;
	/* Get length of current frame. */
	frame_length = (FRAME_OVERHEAD + payload_length); 

	/* Set payload pointer. */     
	frame_ptr = temp_frame_ptr = (uint8_t *)tx_frame_info +
	LARGE_BUFFER_SIZE -
	payload_length - FCS_LEN;

	/*
	* Payload is stored to the end of the buffer avoiding payload
	* copying by TAL.
	*/
	for (i = 0; i < payload_length; i++) {
	*temp_frame_ptr++ = *(payload + i);
	}


	/* Source address */
	if (FCF_SHORT_ADDR == src_addr_mode) {
	frame_ptr -= SHORT_ADDR_LEN;
	convert_16_bit_to_byte_array(tal_pib.ShortAddress, frame_ptr);

	fcf |= FCF_SET_SOURCE_ADDR_MODE(FCF_SHORT_ADDR);
	} else {
	frame_ptr -= EXT_ADDR_LEN;
	frame_length += FCF_2_SOURCE_ADDR_OFFSET;

	convert_64_bit_to_byte_array(tal_pib.IeeeAddress, frame_ptr);

	fcf |= FCF_SET_SOURCE_ADDR_MODE(FCF_LONG_ADDR);
	}

		/* Source PAN-Id */
		#if (DST_PAN_ID == SRC_PAN_ID)
		/* No source PAN-Id included, but FCF updated. */
		fcf |= FCF_PAN_ID_COMPRESSION;
		#else
		frame_ptr -= PAN_ID_LEN;
		temp_value = CCPU_ENDIAN_TO_LE16(SRC_PAN_ID);
		convert_16_bit_to_byte_array(temp_value, frame_ptr);
		#endif

	/* Destination address */
	if (FCF_SHORT_ADDR == dst_addr_mode) {
		frame_ptr -= SHORT_ADDR_LEN;
		convert_16_bit_to_byte_array(*((uint16_t *)dst_addr),
				frame_ptr);

		fcf |= FCF_SET_DEST_ADDR_MODE(FCF_SHORT_ADDR);
	} else {
		frame_ptr -= EXT_ADDR_LEN;
		frame_length += PL_POS_DST_ADDR_START;

		convert_64_bit_to_byte_array(*((uint64_t *)dst_addr),
				frame_ptr);

		fcf |= FCF_SET_DEST_ADDR_MODE(FCF_LONG_ADDR);
	}


	/* Destination PAN-Id */
	temp_value = CCPU_ENDIAN_TO_LE16(DST_PAN_ID);
	frame_ptr -= PAN_ID_LEN;
	convert_16_bit_to_byte_array(temp_value, frame_ptr);

	/* Set DSN. */
	frame_ptr--;
	*frame_ptr = msdu_handle;

	/* Set the FCF. */
	fcf |= FCF_FRAMETYPE_DATA;
	if (ack_req) {
	fcf |= FCF_ACK_REQUEST;
	}

	frame_ptr -= FCF_LEN;
	convert_16_bit_to_byte_array(CCPU_ENDIAN_TO_LE16(fcf), frame_ptr);

	/* First element shall be length of PHY frame. */
	frame_ptr--;
	*frame_ptr = frame_length;

	/* Finished building of frame. */
	tx_frame_info->mpdu = frame_ptr;

	return(tal_tx_frame(tx_frame_info, csma_mode, frame_retry));
}

/*
 *
 * \brief This function transmits a sample data frame 
 * \param payload address of the payload to be transmitted
 * \payload_length Length of the payload to be trasnmitted *
 */
void transmit_sample_frame(uint8_t* payload,uint8_t payload_length) 
{
	static uint16_t seq_num = 0;   
	bool ack_req = ACK_REQ;
	bool frame_retry = FRAME_RETRY;
	csma_mode_t csma_mode = CSMA_MODE;
	uint16_t dst_addr = CCPU_ENDIAN_TO_LE16((uint16_t)DST_ADDR);

	transmit_frame(
		FCF_SHORT_ADDR,
		(uint8_t*)&dst_addr,
		FCF_SHORT_ADDR,
		seq_num++,
		payload,
		payload_length,
		ack_req,
		csma_mode,
		frame_retry);

}

/** 
 * \brief The Receiver is switched on Using this function,
  * When PROMISCUOUS_MODE is enabled the receiver is put in RX_ON mode ,else it is switched on in RX_AACK_ON Mode
 */
void init_data_reception()
{
       /*Enable Promiscuous Mode pib attribute to put the transceiver in RX_ON mode.*/
       #ifdef PROMISCUOUS_MODE
		   bool mode = true;
		   tal_rxaack_prom_mode_ctrl(true);
		   tal_pib_set(macPromiscuousMode, (pib_value_t *)&mode);
       #endif
       /*RX_AACK_ON Mode is enabled if Promiscuous Mode is not used,else RX is switched on in RX_ON Mode*/
       tal_rx_enable(PHY_RX_ON); 
}

void app_alert(void)
{
	/* Indicate error - flash an LED */
	
	while(1)
	{
		//led_toggle();
		//delay_ms(100);
	}
}
//--- end of file