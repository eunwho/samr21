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

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 TCP client example --"STRING_EOL \
"-- "BOARD_NAME " --"STRING_EOL	\
"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

typedef struct s_msg_wifi_product {
	uint8_t name[9];
} t_msg_wifi_product;

static t_msg_wifi_product msg_wifi_product = {
	.name = MAIN_WIFI_M2M_PRODUCT_NAME,
};

static uint8_t gau8SocketTestBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];
static SOCKET tcp_client_socket = -1;
static uint8_t wifi_connected;






/** UART module for debug. */
static struct usart_module cdc_uart_module;

#define MAX_RX_BUFFER_LENGTH   14
struct usart_module usart_instance;

void usart_read_callback(struct usart_module *const usart_module);
void usart_write_callback(struct usart_module *const usart_module);

volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];

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

void config_rs485_TX_EN(void){
	struct port_config pin_config;
	port_get_config_defaults( &pin_config);
	pin_config.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA27, &pin_config);
	port_pin_set_output_level(PIN_PA27, false);
}

void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_instance, usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);

	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}

uint8_t readIn485[]={0x02,0x51,'R','F','F','F','F','F','F','F','F',0x03,0x00};

uint8_t writeDout485[]={0x02,0x52,'W','P','A','0','0','F','F','F','F',0x03,0x00};
uint8_t readDout485[]={0x02,0x52,'R','F','F','0','0','F','F','F','F',0x03,0x00};

volatile uint16_t temp;

int main ( void )
{
	
	irq_initialize_vectors();
	system_init();
	delay_init();
	cpu_irq_enable();	
	config_rs485_TX_EN( );

	sio2host_init();
	
//	LCD_Initialize();
	readMacAddress();
	wsndemo_init();
	configure_usart();

	// configure_usart_callbacks();
	// system_interrupt_enable_global();

	port_pin_set_output_level(PIN_PA27, true);
	usart_write_buffer_wait(&usart_instance, writeDout485, sizeof(writeDout485));
	delay_us(100);
	port_pin_set_output_level(PIN_PA27, false);
	delay_ms(10);
	port_pin_set_output_level(PIN_PA27, true);

//--- wifi setting
/*
	if(SystTick_Config(system_cpu_clock_get_hz() / 1000 )) {
		puts("main: Systick configuration error!");
		while(1);
	}
*/
		
	while (true) {
/*
		//port_pin_set_output_level(PIN_PA27, false);
		//usart_read_buffer_job(&usart_instance, rx_buffer, MAX_RX_BUFFER_LENGTH);	
		//port_pin_set_output_level(PIN_PA27, true);
		port_pin_set_output_level(PIN_PA27, true);	
		usart_write_buffer_wait(&usart_instance, readDout485, sizeof(readDout485));
		port_pin_set_output_level(PIN_PA27, false);

		usart_read_buffer_wait(&usart_instance, rx_buffer,13); 

		delay_ms(10);
		port_pin_set_output_level(PIN_PA27, true);
		delay_ms(2000);
*/
		wsndemo_task();
	}

/*	
    while(1)
    {
		port_pin_set_output_level(PIN_PA27, true);
		usart_write_buffer_wait(&usart_instance, readIn485, sizeof(readIn485));
		delay_ms(7);
		port_pin_set_output_level(PIN_PA27, false);
		
		port_pin_set_output_level(PIN_PA27, false);
		if (usart_read_wait(&usart_instance, &temp) == STATUS_OK) {
			port_pin_set_output_level(PIN_PA27, true);
			delay_us(10);
			while (usart_write_wait(&usart_instance, temp) != STATUS_OK) {
			}			
			port_pin_set_output_level(PIN_PA27, false);			
		}
//		delay_ms(1000);		
		// wsndemo_task();
    }
*/
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
