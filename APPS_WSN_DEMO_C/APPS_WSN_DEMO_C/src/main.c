/**
* \file  main.c
*/
#include "asf.h"
#include "sio2host.h"
#include "wsndemo.h"
#include "miwi_api.h"

void readMacAddress(void);

void configure_usart(void);
void config_rs485_TX_EN(void);

struct usart_module usart_instance;
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


uint8_t string[]="Hello World! \r\n";

int main ( void )
{
	
	irq_initialize_vectors();
	system_init();
	delay_init();
	cpu_irq_enable();	
	config_rs485_TX_EN( );

//	LCD_Initialize();
	readMacAddress();
	wsndemo_init();
	configure_usart();
	
    while(1)
    {
		port_pin_set_output_level(PIN_PA27, true);
		usart_write_buffer_wait(&usart_instance, string, sizeof(string));
		delay_ms(10);
		port_pin_set_output_level(PIN_PA27, false);
		delay_ms(1000);		
		// wsndemo_task();
    }

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
