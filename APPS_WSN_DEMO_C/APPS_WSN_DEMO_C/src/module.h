/*
 * module.h
 *
 * Created: 2019-04-02 오후 5:26:24
 *  Author: ew2
 */ 


#ifndef __MODULE_H_
#define __MODULE_H_

extern void configure_i2c_master(void);
//extern void i2c_clock_init();
//extern void i2c_pin_init();
//extern void pin_set_peripheral_function(uint32_t pinmux);

extern void usart_read_callback(struct usart_module *const usart_module);
extern void usart_write_callback(struct usart_module *const usart_module);
extern void config_i2c_GLCD_Select(void);
extern enum status_code _i2c_master_wait_for_bus( struct i2c_master_module *const module);
extern enum status_code _i2c_master_address_response( struct i2c_master_module *const module);
extern enum status_code _i2c_master_send_hs_master_code( 
	struct i2c_master_module *const module,	uint8_t hs_master_code);

extern void gLcdInit(void);
extern void gLcdShow(uint8_t * page);
extern void gLcdShow2(uint8_t * page);
enum status_code i2cOut( struct i2c_master_module *const module, uint8_t toSlaveByte);

#endif /* MODULE_H_ */