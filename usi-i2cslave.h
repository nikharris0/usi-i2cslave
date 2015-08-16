#ifndef USI_I2CSLAVE_H_
#define USI_I2CSLAVE_H_

#include <stdint.h>
#include <avr/io.h>

#define USI_RX_BUFFER_SIZE	8
#define USI_RX_BUFFER_MASK	(USI_RX_BUFFER_SIZE - 1)

#define USI_TX_BUFFER_SIZE	8
#define USI_TX_BUFFER_MASK	(USI_TX_BUFFER_SIZE - 1)

/* attiny2313 specific */
#define USI_DDR			DDRB
#define USI_PORT		PORTB
#define USI_PIN			PINB
#define USI_PORT_SDA	PORTB5
#define USI_PORT_SCL	PORTB7
#define USI_PIN_SDA		PINB5
#define USI_PIN_SCL		PINB7

/* states */
#define USI_SLAVE_CHECK_ADDR					0x00
#define USI_SLAVE_SEND_DATA						0x01
#define USI_SLAVE_REQ_REPLY_FROM_SEND_DATA		0x02
#define USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA	0x03
#define USI_SLAVE_REQUEST_DATA					0x04
#define USI_SLAVE_GET_DATA_AND_ACK				0x05

#endif
