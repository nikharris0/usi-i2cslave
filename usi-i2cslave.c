#include <avr/interrupt.h>
#include "usi-i2cslave.h"

static uint8_t usi_addr;

/* circular buffer for received data */
static uint8_t usi_rx_buffer[USI_RX_BUFFER_SIZE];
static volatile uint8_t usi_rx_head;
static volatile uint8_t usi_rx_tail;

/* circular buffer for transmit data */
static uint8_t usi_tx_buffer[USI_TX_BUFFER_SIZE];
static volatile uint8_t usi_tx_head;
static volatile uint8_t usi_tx_tail;

static volatile uint8_t slave_state;

void i2c_slave_init(uint8_t addr)
{
	usi_rx_head = usi_rx_tail = usi_tx_head = usi_tx_tail = 0;

	usi_addr = addr; /* store our address so we know when being addressed by master */

	USI_PORT |= (1 << USI_PORT_SCL); /* SCL high */
	USI_PORT |= (1 << USI_PORT_SDA); /* SDA high */

	USI_DDR |= (1 << USI_PORT_SCL); /* SCL is output */
	USI_DDR &= ~(1 << USI_PORT_SDA); /* SDA is input */

	USICR = (1 << USISIE) | /* enable START condition interrupt */
			(0 << USIOIE) | /* enable counter overflow interrupt */
			(1 << USIWM1) | /* 2 wire mode */
			(0 << USIWM0) |  /* 2 wire mode */
			(1 << USICS1) | /* USICS1, USICS0, and USICLK set clock source for SR */
							/* no USI counter overflow until we get our first START */
							/* apparently that is troublesome */
			(0 << USICS0) | /* external positive edge for shift register clock */
			(0 << USICLK) | /* external both edges for counter clock */
			(0 << USITC);   /* no toggle clock pin */
}

void i2c_slave_transmit_byte(unsigned char b)
{
	uint8_t tmp_head;
	tmp_head = (usi_tx_head + 1) % USI_TX_BUFFER_MASK; /* determine write index */
	while(tmp_head == usi_tx_tail); /* wait until buffer is no longer full */

	usi_tx_buffer[tmp_head] = b; /* write byte to tx buffer */
	usi_tx_head = tmp_head; /* new tx head */
}

unsigned char i2c_slave_receive_byte(void)
{
	uint8_t tmp_tail;
	uint8_t tmp_rx_tail;
	tmp_rx_tail = usi_rx_tail;

	while(tmp_rx_tail == usi_rx_tail); /* wait for new data */
	tmp_tail = (usi_rx_tail + 1) % USI_RX_BUFFER_MASK; /* new byte offset */
	return usi_rx_buffer[tmp_tail];
}

uint8_t i2c_slave_data_available(void)
{
	uint8_t tmp_rx_tail;
	tmp_rx_tail = usi_rx_tail;
	return (usi_rx_head != tmp_rx_tail); /* if current head doesn't  equal the tail there bytes */
}

/* interrupt for START condition */
ISR(USI_START_vect)
{
	uint8_t tmp_usisr;
	tmp_usisr = USISR;

	slave_state = USI_SLAVE_CHECK_ADDR;
	USI_DDR &= ~(1 << USI_PORT_SDA);

	while( (USI_PIN & (1 << USI_PORT_SCL)) && !(tmp_usisr & (1 << USIPF)));

	USICR = (1 << USISIE) | /* keep START interrupt enabled */
			(1 << USIOIE) | /* keep overflow interrupt enabled */
			(1 << USIWM1) | /* 2 wire mode */
			(1 << USIWM0) | /* 2 wire mode, this time with overflow clock held low */
			(1 << USICS1) | /* external positive edge shift register clock source */
			(0 << USICS0) | /* external both edges counter clock source */
			(0 << USICLK) |
			(0 << USITC);   /* don't toggle clock pin */

	USISR = (1 << USISIF) | /* start condition detected */
			(1 << USIOIF) | /* clear counter overflow interrupt flag */
			(1 << USIPF)  | /* clear STOP condition flag */
			(1 << USIDC)  | /* claer collision bit */
			(0 << USICNT0); /* sheet says 4 bit counter value, but ref impl says count 8bits? */

}

ISR(USI_OVERFLOW_vect)
{
	uint8_t tmp_tx_tail;
	uint8_t tmp_usidr;

	switch(slave_state) {
		/* START condition was received, so proceed with processing */
		case USI_SLAVE_CHECK_ADDR:
			/* if the address is general call or matches our address */
			if(USIDR == 0 || ((USIDR >> 1) == usi_addr)) {
				/* if the read bit is set then we are sending data */
				if(USIDR & 1) {
					slave_state = USI_SLAVE_SEND_DATA;
				} else { /* otherwise we will be receiving data */
					slave_state = USI_SLAVE_REQUEST_DATA;
					USIDR = 0; /* clear the data register */
					USI_DDR |= (1 << USI_PORT_SDA); /* SDA line to input */
					USISR = (0 << USISIF) | /* do not clear START condition */
							(1 << USIOIF) | /* clear counter overflow interrupt flag */
							(1 << USIPF)  | /* clear STOP condition flag */
							(1 << USIDC)  | /* clear collision flag */
							(0x0E << USICNT0); /* apparently tell USI counter to shift 1 bit? */
				}
			} else { /* this packet is not meant for me so set up for a start condition again */
				USICR = (1 << USISIE) | /* enable start condition interrupt */
						(0 << USIOIE) | /* disable counter overflow interrupt */
						(1 << USIWM1) | /* two wire mode */
						(0 << USIWM0) | /* no hold SCL low during overflow */
						(1 << USICS1) | /* external shift register clock */
						(0 << USICS0) | /* postive edge */
						(0 << USICLK) | /* both edges */
						(0 << USITC);   /* no toggle clock pin */
			}

			break;

		case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
			/* we will get either an ACK or a NACK. if NACK, reset */
			if(USIDR) {
				USICR = (1 << USISIE) | /* enable start condition interrupt */
						(0 << USIOIE) | /* disable counter overflow interrupt */
						(1 << USIWM1) | /* two wire mode */
						(0 << USIWM0) | /* no hold SCL low during overflow */
						(1 << USICS1) | /* external shift register clock */
						(0 << USICS0) | /* postive edge */
						(0 << USICLK) | /* both edges */
						(0 << USITC);   /* no toggle clock pin */
				return;
			}

			/* if we make it here, it's an ACK, so no break here, flow through */

		case USI_SLAVE_SEND_DATA:
			tmp_tx_tail = usi_tx_tail;

			/* if there is still more data to send */
			if(usi_tx_head != tmp_tx_tail) {
				usi_tx_tail = (usi_tx_tail + 1) % USI_TX_BUFFER_MASK;
				USIDR = usi_tx_buffer[usi_tx_tail];

				/* once we're done sending data we will need an ACK or NACK */
				slave_state = USI_SLAVE_REQ_REPLY_FROM_SEND_DATA;

				/* make SDA output */
				USI_DDR |= (1 << USI_PORT_SDA);
				/* clear flags and set status to send out 8 bits */
				USISR = (0 << USISIF) | /* clear START condition interrupt flag */
						(1 << USIOIF) | /* enable counter overflow interrupt */
						(1 << USIPF)  | /* clear STOP condition */
						(1 << USIDC)  | /* clear collision flag */
						(0 << USICNT0); /* apparently shift out 8 bits */

			} else { /* otherwise we are done sending data so we reset*/
				USICR = (1 << USISIE) | /* clear start condition interrupt */
						(0 << USIOIE) | /* disable counter overflow interrupt */
						(1 << USIWM1) | /* two wire mode */
						(0 << USIWM0) | /* no hold SCL low during overflow */
						(1 << USICS1) | /* external shift register clock */
						(0 << USICS0) | /* postive edge */
						(0 << USICLK) | /* both edges */
						(0 << USITC);   /* no toggle clock pin */
			}

			break;

		case USI_SLAVE_REQ_REPLY_FROM_SEND_DATA:
			slave_state = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;

			/* SDA as input so we can receive ACK */
			USI_DDR &= ~(1 << USI_PORT_SDA);
			USIDR = 0;

			/* set status register to read ACK response */
			USISR = (0 << USISIF) | /* clear START condition interrupt flag */
					(1 << USIOIF) | /* enable counter overflow interrupt */
					(1 << USIPF)  | /* cler STOP condition flag */
					(1 << USIDC)  | /* clear collision flag */
					(0x0E << USICNT0); /* shift 1 bit */

			break;


		case USI_SLAVE_REQUEST_DATA:
			/* we will read in data next */
			slave_state = USI_SLAVE_GET_DATA_AND_ACK;

			/* configure status register to read in data */
			USI_DDR &= ~(1 << USI_PORT_SDA);
			USISR = (0 << USISIF) | /* clear START condition interrupt */
					(1 << USIOIF) | /* enable counter overflow interrupt */
					(1 << USIPF)  | /* clear STOP condition flag */
					(1 << USIDC)  | /* clear collision flag */
					(0 << USICNT0); /* shift 8 bits */

			break;

		case USI_SLAVE_GET_DATA_AND_ACK:
			tmp_usidr = USIDR;
			usi_rx_head = (usi_rx_head +1) % USI_RX_BUFFER_MASK;
			usi_rx_buffer[usi_rx_head] = tmp_usidr;

			slave_state = USI_SLAVE_REQUEST_DATA;

			/* set SDA as output and clear data register */
			USI_DDR |= (1 << USI_PORT_SDA);
			USIDR = 0;


			/* configure USISR to send ACK response to master */
			USISR = (0 << USISIF) | /* clear START condition interrupt */
					(1 << USIOIF) | /* enable counter overflow interrupt */
					(1 << USIPF)  | /* clear STOP condition flag */
					(1 << USIDC)  | /* clear collision flag */
					(0x0E << USICNT0); /* shift 1 bits */
	}
}









