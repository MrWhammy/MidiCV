/********************************************************************************
Includes
********************************************************************************/
#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/********************************************************************************
Defines
********************************************************************************/
#define USART_BAUDRATE 31250
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
 
/********************************************************************************
Interrupt Routines
********************************************************************************/
 
// timer0 overflow
ISR(TIMER0_OVF_vect) {
    // XOR PORTA with 0x01 to toggle the second bit up
    PORTB=PORTB ^ 0x01;
}

/********************************************************************************
USART FUNCTONS
********************************************************************************/

void USART_Init( void ) {
	/* Set baud rate */
	UBRRH = (BAUD_PRESCALE >> 8); 
	UBRRL = BAUD_PRESCALE;
	/* Enable receiver and transmitter */ 
	UCSRB = (1<<RXEN)|(1<<TXEN);
	/* Set frame format: 8data, 2stop bit */ 
	UCSRC = (1<<USBS)|(3<<UCSZ0);
}

unsigned char USART_Receive( void ) {
	/* Wait for data to be received */ 
	while ( !(UCSRA & (1<<RXC)) ) {};
	/* Get and return received data from buffer */ 
	return UDR;
}

/********************************************************************************
TIMER FUNCTONS
********************************************************************************/
void TIMER_Init( void ) {
	// enable timer overflow interrupt for both Timer0 and Timer1
    TIMSK=(1<<TOIE0);

    // set timer0 counter initial value to 0
    TCNT0=0x00;
    // start timer0 with /1024 prescaler
    TCCR0B = (1<<CS02) | (1<<CS00);
}

/********************************************************************************
SPI FUNCTONS
********************************************************************************/
void SPI_Init( void ) {
	// set three wire mode and external clock with strobe bit
	// start with clock low, so no USICLK here
	USICR = (1<<USIWM1) | (1<<USICS1) | (1<<USITC);
	// enable DO and USCK as output
	DDRB |= (1<<PB6) | (1<<PB7);
}

void SPI_Write( char output ) {
	USIDR = output;

	USISR = _BV(USIOIF); // clear flag

	PORTB &= ~(_BV(PB7));
 
  	while ( (USISR & _BV(USIOIF)) == 0 ) {
  		// 
   		USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC);
  	}
}

/********************************************************************************
DAC SHIT
********************************************************************************/
void WriteDACValue(uint16_t value) {
	PORTB &= ~(_BV(PB1));
	SPI_Write(0x10 | value >> 8);
	SPI_Write(value & 0xFF);
	PORTB |= _BV(PB1);
}
 
/********************************************************************************
Main
********************************************************************************/
int main( void ) {

	PORTB = 0;
	DDRB = 0;

	// chip select high
	DDRB |= _BV(PB1);
	PORTB |= _BV(PB1);

    // Configure PORTB as output
    DDRB |= _BV(PB0);
    

    //TIMER_Init();
    SPI_Init();



    // enable interrupts
    // sei(); 
    uint16_t output = 0;
    while(true) {
    	//USIDR = output;

    	WriteDACValue(output);

    	// laat maar overflowen
    	output++;
    	if (output >= 4096) {
    		output = 0;
    		PORTB ^=  _BV(PB0);
    	}
    }
}


