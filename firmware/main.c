/********************************************************************************
Includes
********************************************************************************/
#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>

/********************************************************************************
Defines
********************************************************************************/
#define USART_BAUDRATE 31250
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define NOTE_ON 0x40
#define NOTE_OFF 0x50
 
/********************************************************************************
Interrupt Routines
********************************************************************************/


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
void writeDACValue(uint16_t value) {
	PORTB &= ~(_BV(PB1));
	SPI_Write(0x10 | value >> 8);
	SPI_Write(value & 0xFF);
	PORTB |= _BV(PB1);
}

/********************************************************************************
MIDI SHIT
********************************************************************************/
uint16_t calcDACValue(unsigned char note) {
	uint16_t result = note;
	// gewoon efkes shiften om te testen
	return result << 0x5;
} 

void handleNote(unsigned char message, unsigned char channel) {
	unsigned char note = USART_Receive();
    unsigned char velocity = USART_Receive();

    uint16_t output = calcDACValue(note);
    writeDACValue(output);
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
    USART_Init();


    // enable interrupts
    // sei(); 
    while(true) {

    	unsigned char uart = USART_Receive();

    	if ((uart & 0x80) == 0x80) { // midi status message
    		unsigned char message = uart & 0xF0;
    		unsigned char channel = uart & 0x0F;

    		switch (message) {
    			case NOTE_ON:
    			case NOTE_OFF:
    				handleNote(message, channel);
    				break;
    		}

    		// blink LED
    		PORTB ^=  _BV(PB0);
    	}
    	// else ignore
    }
}


