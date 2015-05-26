/********************************************************************************
Includes
********************************************************************************/
#include <avr/io.h>
#include <stdbool.h>
//#include <avr/interrupt.h>
#include <avr/pgmspace.h>

/********************************************************************************
Defines
********************************************************************************/
#define USART_BAUDRATE 31250
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define STRIGGER 1

#define TRANSPOSE 2
#define MIDI_CV_LENGTH 58

#define NOTE_ON 0x90
#define NOTE_OFF 0x80
 
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
DAC
********************************************************************************/
void writeDACValue(uint16_t value) {
	PORTB &= ~(_BV(PB4));
	SPI_Write(0x10 | value >> 8);
	SPI_Write(value & 0xFF);
	PORTB |= _BV(PB4);
}

/********************************************************************************
MIDI
********************************************************************************/
static const uint16_t MIDI_CV[] PROGMEM =
{
    149, 157, 167, 177, 187, 198, 210, 223, 236, 250, 265, 281, 297, 315, 334, 
    354, 375, 397, 420, 445, 472, 500, 530, 561, 595, 630, 667, 707, 749, 794, 
    841, 891, 944, 1000, 1059, 1122, 1189, 1260, 1335, 1414, 1498, 1587, 1682, 
    1782, 1888, 2000, 2119, 2245, 2378, 2520, 2670, 2828, 2997, 3175, 3364, 
    3564, 3775, 4000
};

uint16_t calcDACValue(unsigned char note) {
  // simple table lookup
  unsigned char index = (note - TRANSPOSE * 12) % MIDI_CV_LENGTH; 
	return pgm_read_word(&MIDI_CV[index]);
} 

void handleNote(unsigned char message, unsigned char channel) {
	unsigned char note = USART_Receive();
  unsigned char velocity = USART_Receive();

    if (message == NOTE_OFF
    	|| velocity == 0) {
    	// leave CV as it is
    	// GATE OFF
      #ifdef STRIGGER
      PORTB |= _BV(PB2);
      #endif

      #ifdef VTRIGGER
    	PORTB &= ~(_BV(PB2));
      #endif
    } else { // message == NOTE_ON && velocity > 0 => GATE ON
     	// set CV first, so if previously there was no note, GATE is only set when CV is certainly there
     	// CV
     	uint16_t output = calcDACValue(note);
    	writeDACValue(output);

    	// GATE ON
     	#ifdef STRIGGER
      PORTB &= ~(_BV(PB2));
      #endif

      #ifdef VTRIGGER
      PORTB |= _BV(PB2);
      #endif
    }
}


/********************************************************************************
Main
********************************************************************************/
int main( void ) {

	PORTB = 0;
	DDRB = 0;

	// SPI chip select high
	DDRB |= _BV(PB4);
	PORTB |= _BV(PB4);

    // Configure PORTB as output
    DDRB |= _BV(PB2);
    

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
    	}
    	// else ignore
    }
}


