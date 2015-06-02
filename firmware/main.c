/********************************************************************************
Includes
********************************************************************************/
#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include "usbdrv.h"
#include <util/delay.h>

/********************************************************************************
Defines
********************************************************************************/
#define USART_BAUDRATE 31250 // per MIDI standard
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define TRANSPOSE 2 // number of octaves transposed
#define MIDI_CV_LENGTH 58 // number of output voltages available

#define NOTE_ON 0x90 // NOTE_ON MIDI MESSAGE
#define NOTE_OFF 0x80 // NOTE_OFF MIDI MESSAGE

// defined by convention with usbtest program
#define USB_LED_OFF 0
#define USB_LED_ON  1
#define USB_DATA_OUT 2
 
/********************************************************************************
Interrupt Routines
********************************************************************************/
volatile uint8_t midiBytesReceived = 0;
volatile unsigned char midiReceiveBuffer[3]; 

// From the datasheet, predefined names don't seem to work
// 8 | 0x0007 | USART0, RX | USART0, Rx Complete
ISR(_VECTOR(7))
{
  // read from UDR resets interrupt flag
  unsigned char uart = UDR;

  // if bytesReceived is 3 or bigger and we get here: ignore
  uint8_t bytesReceived = midiBytesReceived;
  if (bytesReceived < 3) {
    // a status message resets the buffer index
    if ((uart & 0x80) == 0x80) {
      if (bytesReceived > 0)
        bytesReceived = 0;

      midiReceiveBuffer[bytesReceived] = uart;
      midiBytesReceived = bytesReceived + 1;
    } else if (bytesReceived > 0) {
      midiReceiveBuffer[bytesReceived] = uart;
      midiBytesReceived = bytesReceived + 1;
    }
    // else data byte received without status byte > ignore
  }
}

/********************************************************************************
USART FUNCTONS
********************************************************************************/

void USART_Init( void ) {
	/* Set baud rate */
	UBRRH = (BAUD_PRESCALE >> 8); 
	UBRRL = BAUD_PRESCALE;
	/* Enable receiver and receive interrupt */ 
	UCSRB = (1<<RXEN)|(1<<RXCIE);
	/* Set frame format: 8data */ 
	UCSRC = (1 << UCSZ0) | (1 << UCSZ1);
}

/********************************************************************************
SPI FUNCTONS
********************************************************************************/
void SPI_Init( void ) {
   // SPI chip select high
  DDRB |= _BV(PB4);
  PORTB |= _BV(PB4);
	// set three wire mode and external clock with strobe bit
	// start with clock low, so no USICLK here
	USICR = (1<<USIWM1) | (1<<USICS1) | (1<<USITC);
	// enable DO and USCK as output
	DDRB |= (1<<PB6) | (1<<PB7);
}

void SPI_Write( char output ) {
  // fill data register
	USIDR = output;

	USISR = _BV(USIOIF); // clear flag

	PORTB &= ~(_BV(PB7));
 
  // loop through shift register
  while ( (USISR & _BV(USIOIF)) == 0 ) {
  		// 
   	USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC);
  }
}

/********************************************************************************
DAC
********************************************************************************/
void writeDACValue(uint16_t value) {
  // chip select
	PORTB &= ~(_BV(PB4));
  // write MSB
	SPI_Write(0x10 | value >> 8);
  // write LSB
	SPI_Write(value & 0xFF);
  // chip deselect latches output
	PORTB |= _BV(PB4);
}

/********************************************************************************
MIDI
********************************************************************************/

// to avoid having to calculate, we keep these in prog memory
// prog mem not RAM due to size constraints (prog: 2K, ram: 128B)
// calculated with A as 1000, Hz/V, 12TET
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

// no need for volatile, set from (subroutine of) main loop
unsigned char lastNote = 0;
unsigned char noteNames[12] = "CdDeEFgGaAbB";

void handleNote(unsigned char message, unsigned char channel) {
	unsigned char note = midiReceiveBuffer[1];
  unsigned char velocity = midiReceiveBuffer[2];

    if (message == NOTE_OFF
    	|| velocity == 0) {
    	// leave CV as it is
    	// GATE OFF (active low for Korg)
      PORTB |= _BV(PB2);
    } else { // message == NOTE_ON && velocity > 0 => GATE ON
     	// set CV first, so if previously there was no note, GATE is only set when CV is certainly there
     	// CV
     	uint16_t output = calcDACValue(note);
    	writeDACValue(output);

    	// GATE ON (active low for Korg)
      PORTB &= ~(_BV(PB2));
    }

    // in case USB out asks for it
    lastNote = note;
}

void handleMidiBuffer() {
  unsigned char uart = midiReceiveBuffer[0];
  unsigned char message = uart & 0xF0;
  unsigned char channel = uart & 0x0F;

  switch (message) {
    case NOTE_ON:
    case NOTE_OFF:
      handleNote(message, channel);
      break;
  }
}

/********************************************************************************
USB
********************************************************************************/

// this gets called when custom control message is received
USB_PUBLIC uchar usbFunctionSetup(uchar data[8]) {
    usbRequest_t *rq = (void *)data; // cast data to correct type
        
    switch(rq->bRequest) { // custom command is in the bRequest field
    case USB_LED_ON:
        PORTB |= _BV(PB2); // turn LED on
        return 0;
    case USB_LED_OFF: 
        PORTB &= ~(_BV(PB2)); // turn LED off
        return 0;
    case USB_DATA_OUT: // send data to PC
        // send 1 character, the last played note
        usbMsgPtr = &noteNames[lastNote % 12];
        return 1;
    }

    return 0; // should not get here
}

/********************************************************************************
Main
********************************************************************************/
int main( void ) {
  // reset
  PORTB = 0;
  DDRB = 0;

  // Configure PB2 (GATE) as output
  DDRB |= _BV(PB2);
  // GATE OFF on startup (active low for Korg)
  PORTB |= _BV(PB2);

    wdt_enable(WDTO_1S); // enable 1s watchdog timer

  // init SPI and UART
    SPI_Init();
    USART_Init();

    // ========= USB INIT =============
    usbInit();
        
    usbDeviceDisconnect(); // enforce re-enumeration
    uchar i;
    for(i = 0; i<250; i++) { // wait 500 ms
        wdt_reset(); // keep the watchdog happy
        _delay_ms(2);
    }
    usbDeviceConnect();
    // ==============
        
    sei(); // Enable interrupts after re-enumeration
        
    while(1) {
        wdt_reset(); // keep the watchdog happy
        // all usb functions are called from this function
        usbPoll(); // needs to be called at least once every 50ms

        // full note_on/note_off
        if (midiBytesReceived == 3) {
           handleMidiBuffer();
           // reset for new receive
           midiBytesReceived = 0;
        }
    }
}

