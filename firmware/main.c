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

#define TRANSPOSE 0 // number of octaves transposed
#define MIDI_CV_LENGTH 24 // number of output voltages available

#define NOTE_OFF 0x80 // NOTE_OFF MIDI MESSAGE
#define NOTE_ON 0x90 // NOTE_ON MIDI MESSAGE
#define AFTERTOUCH 0xA0 // polyphonic key pressure
#define CONTROL_CHANGE 0xB0 // control change
#define PROGRAM_CHANGE 0xC0 // program change
#define CHANNEL_PRESSURE 0xD0 // channel pressure after touch
#define PITCH_BEND 0xE0 // pitch bend change
#define SYSTEM_MSG 0xF0 // system real time or common
#define SYSTEM_REAL_TIME 0xF8 // system real time

// defined by convention with usbtest program
#define USB_LED_OFF 0
#define USB_LED_ON  1
#define USB_DATA_OUT 2

// define by our own convention
// CONFIG section
#define HIGHNOTE 0
// #define STRIGGER 1
 
/********************************************************************************
Interrupt Routines
********************************************************************************/
volatile unsigned char statusByte = 0;
volatile uint8_t dataBytesReceived = 0;
volatile unsigned char dataBytesBuffer[2]; 

// From the datasheet, predefined names don't seem to work
// 8 | 0x0007 | USART0, RX | USART0, Rx Complete
ISR(_VECTOR(7))
{
  // read from UDR resets interrupt flag
  unsigned char uart = UDR;

  // ignore all system real time messages
  if ((uart & SYSTEM_REAL_TIME) == SYSTEM_REAL_TIME)
    return;

  if ((uart & 0x80) == 0x80) {
    statusByte = uart;
    dataBytesReceived = 0;
  } else if (statusByte > 0) {
    if (dataBytesReceived < 2) { // only sysex message can be longer than two
      dataBytesBuffer[dataBytesReceived] = uart;
      dataBytesReceived++;
    }
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
  // write MSB (first nibble of value is 0 by convention (highest value is 0x0FFF, 4096))
	SPI_Write(0x10 | value >> 8);
  // write LSB
	SPI_Write(value & 0xFF);
  // chip deselect latches output
	PORTB |= _BV(PB4);
}

/********************************************************************************
CONFIG
********************************************************************************/
unsigned char config = 0;

void readConfig() {
  // read from EEPROM
}

void writeConfig(unsigned char newConf) {
  config = newConf;
  // writethrough to EEPROM
}

/********************************************************************************
GATE
********************************************************************************/
void gateInit() {
  // Configure PB2 (GATE) as output
  DDRB |= _BV(PB2);
}

void gateOn() {
  //if ((config & _BV(STRIGGER)) == _BV(STRIGGER)) {
    // (active low for Korg)
    PORTB &= ~(_BV(PB2));
 // } else {
 //   PORTB |= _BV(PB2);
  //}
}

void gateOff() {
  // GATE OFF 
  //if ((config & _BV(STRIGGER)) == _BV(STRIGGER)) {
    // (active low for Korg)
    PORTB |= _BV(PB2);
  //} else {
  //  PORTB &= ~(_BV(PB2));
  //}
}

/********************************************************************************
MIDI
********************************************************************************/

// to avoid having to calculate, we keep these in prog memory
// prog mem not RAM due to size constraints (prog: 2K, ram: 128B)
// calculated with C as 1000, Hz/V, 12TET
static const uint16_t MIDI_CV[] PROGMEM =
{
    1000, 1059, 1122, 1189, 1260, 1335, 1414, 1498, 1587, 1682, 
    1782, 1888, 2000, 2119, 2245, 2378, 2520, 2670, 2828, 2997, 3175, 3364, 
    3564, 3775
};

uint16_t calcDACValue(unsigned char note) {
  // simple table lookup
  unsigned char index = note % MIDI_CV_LENGTH; 
	return pgm_read_word(&MIDI_CV[index]);
} 

// no need for volatile, set from (subroutine of) main loop
unsigned char noteNames[13] = "CdDeEFgGaAbB ";

// 10 is randomly chosen
unsigned char noteStack[10] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
int8_t noteStackPointer = -1;
unsigned char config;

// unsigned char findHighestNote() {
//   unsigned char highestNote = 0;
//   int tempPointer = 0;
//   while (tempPointer <= noteStackPointer) {
//     unsigned char note = noteStack[tempPointer];
//     if (note < 0xFF && note > highestNote) 
//       highestNote = note;
//     tempPointer++;
//   }
//   return highestNote;
// }

unsigned char findLatestNote() {
  return noteStack[noteStackPointer];
}

unsigned char findNote() {
  // if ((config & _BV(HIGHNOTE)) == _BV(HIGHNOTE))
  //   return findHighestNote();
  // else // latest note
    return findLatestNote();
}

// play what's on top of the stack, or stop playing if nothing's on top of the stack
void playNote() {
  if (noteStackPointer > -1) {
      // set CV first, so if previously there was no note, GATE is only set when CV is certainly there
      // CV
      uint16_t output = calcDACValue(findNote());
      writeDACValue(output);

      gateOn();
    } else { // message == NOTE_ON && velocity > 0 => GATE ON
      // leave CV as it is
      gateOff();
    }
  }

void handleNote(unsigned char message, unsigned char channel) {
	unsigned char note = dataBytesBuffer[0];
  unsigned char velocity = dataBytesBuffer[1];

    if (message == NOTE_OFF
      || velocity == 0) {
    	int8_t tempPointer = noteStackPointer;

      // find note that was released in noteStack
      while (tempPointer > 0 && noteStack[tempPointer] != note)
        tempPointer--;

      if (tempPointer > -1) { // note found in stack
        // set note off 
        noteStack[tempPointer] = 0xFF;
        if (tempPointer == noteStackPointer) {
          // set noteStackPointer to last played note
          while (noteStackPointer > -1 && noteStack[noteStackPointer] == 0xFF)
            noteStackPointer--;
        } // else leave noteStackPointer alone
      } // else note not found, ignore

    } else { // message == NOTE_ON && velocity > 0 => GATE ON
     	if (noteStackPointer >= 9) {
        // stack overflow, restart
        noteStackPointer = -1;
      }

      // add the new note
      noteStackPointer++; // pointer starts at -1, so add1 first
      noteStack[noteStackPointer] = note;
    }

    // set CV/GATE according to stack
    playNote();
}

// this should be called every so often to ensure handling
// of the midi buffer
void midiPoll() {
  unsigned char status = statusByte;
  unsigned char message = status & 0xF0; // first nibble is the message
  unsigned char channel = status & 0x0F; // second nibble is the channel (for channel voice messages)

  switch (message) {
    case NOTE_OFF:
    case NOTE_ON:
      if (dataBytesReceived == 2) {
        handleNote(message, channel);
        dataBytesReceived = 0;
      }
      break;
    case AFTERTOUCH:
    case CONTROL_CHANGE:
    case PITCH_BEND:
      if (dataBytesReceived == 2) {
        dataBytesReceived = 0;
      }
      break;
    case CHANNEL_PRESSURE:
    case PROGRAM_CHANGE:
      if (dataBytesReceived == 1) {
        dataBytesReceived = 0;
      }
      break;
    case SYSTEM_MSG:
      dataBytesReceived = 0;
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
        writeConfig(config | _BV(HIGHNOTE)); // write highest note
        return 0;
    case USB_LED_OFF: 
        writeConfig(config & ~_BV(HIGHNOTE)); // write latest note
        return 0;
    case USB_DATA_OUT: // send data to PC
        // send 1 character, the note playing
        if (noteStackPointer > -1) {
          usbMsgPtr = &noteNames[noteStack[noteStackPointer] % 12];
        } else {
          // or   if no note playing
          usbMsgPtr = &noteNames[13];
        }
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


  readConfig();

  gateInit();
  gateOff();

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

        midiPoll();
    }
}

