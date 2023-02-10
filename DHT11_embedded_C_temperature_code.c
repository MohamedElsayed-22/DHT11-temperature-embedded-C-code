#define DHT11_PIN 2 // change this according to the pin on the chip to which the module is connected
#define F_CPU 16000000UL
#define BAUD 9600
#define BRC ((F_CPU/16/BAUD)-1)
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <util/setbaud.h>
#include <util/delay.h>

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define _BV(bit) (1 << (bit))
#define _SFR_MEM_ADDR(sfr) ((uint16_t) &(sfr))
#define _SFR_ADDR(sfr) _SFR_MEM_ADDR(sfr)
#define _MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))
#define _SFR_BYTE(sfr) _MMIO_BYTE(_SFR_ADDR(sfr))

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

/*
  DHT11 temperature module works as follows:
  1- It receives a request from the micro controller to start the connection.
  2- It sends a feedback which is considered the response upon the request sent to it.
  3- It sends data of total 40 bits long (5 segments) as follows:
    a- first two segments contain humidity value in decimal integer form.
      note: 1st 8-bits are integer part and next 8 bits are fractional part.
    b- Next two segments contain temperature value in decimal integer form (Celsius).
    c- Last segment is the checksum which holds checksum of first four segments.
*/

//ReadData() is used to get the data from the module
int ReadDigitalData(uint8_t pin)
{
	if (PIND & 4) return 1;
	return 0;
}

//microsec()  returns the number of microseconds since the board has booted
unsigned long microsec() {
    unsigned long m;
    uint8_t oldSREG = SREG, t;
    m = 0;
    t = TCNT0;
    if ((TIFR0 & _BV(TOV0)) && (t < 255))
        m++;
    SREG = oldSREG;
    return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}


// dht11 is slow module so it needs about 2 sec of delay.
void wait_for_dht11(){
  _delay_ms(2000);
}

// AVR send start pulse/request
void start_signal(uint8_t dht11_pin){
  DDRD = 0;
	DDRD |= (1<<dht11_pin);
  PORTD = 0;
  // set to low pin
	PORTD &= ~(1<<dht11_pin);
  // wait for 18ms
	_delay_ms(18);
  PORTD = 0;
  // set to high pin
	PORTD |= (1<<dht11_pin);
  DDRD &= ~(1<<dht11_pin);
  // set to high pin
  PORTD |= (1<<dht11_pin);
}


struct temp{
   uint8_t tempInt;
   uint8_t tempFloat;
};
struct temp read_dht11(uint8_t dht11_pin){
  uint16_t rawTemperature = 0;
  uint16_t data = 0;
  struct temp temp;
  unsigned long startTime;

  for ( int8_t i = -3 ; i < 80; i++ ) {
    int8_t live;
    startTime = microsec();

    do {
      live = (unsigned long)(microsec() - startTime);
      if ( live > 90 )return;
    }
    while (ReadDigitalData(dht11_pin) == (i & 1) ? 1 : 0);
    if ( i >= 0 && (i & 1) ) {
      data <<= 1;
      // bit 0 is maximum 30 usecs and bit 1 is at least 68 usecs.
      if ( live > 30 ) {
        data |= 1; // we got a one
      }
    }
    switch ( i ) {
      case 63:
        rawTemperature = data;
      case 79:
        data = 0;
        break;
    }
  }

  // temperature integer part
  temp.tempInt = rawTemperature >> 8;
  rawTemperature = rawTemperature << 8;
  // temperature float part
  temp.tempFloat = rawTemperature >> 8;

  return temp;
}

