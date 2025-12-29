
#include "light_apa102.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define setOutput(ddr, pin) ((ddr) |= (1 << (pin)))
#define setLow(port, pin) ((port) &= ~(1 << (pin)))
#define setHigh(port, pin) ((port) |= (1 << (pin)))
#define getValue(port, pin) ((port) & (1 << (pin)))


struct cRGB AngleData[64];


#define APA102_Transfer(data) do {                                 \
	while (!(UCSR0A & (1 << UDRE0)));  \
	UDR0 = (data);                     \
} while (0)



inline void APA102_init(void) 
{
  setOutput(apa102_ddr, apa102_clk);
  setOutput(apa102_ddr, apa102_data);
  setLow(apa102_port, apa102_clk);

	// Baud rate must be set to 0 prior to enabling the USART as SPI
	// master, to ensure proper initialization of the XCK line.
	UBRR0 = 0;

	// Set USART to Master SPI mode.
	UCSR0C = (1<<UMSEL01) | (1<<UMSEL00);
	// Enable TX only
	UCSR0B = (1 << TXEN0);
	// Set baud rate. Must be set _after_ enabling the transmitter.
	UBRR0 = 0;  
   
}



 
 
void inline apa102_setleds(struct cRGB *ledarray, uint16_t leds)
{
  uint16_t i;
  uint8_t *rawarray=(uint8_t*)ledarray;
  //SPI_init();
  
  APA102_Transfer(0x00);  // Start Frame
  APA102_Transfer(0x00);
  APA102_Transfer(0x00);
  APA102_Transfer(0x00);
 
  for (i=0; i<(leds+leds+leds); i+=3)
  {
    APA102_Transfer(0xff);  // Maximum global brightness
    APA102_Transfer(rawarray[i+0]);
    APA102_Transfer(rawarray[i+1]);
    APA102_Transfer(rawarray[i+2]);
  }
  
  // End frame: 8+8*(leds >> 4) clock cycles    
  for (i=0; i<leds; i+=16)
  {
    APA102_Transfer(0xff);  // 8 more clock cycles
  }
}




void inline apa102_setleds2(uint8_t *ledarray, uint16_t leds)
{
	uint16_t i;
	
	//SPI_init();
	
	APA102_Transfer(0x00);  // Start Frame
	APA102_Transfer(0x00);
	APA102_Transfer(0x00);
	APA102_Transfer(0x00);
	
	for (i=0; i<(leds+leds+leds); i+=3)
	{
		APA102_Transfer(0xff);  // Maximum global brightness
		APA102_Transfer(ledarray[i+0]);
		APA102_Transfer(ledarray[i+1]);
		APA102_Transfer(ledarray[i+2]);
	}
	
	// End frame: 8+8*(leds >> 4) clock cycles
	for (i=0; i<leds; i+=16)
	{
		APA102_Transfer(0xff);  // 8 more clock cycles
	}
}



void APA102_SetAll(uint8_t Val)
{
 for (uint8_t i = 0; i < 64; i++)
 {
	AngleData[i].b = Val;
	AngleData[i].g = Val;
	AngleData[i].r = Val; 
 }
 	
}