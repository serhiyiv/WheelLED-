/**
 * Project: AVR ATtiny USB Tutorial at http://codeandlife.com/
 * Author: Joonas Pihlajamaa, joonas.pihlajamaa@iki.fi
 * Inspired by V-USB example code by Christian Starkjohann
 * Copyright: (C) 2012 by Joonas Pihlajamaa
 * License: GNU GPL v3 (see License.txt)
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "usbdrv.h"

#define F_CPU 12000000L
#include <util/delay.h>








#define ERASE_CHIP   0
#define ERASE_IMAGE    1
#define GOTO_IMAGE     2
#define WRITE_IMAGE    3

#define TEST    4



#define USB_DATA_OUT 20
#define USB_DATA_IN 21






int MEMORY_POS = 0;
int IMAGE_SIZE = 12288;


#define MODE_USB        0
#define MODE_CYCLING    1
#define MODE_SLEEP      2

volatile uint8_t DeviceMode = MODE_USB;  // 0 - USB data   / 1 - Cycling

// TIMERS
volatile double long   CPU_CLOCK = 12000;
volatile double long   Timer_Prescaller = 8;
volatile double long   Timer_Resolution = 0.05;
volatile unsigned long int  Ticks = 50000;
volatile unsigned long int  TicksTemp = 0;
volatile unsigned long int  TicksSleep = 0;
volatile unsigned long int  TicksImageTime = 0;


// IMAGES
volatile uint8_t  ImagesCnt   = 1;
volatile uint8_t  NextImage = 0;


volatile unsigned long int   AngleCount = 360;
volatile unsigned long int  AngleTime = 200;



// PROCESSING
volatile uint8_t Display = 1;
volatile uint8_t ClearLed = 0;
volatile uint8_t FirstCircle = 0;
volatile uint8_t EnteringSleep = 0;
volatile uint8_t AfterSleepCircles = 0;
volatile unsigned long int AngleIDX = 0;
volatile unsigned long LastAngle = -1;
volatile uint8_t ImageIDX = 0;
volatile unsigned long int CurrentImageAddress = 0;

//Settings


#define READ_LINE(port, pin)    port & (1<<pin)
#define setOutput(ddr, pin) ((ddr) |= (1 << (pin)))
#define setLow(port, pin) ((port) &= ~(1 << (pin)))
#define setHigh(port, pin) ((port) |= (1 << (pin)))


static uchar replyBuf[16] = "Hello, USB!";
static uchar dataReceived = 0, dataLength = 0; // for USB_DATA_IN


#include "l74hc595/l74hc595.c"
#include "SST25VFxxxx/SST25VFxxxx.c"




void Delay_ms(int n)
{
	while (n--)
	{
		_delay_ms(1);
	}
}




// this gets called when custom control message is received
USB_PUBLIC uchar usbFunctionSetup(uchar data[8]) {
	usbRequest_t *rq = (void *)data; // cast data to correct type
	
	
	switch(rq->bRequest) { // custom command is in the bRequest field
     
    case ERASE_CHIP:
         sst25vf_ChipErase();
		 return 0;
    case USB_DATA_OUT: // send data to PC
         usbMsgPtr = replyBuf;
         return sizeof(replyBuf);
    case USB_DATA_IN: // receive data from PC
		 dataLength  = (uchar)rq->wLength.word;
         dataReceived = 0;
		
		 if(dataLength  > sizeof(replyBuf)) // limit to buffer size
			dataLength  = sizeof(replyBuf);
			
		 return USB_NO_MSG; // usbFunctionWrite will be called now
   
    case ERASE_IMAGE: 
	     MEMORY_POS = rq->wValue.bytes[0] * IMAGE_SIZE;
         Block_Erase_4K(MEMORY_POS);
		 
		 Block_Erase_4K(MEMORY_POS + 4096);
		 
		 Block_Erase_4K(MEMORY_POS + 8192);
		 return 0;
		 
    case GOTO_IMAGE: 
	     MEMORY_POS = rq->wValue.bytes[0] * IMAGE_SIZE;
		 return 0;
		    
    case WRITE_IMAGE: 
	     sst25vf_WriteByte(MEMORY_POS, rq->wValue.bytes[0]);
	     MEMORY_POS++;
		 return 0;    
 
    case TEST:
    		l74hc595_setregallon(0);
    		l74hc595_setchipbyte(0, sst25vf_ReadByte(rq->wValue.bytes[0]));
			l74hc595_shiftout(); 
    return 0; 
 
   
  /* 1 ROW = 24 x 8 bit (FRONT + REAR) 
   1 Image = 1 ROW x 360 deg. = 24 x 360 = 8640  (x 8 bit FF)
   
   1mbit * 1024 kbit * 1024bit   / 8 = 131072 x 8bit
   16mbit = 131072 * 16 = 2097152 x 8bit
   Images = 2097152 / 8640 = 242 
   
   ERASE 4k = 4096 x 8 bit
   if  erase cycle takes 4k it means we have to use 4096 x3 times to erase our 8640 image. 
   so our total image size (with empty part after 4096) will be 12288 x 8 bit
   
   so 16 mbit will hold 170 images
   
   */
   
   

    }

    return 0; // should not get here
}

// This gets called when data is sent from PC to the device
USB_PUBLIC uchar usbFunctionWrite(uchar *data, uchar len) {
	uchar i;
			
	for(i = 0; dataReceived < dataLength && i < len; i++, dataReceived++)
		replyBuf[dataReceived] = data[i];
		
    return (dataReceived == dataLength); // 1 if we received it all, 0 if not
}

/*

Timer Count = Required Delay / Clock time period (1 / Frequency Mhz) - 1

0.01ms / (1 / 12000) - 1 =
0.01ms / 0,0000833333333 - 1= 120,000000048 - 1 = 119,000000048
Timer Count = 119,000000048 ticks to reach 0.01ms

prescaler 8
0.1ms / (1 / (12000 / 8)) - 1 =
0.1ms / 0,00066666666667 - 1= 149,99999999925 - 1 = 148,99999999925
Timer Count = 149 ticks to reach 0.1ms

*/

void Init_Ticker(void)
{
	
	TCCR0B= 0x00;
	TCCR0A= 0x00;
	TCNT0 = 0;
	TCCR0A |= (1 << WGM01); // Configure timer 1 for CTC mode
	TIMSK0 |= (1 << OCIE0A); // Enable CTC interrupt
	OCR0A   = 74;// 0.5 / 0.004;
	TCCR0B |= (1 << CS01);	
	sei();
	
	
	
	
	/*
	// set up timer with prescaler = 8 and CTC mode
	TCCR1B = 0x00;
	TCCR1B |= (1 << WGM12)|(1 << CS11);
	// initialize counter
	TCNT1 = 0;
	// initialize compare value
	OCR1A = 149;
	
	
	TIMSK1 |= (1 << OCIE1A); // Enable CTC interrupt
	*/

	

}


ISR(TIMER0_COMPA_vect)
{
	if (DeviceMode == MODE_CYCLING)
	{
		
		
 Ticks++;
 TicksTemp++;
 TicksSleep++;
 TicksImageTime++;
 
 if (TicksSleep >=  120000) // Time in seconts * 1000ms /   Time resolution = number of ticks   3000ms/0.05 = 60000 ticks = 3seconds
 {
	EnteringSleep = 1;
	TicksSleep = 0; 
 }
 
 if (TicksTemp >= AngleTime)
 {
	 AngleIDX++;
	 
	 if (AngleIDX > AngleCount -1)
	 {
		 //AngleIDX = 0;
		Display = 0;
        ClearLed = 1;
		TicksTemp = 0;
	 } 
	 else
	 {
	   TicksTemp = 0;
	   Display = 1;		 
	 }

 }
 
 if (ImagesCnt > 1)
 {
  if (TicksImageTime >=  100000) // Time in seconts * 1000ms /   Time resolution = number of ticks   3000ms/0.05 = 60000 ticks = 3seconds
  {
	  NextImage = 1;
	  TicksImageTime =0;	  
	  
  }
 } 
  
 
 }
}

ISR(TIMER1_COMPA_vect)
{
	 
}


void Hall_Action (void)
{
	wdt_disable();
	//cli();
	DeviceMode = MODE_CYCLING;
	AngleTime = Ticks / AngleCount;


	Ticks = 0;
	TicksTemp = 0;
	AngleIDX = 0;
	Display = 1;
	TicksSleep = 0;
	EnteringSleep = 0;
	AfterSleepCircles = 0;
	// need reset timer
	Init_Ticker();

	sei();	
}


ISR(INT1_vect)
{ 

if (DeviceMode == MODE_SLEEP)
{
	 AfterSleepCircles++;
	 if (AfterSleepCircles >= 3)
	 {
		 Hall_Action();
	 }
 	
} else

{
 Hall_Action();
}
 
	
	

}









 

 







void Init_Hall(void)
{
	 EIMSK |= _BV(INT1);  //Enable INT1
	 EICRA |= _BV(ISC11); //Trigger on falling edge of INT0
	 sei();	
}



void DisplayAngle (void)
{
 
 //uint8_t arrrrr[24];
 
  //1 angle = 24 x 8 bit
  
  /* VARIANT 1
  sst25vf_Read(IDX * 24, arrrrr, 24);
   for (int a = 0; a < 24; a++)
   {
	   l74hc595_setchipbyte(a, arrrrr[a]);
   }
  l74hc595_shiftout();
  */
  
  
 // VARIANT 2
 sst25vf_Read_Reverse(ImageIDX * IMAGE_SIZE + AngleIDX * 24, l74hc595_icarray, 24); 
 l74hc595_shiftout(); 
 
  
/*//  VARIANT 3
for (int a = 0; a < 24; a++)
  {
	l74hc595_setchipbyte(a, sst25vf_ReadByte(AngleIDX * 24 + a));  
  }
   l74hc595_shiftout(); 
   */
}


void Init_USB (void)
{
	uchar i;
    wdt_enable(WDTO_1S); // enable 1s watchdog timer
    usbInit();
	
    usbDeviceDisconnect(); // enforce re-enumeration
    for(i = 0; i<250; i++) { // wait 500 ms
        wdt_reset(); // keep the watchdog happy
        _delay_ms(2);
    }
    usbDeviceConnect();
	
	sei(); 
}

int main() {
	
	Init_Hall();
	l74hc595_init();	
	SST25VF_init();
	Init_USB();
	

   while(1)
    {
		
	if (EnteringSleep == 1)
	{
		/*DeviceMode = MODE_SLEEP;
			TCCR0B= 0x00;
			TCCR0A= 0x00;
			TCNT0 = 0;
		EnteringSleep = 0;
		l74hc595_setchipbyte(0, 0b11111101);
		l74hc595_shiftout();
		Delay_ms(1000);
		l74hc595_setregallon(1);*/
	}
		
		
	if (DeviceMode == MODE_USB)
	{
		wdt_reset(); // keep the watchdog happy
		usbPoll();
	}

	else
	
	if (DeviceMode == MODE_CYCLING)
	{	
		
		
	/*	if (NextImage == 1)
		{
			NextImage = 0;
			ImageIDX++;
			if (ImageIDX >= ImagesCnt - 1)
			{
				ImageIDX = 0;
			}
			
		}	*/
		
	  if (Display == 1)
	  {
		  Display = 0;
		  DisplayAngle();
		 
	  }
	 
	 if (ClearLed == 1)
	 {
		 			l74hc595_setregallon(1);
		 			
					 ClearLed = 0;
	 }
	 
	 	   
	 }
	 
    }
	
	
    return 0;
}


/*

int main() {

Init_Hall();
l74hc595_init();
l74hc595_setregallon();
l74hc595_shiftout();
SST25VF_init();



uchar i;
wdt_enable(WDTO_1S); // enable 1s watchdog timer
usbInit();

usbDeviceDisconnect(); // enforce re-enumeration
for(i = 0; i<250; i++) { // wait 500 ms
wdt_reset(); // keep the watchdog happy
_delay_ms(2);
}
usbDeviceConnect();

sei(); // Enable interrupts after re-enumeration


while(1) {
if (DeviceMode == MODE_USB)
{
wdt_reset(); // keep the watchdog happy
usbPoll();
}

else

{

if (Display == 1)
{

for (int s = 0; s < 360; s++)
{
DisplayAngle(s);
Delay_ms(5);
}


//DisplayAngle(AngleIDX);
//Display = 0;
}





}



}

return 0;
}
*/
