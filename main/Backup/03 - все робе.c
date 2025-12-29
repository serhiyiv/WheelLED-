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
#include <avr/eeprom.h>

#include "usbdrv.h"

#define F_CPU 12000000L
#include <util/delay.h>








#define ERASE_CHIP   0
#define ERASE_IMAGE    1
#define GOTO_IMAGE     2
#define WRITE_IMAGE    3


#define SETTINGS_GOTO    4
#define SETTINGS_WRITE   5
#define SETTINGS_READ    6
#define SETTINGS_UPDATE  7

#define TEST    22
#define USB_DATA_OUT 20
#define USB_DATA_IN 21






int MEMORY_POS = 0;
int IMAGE_SIZE = 12288;
int SETTINGS_POS = 0;

#define MODE_USB        0
#define MODE_CYCLING    1
#define MODE_SLEEP      2

volatile uint8_t DeviceMode = MODE_USB;  // 0 - USB data   / 1 - Cycling

// TIMERS
int   CPU_CLOCK = 12000;
int   Timer_Prescaller = 8;
float   Timer_Resolution = 0;
int  Timer_Counts = 50;
float Timer_Overflow;

volatile  unsigned long int  Ticks = 50000;
volatile  unsigned long int  TicksTemp = 0;
volatile  unsigned long int  TicksSleep = 0;
volatile  unsigned long int  TicksImageTime = 0;


// IMAGES

volatile uint8_t  NextImage = 0;
volatile uint8_t  FirstRun   = 1;


#define SIDE_LEFT   0
#define SIDE_RIGHT  1

volatile uint8_t  DisplaySide = SIDE_LEFT;



volatile unsigned long int   AngleCount = 360;
volatile unsigned long int  AngleTime = 200;



// PROCESSING
volatile uint8_t Display = 1;
volatile uint8_t ClearLed = 0;
volatile uint8_t FirstCircle = 0;
volatile uint8_t EnteringSleep = 0;
volatile uint8_t AfterSleepCircles = 0;
volatile unsigned long int AngleIDX = 0;
volatile unsigned long int AngleIDX_wOffset = 0;
volatile unsigned long int LastAngle = -1;
volatile uint8_t ImageIDX = 0;
volatile unsigned long int CurrentImageAddress = 0;




//Settings
int SETTINGS_IMAGES_CHANGE_TIME = 9000;
volatile uint8_t SETTINGS_IMAGES_COUNT = 1;
uint8_t SETTINGS_DISPLAY_MODE = 0;   
// 0 - Same Image on Both Sides
// 1 - Different Images on Different Sides
// 2 - Front only. Rear OFF
// 3 - Rear Only. Front OFF
// 4 - Toggle Sides. One side always Off
volatile unsigned long int SETTINGS_START_IMAGE_IDX = 0;
volatile unsigned long int SETTINGS_ANGLE_OFFSET = 0;
uint8_t SETTINGS_REVERSE_DIRECTION = 0;
uint8_t SETTINGS_SLEEP_TIME = 0;



#define READ_LINE(port, pin)    port & (1<<pin)
#define setOutput(ddr, pin) ((ddr) |= (1 << (pin)))
#define setLow(port, pin) ((port) &= ~(1 << (pin)))
#define setHigh(port, pin) ((port) |= (1 << (pin)))



// Buttons
#define Button_DDR  DDRD
#define Button_PORT PORTD
#define Button_Pin PIND
#define Button_Pin1 PIND0
#define Button_Pin2 PIND1
#define DEBOUNCE_TIME 25
#define DEBOUNCE_TIME_LONG 1000


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


void Init_Settings (void)
{
	
	
	SETTINGS_START_IMAGE_IDX    = eeprom_read_byte(0); ImageIDX = 0;
	SETTINGS_ANGLE_OFFSET       = eeprom_read_byte(1) * 2; AngleIDX_wOffset = SETTINGS_ANGLE_OFFSET;
	SETTINGS_REVERSE_DIRECTION  = eeprom_read_byte(2);
	SETTINGS_DISPLAY_MODE       = eeprom_read_byte(3);
	SETTINGS_IMAGES_COUNT       = eeprom_read_byte(4);
	SETTINGS_IMAGES_CHANGE_TIME = eeprom_read_byte(5);





	
}


 
   
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
		 		 
	case SETTINGS_GOTO:
	     SETTINGS_POS = rq->wValue.bytes[0];
	return 0;	 
	
	case SETTINGS_WRITE:
	     eeprom_write_byte(SETTINGS_POS, rq->wValue.bytes[0]);
	return 0;		
	case SETTINGS_READ:
	
	return 0;		
	case SETTINGS_UPDATE:
	 Init_Settings();
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
	
Ticks++;
TicksTemp++;
//TicksSleep++;

/*
if (TicksSleep >=  3 * 1000 / 0.0333333333335) // Time in seconts * 1000ms /   Time resolution = number of ticks   3000ms/0.05 = 60000 ticks = 3seconds
{
	EnteringSleep = 1;
	TicksSleep = 0;
}*/

if (TicksTemp >= AngleTime)
{
	volatile int TempAngle = AngleIDX;
	
	if (SETTINGS_REVERSE_DIRECTION == 1)
	{
		TempAngle--;
		if (TempAngle < 0 )
		{
			AngleIDX = AngleCount -1 ;
			Display = 0;
			ClearLed = 1;
			TicksTemp = 0;
		}
		else
		{
			TicksTemp = 0;
			Display = 1;
			AngleIDX = TempAngle;
		}		
		
		
		
	} 
	else
	{
		TempAngle++;
		if (TempAngle > AngleCount -1 )
		{
			AngleIDX = 0;
			Display = 0;
			ClearLed = 1;
			TicksTemp = 0;
		}
		else
		{
			TicksTemp = 0;
			Display = 1;
			AngleIDX = TempAngle;
		}	
				
	}
	

	
	
	

}

if (SETTINGS_IMAGES_COUNT > 1)
{
	TicksImageTime++;
	if (TicksImageTime >=  40000) // Time in seconts * 1000ms /   Time resolution = number of ticks   3000ms/0.05 = 60000 ticks = 3seconds
	{
		NextImage = 1;
		TicksImageTime =0;
		
	}
}


}		

  



void Hall_Action (void)
{
	
	if (FirstRun == 1)
	{
		FirstRun = 0;
		wdt_disable();
		DeviceMode = MODE_CYCLING;
		Init_Ticker();
		sei();
	}


	if (NextImage == 1)
	{
		NextImage = 0;
		uint8_t TempImageIDX = ImageIDX;		
		
		TempImageIDX++;
				
		if (TempImageIDX > SETTINGS_IMAGES_COUNT - 1)
		{
			ImageIDX = 0;
		}
		else
		{
			ImageIDX = TempImageIDX;
		}
		TicksImageTime = 0;
		
		
		if (SETTINGS_DISPLAY_MODE == 4)
		{
		 if (DisplaySide == SIDE_LEFT) {DisplaySide = SIDE_RIGHT;} 
		 else { DisplaySide = SIDE_LEFT; }		 	
		}
			
	}







	volatile  int TTT = Ticks;	
	AngleTime = TTT / 360;
	
	
	//AngleTime =((((Ticks*50)+TCNT0)*0.00066666666667) / 360) / 0,00066666666667 - 1;
	
	
	
  
	
	if (SETTINGS_REVERSE_DIRECTION == 1) 
	{
		AngleIDX = AngleCount - 1;
	} else
	{
	    AngleIDX = 0;
	}
	
	
	//AngleIDX_wOffset = AngleIDX + SETTINGS_ANGLE_OFFSET;
	
	TCNT0 = 0;
	Ticks = 0;
	TicksTemp = 0;
	
	Display = 1;
	TicksSleep = 0;
	EnteringSleep = 0;
	AfterSleepCircles = 0;	
	ClearLed = 0;	
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


if (SETTINGS_DISPLAY_MODE == 0)// 0 - Same Image on Both Sides
{
	sst25vf_Read_Reverse(ImageIDX * IMAGE_SIZE + AngleIDX * 24, l74hc595_icarray, 24); 
	l74hc595_shiftout();
} 
else if (SETTINGS_DISPLAY_MODE == 1)// 1 - Different Images on Different Sides
{
	uint8_t   NextImageIDX ;
	uint8_t  TempNextImageIDX = ImageIDX + 1;
	
	if (TempNextImageIDX > SETTINGS_IMAGES_COUNT - 1)
	{
		NextImageIDX = 0;
	}
	else
	{
	  NextImageIDX = TempNextImageIDX;	
	}

	for (int a = 0; a < 12; a++)
	{
		l74hc595_setchipbyte(a, sst25vf_ReadByte(ImageIDX * IMAGE_SIZE + AngleIDX * 24 + a));
		l74hc595_setchipbyte(12 + a, sst25vf_ReadByte(NextImageIDX * IMAGE_SIZE + AngleIDX * 24 + a + 12));
	}
	l74hc595_shiftout();	
}

else if (SETTINGS_DISPLAY_MODE == 2 ||SETTINGS_DISPLAY_MODE == 3) // 2 - Front only. Rear OFF  // 3 - Rear Only. Front OFF
{
  l74hc595_setregallon(0);
	for (int a = 0; a < 12; a++)
	{
		if (SETTINGS_DISPLAY_MODE == 2) { l74hc595_setchipbyte(a, sst25vf_ReadByte(ImageIDX * IMAGE_SIZE + AngleIDX * 24 + a));}
		if (SETTINGS_DISPLAY_MODE == 3) { l74hc595_setchipbyte(12 + a, sst25vf_ReadByte(ImageIDX * IMAGE_SIZE + AngleIDX * 24 + a + 12));}
	}
	l74hc595_shiftout();
	
}
else if (SETTINGS_DISPLAY_MODE == 4) // 4 - Toggle Sides. One side always Off
{
	l74hc595_setregallon(0);
	for (int a = 0; a < 12; a++)
	{
		if (DisplaySide == SIDE_LEFT) { l74hc595_setchipbyte(a, sst25vf_ReadByte(ImageIDX * IMAGE_SIZE + AngleIDX * 24 + a));}
		if (DisplaySide == SIDE_RIGHT) { l74hc595_setchipbyte(12 + a, sst25vf_ReadByte(ImageIDX * IMAGE_SIZE + AngleIDX * 24 + a + 12));}
	}
	l74hc595_shiftout();	
}
 
 




 
 
 
 
 
 
 

 
 
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














void Init_Buttons (void)
{

 
 Button_DDR = 1<<Button_Pin1;		// Set Button_Pin1 as input
 Button_DDR = 1<<Button_Pin2;		// Set Button_Pin2 as input
 Button_PORT = 1<<Button_Pin1;		// Enable PD2 pull-up resistor
 Button_PORT = 1<<Button_Pin1;	
 sei();
 
 	
}
	

uint8_t Button_Pressed(uint8_t Pin)
{
	uint8_t ButtonPin = READ_LINE(Button_Pin, Pin);
	if (ButtonPin == 0)
	{
		Delay_ms(DEBOUNCE_TIME);
		ButtonPin = READ_LINE(Button_Pin, Pin);
		if (ButtonPin == 0)
		{
			
		   Delay_ms(DEBOUNCE_TIME_LONG);
		   ButtonPin = READ_LINE(Button_Pin, Pin);	
			if (ButtonPin == 0)
			{
				return 2;
			}
			
			
			
			return 1;
		}
		
	}
	return 0;
}

int main() {
	
	
	//Timer_Resolution = 1 / (CPU_CLOCK / Timer_Prescaller);
	
	
	
	
	Init_Hall();
	l74hc595_init();	
	SST25VF_init();
	Init_USB();
  Init_Buttons();
  Init_Settings();
    sei();
	

   while(1)
    {
    uint8_t ButtonRes = Button_Pressed(Button_Pin1);	
    if ( ButtonRes == 1)
    {
      l74hc595_setchipbyte(0, 0b11111101);
      l74hc595_shiftout();	
    }
    else
    if ( ButtonRes == 2)
    {
      l74hc595_setchipbyte(0, 0b11111011);
      l74hc595_shiftout();
    }	
      
	if (EnteringSleep == 1)
	{
		DeviceMode = MODE_SLEEP;
			TCCR0B= 0x00;
			TCCR0A= 0x00;
			TCNT0 = 0;
		EnteringSleep = 0;
		l74hc595_setchipbyte(0, 0b11111101);
		l74hc595_shiftout();
		Delay_ms(1000);
		l74hc595_setregallon(1);
	}	
	
	  if (DeviceMode == MODE_USB)
	  {
		  wdt_reset(); // keep the watchdog happy
		  usbPoll();
	  }

	  else
	  
	  if (DeviceMode == MODE_CYCLING)
	  {
		
		
		
		
		if (Display == 1)
		{
			//Display=0; 
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

