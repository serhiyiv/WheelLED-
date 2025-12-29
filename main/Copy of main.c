
#define F_CPU 12000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

#include "usbdrv.h"
#include "l74hc595/l74hc595.c"
#include "SST25VFxxxx/SST25VFxxxx.c"
#include "Light_apa102/light_apa102.c"
#include "TWI/i2c.c"

#define MODE_USB        0
#define MODE_CYCLING    1
#define MODE_SLEEP      2
volatile uint8_t DeviceMode = MODE_USB;

#define SIDE_LEFT   0
#define SIDE_RIGHT  1
volatile uint8_t  DisplaySide = SIDE_LEFT;

static uint8_t USB_BUFFER_INT[1];
static uchar USB_BUFFER_CHAR[16] = "Hello, USB!";
static uchar dataReceived = 0, dataLength = 0; // for USB_DATA_IN

// USB Commands
#define CMD_ERASE_CHIP       0
#define CMD_ERASE_IMAGE      1
#define CMD_GOTO_IMAGE       2
#define CMD_WRITE_IMAGE      3
#define CMD_SETTINGS_GOTO    4
#define CMD_SETTINGS_WRITE   5
#define CMD_SETTINGS_READ    6
#define CMD_SETTINGS_UPDATE  7
#define CMD_READ_FLASH       8
#define CMD_READ_EEPROM      9
#define CMD_SET_CONNECTED    10

#define TEST    22
#define USB_DATA_OUT 20
#define USB_DATA_IN 21

// Settings
volatile uint32_t SETTINGS_IMAGES_CHANGE_TIME = 9000;
volatile uint32_t SETTINGS_IMAGES_COUNT = 1;
volatile uint8_t  SETTINGS_DISPLAY_MODE = 0;
// 0 - Same Image on Both Sides
// 1 - Different Images on Different Sides
// 2 - Front only. Rear OFF
// 3 - Rear Only. Front OFF
// 4 - Toggle Sides. One side always Off
volatile uint32_t SETTINGS_START_IMAGE_IDX = 0;
volatile uint32_t SETTINGS_ANGLE_OFFSET = 0;
volatile uint8_t  SETTINGS_REVERSE_DIRECTION = 0;
volatile uint32_t SETTINGS_SLEEP_TIME = 0;

// Buttons
#define BUTTON_PORT PORTD       /* PORTx - register for button output */
#define BUTTON_PIN  PIND         /* PINx - register for button input */
#define BUTTON_PREV PIND1          /* bit for button input/output */
#define BUTTON_NEXT PIND0          /* bit for button input/output */



volatile uint32_t MEMORY_POS = 0;
volatile uint8_t  SETTINGS_POS = 0;

volatile uint32_t IMAGE_SIZE = 36864; // EQUAL TO 4096
volatile uint32_t AngleCount = 300;

// FLAGS
volatile uint8_t Display = 1;
volatile uint8_t AfterSleepCircles = 0;
volatile uint8_t FirstRun = 1;
volatile uint8_t ImageLocked = 0;
volatile uint8_t LoadingIntoLEDS = 0;

// Processing
volatile uint32_t AngleIDX = 0;
volatile uint32_t AngleIDX_wOffset = 0;
volatile uint32_t ImageIDX = 0;
volatile uint32_t ImageTime = 0;
volatile uint32_t InactiveTime = 0;



volatile uint32_t ANGLE_LED_COUNT_SINGLE = 32;
volatile uint32_t ANGLE_LED_COUNT_RGB  =   96;  // ANGLE_LED_COUNT_SINGLE * 3

uint8_t TWI_MODE = 0; // 1 = Master/ 0 = Slave / 2 = OFF

#define TWI_ADDRESS_MASTER 0xA0
#define TWI_ADDRESS_SLAVE 0x07



unsigned char TWI_Buffer[100];
#define StrCompare(s1,s2)  (!strcmp ((s1),(s2)))

/// Кольорову палітру закинемо в внутрішній ЕЕПРОМ і будем звідти зчитувати

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
	SETTINGS_ANGLE_OFFSET       = eeprom_read_byte((const uint8_t *)1) * 2; 
	SETTINGS_REVERSE_DIRECTION  = eeprom_read_byte((const uint8_t *)2);
	SETTINGS_DISPLAY_MODE       = eeprom_read_byte((const uint8_t *)3);
	SETTINGS_IMAGES_COUNT       = eeprom_read_byte((const uint8_t *)4);
	SETTINGS_IMAGES_CHANGE_TIME = (eeprom_read_byte((const uint8_t *)5) * 46875UL);
	SETTINGS_SLEEP_TIME = (eeprom_read_byte((const uint8_t *)6) * 46875UL);
	
}


 
  
// this gets called when custom control message is received
USB_PUBLIC uchar usbFunctionSetup(uchar data[8]) {
	usbRequest_t *rq = (void *)data; // cast data to correct type
	
	
	switch(rq->bRequest) { // custom command is in the bRequest field
     
    case CMD_ERASE_CHIP:
         sst25vf_ChipErase();
		 return 0;
    case USB_DATA_OUT: // send data to PC
        USB_BUFFER_INT[0] = 121;
	    usbMsgPtr = USB_BUFFER_INT;
        return sizeof(USB_BUFFER_INT);
         
		 
    case CMD_READ_FLASH: // send data to PC
       	 USB_BUFFER_INT[0] = sst25vf_ReadByte(MEMORY_POS);
	     MEMORY_POS++;
	     usbMsgPtr = USB_BUFFER_INT;
	     return sizeof(USB_BUFFER_INT);	 
		
    case CMD_READ_EEPROM: // send data to PC
		USB_BUFFER_INT[0] = eeprom_read_byte(SETTINGS_POS);
		
		usbMsgPtr = USB_BUFFER_INT;
		return sizeof(USB_BUFFER_INT);		
		
	 
		 
		 
    case USB_DATA_IN: // receive data from PC
		 dataLength  = (uchar)rq->wLength.word;
         dataReceived = 0;
		
		 if(dataLength  > sizeof(USB_BUFFER_CHAR)) // limit to buffer size
			dataLength  = sizeof(USB_BUFFER_CHAR);
			
		 return USB_NO_MSG; // usbFunctionWrite will be called now
   
    case CMD_ERASE_IMAGE: 
	     MEMORY_POS = rq->wValue.bytes[0] * IMAGE_SIZE;
    	
		 uint32_t ERASE_POS = MEMORY_POS;		 
		 uint8_t NumErases = IMAGE_SIZE / 4096;//uint8_t NumErases = IMAGE_SIZE / 4096;
		 for (uint8_t I = 0; I < NumErases; I++)
		 {
			 Block_Erase_4K(ERASE_POS);
			 Delay_ms(10);
			 ERASE_POS += 4096;
		 }
				 		 
		
		 return 0;
		 
    case CMD_GOTO_IMAGE: 
	     MEMORY_POS = rq->wValue.bytes[0] * IMAGE_SIZE;
		 return 0;
		    
    case CMD_WRITE_IMAGE: 
	     sst25vf_WriteByte(MEMORY_POS, rq->wValue.bytes[0]);
	     MEMORY_POS++;
		 return 0;    
		 		 
	case CMD_SETTINGS_GOTO:
	     SETTINGS_POS = rq->wValue.bytes[0];
	return 0;	 
	
	case CMD_SETTINGS_WRITE:
	     eeprom_write_byte(SETTINGS_POS, rq->wValue.bytes[0]);
	return 0;		
	case CMD_SETTINGS_READ:
	  // eeprom_read_byte(SETTINGS_POS);
	return 0;		
	case CMD_SETTINGS_UPDATE:
	 Init_Settings();
	return 0;
	
	case CMD_SET_CONNECTED:
		DeviceMode = MODE_USB;
		FirstRun = 1;
		TCCR0B= 0x00;
		TCCR0A= 0x00;
		TCNT0 = 0;

		TCCR1B= 0x00;
		TCCR1A= 0x00;
		TCNT1 = 0;
		//l74hc595_setregallon(1);
	return 0; 
	
    case TEST:
    		/*l74hc595_setregallon(0);
    		l74hc595_setchipbyte(0, sst25vf_ReadByte(rq->wValue.bytes[0]));
			l74hc595_shiftout(); */
    return 0; 
  

    }

    return 0; // should not get here
}
// This gets called when data is sent from PC to the device
USB_PUBLIC uchar usbFunctionWrite(uchar *data, uchar len) {
	uchar i;
			
	for(i = 0; dataReceived < dataLength && i < len; i++, dataReceived++)
		USB_BUFFER_CHAR[dataReceived] = data[i];
		
    return (dataReceived == dataLength); // 1 if we received it all, 0 if not
}



void Init_AngleTimer(void)
{	
	//TCCR0B= 0x00;
	//TCCR0A= 0x00;
	TCNT0 = 0;
	TCCR0A |= (1 << WGM01); // Configure timer 1 for CTC mode
	TIMSK0 |= (1 << OCIE0A); // Enable CTC interrupt
	OCR0A   =  TCNT1  / AngleCount;// 0.5 / 0.004;
	TCCR0B |= (1 << CS02);
}

void Init_TotalTimer(void)
{
	// set up timer with prescaler = 8
	TCCR1B |= (1 << CS12);	
	// initialize counter
	TCNT1 = 0;	
	// enable overflow interrupt
	TIMSK1 |= (1 << TOIE1);	
	//0,0213333333333333   -OCR0 - 32
	
	//0,0213333333333333 * 65535 = 1398.0799999999978155
	// 1 Sec = 46875	
}

ISR(TIMER1_OVF_vect)
{
	InactiveTime++; 
}

ISR(TIMER0_COMPA_vect) // ANGLE CHANGE TIMER
{	

if (SETTINGS_REVERSE_DIRECTION == 1)
{
	AngleIDX--;
	if (AngleIDX < 0 )
	{
		Display = 0;
	}
	else
	{
		Display = 1;
	}		
}
else
{
	AngleIDX++;
    if (AngleIDX > AngleCount -1 )
	{
	  Display = 0;
	}
	else
	{			
	  Display = 1;
	  
	 AngleIDX_wOffset =SETTINGS_ANGLE_OFFSET+ AngleIDX ; 
	 if (AngleIDX_wOffset > AngleCount - 1)
	 {
		 AngleIDX_wOffset = (AngleIDX + SETTINGS_ANGLE_OFFSET) - (AngleCount - 1);
	 }
	   
	  
	  
	}		
}

}




void Hall_Action (void)
{
	
	if (FirstRun == 1)
	{
		FirstRun = 0;
		ImageTime = 0; 
		DeviceMode = MODE_CYCLING;
		//wdt_disable();
		Init_TotalTimer();
		Init_AngleTimer();
	}

if (InactiveTime > 0) // SPEED < 5.2 kn  nothing is displayed,  resets inactive time to keep timer go on
{
	Display = 0;
	InactiveTime = 0;
	
	APA102_SetAll(0);
	AngleData[29].r = 255;
	AngleData[30].r = 255;
	AngleData[31].r = 255;
	apa102_setleds(AngleData, 64);
	

}
else
{
	

// 1 Sec = 46875// 5 sec = 234375 
 ImageTime = ImageTime + TCNT1 + InactiveTime * 65535;
 if (ImageTime >= SETTINGS_IMAGES_CHANGE_TIME)
 {
	
	
	if (ImageLocked == 0)
	{
	
	 ImageIDX ++;
	 if (ImageIDX > SETTINGS_IMAGES_COUNT - 1)
	 {
		 ImageIDX = 0;
	 }
	 
	 if (SETTINGS_DISPLAY_MODE == 4)
	 {
		 if (DisplaySide == SIDE_LEFT) {DisplaySide = SIDE_RIGHT;}
		 else { DisplaySide = SIDE_LEFT; }
	 } 
	 
	 
	} 
	 
	 
  ImageTime = 0; 
 }


 
 
  if (SETTINGS_REVERSE_DIRECTION == 1)
  {
	AngleIDX = AngleCount - 1;
	AngleIDX_wOffset = AngleIDX - SETTINGS_ANGLE_OFFSET;
  } else
  {
	AngleIDX = 0;
	AngleIDX_wOffset = AngleIDX + SETTINGS_ANGLE_OFFSET;
  }
  
 
 
 
 Display = 1;
 InactiveTime = 0;
 Init_AngleTimer();
 TCNT1 = 0;
 
 sei();
  }

}


ISR(INT1_vect)
{ 
	if (DeviceMode == MODE_SLEEP)
	{
		AfterSleepCircles++;
		if (AfterSleepCircles >= 3)
		{
			FirstRun = 1;
			AfterSleepCircles = 0;
			Hall_Action();
		}
		
	} 
	else
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

void Upload_LEDS(uint32_t startAddr, uint16_t nLeds)
{
 uint32_t i;	

	APA102_Transfer(0x00);  // Start Frame
	APA102_Transfer(0x00);
	APA102_Transfer(0x00);
	APA102_Transfer(0x00);	

	sst25vf_Select();
	spiSendByte(SST25VF_RD);
	SendAddr(startAddr);
	
	for(i=0;i<nLeds+nLeds+nLeds;i+=3)
	{
		APA102_Transfer(0xff);  // Maximum global brightness   
		APA102_Transfer(sst25vf_GetByte()); // Read B
		APA102_Transfer(sst25vf_GetByte());	// Read G
		APA102_Transfer(sst25vf_GetByte());	// Read R	
		
	}
	sst25vf_DeSelect();
	
	// End frame: 8+8*(leds >> 4) clock cycles
	for (i=0; i<nLeds; i+=16)
	{
		APA102_Transfer(0xff);  // 8 more clock cycles
	}	
			
}


void DisplayAngle (void)
{

 if(LoadingIntoLEDS == 0)
 {
   LoadingIntoLEDS = 1;
	if (SETTINGS_DISPLAY_MODE == 0)// 0 - Same Image on Both Sides
	{	
      Upload_LEDS(ImageIDX * IMAGE_SIZE + AngleIDX_wOffset * ANGLE_LED_COUNT_RGB, ANGLE_LED_COUNT_SINGLE);		
	} 
	else if (SETTINGS_DISPLAY_MODE == 1)// 1 - Different Images on Different Sides
	{
		
	}
	else if (SETTINGS_DISPLAY_MODE == 2 ||SETTINGS_DISPLAY_MODE == 3) // 2 - Front only. Rear OFF  // 3 - Rear Only. Front OFF
	{
	
	}
	else if (SETTINGS_DISPLAY_MODE == 4) // 4 - Toggle Sides. One side always Off
	{
	
	}
	
   LoadingIntoLEDS = 0;
  }
 
}


void Init_USB (void)
{
	uchar i;
	//wdt_enable(WDTO_1S); // enable 1s watchdog timer
	usbInit();
	
	usbDeviceDisconnect(); // enforce re-enumeration
	for(i = 0; i <250; i++) { // wait 500 ms
		//wdt_reset(); // keep the watchdog happy
		_delay_ms(2);
	}
	usbDeviceConnect();
	sei();
}



int ButtonState(uint8_t Button)
{
	if (bit_is_clear(BUTTON_PIN, Button))
	{
		return 1;
	}
	return 0;
}


void Init_Buttons (void)
{
 //BUTTON_PORT |= _BV(BUTTON_PREV);
 BUTTON_PORT |= _BV(BUTTON_NEXT);	
}
	


void ProcessButtons(void)
{
	/*if (ButtonState(BUTTON_PREV))
	{	
		Delay_ms(200);
		if (ButtonState(BUTTON_PREV))
		{
			
			if (ImageLocked == 0)
			{
				ImageLocked = 1;
				/*l74hc595_setregallon(0);
				l74hc595_setchipbyte(11, 0b01101101);
				l74hc595_shiftout();
				Delay_ms(500);*/
		/*	} 
			else
			{
				ImageLocked = 0;
				/*l74hc595_setregallon(0);
				l74hc595_setchipbyte(11,0b10110110 );
				l74hc595_shiftout();
				Delay_ms(500);	*/		
		/*	}
						
		}
		else
		{
			ImageIDX++;		
			/*l74hc595_setregallon(0);
			l74hc595_setregoff(ImageIDX);
			l74hc595_shiftout();
			Delay_ms(500);*/
	/*	}
		
		
	}
	*/



	if (ButtonState(BUTTON_NEXT))
	{
		
		Delay_ms(1000);
		if (ButtonState(BUTTON_NEXT))
		{
			
		  wdt_enable(WDTO_1S); // enable 1s watchdog timer
		 _delay_ms(1000);
			
			
			
			
			
			
			
			
			
			/*uint8_t LedValue;
			SETTINGS_DISPLAY_MODE++;
			if (SETTINGS_DISPLAY_MODE > 4)
			{
				SETTINGS_DISPLAY_MODE = 0;
			}*/
			//l74hc595_setregallon(0);
			/*switch (SETTINGS_DISPLAY_MODE)
			{
			case 0: l74hc595_setchipbyte(5, 0b11111110);break;
			case 1: l74hc595_setchipbyte(5, 0b11110111);break;
			case 2: l74hc595_setchipbyte(5, 0b10111111);break;
			case 3: l74hc595_setchipbyte(6, 0b11111101);break;
			case 4: l74hc595_setchipbyte(6, 0b11101111);break;
			}
			l74hc595_shiftout();
			*/
			
			
			/*Delay_ms(500);*/	
		}
		/*else
		{
			ImageIDX--;
			
			/*l74hc595_setregallon(0);
			l74hc595_setregoff(ImageIDX);
			l74hc595_shiftout();
			Delay_ms(500);	*/		
			
		/*}*/
		
	}
}


void i2cSlaveReceiveService(u08 receiveDataLength, u08* receiveData)
{

	uint8_t TWI_Received;
	
	TWI_Received = atoi(receiveData);
	ImageIDX= TWI_Received;
	
	
	
	/*if (TWI_Received == 177)
	{
		APA102_SetAll(0);
		AngleData[0].r = 255;
		AngleData[1].g = 255;
		AngleData[2].b = 255;
		apa102_setleds(AngleData, 64);
	}

	if (TWI_Received == 178)
	{
		APA102_SetAll(0);
		AngleData[0].r = 255;
		AngleData[1].r = 255;
		AngleData[2].r = 255;
		apa102_setleds(AngleData, 64);
		
	}
	
	
	sprintf(TWI_Buffer,"%s",receiveData,receiveDataLength);
	
	if (StrCompare(TWI_Buffer , "on"))
	{

		setHigh(PORTC, PC0);
	}

	if (StrCompare(TWI_Buffer , "off"))
	{
		setLow(PORTC, PC0);
		
	}
	*/
	
	
	
}






void TWI_Send_String(char * Data)
{
	sprintf(TWI_Buffer,"%s",Data);TWI_Buffer[strlen(TWI_Buffer)]=0x00;
	i2cMasterSend(TWI_ADDRESS_SLAVE,strlen(TWI_Buffer)+1/*(0x00)*/,&TWI_Buffer[0]);
}

void TWI_Send_Int(uint8_t  Data)
{
	sprintf(TWI_Buffer,"%d",Data);TWI_Buffer[strlen(TWI_Buffer)]=0x00;
	i2cMasterSend(TWI_ADDRESS_SLAVE,strlen(TWI_Buffer)+1/*(0x00)*/,&TWI_Buffer[0]);
}


void Init_TWI(void)
{
	

	 
	 if (TWI_MODE == 0) // SLAVE
	 {
		  i2cInit(0);
		  // set local device address and allow response to general call
		  i2cSetLocalDeviceAddr(TWI_ADDRESS_SLAVE, TRUE);
		 i2cSetSlaveReceiveHandler( i2cSlaveReceiveService ); 
	 } else
	 {
		  i2cInit(1);
		 	 // set local device address and allow response to general call
		 	 i2cSetLocalDeviceAddr(TWI_ADDRESS_MASTER, TRUE);
	 }

}


ISR (PCINT0_vect)
{
//https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328
}


void TurnOff_Peripherals(void)
{
	
}


void PowerOff(void)
{
	TurnOff_Peripherals();
	set_sleep_mode(SLEEP_MODE_STANDBY);
	sleep_enable();
	sleep_cpu();	
}

void Init_Power(void)
{
 
 DDRB &= ~(1 << DDB0);         // Clear the PB0 pin
 // PB0 (PCINT0 pin) is now an input

 PORTB |= (1 << PORTB0);        // turn On the Pull-up
 // PB0 is now an input with pull-up enabled

 PCICR |= (1 << PCIE0);     // set PCIE0 to enable PCMSK0 scan
 PCMSK0 |= (1 << PCINT0);   // set PCINT0 to trigger an interrupt on state change

 sei();                     // turn on interrupts

	
}





int main() {
 
 Init_Settings();	
 Init_Hall();
 l74hc595_init();
 APA102_init();	
 SST25VF_init();
 Init_USB();
 Init_Buttons();
 
 Init_TWI();
 //Init_TWI(Master);

 
 sei();
	
APA102_SetAll(0);
AngleData[5].r = 255;
apa102_setleds(AngleData, 64);


https://learn.adafruit.com/led-tricks-gamma-correction/the-quick-fix





  while(1)
  {
		
    ProcessButtons();	
    usbPoll();

	 if (DeviceMode == MODE_CYCLING)
	 {
		if (InactiveTime > 0)
		{
			Display = 0;
				APA102_SetAll(0);
				AngleData[29].g = 255;
				AngleData[30].g = 255;
				AngleData[31].g = 255;
				apa102_setleds(AngleData, 64);

		}

		if ((InactiveTime * 65535UL) + TCNT1 >= SETTINGS_SLEEP_TIME)
		{
			DeviceMode = MODE_SLEEP;
			TCCR0B= 0x00;
			TCCR0A= 0x00;
			TCNT0 = 0;
		
			TCCR1B= 0x00;
			TCCR1A= 0x00;
			TCNT1 = 0;
		
			InactiveTime = 0;
			
			APA102_SetAll(0);
			AngleData[29].r = 255;
			AngleData[30].r = 255;
			AngleData[31].r = 255;
			apa102_setleds(AngleData, 64);			
			Delay_ms(1000);
			APA102_SetAll(0);
			apa102_setleds(AngleData, 64);

		}

		if (Display == 1)
		{
			Display = 0;
			DisplayAngle();			
		}

	 }
	  
	  
	  
	 
  }
	
	
    return 0;
}

