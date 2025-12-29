#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include "SST25VFxxxx.h"

#define setLow(port, pin) ((port) &= ~(1 << (pin)))



uint8_t sst25vf_GetByte(void)
{
	return spiSendByte(0xFF);
}


void SST25VF_init(void)
{			
	DDRB |= ( 1 << SST25VF_PIN_MOSI ) | ( 1 << SST25VF_PIN_SCK ) | ( 1 << SST25VF_PIN_CE );
	// make sure the MISO pin is input
	DDRB &= ~( 1 << SST25VF_PIN_MISO );
	
	// set up the SPI module: SPI enabled, MSB first, master mode,
	//  clock polarity and phase = 0, F_osc/8
	SPSR = ( 1 <<SPI2X);     // set double SPI speed for F_osc/8
	SPCR = ( 1 << SPE ) | ( 1 << MSTR ) ;
	
	OUTPUT_HIGH(SST25VF_PORT, SST25VF_PIN_CE);
	sst25vf_WRSR(0x00);
	sst25vf_DBSY();       
}



uint8_t sst25vf_RDSR(void)
{
	uint8_t result;
	sst25vf_Select();
	spiSendByte(SST25VF_RDSR);
	result = spiGetByte();
	sst25vf_DeSelect();
	return(result);
}

void sst25vf_WRSR(uint8_t status)
{
	sst25vf_EWSR();
	sst25vf_Select();
	spiSendByte(SST25VF_WRSR);
	spiSendByte(status);
	sst25vf_DeSelect();
}

 void sst25vf_Select(void)
{
	SST25VF_PORT &= ~(1<<SST25VF_PIN_CE); // 0
}

 void sst25vf_DeSelect(void)
{
	SST25VF_PORT |= (1<<SST25VF_PIN_CE);// 1
}



void sst25vf_Command(uint8_t data){
	sst25vf_Select();
	spiSendByte(data);
	sst25vf_DeSelect();
}


uint8_t spiSendByte(uint8_t data)
{
	SPDR = data;
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}


static void SendAddr(uint32_t addr)
{
	spiSendByte((addr & 0xFF0000) >> 16);
	spiSendByte((addr & 0xFF00) >> 8);
	spiSendByte(addr & 0xFF);
}

void sst25vf_Busy(void)
{

	while ((sst25vf_RDSR() & 0x03) == 0x03)	/* waste time until not busy */
	sst25vf_RDSR();
	
}


void sst25vf_WriteByte(uint32_t startAddr, const uint8_t data)
{
	sst25vf_WREN();
	sst25vf_Select();
		
	spiSendByte(SST25VF_WRBYTE);
	SendAddr(startAddr);
	spiSendByte(data);
	sst25vf_DeSelect();
	sst25vf_Busy();
}


void sst25vf_Read(uint32_t startAddr, uint8_t *data, uint16_t nBytes){
	uint16_t i;
	sst25vf_Select();
	spiSendByte(SST25VF_RD);
	SendAddr(startAddr);
	for(i=0;i<nBytes;i++)
	{
		data[i] = spiGetByte();
	}
	sst25vf_DeSelect();
}





void sst25vf_Read_Reverse(uint32_t ReadAddress, uint8_t *data, uint16_t WriteAddress, uint16_t nBytes){
	uint16_t i;
	sst25vf_Select();
	spiSendByte(SST25VF_RD);
	SendAddr(ReadAddress);
	for(i=0;i<nBytes;i++)
	{
		data[WriteAddress - 1 - i] = spiGetByte();
	}
	sst25vf_DeSelect();
}



unsigned char sst25vf_ReadByte(unsigned long Dst)
{
	unsigned char byte = 0;
	sst25vf_Select();			
	spiSendByte(0x03); 		
	spiSendByte(((Dst & 0xFFFFFF) >> 16));	
	spiSendByte(((Dst & 0xFFFF) >> 8));
	spiSendByte(Dst & 0xFF);
	byte = spiGetByte();
	sst25vf_DeSelect();			
	return byte;			
}

unsigned char sst25vf_ReadByte_Speed(unsigned long Dst)
{
	unsigned char byte = 0;
	sst25vf_Select();
	spiSendByte(0x0B);
	spiSendByte(((Dst & 0xFFFFFF) >> 16));
	spiSendByte(((Dst & 0xFFFF) >> 8));
	spiSendByte(Dst & 0xFF);
	byte = spiGetByte();
	sst25vf_DeSelect();
	return byte;

}



void sst25vf_ChipErase(void)
{sst25vf_WREN();
	sst25vf_Select();
	spiSendByte(SST25VF_CHIPERASE);
	sst25vf_DeSelect();
	sst25vf_Busy();
}


void Block_Erase_4K(unsigned long Dst)
{sst25vf_WREN();
	sst25vf_Select();				/* enable device */
	spiSendByte(SST25VF_ERASE4k);			/* send 64KByte Block Erase command */
	spiSendByte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	spiSendByte(((Dst & 0xFFFF) >> 8));
	spiSendByte(Dst & 0xFF);
	sst25vf_DeSelect();				/* disable device */
	sst25vf_Busy();
}
void Block_Erase_32K(unsigned long Dst)
{sst25vf_WREN();
	sst25vf_Select();				/* enable device */
	spiSendByte(SST25VF_ERASE32k);			/* send 64KByte Block Erase command */
	spiSendByte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	spiSendByte(((Dst & 0xFFFF) >> 8));
	spiSendByte(Dst & 0xFF);
	sst25vf_DeSelect();				/* disable device */
	sst25vf_Busy();
}
void Block_Erase_64K(unsigned long Dst)
{sst25vf_WREN();
	sst25vf_Select();				/* enable device */
	spiSendByte(SST25VF_ERASE64k);			/* send 64KByte Block Erase command */
	spiSendByte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	spiSendByte(((Dst & 0xFFFF) >> 8));
	spiSendByte(Dst & 0xFF);
	sst25vf_DeSelect();				/* disable device */
	sst25vf_Busy();
}



void Read_Cont(unsigned long Dst, unsigned long no_bytes)
{
	unsigned long i = 0;
	sst25vf_Select();				/* enable device */
	spiSendByte(0x03); 			/* read command */
	spiSendByte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	spiSendByte(((Dst & 0xFFFF) >> 8));
	spiSendByte(Dst & 0xFF);
	for (i = 0; i < no_bytes; i++)		/* read until no_bytes is reached */
	{
		upper_128[i] = spiGetByte();	/* receive byte and store at address 80H - FFH */
	}
	sst25vf_DeSelect();				/* disable device */

}

void sst25vf_AAIStart(uint32_t startAddr, const uint8_t D0, const uint8_t D1)
{
	sst25vf_WREN();
	sst25vf_Select();	
	spiSendByte(SST25VF_WRAAI);
	SendAddr(startAddr);
	spiSendByte(D0);
	spiSendByte(D1);
	sst25vf_DeSelect();
}


void sst25vf_AAICont(const uint8_t D0, const uint8_t D1)
{
	sst25vf_Select();
	spiSendByte(SST25VF_WRAAI);
	spiSendByte(D0);
	spiSendByte(D1);
	sst25vf_DeSelect();
}



void sst25vf_Write(uint32_t startAddr, const uint8_t *data, uint16_t nBytes)
{
	uint16_t i;
	
	// If odd start address
	if(startAddr & 0x01){
		// write 1 byte
		sst25vf_WriteByte(startAddr, data[0]);
		startAddr++;
		nBytes--;
		data++;
	}
	
	if(nBytes == 0){
		return; // only one byte. End
	}
	
	i = 0;
	// Write pairs of bytes aligned to even addresses (AAI)
	if(nBytes >= 2){
		sst25vf_AAIStart(startAddr,data[i],data[i+1]);
		i +=2;
		nBytes -= 2;
		sst25vf_Busy();
		
		while(nBytes >= 2){
			sst25vf_AAICont(data[i], data[i+1]);
			i +=2;
			nBytes -= 2;
			sst25vf_Busy();
		}
		sst25vf_WRDI();
		sst25vf_Busy();
	}
	
	// if one byte remaining
	if(nBytes){
		// write 1 byte
		startAddr += i;
		sst25vf_WriteByte(startAddr, data[i]);
	}
}