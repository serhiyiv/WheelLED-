

#define SET_INPUT(portdir,pin)  portdir &= ~(1<<pin)
#define SET_OUTPUT(portdir,pin) portdir |= (1<<pin)
#define OUTPUT_LOW(port,pin)    port &= ~(1<<pin)
#define OUTPUT_HIGH(port,pin)   port |= (1<<pin)
// PINS
#define  SST25VF_PORT PORTB
#define  SST25VF_DDR DDRB
#define  SST25VF_SPCR SPCR
#define  SST25VF_SPDR SPDR
#define  SST25VF_SPSR SPSR


/*ATMEGA 328p
PB5 SCK (SPI Bus Master clock Input)
PB4 MISO (SPI Bus Master Input/Slave Output)
PB3 MOSI (SPI Bus Master Output/Slave Input)
PB2 SS(SPI Bus Master Slave select)
*/

#define  SST25VF_PIN_CE        PINB2
#define  SST25VF_PIN_MOSI      PINB3
#define  SST25VF_PIN_SCK       PINB5
#define  SST25VF_PIN_MISO      PINB4

unsigned char  upper_128[128];



// COMMANDS
#define SST25VF_BUSY       0x01
#define SST25VF_WEL        0x02
#define SST25VF_BP0        0x04
#define SST25VF_BP1        0x08
#define SST25VF_BP2        0x10
#define SST25VF_BP3        0x20
#define SST25VF_AAI        0x40
#define SST25VF_BPL        0x80
#define SST25VF_RD          0x03
#define SST25VF_ERASE4k     0x20
#define SST25VF_ERASE32k    0x52
#define SST25VF_ERASE64k    0xD8
#define SST25VF_CHIPERASE   0x60
#define SST25VF_WRBYTE      0x02
#define SST25VF_WRAAI       0xAD
#define SST25VF_RDSR        0x05
#define SST25VF_EWSR        0x50
#define SST25VF_WRSR        0x01
#define SST25VF_WREN        0x06
#define SST25VF_WRDI        0x04
#define SST25VF_RDID        0x90
#define SST25VF_JEDECID     0x9F
#define SST25VF_EBSY        0x70
#define SST25VF_DBSY        0x80


// PROTOTYPES

uint8_t spiSendByte(uint8_t data);
#define spiGetByte()    spiSendByte(0xFF);
uint8_t sst25vf_GetByte(void);

void SST25VF_init(void);
void sst25vf_Select(void);
void sst25vf_DeSelect(void); 
void sst25vf_WRSR(uint8_t status); 
void sst25vf_Command(uint8_t data);
#define sst25vf_WREN()            sst25vf_Command(SST25VF_WREN)
#define sst25vf_WRDI()            sst25vf_Command(SST25VF_WRDI)
#define sst25vf_EWSR()            sst25vf_Command(SST25VF_EWSR)
#define sst25vf_EBSY()            sst25vf_Command(SST25VF_EBSY)
#define sst25vf_DBSY()            sst25vf_Command(SST25VF_DBSY)

void sst25vf_WriteByte(uint32_t startAddr, const uint8_t data);
unsigned char sst25vf_ReadByte(unsigned long Dst);
unsigned char sst25vf_ReadByte_Speed(unsigned long Dst);
void sst25vf_ChipErase(void);
void sst25vf_Busy(void);
void sst25vf_Read(uint32_t startAddr, uint8_t *data, uint16_t nBytes);
void sst25vf_Read_Reverse(uint32_t ReadAddress, uint8_t *data, uint16_t WriteAddress, uint16_t nBytes);
uint8_t sst25vf_RDSR(void);
void Block_Erase_4K(unsigned long Dst);
void Block_Erase_32K(unsigned long Dst);
void Block_Erase_64K(unsigned long Dst);

void sst25vf_AAIStart(uint32_t startAddr, const uint8_t D0, const uint8_t D1);
void sst25vf_AAICont(const uint8_t D0, const uint8_t D1);
void sst25vf_Write(uint32_t startAddr, const uint8_t *data, uint16_t nBytes);