/*
 * light_apa102_config.h
 *
 * Created: 30.11.2014
 *
 * User Configuration file for the light_apa102_lib
 *
 */ 


#ifndef APA102_CONFIG_H_
#define APA102_CONFIG_H_

///////////////////////////////////////////////////////////////////////
// Define I/O pins
// Clk and Data have to be connected to the same port
///////////////////////////////////////////////////////////////////////

#define apa102_port PORTD    // Data and clk port 
#define apa102_ddr DDRD
#define apa102_data PD1    // Data out pin
#define apa102_clk  PD4    // Clk out pin




///////////////////////////////////////////////////////////////////////
// Define color byte order
///////////////////////////////////////////////////////////////////////

struct cRGB { uint8_t b; uint8_t g; uint8_t r; };   // BGR (APA102 Standard)
//struct cRGB { uint8_t g; uint8_t b; uint8_t r; };   // GBR 



#endif /* APA102_CONFIG_H_ */