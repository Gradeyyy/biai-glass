/**
 * 		lang
 *
 *      2025-02-19
 */
#include "stdint.h"
#include "math.h"
#include "main.h"

#ifndef OPT3001_H_
#define OPT3001_H_

#define OPT3001_ADDR1 	0x44        //GND (Default)
#define OPT3001_ADDR2	0x45        //VDD
#define OPT3001_ADDR3	0x46        //SDA
#define OPT3001_ADDR4	0x47		//SCL

#define OPT3001_ConfigReg	0x01
#define OPT3001_ResultReg	0x00
#define OPT3001_SetLow	    0x02
#define OPT3001_SetHigh     0x03
#define OPT3001_ManuID      0x7E
#define OPT3001_DeviceID    0x7F



void opt3001_writedata(uint8_t register_addr, uint16_t data ); //This function is used to write data to different registers.

uint16_t opt3001_readdata(uint8_t register_addr);              //This function is used to read raw data from different registers.

uint8_t opt3001_init(void);              //Initialize the sensor

float calculate_lux(void);               //get the ambient light level value from the sensor(in Lux)

uint16_t calculate_lux_int(void);

uint16_t read_devid(void);              //Read the device id

uint16_t read_manufacturer_id(void);    //Read the manufacturer id

void opt3001_shutdown(void);

uint16_t calculate_lux_int(void);



#endif /* OPT3001_H_ */
