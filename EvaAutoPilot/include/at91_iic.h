#ifndef AT91IIC_H
#define AT91IIC_H

#include "AT91RM9200.h"
#include "lib_AT91RM9200.h" 
  
#define AT91C_EEPROM_I2C_ADDRESS  	(0x21<<16) 
  
#define AT91C_MASTER_CLOCK			59904000
//#define AT91C_TWI_CLOCK 			400000

#define AT91C_EEPROM_READ_OK			0
#define AT91C_EEPROM_WRITE_OK			0

extern int AT91F_TWI_Write (const AT91PS_TWI, int, char *, int );
extern int AT91F_TWI_Read  (const AT91PS_TWI, int, char *, int );
extern void AT91F_SetTwiClock(const AT91PS_TWI pTwi);
extern void AT91_IIC_init(void);

#endif