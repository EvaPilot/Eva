//*----------------------------------------------------------------------------
//*      ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : twi.h
//* Object              :
//*----------------------------------------------------------------------------
#ifndef twi_h
#define twi_h

#define AT91C_EEPROM_I2C_ADDRESS  	(0x50<<16)

#define AT91C_EEPROM_READ_OK			0
#define AT91C_EEPROM_WRITE_OK			0

extern int AT91F_TWI_Write (const AT91PS_TWI, int, char *, int );
extern int AT91F_TWI_Read  (const AT91PS_TWI, int, char *, int );

#endif