#include "at91_iic.h"  

int h125 = 125;
extern  int		count_twi;
extern  char 	twi_error;                  
//*----------------------------------------------------------------------------
//* \fn    AT91F_SetTwiClock
//* \brief Initialization
//*----------------------------------------------------------------------------
void AT91F_SetTwiClock(const AT91PS_TWI pTwi)
{        
	int sclock;  

	/* Here, CKDIV = 1 and CHDIV=CLDIV  ==> CLDIV = CHDIV = 1/4*((Fmclk/FTWI) -6)*/

	sclock = (10*AT91C_MASTER_CLOCK /400000);
	if (sclock % 10 >= 5)
		sclock = (sclock /10) - 5;
	else
		sclock = (sclock /10)- 6;
	sclock = (sclock + (4 - sclock %4)) >> 2;	// div 4

    pTwi->TWI_CWGR	= 0x00010000 | sclock | (sclock << 8);
}  
  
//*=========================================================
//*		WRITE
//*=========================================================
//*----------------------------------------------------------------------------
//* \fn    AT91F_TWI_Write
//* \brief Send n bytes to a slave device
//*----------------------------------------------------------------------------
int AT91F_TWI_Write(const AT91PS_TWI pTwi ,int address, char *data2send, int size)
{
	unsigned int status;
	float www;

	// Set the TWI Master Mode Register
	
	
	pTwi->TWI_MMR = ( AT91C_EEPROM_I2C_ADDRESS | AT91C_TWI_IADRSZ_NO ) & ~AT91C_TWI_MREAD;	
//	pTwi->TWI_MMR = ( AT91C_EEPROM_I2C_ADDRESS | AT91C_TWI_IADRSZ_1_BYTE ) & ~AT91C_TWI_MREAD;	
	
	// Set TWI Internal Address Register
	pTwi->TWI_IADR = address;

	status = pTwi->TWI_SR;
		
	pTwi->TWI_THR = *(data2send++);
	
	pTwi->TWI_CR = AT91C_TWI_START;
		
//	while (size-- >1){
	{
		// Wait THR Holding register to be empty
		count_twi = 0;
		while (!(pTwi->TWI_SR & AT91C_TWI_TXRDY))
		{
			www = (float)count_twi;
			if (www > h125)
			{
				twi_error = 1;
				count_twi = 0;
//				goto break1;
				goto break2;
			}
		}
	
		// Send first byte
		pTwi->TWI_THR = *(data2send++);
		
	}

	break1:;
	pTwi->TWI_CR = AT91C_TWI_STOP;		

	status = pTwi->TWI_SR;

	// Wait transfer is finished
		count_twi = 0;
    while (!(pTwi->TWI_SR & AT91C_TWI_TXCOMP))
		{
			www = (float)count_twi;
			if (www > h125)
			{
				twi_error = 1;
				count_twi = 0;
				goto break2;
			}
		}

	break2:;		
	return AT91C_EEPROM_WRITE_OK;
}

//*=========================================================
//*		READ
//*=========================================================
//*----------------------------------------------------------------------------
//* \fn    AT91F_TWI_Read
//* \brief Read n bytes from a slave device
//*----------------------------------------------------------------------------
int AT91F_TWI_Read(const AT91PS_TWI pTwi , int address, char *data, int size)
{
	unsigned int status;
	char tmd;
	float www;
	char i;
	
	// Set the TWI Master Mode Register
	pTwi->TWI_MMR = AT91C_EEPROM_I2C_ADDRESS | AT91C_TWI_IADRSZ_NO | AT91C_TWI_MREAD;	
	
	// Set TWI Internal Address Register
	pTwi->TWI_IADR = address;
	
	// Start transfer
	pTwi->TWI_CR = AT91C_TWI_START;
	
	status = pTwi->TWI_SR;
		
//	while (size-- >1)
	for (i=0;i<2;i++)
	{
		
		// Wait RHR Holding register is full

		count_twi = 0;

		while (!(pTwi->TWI_SR & AT91C_TWI_RXRDY))
		{
			www = (float)count_twi;
			if (www > h125)
			{
				twi_error = 1;
				count_twi = 0;
//				goto break3;
				goto break4;
			}
						
		}

		// Read byte
		*(data++) = pTwi->TWI_RHR;
	
	}

	break3:;
	pTwi->TWI_CR = AT91C_TWI_STOP;

	status = pTwi->TWI_SR;

	// Wait transfer is finished
		count_twi = 0;
    while (!(pTwi->TWI_SR & AT91C_TWI_TXCOMP))
		{
			www = (float)count_twi;
			if (www > h125)
			{
				twi_error = 1;
				count_twi = 0;
				goto break4;
			}
		}
    
	break4:;
	// Read last byte
	*data = pTwi->TWI_RHR;
		
	return AT91C_EEPROM_READ_OK;
}


void	AT91_IIC_init(void)
{
	AT91F_TWI_CfgPIO ();
	AT91F_PIO_CfgOpendrain(AT91C_BASE_PIOA, (unsigned int) AT91C_PA25_TWD);

	// Configure PMC by enabling TWI clock
	AT91F_TWI_CfgPMC ();
	
	// Configure TWI in master mode
	AT91F_TWI_Configure (AT91C_BASE_TWI);
		
	// Set TWI Clock Waveform Generator Register	
	AT91F_SetTwiClock(AT91C_BASE_TWI);
}