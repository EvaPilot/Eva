//*----------------------------------------------------------------------------
//*         ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : dataflash.c
//* Object              : High level functions for the dataflash
//* Creation            : HIi   10/10/2003
//*----------------------------------------------------------------------------

#include <dataflash.h>

extern int tgtnum;

AT91S_DATAFLASH_INFO dataflash_info[CFG_MAX_DATAFLASH_BANKS];
static AT91S_DataFlash DataFlashInst;

int cs[][CFG_MAX_DATAFLASH_BANKS] = {
	{CFG_DATAFLASH_LOGIC_ADDR_CS0, 0},	/* Logical adress, CS */
	{CFG_DATAFLASH_LOGIC_ADDR_CS3, 3}
};

int AT91F_DataflashInit (void)
{
	int i;
	int dfcode;
		
	AT91F_SpiInit ();

	for (i = 0; i < CFG_MAX_DATAFLASH_BANKS; i++) {
		dataflash_info[i].Desc.state = IDLE;
		dataflash_info[i].id = 0;
		dataflash_info[i].Device.pages_number = 0;
		dfcode = AT91F_DataflashProbe (cs[i][1], &dataflash_info[i].Desc);

		switch (dfcode) {

		case AT45DB041:
			dataflash_info[i].Device.pages_number = 2048;
			dataflash_info[i].Device.pages_size = 264;
			dataflash_info[i].Device.page_offset = 9;
			dataflash_info[i].Device.byte_mask = 0x100;
			dataflash_info[i].Device.cs = cs[i][1];
			dataflash_info[i].Desc.DataFlash_state = IDLE;
			dataflash_info[i].logical_address = cs[i][0];
			dataflash_info[i].id = dfcode;
		
		default:
			break;
		}
	}			
	return (1);
}


void AT91F_DataflashPrintInfo(void)
{
	
}


/*------------------------------------------------------------------------------*/
/* Function Name       : AT91F_DataflashSelect 					*/
/* Object              : Select the correct device				*/
/*------------------------------------------------------------------------------*/
AT91PS_DataFlash AT91F_DataflashSelect (AT91PS_DataFlash pFlash,
										unsigned int *addr)
{
	char addr_valid = 0;
	int i;

	for (i = 0; i < CFG_MAX_DATAFLASH_BANKS; i++)
		if ((*addr & 0xFF000000) == dataflash_info[i].logical_address) {
			addr_valid = 1;
			break;
		}
	if (!addr_valid) {
		pFlash = (AT91PS_DataFlash) 0;
		return pFlash;
	}
	pFlash->pDataFlashDesc = &(dataflash_info[i].Desc);
	pFlash->pDevice = &(dataflash_info[i].Device);
	*addr -= dataflash_info[i].logical_address;
	return (pFlash);
}


/*------------------------------------------------------------------------------*/
/* Function Name       : addr_dataflash 					*/
/* Object              : Test if address is valid				*/
/*------------------------------------------------------------------------------*/
int addr_dataflash (unsigned long addr)
{
	int addr_valid = 0;
	int i;

	for (i = 0; i < CFG_MAX_DATAFLASH_BANKS; i++) {
		if ((((int) addr) & 0xFF000000) ==
			dataflash_info[i].logical_address) {
			addr_valid = 1;
			break;
		}
	}

	return addr_valid;
}

/*------------------------------------------------------------------------------*/
/* Function Name       : read_dataflash 					*/
/* Object              : dataflash memory read					*/
/*------------------------------------------------------------------------------*/
int read_dataflash (unsigned long addr, unsigned long size, char *result)
{
	unsigned int AddrToRead = addr;
	AT91PS_DataFlash pFlash = &DataFlashInst;

	pFlash = AT91F_DataflashSelect (pFlash, &AddrToRead);
	if (pFlash == 0)
		return -1;

	return (AT91F_DataFlashRead (pFlash, AddrToRead, size, result));
}


/*-----------------------------------------------------------------------------*/
/* Function Name       : write_dataflash 				       */
/* Object              : write a block in dataflash			       */
/*-----------------------------------------------------------------------------*/
int write_dataflash (unsigned long addr_dest, unsigned int addr_src,
		     unsigned int size)
{
	unsigned int AddrToWrite = addr_dest;
	AT91PS_DataFlash pFlash = &DataFlashInst;

	pFlash = AT91F_DataflashSelect (pFlash, &AddrToWrite);
	if (AddrToWrite == -1)
		return -1;

	return AT91F_DataFlashWrite (pFlash, (unsigned char *) addr_src, AddrToWrite, size);
}

/*-----------------------------------------------------------------------------*/
/* Function Name       : erase_dataflash 				       */
/* Object              : erase a page in dataflash			       */
/*-----------------------------------------------------------------------------*/
void erase_dataflash(unsigned int pagenumber)
{
	int ipage,ii;
	unsigned int totalpages;
	unsigned int eraseaddr = CFG_DATAFLASH_LOGIC_ADDR_CS0;
	
	
	AT91PS_DataFlash pFlash = &DataFlashInst;	
	pFlash = AT91F_DataflashSelect (pFlash, &eraseaddr);
	AT91F_SpiEnable(pFlash->pDevice->cs);
	
	if(pagenumber == 0)
		{
			totalpages = 902;
			ii = 899;

//			totalpages = 601;
//			ii = 589;

//			totalpages = 508;
//			ii = 496;

//			totalpages = 694;
//			ii = 682;

//			totalpages = 446;
//			ii = 434;

		}
	else
	if (pagenumber==9997)
		{
			totalpages = 903 + tgtnum * (25/264);
			ii = 902;
			if (totalpages > 928) totalpages = 928;
			if (totalpages < 902) totalpages = 902;
		}
	else
	if (pagenumber==9998)
		{
			totalpages = 1940;
			ii = 1163;
		}
	else
	if (pagenumber==9999)
		{
			totalpages = 1278;
			ii = 1241;
		}
	else	
	if (pagenumber==1)
		{
			totalpages = 100;
			ii = 0;
		}
	else
		{	
			totalpages = pagenumber;
			ii = 744;
		}
	
	for(ipage = ii; ipage < totalpages; ipage++)
	{

		AT91F_PageErase(pFlash, (unsigned int)ipage);
		AT91F_DataFlashWaitReady(pFlash->pDataFlashDesc, AT91C_DATAFLASH_TIMEOUT);
	}
}
