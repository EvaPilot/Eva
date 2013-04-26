#include <flash.h>
#include <stdio.h>
#include <string.h>

#define CFG_MAX_FLASH_BANKS 1

extern void AT91F_DBGU_Printk(char *buffer);
extern unsigned int GetTickCount(void);
extern void	AT91F_TickCounClr(void);
char flashmsg[256];

#define FLASH_BANK_SIZE 0x400000	/* 4 MB */
#define MAIN_SECT_SIZE  0x20000		/* 128 KB */
#define CFG_FLASH_BASE	0x10000000

/*	INTEL 38F320J3A	codes */
#define CMD_READ_ARRAY		0xff
#define CMD_READ_STATUS		0x70
#define CMD_CLR_STATUS		0x50
#define CMD_WRITE_BUFFER	0xe8
#define CMD_WRITE_WORD		0x40
#define CMD_WRITE_BYTE		0x10
#define CMD_ERASE_BLOCK		0x20
#define CMD_ERASE_SUSPEND	0xb0
#define CMD_ERASE_CONFIRM	0xd0
#define CMD_CONFIGURATION	0xbb
#define CMD_SETCLR_BLOCK	0x60
#define CMD_SET_BLOCK		0xcc
#define	CMD_CLR_BLOCK		0xd0
#define CMD_PROTECT_PROG	0xc0

#define IDENT_CODE			0x90
#define IDENT_FLASH_ADDR	(*(volatile unsigned char *)(CFG_FLASH_BASE))
#define MEM_FLASH_ADDR		(*(volatile unsigned char *)(CFG_FLASH_BASE))

#define BIT_ERASE_DONE		0x0080
#define BIT_RDY_MASK		0x0080
#define BIT_PROGRAM_ERROR	0x0020
#define BIT_TIMEOUT			0x80000000 /* our flag */

#define READY 1
#define ERR   2
#define TMO   4

/*-----------------------------------------------------------------------
 */
void flash_identification (void)
{
	volatile unsigned char manuf_code, device_code;

	IDENT_FLASH_ADDR = IDENT_CODE;

	manuf_code = *(volatile unsigned char *) CFG_FLASH_BASE;
	device_code = *(volatile unsigned char *)(CFG_FLASH_BASE + 2);

	if(manuf_code == INTEL_MANUFACT)
	{		AT91F_DBGU_Printk("Intel: ");
	}
	else
	{
		AT91F_DBGU_Printk("NO intel manufactory.");	
	}

	switch(device_code)
	{
		case	INTEL_ID_28F320J3A:
				AT91F_DBGU_Printk("INTEL28F320J3A (32Mbit)\n\r");
				break;
		
		case	INTEL_ID_28F640J3A:
				AT91F_DBGU_Printk("INTEL28F640J3A (64Mbit)\n\r");
				break;
		
		default:
				AT91F_DBGU_Printk("Identification is not INTEL28F320.\n\r");
				break;		
	}
	
	IDENT_FLASH_ADDR = CMD_READ_ARRAY;
}

/*-----------------------------------------------------------------------
 */

int flash_erase (int s_first, int s_last)
{
	unsigned char result;
	unsigned int	tick;	
	volatile unsigned char *addr;
	
	int sect;
	int rc = ERR_OK;
	int chip1;

	if ((s_first < 0) || (s_first > s_last)) {
		AT91F_DBGU_Printk("\n\return code is ERR_INVAL.\n\r");
		return ERR_INVAL;
	}
	for (sect = s_first; sect <= s_last; sect++)
	{
//		sprintf (flashmsg,"Erasing sector %2d ... ", sect);
//		AT91F_DBGU_Printk(flashmsg);

//		AT91F_TickCounClr();
		tick = GetTickCount();
	
		addr = (volatile unsigned char *) (CFG_FLASH_BASE + 128 * 1024 * sect );		
		*addr = CMD_ERASE_BLOCK;
		*addr = CMD_ERASE_CONFIRM;

		/* wait until flash is ready */
		chip1 = 0;

		do {
			result = *addr;

			/* check timeout */
			if (GetTickCount () > (tick + FLASH_ERASE_TIMEOUT))
			{
				*addr = CMD_READ_ARRAY;
				chip1 = TMO;				
				break;
			}
			
			if(!chip1 && (result & 0x80))
				chip1 = READY;

		} while (!chip1);

		*addr = CMD_READ_ARRAY;

		if (chip1 == ERR) {
			rc = ERR_PROG_ERROR;
			AT91F_DBGU_Printk("\n\r return code is ERR_PROG_ERROR.\n\r");
			goto outahere;
		}
		if (chip1 == TMO) {
			rc = ERR_TIMOUT;
			AT91F_DBGU_Printk("\n\r return code is TMO.\n\r");
			goto outahere;
		}

//		AT91F_DBGU_Printk ("ok.\r");
	}

outahere:
	/* allow flash to settle - wait 10 ms */
	tick = 0;
	while(tick++ < 10000)	;
	
//	AT91F_DBGU_Printk ("\n\r");

	return rc;
}

unsigned char blankcheck(unsigned long addr, unsigned long cnt)
{
	unsigned char ret = 0;
	unsigned int i;
	
	unsigned char * addrchk = (unsigned char *)(addr);
	
	for(i = 0; i < cnt; i++)
	{
		if(*addrchk++ != 0xff)
			return ret;
	}
	
	ret = 1;
	return ret;	
}


/*-----------------------------------------------------------------------
 * Copy memory to flash
 */
 
int write_byte(unsigned long dest, unsigned char udata)
{
	unsigned int tick;
	volatile unsigned char *addr = (volatile unsigned char *)dest;
	unsigned char flstatus;
	int rc = ERR_OK;
	int chip1;
	
	unsigned char utemp = udata;
	
	////before write data, make sure the sect is blank.
	
//	AT91F_TickCounClr();
	tick = GetTickCount();	
	
	*addr = CMD_WRITE_BYTE;
	*addr = utemp;
	
	/* wait until flash is ready */
	chip1 = 0;
	do {
		flstatus = *addr;

		if(GetTickCount () > (tick + FLASH_ERASE_TIMEOUT))
		{
			*addr = CMD_READ_ARRAY;
			chip1 = TMO;
			break;
		}
		
		if(!chip1 && (flstatus & 0x80))
			chip1 = READY;

	} while (!chip1);
	
	*addr = CMD_READ_ARRAY;
	
//	if(*addr != utemp)
//		rc = ERR_PROG_ERROR;	
	
	if (chip1 == ERR)
		rc = ERR_PROG_ERROR;

	return rc;	
}

/*
volatile static int write_byte(unsigned long dest, unsigned char udata)
{
	unsigned int tick;
	volatile unsigned char *addr = (volatile unsigned char *)dest;
	unsigned char flstatus;
	int rc = ERR_OK;
	int chip1;
	
	unsigned char utemp = udata;
	
	////before write data, make sure the sect is blank.
	
//	AT91F_TickCounClr();
	tick = GetTickCount();	
	
	*addr = CMD_WRITE_BYTE;
	*addr = utemp;
	
	// wait until flash is ready 
	chip1 = 0;
	do {
		flstatus = *addr;

		if(GetTickCount () > (tick + FLASH_ERASE_TIMEOUT))
		{
			*addr = CMD_READ_ARRAY;
			chip1 = TMO;
			break;
		}
		
		if(!chip1 && (flstatus & 0x80))
			chip1 = READY;

	} while (!chip1);
	
	*addr = CMD_READ_ARRAY;
	
	if(*addr != utemp)
		rc = ERR_PROG_ERROR;	
	
	if (chip1 == ERR)
		rc = ERR_PROG_ERROR;

	return rc;	
}
*/
int	writebytes(unsigned char * pbuf, unsigned long blockaddr,
					unsigned long dest, unsigned long nlength)
{
	unsigned int tick;
	volatile unsigned char *addr;
	unsigned char flstatus;
	int rc = ERR_OK;
	int chip1;
	
	unsigned int total;
	unsigned int i;
		
	////before write data, make sure the sect is blank.
	
//	AT91F_TickCounClr();
	tick = GetTickCount();

	if(nlength > 32)	
		total = 32;
	else
		total = nlength;
	addr = (unsigned char *)blockaddr;	
	*addr = CMD_WRITE_BUFFER;
	*addr = total - 1;
	
	addr = (unsigned char *)dest;
	for(i= 0; i < total; i++)
		*addr++ = *pbuf++;
	
	addr = (unsigned char *)blockaddr;
	*addr = 	CMD_ERASE_CONFIRM;	
	
	
	/* wait until flash is ready */
	chip1 = 0;
	do {
		flstatus = *addr;

		if(GetTickCount () > (tick + FLASH_ERASE_TIMEOUT))
		{
			*addr = CMD_READ_ARRAY;
			chip1 = TMO;
			break;
		}
		
		if(!chip1 && (flstatus & 0x80))
			chip1 = READY;

	} while (!chip1);
	
	*addr = CMD_READ_ARRAY;
	
	if (chip1 == ERR)
		rc = ERR_PROG_ERROR;

	return rc;		
	
}

/*-----------------------------------------------------------------------
 * Copy memory to flash.
 */
int write_buff (unsigned char * src, unsigned long addr, unsigned long cnt)
{
	int rc;
	unsigned long i;
	
	unsigned long	blockaddr;
	unsigned long	destaddr;
	unsigned long	lpacket;
	
	if((addr < CFG_FLASH_BASE) || (addr >= (CFG_FLASH_BASE + 64 * 128 * 1024)))
		return ERR_INVAL;
	
	i = 0;
	blockaddr = CFG_FLASH_BASE;
	
	do
	{
		blockaddr += 128*1024;	
	}while(blockaddr <= addr);
	blockaddr = blockaddr - 128*1024;
	
	destaddr = addr;
	
	if(cnt == 1)
	{
		rc = write_byte (addr, *src);			//只有一个字节，直接写入完毕
		return rc;
	}
	
	lpacket = 0x20 - (addr & 0x1f); 
	if(lpacket < cnt)
		writebytes(src, blockaddr, destaddr, lpacket);
	else
	{
		writebytes(src, blockaddr, destaddr, cnt);	// 没有到边界，直接一次写入完毕
		return ERR_OK;
	}
	destaddr += lpacket;
	src += lpacket;
	lpacket = cnt - lpacket;

	
	while(lpacket > 32)
	{		
		do
		{
			blockaddr += 128*1024;	
		}while(blockaddr <= destaddr);
		blockaddr = blockaddr - 128*1024;
		
		writebytes(src, blockaddr, destaddr, 32);	//  循环写入32字节数据
		
		src += 32;
		destaddr += 32;
		lpacket -= 32;		
	}
	
	if(lpacket != 0)
	{
		do
		{
			blockaddr += 128*1024;	
		}while(blockaddr <= destaddr);
		blockaddr = blockaddr - 128*1024;
		
		writebytes(src, blockaddr, destaddr, lpacket);	
	}
	
	return ERR_OK;
}
