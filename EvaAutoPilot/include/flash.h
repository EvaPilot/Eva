/*
 * (C) Copyright 2000, 2001
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef _FLASH_H_
#define _FLASH_H_

#define ulong (unsigned long)
#define ushort (unsigned short)
#define uchar  (unsigned char)

#define CFG_MAX_FLASH_SECT 1
/*-----------------------------------------------------------------------
 * FLASH Info: contains chip specific data, per FLASH bank
 */

/*
 * Values for the width of the port
 */
#define FLASH_CFI_8BIT		0x01
#define FLASH_CFI_16BIT		0x02
#define FLASH_CFI_32BIT		0x04
#define FLASH_CFI_64BIT		0x08
/*
 * Values for the width of the chip
 */
#define FLASH_CFI_BY8		0x01
#define FLASH_CFI_BY16		0x02
#define FLASH_CFI_BY32		0x04
#define FLASH_CFI_BY64		0x08


/*-----------------------------------------------------------------------
 * return codes from flash_write():
 */
#define ERR_OK						0
#define ERR_TIMOUT					1
#define ERR_NOT_ERASED				2
#define ERR_PROTECTED				4
#define ERR_INVAL					8
#define ERR_ALIGN					16
#define ERR_UNKNOWN_FLASH_VENDOR	32
#define ERR_UNKNOWN_FLASH_TYPE		64
#define ERR_PROG_ERROR				128

/*-----------------------------------------------------------------------
 * Protection Flags for flash_protect():
 */
#define FLAG_PROTECT_SET	0x01
#define FLAG_PROTECT_CLEAR	0x02

/*-----------------------------------------------------------------------
 * Device IDs
 */
//#define INTEL_MANUFACT	0x00890089	/* INTEL   manuf. ID in D23..D16, D7..D0 */
#define INTEL_MANUFACT		0x89

#define INTEL_ID_28F320J3A  0x16	/*  32M = 128K x  32	*/
#define INTEL_ID_28F640J3A  0x17	/*  64M = 128K x  64	*/
#define INTEL_ID_28F128J3A  0x18	/* 128M = 128K x 128	*/

/*-----------------------------------------------------------------------
 * Timeout constants:
 *
 * We can't find any specifications for maximum chip erase times,
 * so these values are guestimates.
 */
#define FLASH_ERASE_TIMEOUT	10000	/* timeout for erasing in ms		*/
#define FLASH_WRITE_TIMEOUT	500	/* timeout for writes  in ms		*/


void flash_identification (void);
int flash_erase (int s_first, int s_last);
unsigned char blankcheck(unsigned long addr, unsigned long cnt);
//volatile static int write_byte(unsigned long dest, unsigned char udata);
int write_byte(unsigned long dest, unsigned char udata);
int	writebytes(unsigned char * pbuf, unsigned long blockaddr,unsigned long dest, unsigned long nlength);
extern int write_buff (unsigned char * src, unsigned long addr, unsigned long cnt);

#endif /* _FLASH_H_ */
