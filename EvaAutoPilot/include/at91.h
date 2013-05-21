//
//  at91.h
//  EvaAutoPilot
//
//  Copyright (c) 2013  www.hexairbot.com. All rights reserved.
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License V2
//  as published by the Free Software Foundation.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.

#ifndef at91_h
#define at91_h

#define AT91C_MASTER_CLOCK              59904000
#define AT91C_BAUD_RATE                 115200

#define AT91C_DOWNLOAD_BASE_ADDRESS     0x20000000
#define AT91C_DOWNLOAD_MAX_SIZE         0x00020000

#define AT91C_OFFSET_VECT6              0x14        //* Offset for ARM vector 6

#define AT91C_UBOOT_ADDR			    0x20400000
#define AT91C_UBOOT_SIZE			    128*1024
#define AT91C_UBOOT_DATAFLASH_ADDR	    0xC0008000
#define AT91C_PLLA_VALUE			    0x2026BE04	// crystal= 18.432MHz

#define AT91C_VERSION   "VER 1.01"

extern void AT91F_DBGU_Printk(char *);
extern void AT91F_UART0_Printk(char *);
extern void AT91F_UART1_Printk(char *);
extern void AT91F_UART2_Printk(char *);

extern void AT91F_ST_ASM_HANDLER(void);
extern void AT91F_UART0_ASM_HANDLER(void);
extern void AT91F_UART1_ASM_HANDLER(void);
extern void AT91F_UART2_ASM_HANDLER(void);
extern void AT91F_IRQ0_ASM_HANDLER(void);

#endif