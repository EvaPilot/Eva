//
//  at91_iic.h
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

#ifndef AT91IIC_H
#define AT91IIC_H

#include "AT91RM9200.h"
#include "lib_AT91RM9200.h"
  
#define AT91C_EEPROM_I2C_ADDRESS  	(0x21<<16) 
#define AT91C_MASTER_CLOCK			59904000
#define AT91C_EEPROM_READ_OK		0
#define AT91C_EEPROM_WRITE_OK		0

int AT91F_TWI_Write (const AT91PS_TWI, int, char *, int );
int AT91F_TWI_Read  (const AT91PS_TWI, int, char *, int );
void AT91F_SetTwiClock(const AT91PS_TWI pTwi);
void AT91_IIC_init(void);

#endif