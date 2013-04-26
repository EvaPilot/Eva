//*----------------------------------------------------------------------------
//*      ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : main.h
//* Object              :
//*
//* 1.0 27/03/03 HIi    : Creation
//* 1.01 03/05/04 HIi   : AT9C_VERSION incremented to 1.01
//*----------------------------------------------------------------------------

#ifndef main_h
#define main_h

#include    "embedded_services.h"

#define AT91C_MASTER_CLOCK              59904000
#define AT91C_BAUD_RATE                 115200

#define AT91C_DOWNLOAD_BASE_ADDRESS     0x20000000
#define AT91C_DOWNLOAD_MAX_SIZE         0x00020000

#define AT91C_OFFSET_VECT6              0x14        //* Offset for ARM vector 6

#define AT91C_UBOOT_ADDR			0x20400000
#define AT91C_UBOOT_SIZE			128*1024
#define AT91C_UBOOT_DATAFLASH_ADDR		0xC0008000
#define AT91C_PLLA_VALUE			0x2026BE04	// crystal= 18.432MHz
#define DELAY_MAIN_FREQ				1000
//#define AUTO_HAND					1
//#define TRKPOSNUM					8
#define PI							3.1415926535897932
#define OUTLENGTH 99		//输出每包字节数
#define DISP_LINE_LEN	16 

//*interface in define
#define LAMP *pmem_LED

#define OUT_FXDUO *pmem_FANGXIANGl
#define OUT_FYDUO *pmem_FUYIl
#define OUT_SJDUO *pmem_SHENGJIANGl
#define OUT_YMDUO *pmem_YOUMENl
#define OUT_NEW *pmem_NEW
#define OUT_HW7 *pmem_HW7
#define OUT_SJNEW *pmem_SJNEW
#define OUT_FYNEW *pmem_FYNEW


#define AT91C_VERSION   "VER 1.01"
// Global variables and functions definition

//******************************************
//in buffer define
//******************************************
//out buffer define

#define OUT_NIAN SndBuf[25+18]
#define OUT_YUE SndBuf[26+18]
#define OUT_RI SndBuf[27+18]
#define OUT_SHI SndBuf[28+18]
#define OUT_FEN SndBuf[29+18]
#define OUT_MIAO SndBuf[30+18]
#define OUT_INA_LATTI SndBuf[4+18]
#define OUT_INA_LONGI SndBuf[8+18]
#define OUT_INA_GPSHIGH SndBuf[12+18]
#define OUT_INA_VELS SndBuf[16+18]
#define OUT_INA_HANGXIANG SndBuf[20+18]
#define OUT_INA_SATELNUM SndBuf[24+18]

#define OUT_HAND_AUTOHANDLE SndBuf[31+18]
#define OUT_HAND_FXDUO SndBuf[32+18]
#define OUT_HAND_FYDUO SndBuf[33+18]
#define OUT_HAND_SJDUO SndBuf[34+18]
#define OUT_HAND_YMDUO SndBuf[35+18]
#define OUT_ATL_FXDUO SndBuf[36+18]
#define OUT_ATL_FYDUO SndBuf[37+18]
#define OUT_ATL_SJDUO SndBuf[38+18]
#define OUT_ATL_YMDUO SndBuf[39+18]

#define OUT_HIGH_TEMP SndBuf[40+18]
#define OUT_HIGH_P SndBuf[42+18]
#define OUT_HIGH_GROUNDP SndBuf[44+18]
#define OUT_HIGH_AIR SndBuf[48+18]

#define OUT_ADX_REF25 SndBuf[50+18]

#define	KONGSU_L		SndBuf[50+18]

#define OUT_ADX_FUYANGPING SndBuf[54+18]
#define OUT_ADX_HENGGUNPING SndBuf[58+18]
#define OUT_ADX_FUYANG SndBuf[62+18]
#define OUT_ADX_HENGGUN SndBuf[66+18]
#define OUT_ADX_HANGXIANG SndBuf[70+18]

#define OUT_ARM_TARGET SndBuf[74+18]
#define OUT_ARM_AUTOHANDLE SndBuf[75+18]
#define OUT_ARM_FXDUO SndBuf[76+18]
#define OUT_ARM_FYDUO SndBuf[77+18]
#define OUT_ARM_SJDUO SndBuf[78+18]
#define OUT_ARM_YMDUO SndBuf[79+18]
#define OUT_ARM_FXZHONG SndBuf[80+18]
#define OUT_ARM_FYZHONG SndBuf[81+18]
#define OUT_ARM_SJZHONG SndBuf[82+18]
#define OUT_ARM_YMZHONG SndBuf[83+18]
#define OUT_ARM_REFHIGH SndBuf[84+18]
#define OUT_ARM_CUR_TAR_ANGLE SndBuf[86+18]
#define OUT_ARM_CUR_SHOULD_ANGLE SndBuf[90+18]
#define OUT_ARM_DELTA_ANGLE SndBuf[94+18]

#define OUT_VOLTAGE SndBuf[73+18]
#define OUT_VOLTAGE2 SndBuf[86+18]
#define OUT_VOLTAGE3 SndBuf[87+18]


extern unsigned int GetTickCount(void);
#endif