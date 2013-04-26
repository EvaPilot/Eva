//*----------------------------------------------------------------------------
//*         ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : init.c
//* Object              : Low level initialisations written in C
//* Creation            : HIi   10/10/2003
//*
//*----------------------------------------------------------------------------

#include "AT91RM9200.h"
#include "lib_AT91RM9200.h"
#define YJJ_AT91C_US_ASYNC_MODE ( AT91C_US_USMODE_NORMAL + \
                        AT91C_US_NBSTOP_1_BIT + \
                        AT91C_US_PAR_EVEN + \
                        AT91C_US_CHRL_8_BITS + \
                        AT91C_US_CLKS_CLOCK )

//*----------------------------------------------------------------------------
//* \fn    AT91F_DBGU_Printk
//* \brief This function is used to send a string through the DBGU channel (Very low level debugging)
//*----------------------------------------------------------------------------
void AT91F_DBGU_Printk(
	char *buffer) // \arg pointer to a string ending by \0
{
	while(*buffer != '\0') {
		while (!AT91F_US_TxReady((AT91PS_USART)AT91C_BASE_DBGU));
		AT91F_US_PutChar((AT91PS_USART)AT91C_BASE_DBGU, *buffer++);
	}
}

void AT91F_UART0_Printk(
	char *buffer) // \arg pointer to a string ending by \0
{
	while(*buffer != '\0') {
		while (!AT91F_US_TxReady((AT91PS_USART)AT91C_BASE_US0));
		AT91F_US_PutChar((AT91PS_USART)AT91C_BASE_US0, *buffer++);
	}
}

void AT91F_UART1_Printk(
	char *buffer) // \arg pointer to a string ending by \0
{
	while(*buffer != '\0') {
		while (!AT91F_US_TxReady((AT91PS_USART)AT91C_BASE_US1));
		AT91F_US_PutChar((AT91PS_USART)AT91C_BASE_US1, *buffer++);
	}
}

void AT91F_UART2_Printk(
	char *buffer) // \arg pointer to a string ending by \0
{
	while(*buffer != '\0') {
		while (!AT91F_US_TxReady((AT91PS_USART)AT91C_BASE_US2));
		AT91F_US_PutChar((AT91PS_USART)AT91C_BASE_US2, *buffer++);
	}
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_DataAbort
//* \brief This function reports an Abort
//*----------------------------------------------------------------------------
void AT91F_SpuriousHandler() 
{
	AT91F_DBGU_Printk("-F- Spurious Interrupt detected\n\r");
	while (1);
}


//*----------------------------------------------------------------------------
//* \fn    AT91F_DataAbort
//* \brief This function reports an Abort
//*----------------------------------------------------------------------------
void AT91F_DataAbort() 
{
	AT91F_DBGU_Printk("-F- Data Abort detected\n\r");
	while (1);
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_FetchAbort
//* \brief This function reports an Abort
//*----------------------------------------------------------------------------
void AT91F_FetchAbort()
{
	AT91F_DBGU_Printk("-F- Prefetch Abort detected\n\r");
	while (1);
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_Undef
//* \brief This function reports an Abort
//*----------------------------------------------------------------------------
void AT91F_Undef() 
{
	AT91F_DBGU_Printk("-F- Undef detected\n\r");
	while (1);
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_UndefHandler
//* \brief This function reports that no handler have been set for current IT
//*----------------------------------------------------------------------------
void AT91F_UndefHandler() 
{
	AT91F_DBGU_Printk("-F- Undef detected\n\r");
	while (1);
}


//*--------------------------------------------------------------------------------------
//* Function Name       : AT91F_InitSdram
//* Object              : Initialize the SDRAM
//* Input Parameters    :
//* Output Parameters   :
//*--------------------------------------------------------------------------------------
void AT91F_InitSdram()
{
	int *pRegister;
	
	//* Configure PIOC as peripheral (D16/D31)
	
	AT91F_PIO_CfgPeriph(
						 AT91C_BASE_PIOC, // PIO controller base address
						 0xFFFF0030,
						 0
						);
	
	//*Init SDRAM
	pRegister = (int *)0xFFFFFF98;
	*pRegister = 0x3bbbe3d4;//*0x2188c155; 
	pRegister = (int *)0xFFFFFF90;
	*pRegister = 0x12;	//*0x2; 
	pRegister = (int *)0x20000000;
	*pRegister = 0; 
	pRegister = (int *)0xFFFFFF90;
	*pRegister = 0x14;	//*0x4; 
	pRegister = (int *)0x20000000;
	*pRegister = 0; 
	*pRegister = 0; 
	*pRegister = 0; 
	*pRegister = 0; 
	*pRegister = 0; 
	*pRegister = 0; 
	*pRegister = 0; 
	*pRegister = 0; 
	pRegister = (int *)0xFFFFFF90;
	*pRegister = 0x13;	//*0x3; 
	pRegister = (int *)0x20000080;
	*pRegister = 0; 

	pRegister = (int *)0xFFFFFF94;
	*pRegister = 0x2e0; 
	pRegister = (int *)0x20000000;
	*pRegister = 0; 

	pRegister = (int *)0xFFFFFF90;
	*pRegister =0x10;	//* 0x00; 
	pRegister = (int *)0x20000000;
	*pRegister = 0; 

}

void AT91F_SetPLL(void)
{
	volatile int tmp = 0;//////////////processor clk = 179.712MHz  master clk = processor clk / 3 = 59.904
	
	/* APMC Initialization for Crystal */
	AT91PS_PMC pApmc = (AT91PS_PMC)AT91C_BASE_PMC;
	AT91PS_CKGR pCkgr =  (AT91PS_CKGR)AT91C_BASE_CKGR;
	
	pApmc->PMC_IDR = 0xFFFFFFFF;
	/* -Setup the PLL A */	
	pCkgr->CKGR_PLLAR = 0x2026BE04; 
	
	while(!(pApmc->PMC_SR & AT91C_PMC_LOCKA) && (tmp++ < 1000));

	/* Write in the MCKR dirty value concerning the clock selection CSS then overwrite it in a second sequence */
	pApmc->PMC_MCKR = 0x203;
	/* Wait until the master clock is established */
	tmp = 0;
	while(!(pApmc->PMC_SR & AT91C_PMC_MCKRDY) && (tmp++ < 1000));

	/* - Commuting Master Clock from PLLB to PLLA/3 */
	pApmc->PMC_MCKR = 0x202;
	/* Wait until the master clock is established */
	tmp = 0;
	while(!(pApmc->PMC_SR & AT91C_PMC_MCKRDY) && (tmp++ < 1000));

	/* Setup MEMC to support all connected memories (CS0 = FLASH; CS1=SDRAM) */
//	AT91C_BASE_EBI->EBI_CSA = AT91C_EBI_CS1A;

	/* com set CS0 cs for flash */
//	AT91C_BASE_SMC2->SMC2_CSR[0] = 0x00003284;
}



//*----------------------------------------------------------------------------
//* \fn    AT91F_InitFlash
//* \brief This function performs low level HW initialization
//*----------------------------------------------------------------------------
void AT91F_InitMemories()
{
	int *pEbi = (int *)0xFFFFFF60;

//* Setup MEMC to support all connected memories (CS0 = FLASH; CS1=SDRAM)
	pEbi  = (int *)0xFFFFFF60;
	*pEbi = 0x00000002;

//* CS0 cs for flash
	pEbi  = (int *)0xFFFFFF70;
	*pEbi = 0x00003284;
	
	AT91F_InitSdram();

}



//*----------------------------------------------------------------------------
//* \fn    AT91F_LowLevelInit
//* \brief This function performs very low level HW initialization
//*----------------------------------------------------------------------------
void AT91F_LowLevelInit(void)
{

	AT91F_SetPLL();
	
	
	// Init Interrupt Controller
	AT91F_AIC_Open(
		AT91C_BASE_AIC,          // pointer to the AIC registers
		AT91C_AIC_BRANCH_OPCODE, // IRQ exception vector
		AT91F_UndefHandler,      // FIQ exception vector
		AT91F_UndefHandler,      // AIC default handler
		AT91F_SpuriousHandler,   // AIC spurious handler
		0);                      // Protect mode

	// Perform 8 End Of Interrupt Command to make sýre AIC will not Lock out nIRQ 
	AT91F_AIC_AcknowledgeIt(AT91C_BASE_AIC);
	AT91F_AIC_AcknowledgeIt(AT91C_BASE_AIC);
	AT91F_AIC_AcknowledgeIt(AT91C_BASE_AIC);
	AT91F_AIC_AcknowledgeIt(AT91C_BASE_AIC);
	AT91F_AIC_AcknowledgeIt(AT91C_BASE_AIC);
	AT91F_AIC_AcknowledgeIt(AT91C_BASE_AIC);
	AT91F_AIC_AcknowledgeIt(AT91C_BASE_AIC);
	AT91F_AIC_AcknowledgeIt(AT91C_BASE_AIC);

	AT91F_AIC_SetExceptionVector((unsigned int *)0x0C, AT91F_FetchAbort);
	AT91F_AIC_SetExceptionVector((unsigned int *)0x10, AT91F_DataAbort);
	AT91F_AIC_SetExceptionVector((unsigned int *)0x4, AT91F_Undef);

	//Initialize SDRAM and Flash
	AT91F_InitMemories();
	
	// Open PIO for DBGU
	AT91F_DBGU_CfgPIO();

	// Configure DBGU 
	AT91F_US_Configure (
		(AT91PS_USART) AT91C_BASE_DBGU,          // DBGU base address
		59904000,             // 48 MHz
		AT91C_US_ASYNC_MODE,        // mode Register to be programmed
		115200 ,              // baudrate to be programmed
//		38400 ,               // baudrate to be programmed
		0);                   // timeguard to be programmed

	// Enable Transmitter
	AT91F_US_EnableTx((AT91PS_USART)AT91C_BASE_DBGU);
	// Enable Receiver
	AT91F_US_EnableRx((AT91PS_USART)AT91C_BASE_DBGU);
	
//	AT91F_DBGU_Printk("\n\r-I- AT91F_LowLevelInit(): Debug channel initialized\n\r");
	
	/////////////////////////////////////// configure UART0 ////////////
	AT91F_US0_CfgPMC();
	
	AT91F_PIO_CfgPeriph(
		AT91C_BASE_PIOA, // PIO controller base address
		((unsigned int) AT91C_PA17_TXD0    ) |
		((unsigned int) AT91C_PA21_RTS0    ) |
		((unsigned int) AT91C_PA18_RXD0    ) |
		((unsigned int) AT91C_PA20_CTS0    ), // Peripheral A
		0); // Peripheral B
		 
	AT91F_US_Configure (
		(AT91PS_USART) AT91C_BASE_US0,          // UART0 base address
		59904000,             // 48 MHz
		AT91C_US_ASYNC_MODE,        // mode Register to be programmed
		9600 ,              // baudrate to be programmed
		0);                   // timeguard to be programmed	
	// Enable Transmitter
	AT91F_US_EnableTx((AT91PS_USART)AT91C_BASE_US0);
	// Enable Receiver

	
//	AT91F_UART0_Printk("\n\r  UART0 Debug channel initialized\n\r");

	/////////////////////////////////////// configure UART1 ////////////
	AT91F_US1_CfgPMC();
	
	AT91F_PIO_CfgPeriph(
		AT91C_BASE_PIOB, // PIO controller base address
		((unsigned int) AT91C_PB20_TXD1    ) |
		((unsigned int) AT91C_PB26_RTS1    ) |
		((unsigned int) AT91C_PB21_RXD1    ) |
		((unsigned int) AT91C_PB24_CTS1    ), // Peripheral A
		0); // Peripheral B
		
	AT91F_US_Configure (
		(AT91PS_USART) AT91C_BASE_US1,          // UART1 base address
		59904000,             // 48 MHz
		AT91C_US_ASYNC_MODE,        // mode Register to be programmed
		9600 ,              // baudrate to be programmed
		0);                   // timeguard to be programmed	
	// Enable Transmitter
	AT91F_US_EnableTx((AT91PS_USART)AT91C_BASE_US1);
	// Enable Receiver
	AT91F_US_EnableRx((AT91PS_USART)AT91C_BASE_US1);	
	
//	AT91F_US_SetTimeguard (	(AT91PS_USART)AT91C_BASE_US1,10 ) ;
//	AT91F_UART1_Printk("\n\r  UART1 Debug channel initialized\n\r");	

	/////////////////////////////////////// configure UART2 ////////////
	AT91F_US2_CfgPMC();
	
	AT91F_PIO_CfgPeriph(
		AT91C_BASE_PIOA, // PIO controller base address
		((unsigned int) AT91C_PA23_TXD2    ) |
		((unsigned int) AT91C_PA31_RTS2    ) |
		((unsigned int) AT91C_PA22_RXD2    ) |
		((unsigned int) AT91C_PA30_CTS2    ) , // Peripheral A
		0); // Peripheral A
		
	AT91F_US_Configure (
		(AT91PS_USART) AT91C_BASE_US2,          // UART0 base address
		59904000,             // 48 MHz
		AT91C_US_ASYNC_MODE,        // mode Register to be programmed
		9600 ,              // baudrate to be programmed
		0);                   // timeguard to be programmed	
	// Enable Transmitter
	AT91F_US_EnableTx((AT91PS_USART)AT91C_BASE_US2);
	// Enable Receiver
	AT91F_US_EnableRx((AT91PS_USART)AT91C_BASE_US2);	
	
//	AT91F_US_SetTimeguard (	(AT91PS_USART)AT91C_BASE_US2,10 ) ;
//	AT91F_UART2_Printk("\n\r  UART1 Debug channel initialized\n\r");	
	
}

