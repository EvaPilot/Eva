#ifndef UARTCOMM_h
#define UARTCOMM_h

#include <AT91RM9200.h> 
#include <lib_AT91RM9200.h> 


struct UART_DATA {
	unsigned char send_buf[4096];
	unsigned char send_tpr[2048];
	
	unsigned char rev_buf[4096];
	unsigned char rev_rpr[2048];
	

	int  send_buf_index ;
	int  send_buf_end	;
	int  send_buf_flag  ;
	
	int  rev_buf_index ;
	int  rev_buf_end	;
	int  rev_buf_count  ;
	
	
	AT91PS_PDC pdc;

	int baudrate;	
	int iuart;	
	
	int preRxRdy;
	int preTxRdy;
	int preRxRcr;
	int preTxTcr;
} ;

extern struct UART_DATA uart_dataDBGU;
extern struct UART_DATA uart_data1;
extern struct UART_DATA uart_data2;
extern int Uart5ms ;

extern void  uart_senddata(struct UART_DATA *uart_data,unsigned char* buf,int nlen) ;

#endif