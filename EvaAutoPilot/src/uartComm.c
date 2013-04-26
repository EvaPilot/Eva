
#include "uartComm.h"


struct UART_DATA uart_data1;
struct UART_DATA uart_data2;
struct UART_DATA uart_dataDBGU;
int Uart5ms = 0;


void uart_init(struct UART_DATA *uart_data)
{
	
	uart_data->send_buf_index = 0;
	uart_data->send_buf_end	 = 0;
	uart_data->send_buf_flag  = 0 ;

	uart_data->rev_buf_index = 0;
	uart_data->rev_buf_end	 = 0;
	uart_data->rev_buf_count  = 1024 ;
	
	
	uart_data->preRxRdy  = 0;
	uart_data->preTxRdy  = 0;
	
	uart_data->preRxRcr  = 0;
	uart_data->preTxTcr  = 0;
	
	AT91F_PDC_DisableRx(uart_data->pdc);
	AT91F_PDC_DisableTx(uart_data->pdc);
	AT91F_PDC_SetRx(uart_data->pdc, (char*)uart_data->rev_rpr, uart_data->rev_buf_count);	
	//AT91F_PDC_SetNextRx(uart_data->pdc, (char*)uart_data->rev_rpr, uart_data->rev_buf_count);
	AT91F_PDC_EnableRx(uart_data->pdc);
	AT91F_PDC_EnableTx(uart_data->pdc);
	
	
}

void uart_dosend(struct UART_DATA *uart_data)
{
	int nlen =0;
	int i = 0;
	int tcr;
	if (uart_data->send_buf_flag!=0)//intrrupts protection
		return;
	
			
	nlen = 	uart_data->send_buf_end - uart_data->send_buf_index;
	if (nlen<1) 
		return;	
			
	tcr =  uart_data->pdc-> PDC_TCR; 
	if (tcr>0) 
		return;		

	
	if (nlen>1024)
		nlen =1024;

	for (i=0;i<nlen;i++)
	{
		uart_data->send_tpr[i] = uart_data->send_buf[uart_data->send_buf_index];
		uart_data->send_buf_index++;
	}
	AT91F_PDC_SetTx(uart_data->pdc,(char *)(uart_data->send_tpr),nlen);
		
	
	//AT91F_DBGU_InterruptEnable(AT91C_BASE_DBGU, 0x18);
	if (uart_data->send_buf_index >= uart_data->send_buf_end)
	{
		uart_data->send_buf_index = 0;	
		uart_data->send_buf_end = 0;	
		//AT91F_DBGU_InterruptEnable(AT91C_BASE_DBGU, 0x08);
	}
	else
	{
		if(uart_data->send_buf_end >1024)
		{
			
			nlen = uart_data->send_buf_end - uart_data->send_buf_index;
			for (i=0;i<nlen;i++)
			{
				uart_data->send_buf[i] = uart_data->send_buf[uart_data->send_buf_index];
				uart_data->send_buf_index++;
			}
			uart_data->send_buf_index = 0;	
			uart_data->send_buf_end = nlen;	
		}
	}
	
	
}

void  uart_senddata(struct UART_DATA *uart_data,unsigned char* buf,int nlen) 
{

	
	
	int i = 0;
	uart_data->send_buf_flag = 1;
	
	
	if ((uart_data->send_buf_end+nlen)>2048)
	{
		uart_data->send_buf_end = 0;
		uart_data->send_buf_index = 0;
	}

	for (i=0;i<nlen;i++)
	{
		uart_data->send_buf[uart_data->send_buf_end] = buf[i];
		uart_data->send_buf_end++;
	}
	
	uart_data->send_buf_flag = 0;
	
	uart_dosend(uart_data);

}
int uart_doread(struct UART_DATA *uart_data)
{
	int rcr =0;
	int nlen = 0;
	int i;

	
	rcr = uart_data->pdc-> PDC_RCR;
	if (rcr < uart_data->rev_buf_count)
	{
		nlen = uart_data->rev_buf_count - rcr; 
		
			
		if ((uart_data->rev_buf_end+nlen)>2048)
		{
			uart_data->rev_buf_end = 0;
			uart_data->rev_buf_index = 0;
		}
		for (i=0;i<nlen;i++)
		{
			uart_data->rev_buf[uart_data->rev_buf_end] = uart_data->rev_rpr[i];
			uart_data->rev_buf_end++;
		}
		

		AT91F_PDC_SetRx(uart_data->pdc, (char*)uart_data->rev_rpr, uart_data->rev_buf_count);
		
		
		
	}
	return nlen;

}
void uart_doIrq(struct UART_DATA *uart_data)
{
	
	
	
	if(AT91C_BASE_DBGU->DBGU_CSR & 0x08 )
	{
		uart_doread(uart_data);
	}
	if(AT91C_BASE_DBGU->DBGU_CSR & 0x10 )//send over
	{
		uart_dosend(uart_data);
	
	}
	

}

