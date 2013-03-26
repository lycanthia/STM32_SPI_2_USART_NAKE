/*
 *  user/Task/uart.c
 *
 *  Copyright (C) lycanthia(bdyyc@hotmail_com) in github.com
 *
 *  2013.3.23
 *
 *  STM_USART1 <---> PICLF1823,HT95R54
 *  STM_USART2 <---> STC-ThreeCard
 *  STM_USART3 <---> Elec-Modem
 */
#include <Task/uart.h>
#include <Task/spi.h>
#include <Driver/flash.h>

u8 uart1_tx_buf[SIZEOF_TXBUF];
u8 uart1_rx_buf[SIZEOF_TXBUF];
u8 uart2_tx_buf[SIZEOF_TXBUF];
u8 uart2_rx_buf[SIZEOF_TXBUF];
u8 uart3_tx_buf[SIZEOF_TXBUF];
u8 uart3_rx_buf[SIZEOF_TXBUF];
__uart st_uart1_tx = {
	INDEX_USART1,
	0,0,
	uart1_tx_buf
};
__uart st_uart1_rx = {
	INDEX_USART1,
	0,0,
	uart1_rx_buf
};
__uart st_uart2_tx = {
	INDEX_USART2,
	0,0,
	uart2_tx_buf
};
__uart st_uart2_rx = {
	INDEX_USART2,
	0,0,
	uart2_rx_buf
};
__uart st_uart3_tx = {
	INDEX_USART3,
	0,0,
	uart3_tx_buf
};
__uart st_uart3_rx = {
	INDEX_USART3,
	0,0,
	uart3_rx_buf
};




__puart uart1_tx = &st_uart1_tx;
__puart uart1_rx = &st_uart1_rx;
__puart uart2_tx = &st_uart2_tx;
__puart uart2_rx = &st_uart2_rx;
__puart uart3_tx = &st_uart3_tx;
__puart uart3_rx = &st_uart3_rx;


void memclr(u8 *p,u8 len);

void uart1_init(void)
{	
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	u32 baudrate = flash_read(UART_BAUD_ADDRESS(0));
		/** USART1 GPIO Configuration	
		 PA9	 ------> USART1_TX
		 PA10	 ------> USART1_RX
	*/
	/*Configure GPIO pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*Configure GPIO pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	if((baudrate == 0xffffffff)||(baudrate == 0))
		baudrate = 9600;
	USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_Init(USART1, &USART_InitStructure);
	USART_ClockInit(USART1, &USART_ClockInitStructure);
  	
	  /* Enable USART1 DMA TX request */
//  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

  /* Enable the USART1 Receive Interrupt */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  /* Enable DMA Channel4 */
//  DMA_Cmd(DMA1_Channel4, ENABLE);

  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);
}

void uart2_init(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	u32 baudrate = flash_read(UART_BAUD_ADDRESS(1));
	/** USART2 GPIO Configuration	
		 PA2	 ------> USART2_TX
		 PA3	 ------> USART2_RX
	*/

	/*Configure GPIO pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*Configure GPIO pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	if((baudrate == 0xffffffff)||(baudrate == 0))
		baudrate = 4800;
	USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_Init(USART2, &USART_InitStructure);
	USART_ClockInit(USART2, &USART_ClockInitStructure);

  /* Enable USART2 DMA TX request */
//  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

  /* Enable the USART2 Receive Interrupt */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  /* Enable DMA Channel7 */
//  DMA_Cmd(DMA1_Channel7, ENABLE);

  /* Enable the USART2 */
  USART_Cmd(USART2, ENABLE);
}

void uart3_init(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	u32 baudrate = flash_read(UART_BAUD_ADDRESS(2));
	/** USART3 GPIO Configuration	
		 PB10	 ------> USART3_TX
		 PB11	 ------> USART3_RX
	*/

	/*Configure GPIO pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*Configure GPIO pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/** Configure pins as GPIO
		 PA8	 ------> GPIO_Output
		 PA11	 ------> GPIO_Output
		 PA12	 ------> GPIO_Output
		 PB5	 ------> GPIO_Output
	*/
	/*Configure GPIO pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*Configure GPIO pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	if((baudrate == 0xffffffff)||(baudrate == 0))
		baudrate = 9600;
	USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_Init(USART3, &USART_InitStructure);
	USART_ClockInit(USART3, &USART_ClockInitStructure);

  /* Enable USART3 DMA TX request */
//  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

  /* Enable the USART3 Receive Interrupt */
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  /* Enable DMA Channel2 */
// DMA_Cmd(DMA1_Channel2, ENABLE);

  /* Enable the USART2 */
  USART_Cmd(USART3, ENABLE);
	
}
void var_init(void)
{

}
void Task_Uart( void *p_arg )
{
		var_init();
    while ( 1 )
    {
 //       OSTimeDly( 250 );		
				GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_SET);
    }        

}
