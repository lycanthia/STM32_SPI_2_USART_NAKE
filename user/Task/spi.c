/*
 *  user/Task/SPI.c
 *
 *  Copyright (C) lycanthia(bdyyc@hotmail_com) in github.com
 *
 *  2013.3.23
 *
 *  STM_SPI <---> RT5350
 * 
 */


#include <Task/spi.h>
#include <Task/uart.h>
#include <Driver/flash.h>

const u32 baud_table[16] = {
300,
600,
1200,
2400,
4800,
9600,
19200,
38400,
57600,
115200,
230400
};
USART_TypeDef *usart_table[3] = {USART1,USART2,USART3};

u16 spi_buf[SIZEOF_RXBUF];
__spi st_spi_rx = {
	0,
	0,0,
	spi_buf
};
__pspi spi_rx = &st_spi_rx;

#define  ST_UART_RX(n)	uart##n##_rx

void spi2_init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

		/** SPI2 GPIO Configuration	
		 PB12	 ------> SPI2_NSS
		 PB13	 ------> SPI2_SCK
		 PB14	 ------> SPI2_MISO
		 PB15	 ------> SPI2_MOSI
	*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		/*Configure GPIO pin */
		//PB1	 ------> GPIO_Output
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

    SPI_Cmd( SPI2, DISABLE );   //       
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;       
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;  
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;  
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;       
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( SPI2, &SPI_InitStructure );
		
		/* Enable SPI2 RXNE interrupt */
		SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
		
    /* Enable SPI1  */
    SPI_Cmd( SPI2, ENABLE );
    SPI2->DR = 0x00;
}

void spi1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	/** SPI1 GPIO Configuration	
		 PA4	 ------> SPI1_NSS
		 PA5	 ------> SPI1_SCK
		 PA6	 ------> SPI1_MISO
		 PA7	 ------> SPI1_MOSI
	*/

	/*Configure GPIO pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SPI_Cmd( SPI1, DISABLE );   //       
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;       
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;  
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;  
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;       
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init( SPI1, &SPI_InitStructure );
	/* Enable SPI1  */
	SPI_Cmd( SPI1, ENABLE );
	SPI1->DR = 0x00;
	
}

void spi_master_write(SPI_TypeDef *SPIx,u16 txd)
{
		/* Wait for SPI Tx buffer empty */
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
    /* Send SPI1 data */
    SPI_I2S_SendData(SPIx, txd); 
}

u16 spi_master_read(SPI_TypeDef *SPIx)
{
		SPI_I2S_SendData(SPIx, 0);	
		/* Wait for SPI2 data reception */
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
	
		return SPIx->DR;
}

void spi_slave_write(SPI_TypeDef *SPIx,u16 txd)
{
	wr_io(SPI2_IRQ,0);			//generate a externel interrupt to rt5350.
	SPIx->DR = txd;
  /* Wait for SPI2 data reception */
  while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);	
	wr_io(SPI2_IRQ,1);
}





void check_com_2_spi(__puart uart_rx)
{
	u8 i = 0;
	if(!uart_rx->cnt){
		for(i=0;i<uart_rx->cnt;i++)
			spi_slave_write(SPI2,uart_rx->buf[i]|uart_rx->index);
//		OS_ENTER_CRITICAL(  );      /* Tell uC/OS-II that we are starting an ISR */
		NVIC_SETPRIMASK();                    // Disable Interrupts
    uart_rx->cnt = 0;
		NVIC_RESETPRIMASK();                  // Enable Interrupts
//    OS_EXIT_CRITICAL(  );		
	}
}
void uart_broadcast(u8 dat)
{
	USART_SendData(USART1, dat);
	USART_SendData(USART2, dat);
	USART_SendData(USART3, dat);
  while (!(USART1->SR & USART_FLAG_TXE));
	while (!(USART2->SR & USART_FLAG_TXE));
	while (!(USART3->SR & USART_FLAG_TXE));
}

void uart_send(u8 index,u8 dat)
{
	USART_SendData(usart_table[index], dat);
  while (!(usart_table[index]->SR & USART_FLAG_TXE));
}
void uart_setbaud(u8 index,u8 dat)
{
	USART_InitTypeDef USART_InitStructure;
 	USART_Cmd(usart_table[index], DISABLE);
	USART_InitStructure.USART_BaudRate = baud_table[dat];
	USART_Init(usart_table[index], &USART_InitStructure);	
	flash_write(UART_BAUD_ADDRESS(index),baud_table[dat]);
	USART_Cmd(usart_table[index], ENABLE);
}
void check_spi_2_com(void)
{
	u8 i = 0;
	u8 ibaud = 0;
	u8 ibroad = 0;
	u8 index = 0;
	u16 buf = 0;
	for(i=0; i<spi_rx->cnt; i++){
		buf = spi_rx->buf[i];
		ibaud = buf & MASK_SETBAUD;
		ibroad = buf & MASK_BROAD;
		index = ((buf & MASK_COMSEL) >> 12);
		
		if(ibaud){
			if(index)
				uart_setbaud(index-1,(u8)buf);								//set baudrate to com specified.
		}
		else if(ibroad){
			uart_broadcast((u8)buf);	
		}
		else if(index)
			uart_send(index-1,(u8)buf);											//uart-send data to com specified.	
	}
	
	NVIC_SETPRIMASK();                    // Disable Interrupts
	spi_rx->cnt = 0;
	NVIC_RESETPRIMASK();                  // Enable Interrupts
}
void task_spi( void  )
{
	u8 i = 0;
    while ( 1 )
    {
//      OSTimeDly( 10 );
			check_com_2_spi(ST_UART_RX(1));
			check_com_2_spi(ST_UART_RX(2));
			check_com_2_spi(ST_UART_RX(3));
			
			if(spi_rx->cnt)
				check_spi_2_com();
    }        

}
