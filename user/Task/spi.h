/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __SPI_H__
#define __SPI_H__
#ifdef __cplusplus
 extern "C" {
#endif


#include <Cfg/config.h>

	 
#define  MASK_SETBAUD 	0x8000
#define	 MASK_BROAD			0x4000
#define	 MASK_COMSEL		0x3000	 
	 
#define  INDEX_USART1		0x1000
#define  INDEX_USART2		0x2000
#define  INDEX_USART3		0x3000

//无效数据,用于RT5350读STM32串口数据.
#define  SPI_NULL					0x0000		
#define	 SPI_BROADCAST		0xff00 

typedef struct{
		u16 index;
		u16 cnt;
		u16 all;
		u16 *buf;
}__spi,*__pspi;

extern __pspi spi_rx;	 
	 
void spi1_init(void);
void spi2_init(void);
void spi_master_write(SPI_TypeDef *SPIx,u16 txd);
u16 spi_master_read(SPI_TypeDef *SPIx);

void task_spi( void  );

#ifdef __cplusplus
}
#endif
#endif