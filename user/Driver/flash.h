/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __FLASH_H__
#define __FLASH_H__
#ifdef __cplusplus
 extern "C" {
#endif

#include <Cfg/config.h>

/* Flash Control Register bits */
#define CR_PG_Set                ((u32)0x00000001)
#define CR_PG_Reset              ((u32)0x00001FFE) 
	 
/* Delay definition */   
#define EraseTimeout             ((u32)0x00000FFF)
#define ProgramTimeout           ((u32)0x0000000F)
	 
	 
#define EEPROM_START_ADDRESS    ((u32)0x0800d800)       /* EEPROM emulation start address: after 54KByte of used Flash memory */
#define UART_BAUD_ADDRESS(n)   	EEPROM_START_ADDRESS + (PAGE_SIZE*(n))        //2048-9216

	 
void flash_write( u32 addr, u32 val );
u32 flash_read( u32 addr ); 
	 
	 
#ifdef __cplusplus
}
#endif
#endif