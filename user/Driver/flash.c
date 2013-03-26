/*
 *  user/Driver/flash.c
 *
 *  Copyright (C) lycanthia(bdyyc@hotmail_com) in github.com
 *
 *  2013.3.25
 *
 * 
 * 
 */

#include <Driver/flash.h>


void flash_write( u32 addr, u32 val )
{
		FLASH_Status   status = FLASH_COMPLETE;
	
    FLASH_ErasePage( addr );
	
//    OS_ENTER_CRITICAL(  );
    status = FLASH_WaitForLastOperation( ProgramTimeout );
    FLASH->CR |= CR_PG_Set;

    *(vu16 *)addr 			= (u16)(val >> 16);
		*(vu16 *)(addr + 2) = (u16)(val & 0xffff);

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation( ProgramTimeout );
    if ( status != FLASH_BUSY )
    {
        FLASH->CR &= CR_PG_Reset;
    }
 //   OS_EXIT_CRITICAL(  );
    
}
u32 flash_read( u32 addr )
{
		u16 vh = 0;
		u16 vl = 0;
		u32 ret = 0;
		vh = (*(vu16 *)addr);
	  vl = (*(vu16 *)(addr+2));
		ret = (vh << 16) + vl;
    return ret;
}