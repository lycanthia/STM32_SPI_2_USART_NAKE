/*
 *  user/main.c
 *
 *  Copyright (C) lycanthia(bdyyc@hotmail_com) in github.com
 *
 *  2012.4.9
 *
 *  主程序C语言入口, uC/OS-II启动.
 * 
 */

#include <Cfg/config.h>
#include <Task/mainloop.h>
#include <Task/spi.h>
#include <Task/uart.h>

/**
  * @brief  基本应用初始化
  * @param  None
  * @retval None
  */
void application_init( void )
{
	spi1_init();
	spi2_init();
	uart1_init();
	uart2_init();
	uart3_init();
}

/**
  * @brief  初始化任务核心函数：(1) 初始化应用  (2) 初始化操作系统组件
  * @param  None
  * @retval None
  */
void mainloop(void)
{	
    application_init(  );

    task_spi();
}

/**
  * @brief  C入口函数
  * @param  None
  * @retval None
  */
int main( void )
{
    /* Unlock the Flash Program Erase controller */
    FLASH_Unlock(  );

//    tick_init(  );           /* 初始化OS Tick  */
         
		mainloop();

    return ( 0 );
}









void memclr(u8 *p,u8 len)
{
	while(len--)
		*p++ = 0;
}
void mcpy(u8 *d,u8 *s,u8 len)
{
	while(len--)
		*d++ = *s++;	
}
/***************************************************************/
