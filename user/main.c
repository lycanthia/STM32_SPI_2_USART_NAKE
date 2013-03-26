/*
 *  user/main.c
 *
 *  Copyright (C) lycanthia(bdyyc@hotmail_com) in github.com
 *
 *  2012.4.9
 *
 *  ������C�������, uC/OS-II����.
 * 
 */

#include <Cfg/config.h>
#include <Task/mainloop.h>
#include <Task/spi.h>
#include <Task/uart.h>

/**
  * @brief  ����Ӧ�ó�ʼ��
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
  * @brief  ��ʼ��������ĺ�����(1) ��ʼ��Ӧ��  (2) ��ʼ������ϵͳ���
  * @param  None
  * @retval None
  */
void mainloop(void)
{	
    application_init(  );

    task_spi();
}

/**
  * @brief  C��ں���
  * @param  None
  * @retval None
  */
int main( void )
{
    /* Unlock the Flash Program Erase controller */
    FLASH_Unlock(  );

//    tick_init(  );           /* ��ʼ��OS Tick  */
         
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
