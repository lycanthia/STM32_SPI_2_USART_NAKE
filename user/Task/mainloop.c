/*
 *  user/Task/Mainloop.c
 *
 *  Copyright (C) lycanthia(bdyyc@hotmail_com) in github.com
 *
 *  2012.4.9
 *
 *  ��ѭ������
 * 
 */


#include <Task/mainloop.h>

void Task_Main( void *p_arg )
{

    while ( 1 )
    {
//        OSTimeDly( 250 );
			
				GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_SET);
    }        

}
