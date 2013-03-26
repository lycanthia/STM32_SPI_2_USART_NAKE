/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __UART_H__
#define __UART_H__
#ifdef __cplusplus
 extern "C" {
#endif
	 
	 
#include <Cfg/config.h>
typedef struct{
		u16 index;
		u16 cnt;
		u16 all;
		u8 *buf;
}__uart,*__puart;


extern __puart uart1_tx;
extern __puart uart2_tx;
extern __puart uart3_tx;
extern __puart uart1_rx;
extern __puart uart2_rx;
extern __puart uart3_rx;

void uart1_init(void);
void uart2_init(void);
void uart3_init(void);
void Task_Uart( void *p_arg );






#ifdef __cplusplus
}
#endif
#endif