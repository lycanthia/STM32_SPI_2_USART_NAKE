#ifndef __VERSION_H__
#define __VERSION_H__
// <<< Use Configuration Wizard in Context Menu >>>


// <h> 配置
//    <o> 任务堆栈大小      <0x40-0x200:0x08>
#define OS_USER_TASK_STK_SIZE    128

//    <o> 发送缓冲区大小    <0x10-0x200:0x04>
#define SIZEOF_TXBUF				    256

//    <o> 接收缓冲区大小      <0x10-0x200:0x04>
#define SIZEOF_RXBUF    				256
// </h>

// <h> Version
//              <o> Year     <2000-2020>
#define C_YEAR    2013
//              <o> Month       <1-12>            
#define C_MONTH   3
//              <o> Date        <1-31>    
#define C_DATE    26
// </h>

#define TEST 1

#define 	PAGE_SIZE 				0x400 

// <<< end of configuration section >>>

#ifdef TEST
#define out(a,b) O((a),(b))
#else
#define out(a,b)
#endif

#endif
