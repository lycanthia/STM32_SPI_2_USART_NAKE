/****************************Copyright (c)*********************************************                                     
*
*               (c) Copyright 2000-2011, CTB ,All Rights Reserved 
*                            
*                        
*
*---------- 文件信息 ---------------------------------------------------------------
* 文 件 名: 	stm32f10x_target.h
* 创 建 人: 	CTB 	
* 创建日期: 	2011.11.11
* 描    述:     目标板基本初始化
*
*---------- 版本信息-------------------------------------------------------------------
* 版    本: V1.0
*
*--------------------------------------------------------------------------------------
**************************************************************************************/

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef __STM32F10x_TARGET_H
#define __STM32F10x_TARGET_H

int            system_init( void );

void           tick_init( void );

void           wdg_init( void );

void           sleep( u32 cnt );


#endif     /* __STM32F10x_TARGET_H */

/*************************End of file ******************/
