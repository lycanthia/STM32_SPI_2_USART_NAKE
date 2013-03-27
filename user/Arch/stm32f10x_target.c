/*
 *  user/Arch/stm32f10x_target.c
 *
 *  Copyright (C) lycanthia(bdyyc@hotmail_com) in github.com
 *
 *  2012.4.9
 *
 *  System init function. RCC,NVIC,DMA.
 * 
 */

/* Includes ------------------------------------------------------------------ */
#include <Cfg/config.h>
#include <Task/uart.h>

#define USART1_DR_Base  0x40013804
#define USART2_DR_Base  0x40004404
#define USART3_DR_Base  0x40004804

/* Public  variables --------------------------------------------------------- */
ErrorStatus    hse_startup_status;

/* Public  function  --------------------------------------------------------- */



/**
  * @brief  stm32f10x时钟系统配置.
  * @param  None
  * @retval None
  */
static void RCC_Configuration( void )
{
    /* RCC system reset(for debug purpose) */
    //RCC_DeInit(  );
    /* Enable HSE */
    RCC_HSEConfig( RCC_HSE_ON );
    /* Wait till HSE is ready */
    hse_startup_status = RCC_WaitForHSEStartUp(  );
    if ( hse_startup_status == SUCCESS )
    {
        /* HCLK = SYSCLK */
        RCC_HCLKConfig( RCC_SYSCLK_Div1 );
        /* PCLK2 = HCLK */
        RCC_PCLK2Config( RCC_HCLK_Div1 );
        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config( RCC_HCLK_Div2 );
        /* ADCCLK = PCLK2/4 */
        RCC_ADCCLKConfig( RCC_PCLK2_Div4 );

        /* Enable Prefetch Buffer */
        FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable );
        /* Flash 2 wait state */
        FLASH_SetLatency( FLASH_Latency_2 );

        /* PLLCLK = 8MHz * 9 = 72 MHz */
        RCC_PLLConfig( RCC_PLLSource_HSE_Div1, RCC_PLLMul_9 );
        /* Enable PLL */
        RCC_PLLCmd( ENABLE );
        /* Wait till PLL is ready */
        while ( RCC_GetFlagStatus( RCC_FLAG_PLLRDY ) == RESET );
        /* Select PLL as system clock source */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );
        /* Wait till PLL is used as system clock source */
        while ( RCC_GetSYSCLKSource(  ) != 0x08 );
    }
    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA |
                            RCC_APB2Periph_GPIOB |
                            RCC_APB2Periph_AFIO |
                            RCC_APB2Periph_SPI1 |
                            RCC_APB2Periph_USART1, ENABLE );

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2 |
                            RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3, ENABLE );
}


/**
  * @brief  stm32f10x配置基本向量中断
  * @param  None
  * @retval None
  */
static void NVIC_Configuration( void )
{
	NVIC_InitTypeDef NVIC_InitStructure;
#ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable( NVIC_VectTab_RAM, 0x0 );
#else                           /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
#endif

    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );
	
	/* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  /* Enable the USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the USART3 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_Init(&NVIC_InitStructure);
	
//	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQChannel;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable SPI2 interrupt -------------------------------------*/  
  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : DMA_Configuration
* Description    : Configures the DMA.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA_Configuration(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  /* DMA Channel4 (triggered by USART1 Tx event) Config */
  DMA_DeInit(DMA1_Channel4);
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)uart1_tx->buf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = SIZEOF_TXBUF;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  
  /* DMA Channel7 (triggered by USART2 Tx event) Config */
  DMA_DeInit(DMA1_Channel7);
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Base;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)uart2_tx->buf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = SIZEOF_TXBUF;
  DMA_Init(DMA1_Channel7, &DMA_InitStructure);
	
	/* DMA Channel2 (triggered by USART3 Tx event) Config */
  DMA_DeInit(DMA1_Channel2);
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR_Base;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)uart3_tx->buf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = SIZEOF_TXBUF;
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);
}

/**
  * @brief  特殊的GPIO管脚配置
  * @param  None
  * @retval None
  */
static void GPIO_Config( void )
{
    GPIO_InitTypeDef GPIO_InitStructure;   
    GPIO_PinRemapConfig( GPIO_Remap_SWJ_Disable, ENABLE );     

    RCC_MCOConfig( RCC_MCO_HSE );
}


/**
  * @brief  板级初始化函数
  * @param  None
  * @retval None
  */
int SystemInit( void )   //SystemInit
{
    /* System Clocks Configuration */
    RCC_Configuration(  );

    NVIC_DeInit(  );

    /* NVIC configuration */
    NVIC_Configuration(  );
	
//		DMA_Configuration();

//    GPIO_Config(  );
    
    return 0;

}


/**
  * @brief  OS tick 初始化函数
  * @param  None
  * @retval None
  */
void tick_init( void )
{
    /* SysTick end of count event each 0.1ms with input clock equal to 9MHz (HCLK/8, default) */
    //SysTick_SetReload( 9000000 / OS_TICKS_PER_SEC );
    /* Enable SysTick interrupt */
    //SysTick_ITConfig( ENABLE );
    /* Enable the SysTick Counter */
    //SysTick_CounterCmd( SysTick_Counter_Enable );
}


/**
  * @brief  窗口看门狗初始化函数
  * @param  None
  * @retval None
  */
void wdg_init( void )
{
    /* Enable WDG clocks */
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_WWDG, ENABLE );

    /* PCKL1: 36MHZ */
    /* WWDG clock counter = (PCLK1/4096)/8 = 488 Hz (~2 ms)  */
    WWDG_SetPrescaler( WWDG_Prescaler_8 );
    /* Set Window value to 65 */
    WWDG_SetWindowValue( 65 );
    /* Enable WWDG and set counter value to 127, WWDG timeout = ~2 ms * 64 = 130 ms */
    WWDG_Enable( 127 );

    /* Clear EWI flag */
    WWDG_ClearFlag(  );

    /* Enable EW interrupt */
    WWDG_EnableIT(  );

}

/**
  * @brief  普通看门狗初始化
  * @param  None
  * @retval None
  */
void wd_init( void )
{
    /* IWDG timeout equal to 350ms (the timeout may varies due to LSI frequency
       dispersion) ------------------------------------------------------------- */
    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable );

    /* IWDG counter clock: 32KHz(LSI) / 32 = 1KHz */
    IWDG_SetPrescaler( IWDG_Prescaler_32 );

    /* Set counter reload value to 349 */
    IWDG_SetReload( 500 );
    /* Reload IWDG counter */
    IWDG_ReloadCounter(  );

    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
    IWDG_Enable(  );
}


/**
  * @brief  简单的延时函数
  * @param  cnt：time ticks.
  * @retval None
  */
void sleep( u32 cnt )
{
    u32            i = 0;

    for ( i = 0; i < cnt; i++ );
}

/**
  * @brief  延时毫秒
  * @param  ms：毫秒数.
  * @retval None
  */
void delayms( u32 ms )          //ms延时
{
    //OSTimeDly( 10 * ms );
}

#ifdef  DEBUG
/**
  * @brief  Reports the name of the source file and the source line number 
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  *         line: assert_param error line source number
  * @retval None
  */
void assert_failed( u8 * file, u32 line )
{
    /* User can add his own implementation to report the file name and line number,
       ex: O("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while ( 1 )
    {
    }
}
#endif

/*************************End of file ******************/
