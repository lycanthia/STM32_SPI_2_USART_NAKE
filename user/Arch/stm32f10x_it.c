/*
 *  user/Arch/stm32f10x_it.c
 *
 *  Copyright (C) lycanthia(bdyyc@hotmail_com) in github.com
 *
 *  2012.4.9
 *
 *  Interrupt handle.
 * 
 *  Descriptions: 	Main Interrupt Service Routines.
 *                   -This file can be used to describe all the exceptions 
 *                   -subroutines that may occur within user application.
 *                   -When an interrupt happens, the software will branch 
 *                   -automatically to the corresponding routine.
 *                   -The following routines are all empty, user can write code 
 *                   -for exceptions handlers and peripherals IRQ interrupts.
 */

#include "stm32f10x_it.h"
#include <Cfg/config.h>
#include <Task/uart.h>
#include <Task/spi.h>


/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTickHandler( void )
{
   
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles WWDG interrupt request.
  * @param  None
  * @retval None
  */

void WWDG_IRQHandler( void )
{
          
    
    
		
	
               
}

/**
  * @brief  This function handles PVD interrupt request.
  * @param  None
  * @retval None
  */
void PVD_IRQHandler( void )
{
          
    
    

}

/**
  * @brief  This function handles Tamper interrupt request.
  * @param  None
  * @retval None
  */
void TAMPER_IRQHandler( void )
{
          
    
    

}

/**
  * @brief  This function handles RTC global interrupt request.
  * @param  None
  * @retval None
  */
void RTC_IRQHandler( void )
{
          
    
    

           
}

/**
  * @brief  This function handles Flash interrupt request.
  * @param  None
  * @retval None
  */
void FLASH_IRQHandler( void )
{
          
    
    

             
}

/**
  * @brief  This function handles RCC interrupt request.
  * @param  None
  * @retval None
  */
void RCC_IRQHandler( void )
{
          
    
    

                 
}

/**
  * @brief  This function handles External interrupt Line 0 request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler( void )
{
          
    
    

    EXTI_ClearITPendingBit( EXTI_Line0 );
                  
}

/**
  * @brief  This function handles External interrupt Line 1 request.
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler( void )
{
          
    
    

    EXTI_ClearITPendingBit( EXTI_Line1 );
                  
}

/**
  * @brief  This function handles External interrupt Line 2 request.
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler( void )   
{

          
    
    

    EXTI_ClearITPendingBit( EXTI_Line2 );     
                  
}

/**
  * @brief  This function handles External interrupt Line 3 request.
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler( void )
{
          
    
    

    EXTI_ClearITPendingBit( EXTI_Line3 );      
    
}

/**
  * @brief  This function handles External interrupt Line 4 request.
  * @param  None
  * @retval None
  */

void EXTI4_IRQHandler( void )
{
          
    
    

    EXTI_ClearITPendingBit( EXTI_Line4 );
    
}


/**
  * @brief  This function handles DMA1 Channel 1 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler( void )
{
          
    
    

    
}

/**
  * @brief  This function handles DMA1 Channel 2 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel2_IRQHandler( void )
{
          
    
    

    
}


/**
  * @brief  This function handles DMA1 Channel 3 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel3_IRQHandler( void )
{
          
    
    

    
}


/**
  * @brief  This function handles DMA1 Channel 4 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel4_IRQHandler( void )
{
          
    
    

    
}


/**
  * @brief  This function handles DMA1 Channel 5 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel5_IRQHandler( void )
{
          
    
    

    
}


/**
  * @brief  This function handles DMA1 Channel 6 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel6_IRQHandler( void )
{
          
    
    

    
}


/**
  * @brief  This function handles DMA1 Channel 7 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel7_IRQHandler( void )
{
          
    
    

    
}


/**
  * @brief  This function handles ADC1 and ADC2 global interrupts requests.
  * @param  None
  * @retval None
  */
 //3723¶ÔÓ¦900
 //
void ADC1_2_IRQHandler( void )	
{

}



/**
  * @brief  This function handles USB High Priority or CAN TX interrupts requests.                
  * @param  None
  * @retval None
  */
void USB_HP_CAN1_TX_IRQHandler( void )
{
          
    
    

    
}


/**
  * @brief  This function handles USB Low Priority or CAN RX0 interrupts requests.
  * @param  None
  * @retval None
  */
void USB_LP_CAN1_RX0_IRQHandler( void )
{
          
    
    

    
}

/**
  * @brief  This function handles CAN RX1 interrupt request.
  * @param  None
  * @retval None
  */
void CAN1_RX1_IRQHandler( void )
{
          
    
    

    
}


/**
  * @brief  This function handles CAN SCE interrupt request.
  * @param  None
  * @retval None
  */
void CAN1_SCE_IRQHandler( void )
{
          
    
    

    
}


/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler( void )
{
          
    
    

    
}


/**
  * @brief  This function handles TIM1 Break interrupt request.
  * @param  None
  * @retval None
  */
void TIM1_BRK_IRQHandler( void )
{
          
    
    

    
}


/**
  * @brief  This function handles TIM1 overflow and update interrupt request.
  * @param  None
  * @retval None
  */
void TIM1_UP_IRQHandler( void )
{
          
    
    

    
}

/**
  * @brief  This function handles TIM1 Trigger and commutation interrupts requests.
  * @param  None
  * @retval None
  */
void TIM1_TRG_COM_IRQHandler( void )
{
          
    
    

    
}


/**
  * @brief  This function handles TIM1 capture compare interrupt request.
  * @param  None
  * @retval None
  */
void TIM1_CC_IRQHandler( void )
{
          
    
    

    
}


/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */


void TIM2_IRQHandler( void )
{

          
    
    

    TIM_ClearITPendingBit( TIM2, TIM_IT_Update );

                  // Tell uC/OS-II that we are leaving the ISR
}


/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
BitAction      Gbn = Bit_RESET;
void TIM3_IRQHandler( void )
{
          
    
    

    if ( TIM_GetITStatus( TIM3, TIM_IT_Update ) != RESET )
    {
        TIM_ClearITPendingBit( TIM3, TIM_IT_Update );
    }
                  // Tell uC/OS-II that we are leaving the ISR 
}


/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler( void )
{
          
    
    
    if ( TIM_GetITStatus( TIM4, TIM_IT_Update ) != RESET )
    {

        TIM_ClearITPendingBit( TIM4, TIM_IT_Update );
    }
    
}

/**
  * @brief  This function handles I2C1 Event interrupt request.
  * @param  None
  * @retval None
  */
void I2C1_EV_IRQHandler( void )
{
}


/**
  * @brief  This function handles I2C1 Error interrupt request.
  * @param  None
  * @retval None
  */
void I2C1_ER_IRQHandler( void )
{
}


/**
  * @brief  This function handles I2C2 Event interrupt request.
  * @param  None
  * @retval None
  */
void I2C2_EV_IRQHandler( void )
{
}


/**
  * @brief  This function handles I2C2 Error interrupt request.
  * @param  None
  * @retval None
  */
void I2C2_ER_IRQHandler( void )
{
}


/**
  * @brief  This function handles SPI1 global interrupt request.
  * @param  None
  * @retval None
  */
void SPI1_IRQHandler( void )
{
	      
	
	
	
	              
}


/**
  * @brief  This function handles SPI2 global interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler( void )
{
	u16 rxd = 0;
	      	
	rxd = SPI_I2S_ReceiveData(SPI2); 
	if(rxd != SPI_NULL)
		spi_rx->buf[spi_rx->cnt++] = rxd;               
}


/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */

void USART1_IRQHandler( void )
{		
		u8  rxd = 0;
		//u16 spi_txd = INDEX_USART1;
	
		if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
		{
			rxd = USART_ReceiveData(USART1);		
			 
			uart1_rx->buf[uart1_rx->cnt++] = rxd; 
			
			
			//spi_txd |= rxd;
			//spi_write(SPI2,spi_txd);	

			USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		}	
                  
}


/**
  * @brief  This function handles USART2 global interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler( void )
{
		u8  rxd = 0;
		//u16 spi_txd = INDEX_USART2;

		if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
		{
			rxd = USART_ReceiveData(USART2);
			 
			uart2_rx->buf[uart2_rx->cnt++] = rxd;  
			
			
			//spi_txd |= rxd;
			//spi_write(SPI2,spi_txd);
			
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		}	
		              
}


/**
  * @brief  This function handles USART3 global interrupt request.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler( void )
{
		u8  rxd = 0;
		//u16 spi_txd = INDEX_USART3;

		if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
		{
			rxd = USART_ReceiveData(USART3);
			 
			uart3_rx->buf[uart3_rx->cnt++] = rxd;  
			
			
			//spi_txd |= rxd;
			//spi_write(SPI2,spi_txd);		
			
			USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		}	
		              
}


/**
  * @brief  This function handles External lines 15 to 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler( void )      
{
          
    
    
                  
}


/**
  * @brief  This function handles RTC Alarm interrupt request.
  * @param  None
  * @retval None
  */
void RTCAlarm_IRQHandler( void )
{
}


/**
  * @brief  This function handles USB WakeUp interrupt request.
  * @param  None
  * @retval None
  */
void USBWakeUp_IRQHandler( void )
{
}


/**
  * @brief  This function handles TIM8 Break interrupt request.
  * @param  None
  * @retval None
  */
void TIM8_BRK_IRQHandler( void )
{
}


/**
  * @brief  This function handles TIM8 overflow and update interrupt request.
  * @param  None
  * @retval None
  */
void TIM8_UP_IRQHandler( void )
{
}


/**
  * @brief  This function handles TIM8 Trigger and commutation interrupts requests.
  * @param  None
  * @retval None
  */
void TIM8_TRG_COM_IRQHandler( void )
{
}


/**
  * @brief  This function handles TIM8 capture compare interrupt request.
  * @param  None
  * @retval None
  */
void TIM8_CC_IRQHandler( void )
{
}


/**
  * @brief  This function handles ADC3 global interrupt request.
  * @param  None
  * @retval None
  */
void ADC3_IRQHandler(void)
{
         
   
   

   
}


/**
  * @brief  This function handles FSMC global interrupt request.
  * @param  None
  * @retval None
  */
void FSMC_IRQHandler( void )
{
}


/**
  * @brief  This function handles SDIO global interrupt request.
  * @param  None
  * @retval None
  */
void SDIO_IRQHandler( void )
{
}


/**
  * @brief  This function handles TIM5 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler( void )
{
}


/**
  * @brief  This function handles SPI3 global interrupt request.
  * @param  None
  * @retval None
  */
void SPI3_IRQHandler( void )
{
}


/**
  * @brief  This function handles UART4 global interrupt request.
  * @param  None
  * @retval None
  */
void UART4_IRQHandler( void )
{
}


/**
  * @brief  This function handles UART5 global interrupt request.
  * @param  None
  * @retval None
  */
void UART5_IRQHandler( void )
{
}


/**
  * @brief  This function handles TIM6 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM6_IRQHandler( void )
{
}


/**
  * @brief  This function handles TIM7 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM7_IRQHandler( void )
{
}


/**
  * @brief  This function handles DMA2 Channel 1 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Channel1_IRQHandler( void )
{
}


/**
  * @brief  This function handles DMA2 Channel 2 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Channel2_IRQHandler( void )
{
}


/**
  * @brief  This function handles DMA2 Channel 3 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Channel3_IRQHandler( void )
{
}


/**
  * @brief  This function handles DMA2 Channel 4 and DMA2 Channel 5 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Channel4_5_IRQHandler( void )
{
          
    
    

    
}

/************************END OF FILE*****************************/
