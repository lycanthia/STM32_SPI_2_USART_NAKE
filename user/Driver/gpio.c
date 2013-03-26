/*
 *  user/Driver/gpio.c
 *
 *  Copyright (C) lycanthia(bdyyc@hotmail_com) in github.com
 *
 *  2012.4.9
 *
 *  IO²Ù×÷¡£
 * 
 */
 
#include <Driver/gpio.h>

const vu16     io_bit[16] = { 
    0x1, 0x2, 0x4, 0x8,
    0x10, 0x20, 0x40, 0x80,
    0x100, 0x200, 0x400, 0x800,
    0x1000, 0x2000, 0x4000, 0x8000
};


/**
  * @brief  ¶ÁIO×´Ì¬
  * @param  pin£ºIOË÷Òý¡£
  * @retval IOÂß¼­×´Ì¬¡£
  */
u8 rd_io( u8 pin )
{
    GPIO_TypeDef  *GPIOx;
    u8             val = 0;
    switch ( pin >> 4 )
    {
        case 0:
            GPIOx = GPIOA;
            break;
        case 1:
            GPIOx = GPIOB;
            break;
        case 2:
            GPIOx = GPIOC;
            break;
        case 3:
            GPIOx = GPIOD;
            break;
        case 4:
            GPIOx = GPIOE;
            break;
        case 5:
            GPIOx = GPIOF;
            break;
        case 6:
            GPIOx = GPIOG;
            break;
        default:
            return 0;
    }
    val = ( ( ( GPIOx->IDR ) & io_bit[pin & 0xf] ) == 0 ) ? 0 : 1;
    return val;

}

/**
  * @brief  Ð´IO
  * @param  pin: IOË÷Òý¡£
            val: IOÂß¼­×´Ì¬¡£
  * @retval None
  */
void wr_io( u8 pin, u8 val )
{
    GPIO_TypeDef  *GPIOx;
    switch ( pin >> 4 )
    {
        case 0:
            GPIOx = GPIOA;
            break;
        case 1:
            GPIOx = GPIOB;
            break;
        case 2:
            GPIOx = GPIOC;
            break;
        case 3:
            GPIOx = GPIOD;
            break;
        case 4:
            GPIOx = GPIOE;
            break;
        case 5:
            GPIOx = GPIOF;
            break;
        case 6:
            GPIOx = GPIOG;
            break;
        default:
            return;
    }
    GPIO_WriteBit( GPIOx, io_bit[pin & 0xf], ( BitAction ) val );
}

