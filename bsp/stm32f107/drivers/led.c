/*
 * File      : led.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */
#include <rtthread.h>
#include <stm32f10x.h>

// led define PE2/PE3/PE4/PE5
#define led_rcc                    RCC_APB2Periph_GPIOE
#define led_gpio                   GPIOE

void rt_hw_led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(led_rcc,ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(led_gpio, &GPIO_InitStructure);
}

void rt_hw_led_on(rt_uint32_t n)
{
    if(n > 3)
        return;

    GPIO_ResetBits(led_gpio, (1<<(n+2)));
}

void rt_hw_led_off(rt_uint32_t n)
{
    if(n > 3)
        return;

    GPIO_SetBits(led_gpio, (1<<(n+2)));
}
