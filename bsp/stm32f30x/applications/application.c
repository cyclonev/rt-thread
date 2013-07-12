/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>
#include <rtthread.h>
#include <board.h>

int ccm_buffer0[1024] SECTION(".CCM");

int ccm_buffer1[1] SECTION(".CCM");

int ccm_buffer2[128] SECTION(".CCM");

int ccm_bufferz[1024] ;

void rt_init_thread_entry(void* parameter)
{
    ccm_buffer0[1] = 0xAA;
    ccm_buffer1[1] = 0xAA;
    ccm_buffer2[1] = 0xAA;
    ccm_bufferz[1] = 0xAA;
}

int rt_application_init()
{
    rt_thread_t init_thread;

#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 80, 20);
#endif

    return 0;
}

/*@}*/
