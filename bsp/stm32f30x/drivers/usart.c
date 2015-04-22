/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2015, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 * 2015-04-22     aozima       update for STM32F3xx.
 */


#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "usart.h"


/* USART1 hardware */
#define USART1_GPIO_TX          GPIO_Pin_9
#define USART1_TX_PIN_SOURCE    GPIO_PinSource9
#define USART1_GPIO_RX		    GPIO_Pin_10
#define USART1_RX_PIN_SOURCE    GPIO_PinSource10
#define USART1_GPIO			    GPIOA
#define USART1_GPIO_RCC         RCC_AHBPeriph_GPIOA

/* USART2 hardware */
#define USART2_GPIO_TX	        GPIO_Pin_2
#define USART2_TX_PIN_SOURCE    GPIO_PinSource2
#define USART2_GPIO_RX	        GPIO_Pin_3
#define USART2_RX_PIN_SOURCE    GPIO_PinSource3
#define USART2_GPIO	    	    GPIOA
#define USART2_GPIO_RCC     	RCC_AHBPeriph_GPIOB

/* USART3 hardware */
#define USART3_GPIO_TX		    GPIO_Pin_10
#define USART3_TX_PIN_SOURCE    GPIO_PinSource10
#define USART3_GPIO_RX		    GPIO_Pin_11
#define USART3_RX_PIN_SOURCE    GPIO_PinSource11
#define USART3_GPIO			    GPIOB
#define USART3_GPIO_RCC   	    RCC_AHBPeriph_GPIOB

/* STM32 uart driver */
struct stm32_uart
{
    USART_TypeDef* uart_device;
    IRQn_Type irq;
};

static rt_err_t stm32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct stm32_uart* uart;
    USART_InitTypeDef USART_InitStructure;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct stm32_uart *)serial->parent.user_data;

    USART_InitStructure.USART_BaudRate = cfg->baud_rate;

    if (cfg->data_bits == DATA_BITS_8)
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;

    if (cfg->stop_bits == STOP_BITS_1)
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    else if (cfg->stop_bits == STOP_BITS_2)
        USART_InitStructure.USART_StopBits = USART_StopBits_2;

    if (cfg->parity == PARITY_NONE)
    {
        USART_InitStructure.USART_Parity = USART_Parity_No;
    }
    else if (cfg->parity == PARITY_ODD)
    {
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
    }
    else if (cfg->parity == PARITY_EVEN)
    {
        USART_InitStructure.USART_Parity = USART_Parity_Even;
    }

    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uart->uart_device, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(uart->uart_device, ENABLE);

    return RT_EOK;
}

static rt_err_t stm32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct stm32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    switch (cmd)
    {
        /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        UART_DISABLE_IRQ(uart->irq);
        /* disable interrupt */
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, DISABLE);
        break;
        /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        UART_ENABLE_IRQ(uart->irq);
        /* enable interrupt */
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, ENABLE);
        break;
    }

    return RT_EOK;
}

static int stm32_putc(struct rt_serial_device *serial, char c)
{
    struct stm32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    /* Loop until transmit data register is empty */
    while (USART_GetFlagStatus(uart->uart_device, USART_FLAG_TXE) == RESET);

    USART_SendData(uart->uart_device, (uint8_t) c);

    return 1;
}

static int stm32_getc(struct rt_serial_device *serial)
{
    int ch;
    struct stm32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    ch = -1;
    if (USART_GetFlagStatus(uart->uart_device, USART_FLAG_RXNE) == SET)
    {
        ch = USART_ReceiveData(uart->uart_device);
    }

    return ch;
}

static const struct rt_uart_ops stm32_uart_ops =
{
    stm32_configure,
    stm32_control,
    stm32_putc,
    stm32_getc,
};

#if defined(RT_USING_USART1)
/* USART1 device driver structure */
static struct stm32_uart uart1 =
{
    USART1,
    USART1_IRQn,
};
static struct rt_serial_device serial1;

void USART1_IRQHandler(void)
{
    struct stm32_uart* uart;

    uart = &uart1;

    /* enter interrupt */
    rt_interrupt_enter();
    if(USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(&serial1, RT_SERIAL_EVENT_RX_IND);
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_RXNE);
    }

    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }
    if (USART_GetFlagStatus(uart->uart_device, USART_FLAG_ORE) == SET)
    {
        stm32_getc(&serial1);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(RT_USING_USART2)
/* USART2 device driver structure */
static struct stm32_uart uart2 =
{
    USART2,
    USART2_IRQn,
};
static struct rt_serial_device serial2;

void USART2_IRQHandler(void)
{
    struct stm32_uart* uart;

    uart = &uart2;

    /* enter interrupt */
    rt_interrupt_enter();

    if(USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(&serial2, RT_SERIAL_EVENT_RX_IND);
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_RXNE);
    }

    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }

    if (USART_GetFlagStatus(uart->uart_device, USART_FLAG_ORE) == SET)
    {
        stm32_getc(&serial2);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(RT_USING_USART3)
/* USART3 device driver structure */
static struct stm32_uart uart3 =
{
    USART3,
    USART3_IRQn,
};
static struct rt_serial_device serial3;

void USART3_IRQHandler(void)
{
    struct stm32_uart* uart;

    uart = &uart3;

    /* enter interrupt */
    rt_interrupt_enter();

    if(USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(&serial3, RT_SERIAL_EVENT_RX_IND);
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_RXNE);
    }

    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }

    if (USART_GetFlagStatus(uart->uart_device, USART_FLAG_ORE) == SET)
    {
        stm32_getc(&serial3);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

static void RCC_Configuration(void)
{
#ifdef RT_USING_USART1
    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd(USART1_GPIO_RCC, ENABLE);
    /* Enable UART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif

#ifdef RT_USING_USART2
    /* Enable GPIO clocks */
    RCC_AHBPeriphClockCmd(USART2_GPIO_RCC, ENABLE);
    /* Enable USART2 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif

#ifdef RT_USING_USART3
    /* Enable GPIO clocks */
    RCC_AHBPeriphClockCmd(USART3_GPIO_RCC, ENABLE);
    /* Enable USART3 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#endif
}

static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

#ifdef RT_USING_USART1
    /* Configure USART1 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = USART1_GPIO_RX | USART1_GPIO_TX;
    GPIO_Init(USART1_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(USART1_GPIO, USART1_TX_PIN_SOURCE, GPIO_AF_7);
    GPIO_PinAFConfig(USART1_GPIO, USART1_RX_PIN_SOURCE, GPIO_AF_7);
#endif

#ifdef RT_USING_USART2
    /* Configure USART2 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = USART2_GPIO_TX | USART2_GPIO_RX;
    GPIO_Init(USART2_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(USART2_GPIO, USART2_TX_PIN_SOURCE, GPIO_AF_7);
    GPIO_PinAFConfig(USART2_GPIO, USART2_RX_PIN_SOURCE, GPIO_AF_7);
#endif

#ifdef RT_USING_USART3
    /* Configure USART3 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = USART3_GPIO_TX | USART3_GPIO_RX;
    GPIO_Init(USART3_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(USART3_GPIO, USART3_TX_PIN_SOURCE, GPIO_AF_7);
    GPIO_PinAFConfig(USART3_GPIO, USART3_RX_PIN_SOURCE, GPIO_AF_7);
#endif
}

static void NVIC_Configuration(struct stm32_uart* uart)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = uart->irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void rt_hw_usart_init(void)
{
    struct stm32_uart* uart;
    struct serial_configure config;

    RCC_Configuration();
    GPIO_Configuration();

#ifdef RT_USING_USART1
    uart = &uart1;
    config.baud_rate = BAUD_RATE_115200;
    config.bit_order = BIT_ORDER_LSB;
    config.data_bits = DATA_BITS_8;
    config.parity    = PARITY_NONE;
    config.stop_bits = STOP_BITS_1;
    config.invert    = NRZ_NORMAL;

    serial1.ops    = &stm32_uart_ops;
    serial1.config = config;

    NVIC_Configuration(&uart1);

    /* register UART1 device */
    rt_hw_serial_register(&serial1, "usart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
#endif

#ifdef RT_USING_USART2
    uart = &uart2;

    config.baud_rate = BAUD_RATE_115200;
    config.bit_order = BIT_ORDER_LSB;
    config.data_bits = DATA_BITS_8;
    config.parity    = PARITY_NONE;
    config.stop_bits = STOP_BITS_1;
    config.invert    = NRZ_NORMAL;

    serial2.ops    = &stm32_uart_ops;
    serial2.config = config;

    NVIC_Configuration(&uart2);

    /* register UART1 device */
    rt_hw_serial_register(&serial2, "usart2",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
#endif

#ifdef RT_USING_USART3
    uart = &uart3;

    config.baud_rate = BAUD_RATE_115200;
    config.bit_order = BIT_ORDER_LSB;
    config.data_bits = DATA_BITS_8;
    config.parity    = PARITY_NONE;
    config.stop_bits = STOP_BITS_1;
    config.invert    = NRZ_NORMAL;

    serial3.ops    = &stm32_uart_ops;
    serial3.config = config;

    NVIC_Configuration(&uart3);

    /* register UART1 device */
    rt_hw_serial_register(&serial3, "usart3",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
#endif
}
