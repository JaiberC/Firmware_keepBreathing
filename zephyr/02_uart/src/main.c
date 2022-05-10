/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <devicetree.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <console/console.h>
#include <console/tty.h>
#include <string.h>
#include <stdio.h>

////////////////////blinky defines//////////////////////////
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif


////////////////////////usart defines/////////////////////////////////////////////

/* 1000 msec = 1 sec */
#define SLEEP_TIME  100
#define UART_DEVICE_NAME CONFIG_UART_CONSOLE_ON_DEV_NAME
//#define UART_DEVICE_NAME "UART_1"

const struct device *uart_dev;
static volatile bool data_transmitted;
static int char_sent;
static const char fifo_data[] = "Testing Tx....\r\n";
static int tx_data_idx;
static volatile bool data_transmitted;
static volatile bool data_received; 
char buffer_read[5];
volatile int contador=0;
volatile int rx_counter=0;

#define DATA_SIZE	(sizeof(fifo_data) - 1)

// Init
void uart_init()
{
struct uart_config uart_cfg;

	int ret;
	uart_dev = device_get_binding(UART_DEVICE_NAME);
	ret = uart_config_get(uart_dev, &uart_cfg);
	if (!ret) {
		printk("\n======== [%s] ========\n", UART_DEVICE_NAME);
		printk("[%s] uart_config.baudrate=%d\n", UART_DEVICE_NAME, uart_cfg.baudrate);
		printk("[%s] uart_config.parity=%d\n", UART_DEVICE_NAME, uart_cfg.parity);
		printk("[%s] uart_config.stop_bits=%d\n", UART_DEVICE_NAME, uart_cfg.stop_bits);
		printk("[%s] uart_config.data_bits=%d\n", UART_DEVICE_NAME, uart_cfg.data_bits);
		printk("[%s] uart_config.flow_ctrl=%d\n\n\n", UART_DEVICE_NAME, uart_cfg.flow_ctrl);
	}
}




static void uart_fifo_callback(const struct device *dev, void *user_data)
{	contador++;
	uint8_t recvData;
	ARG_UNUSED(user_data);
	if (!uart_irq_update(dev)) {
		printk("retval should always be 1\n");
		return;
	}
	
	//uart_irq_tx_enable(uart_dev);
	if (uart_irq_tx_ready(dev) && tx_data_idx < DATA_SIZE) {

		if (uart_fifo_fill(dev,
				   (uint8_t *)&fifo_data[tx_data_idx++], 1) > 0) {
			data_transmitted = true;
			char_sent++;
		}
		
		if (tx_data_idx == DATA_SIZE) {
			uart_irq_tx_disable(dev);
		}
	}

	/* Verify uart_irq_rx_ready() */
        if (uart_irq_rx_ready(dev)) {
                /* Verify uart_fifo_read() */
        	rx_counter++;
            	uart_fifo_read(dev, &recvData, 1);
		uart_irq_rx_disable(uart_dev);
                printk("%c", recvData);
		sprintf(buffer_read,"%s%c",buffer_read,recvData);

            	if ((recvData == '\n') || (recvData == '\r')) {
		        int i;
			i=atoi(buffer_read);
			printk("Char array %s\n",buffer_read);
			printk("Number + 1: %d\n",i+1);
			printk("Number * 2: %d\n",i*2);
			buffer_read[0]=0;
			rx_counter=0;
                }
        }


}





// Main
void main(void)
{

////////////////////gpio led0 init////////////////////////////
	const struct device *dev;
	bool led_is_on = true;
	int ret;

	dev = device_get_binding(LED0);
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}
////////////////////gpio led0 init////////////////////////////

	printk("Hello World! \n");
	
	uart_init();
	uart_irq_callback_set(uart_dev, uart_fifo_callback);

	while (1) {
<<<<<<< HEAD
		//printk("Contador: %d\n",contador);
=======
		printk("Contador: %d\n",contador);
>>>>>>> 1f43912ddb54cb9b8c1e97c42d41a5a18a25811a
		gpio_pin_set(dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		k_msleep(SLEEP_TIME);

		char_sent = 0;
		
		/* Verify uart_irq_callback_set() */
		//uart_irq_callback_set(uart_dev, uart_fifo_callback);
		//printk("Que imprima otra cosa\n");
		
		/* Enable Tx/Rx interrupt before using fifo */
		/* Verify uart_irq_tx_enable() */
		//uart_irq_tx_enable(uart_dev);
		
		/* Verify uart_irq_rx_enable() */
	        uart_irq_rx_enable(uart_dev);
        	//while (data_received == false) {}
				
		tx_data_idx = 0;
		
		/* Verify uart_irq_tx_disable() */
		uart_irq_tx_disable(uart_dev);

	}
}
