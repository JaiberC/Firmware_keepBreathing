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
static const char fifo_data[] = "Testing Tx.\r\n";
static int tx_data_idx;
static volatile bool data_transmitted;

#define DATA_SIZE	(sizeof(fifo_data) - 1)

////// Para el timer
struct k_work my_work;
struct k_timer my_timer;
extern void my_expiry_function(struct k_timer *timer_id);
extern void my_work_handler(struct k_work *dummy);
K_WORK_DEFINE(my_work, my_work_handler);
u32_t start_time;
u32_t stop_time;
u32_t cycles_spent;
u32_t nanoseconds_spent=1;
int sample_time = 100; // En milisegundos


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
	else {
		printk("Problem getting info from uart_dev");	
	}	
	
}




static void uart_fifo_callback(const struct device *dev, void *user_data)
{
	
	ARG_UNUSED(user_data);
	if (!uart_irq_update(dev)) {
		printk("retval should always be 1\n");
		return;
	}
	
	uart_irq_tx_enable(uart_dev);
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
}

// Funciones para la interrupción por tiempo

void my_expiry_function(struct k_timer *dummy) // Se encarga de agregar la tarea al workqueue
{
    k_work_submit(&my_work);
}

void my_work_handler(struct k_work *work) // Función de interrupción por tiempo
{
	stop_time = k_cycle_get_32();
	cycles_spent = stop_time - start_time;
	nanoseconds_spent = cycles_spent/168000;
	start_time = k_cycle_get_32();

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

/////////////////// Timer init //////////////////////////////

	printk("Hello World Again! \n");
	start_time = k_cycle_get_32();
	k_timer_init(&my_timer, my_expiry_function, NULL);
	k_timer_start(&my_timer, K_MSEC(sample_time), K_MSEC(sample_time));
	
///////////////// While infinito ///////////////////////////	

	while (1) {
		gpio_pin_set(dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		k_msleep(SLEEP_TIME);

		printk("Clock ");
		printk("%d\n",nanoseconds_spent);

		char_sent = 0;
		
		/* Verify uart_irq_callback_set() */
		uart_irq_callback_set(uart_dev, uart_fifo_callback);
		
		/* Enable Tx/Rx interrupt before using fifo */
		/* Verify uart_irq_tx_enable() */
		uart_irq_tx_enable(uart_dev);
		
		k_sleep(K_MSEC(1000));
		
		tx_data_idx = 0;
		
		/* Verify uart_irq_tx_disable() */
		uart_irq_tx_disable(uart_dev);
	}
}
