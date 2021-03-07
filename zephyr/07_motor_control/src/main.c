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
#include <drivers/pwm.h>
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
static const char fifo_data[] = "Testing Tx.\r\n";
static int tx_data_idx;
static volatile bool data_transmitted;

#define DATA_SIZE	(sizeof(fifo_data) - 1)
char buffer_read[5];

///////////////////// Timer defines ////////////////////////////////////////////

struct k_work my_work;
struct k_timer my_timer;
extern void my_expiry_function(struct k_timer *timer_id);
extern void my_work_handler(struct k_work *dummy);
K_WORK_DEFINE(my_work, my_work_handler);
u32_t start_time;
u32_t stop_time;
u32_t cycles_spent;
u32_t nanoseconds_spent=1;
int sample_time = 500; // milisecs

////////////////// Encoder Defines ///////////////////////////////////////////
#define CHA_PIN 9

#define MY_DEV_IRQ  24       /* device uses IRQ 24 */
#define MY_DEV_PRIO  2       /* device uses interrupt priority 2 */
/* argument passed to my_isr(), in this case a pointer to the device */
#define MY_IRQ_FLAGS 0       /* IRQ flags */

int pulses = 0;     // Cantidad de pulsos acumulados del encoder

#define SW0_NODE	DT_ALIAS(ch0)  // Declaraciones del canal A del encoder

#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
#define SW0_GPIO_LABEL	DT_GPIO_LABEL(SW0_NODE, gpios)
#define SW0_GPIO_PIN	DT_GPIO_PIN(SW0_NODE, gpios)
#define SW0_GPIO_FLAGS	(GPIO_INPUT | DT_GPIO_FLAGS(SW0_NODE, gpios))
#else
#error "Unsupported board: ch0 devicetree alias is not defined"
#define SW0_GPIO_LABEL	""
#define SW0_GPIO_PIN	0
#define SW0_GPIO_FLAGS	0
#endif

#define SW1_NODE	DT_ALIAS(ch1) // Declaraciones del canal B del encoder

#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
#define SW1_GPIO_LABEL	DT_GPIO_LABEL(SW1_NODE, gpios)
#define SW1_GPIO_PIN	DT_GPIO_PIN(SW1_NODE, gpios)
#define SW1_GPIO_FLAGS	(GPIO_INPUT | DT_GPIO_FLAGS(SW1_NODE, gpios))
#else
#error "Unsupported board: ch1 devicetree alias is not defined"
#define SW1_GPIO_LABEL	""
#define SW1_GPIO_PIN	0
#define SW1_GPIO_FLAGS	0
#endif

// Variables y funciones a usar para las interrupciones del encoder

const struct device *chA, *chB;
static struct gpio_callback chA_cb_data, chB_cb_data;
gpio_port_value_t *values;
uint16_t values_int;
uint8_t old_a, new_a, old_b, new_b;
extern void update_encoder();
extern void decode_encoder();
extern void encoder_irq_a();
extern void encoder_irq_b();

int test;

const struct device *bl, *bh;
uint16_t b_values;

// Variables para el PWM

#define PWM_MOTOR_NODE	DT_ALIAS(pwm_motor)

#if DT_NODE_HAS_STATUS(PWM_MOTOR_NODE, okay)
#define PWM_LABEL	DT_PWMS_LABEL(PWM_MOTOR_NODE)
#define PWM_CHANNEL	DT_PWMS_CHANNEL(PWM_MOTOR_NODE)
#define PWM_FLAGS	DT_PWMS_FLAGS(PWM_MOTOR_NODE)
#else
#error "Unsupported board: pwm-motor devicetree alias is not defined"
#define PWM_LABEL	""
#define PWM_CHANNEL	0
#define PWM_FLAGS	0
#endif



#define MPOS_NODE	DT_ALIAS(ina) // Dirección positiva del motor

#if DT_NODE_HAS_STATUS(MPOS_NODE, okay)
#define MPOS_GPIO_LABEL	DT_GPIO_LABEL(MPOS_NODE, gpios)
#define MPOS_GPIO_PIN	DT_GPIO_PIN(MPOS_NODE, gpios)
#define MPOS_GPIO_FLAGS	(GPIO_INPUT | DT_GPIO_FLAGS(MPOS_NODE, gpios))
#else
#error "Unsupported board: INA devicetree alias is not defined"
#define MPOS_GPIO_LABEL	""
#define MPOS_GPIO_PIN	0
#define MPOS_GPIO_FLAGS	0
#endif

#define MNEG_NODE	DT_ALIAS(inb) // Dirección negativa del motor

#if DT_NODE_HAS_STATUS(MNEG_NODE, okay)
#define MNEG_GPIO_LABEL	DT_GPIO_LABEL(MNEG_NODE, gpios)
#define MNEG_GPIO_PIN	DT_GPIO_PIN(MNEG_NODE, gpios)
#define MNEG_GPIO_FLAGS	(GPIO_INPUT | DT_GPIO_FLAGS(MNEG_NODE, gpios))
#else
#error "Unsupported board: INB devicetree alias is not defined"
#define MNEG_GPIO_LABEL	""
#define MNEG_GPIO_PIN	0
#define MNEG_GPIO_FLAGS	0
#endif

#define ENMOT_NODE	DT_ALIAS(enmot) // Dirección negativa del motor

#if DT_NODE_HAS_STATUS(ENMOT_NODE, okay)
#define ENMOT_GPIO_LABEL	DT_GPIO_LABEL(ENMOT_NODE, gpios)
#define ENMOT_GPIO_PIN	DT_GPIO_PIN(ENMOT_NODE, gpios)
#define ENMOT_GPIO_FLAGS	(GPIO_INPUT | DT_GPIO_FLAGS(ENMOT_NODE, gpios))
#else
#error "Unsupported board: enmot devicetree alias is not defined"
#define ENMOT_GPIO_LABEL	""
#define ENMOT_GPIO_PIN	0
#define ENMOT_GPIO_FLAGS	0
#endif

const struct device *motPos, *motNeg, *motEn, *pwm;
uint32_t pulse_width = 0U;
uint32_t period_usec = 50;
extern void configure_motor();
extern void motor_run();
extern void position_control();
int position_ref=10000;
float kp = 1;
float u = 0;

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
                uart_fifo_read(dev, &recvData, 1);
		uart_irq_rx_disable(uart_dev);
                //printk("%c", recvData);
		sprintf(buffer_read,"%s%c",buffer_read,recvData);

                if ((recvData == '\n') || (recvData == '\r')) {
			printk("========== %s\n",buffer_read);
			buffer_read[0]=0;
                }
        }

}

// Funciones para la interrupción por tiempo

void my_expiry_function(struct k_timer *dummy) // Se encarga de agregar la tarea al workqueue
{
    k_work_submit(&my_work);
}

void my_work_handler(struct k_work *work) // Función de interrupción por tiempo. 
{
	// Aquí debería realizarse el muestreo de sensores y el control
	stop_time = k_cycle_get_32();
	cycles_spent = stop_time - start_time;
	nanoseconds_spent = cycles_spent/168000;
	start_time = k_cycle_get_32();
	position_control();

}

// Funciones para la interrupción por encoder

void encoder_irq_a(){
	// Se utiliza solo un canal para decodificar la dirección
	update_encoder();
	decode_encoder();
}

void encoder_irq_b(){
	// La interrupción por el canal B solo actualiza los valores de los canales
	update_encoder();
}

void update_encoder(struct device *channel){
	// Se actualizan los valores de los pines
	// La captura de valores viejos y nuevos sirve para un tipo de decodificación más completa
	//  				pero que al momento no se implementará hasta que se considere necesario
	old_b = new_b;
	old_a = new_a;
	new_b = gpio_pin_get(chA, SW0_GPIO_PIN);
	new_a = gpio_pin_get(chB, SW1_GPIO_PIN);
}

void decode_encoder(){
	// Encoders en cuadratura. Señales desfasadas 90° entre sí
	// Cuando ocure la interrupción por flanco de subida en A, 
	//			se verifica el valor de B. Dependiendo de la
	//  		dirección, estará en alto o en bajo.
	 if (new_a == 1)    {
         if (new_b == 0) pulses++;    
	      else pulses--;         }      
         else     {        
       		if (new_b == 0) pulses--;      
          else pulses++;     } 
}

void configure_motor(){
	int ret;

	pwm = device_get_binding(PWM_LABEL);
	if (!pwm) {
		printk("Error: didn't find %s device\n", PWM_LABEL);
		return;
	}


	motPos = device_get_binding(MPOS_GPIO_LABEL);
	if (motPos == NULL) {
		printk("Error: didn't find %s device\n", MPOS_GPIO_LABEL);
		return;
	}
	ret = gpio_pin_configure(motPos, MPOS_GPIO_PIN, GPIO_OUTPUT_ACTIVE | MPOS_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, MPOS_GPIO_LABEL, MPOS_GPIO_PIN);
		return;
	}

	motNeg = device_get_binding(MNEG_GPIO_LABEL);
	if (motNeg == NULL) {
		printk("Error: didn't find %s device\n", MNEG_GPIO_LABEL);
		return;
	}
	ret = gpio_pin_configure(motNeg, MNEG_GPIO_PIN, GPIO_OUTPUT_ACTIVE | MNEG_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, MNEG_GPIO_LABEL, MNEG_GPIO_PIN);
		return;
	}

	motEn = device_get_binding(ENMOT_GPIO_LABEL);
	if (motEn == NULL) {
		printk("Error: didn't find %s device\n", ENMOT_GPIO_LABEL);
		return;
	}
	ret = gpio_pin_configure(motEn, ENMOT_GPIO_PIN, GPIO_OUTPUT_ACTIVE | ENMOT_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, ENMOT_GPIO_LABEL, ENMOT_GPIO_PIN);
		return;
	}

	// Habilitar el pin del enable del driver
	gpio_pin_set(motEn,ENMOT_GPIO_PIN,1);
}

void motor_run(int dutycycle){
	int ret;
	if (dutycycle <0){
		dutycycle= dutycycle*(-1);
		gpio_pin_set(motNeg, MNEG_GPIO_PIN, 1);
		gpio_pin_set(motPos, MPOS_GPIO_PIN, 0);
	}
	else{
		gpio_pin_set(motNeg, MNEG_GPIO_PIN, 1);
		gpio_pin_set(motPos, MPOS_GPIO_PIN, 0);
	}

	//pulse_width= 50*dutycycle/100;
	pulse_width= (uint32_t) 25U;
	ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, period_usec, pulse_width, PWM_FLAGS);

	if (ret) {
		printk("Error %d: failed to set pulse width\n", ret);
		return;
	}
}

void position_control(){
	float error;
	error = position_ref-pulses;
	u = kp*error;
	motor_run(u);
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

////////////////// Encoder init ////////////////////////////
	
	////// Channel A

	chA = device_get_binding(SW0_GPIO_LABEL);
	if (chA == NULL) {
		printk("Error: didn't find %s device\n", SW0_GPIO_LABEL);
		return;
	}

	ret = gpio_pin_configure(chA, SW0_GPIO_PIN, SW0_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}

	ret = gpio_pin_interrupt_configure(chA,
					   SW0_GPIO_PIN,
					   GPIO_INT_EDGE_RISING);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}

	gpio_init_callback(&chA_cb_data, encoder_irq_a, BIT(SW0_GPIO_PIN));
	gpio_add_callback(chA, &chA_cb_data);
	printk("Set up button at %s pin %d\n", SW0_GPIO_LABEL, SW0_GPIO_PIN);

	////// Channel B

	chB = device_get_binding(SW1_GPIO_LABEL);
	if (chB == NULL) {
		printk("Error: didn't find %s device\n", SW1_GPIO_LABEL);
		return;
	}

	ret = gpio_pin_configure(chB, SW1_GPIO_PIN, SW1_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, SW1_GPIO_LABEL, SW1_GPIO_PIN);
		return;
	}

	ret = gpio_pin_interrupt_configure(chB,
					   SW1_GPIO_PIN,
					   GPIO_INT_EDGE_RISING);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, SW1_GPIO_LABEL, SW1_GPIO_PIN);
		return;
	}

	gpio_init_callback(&chB_cb_data, encoder_irq_b, BIT(SW1_GPIO_PIN));
	gpio_add_callback(chB, &chB_cb_data);
	printk("Set up button at %s pin %d\n", SW1_GPIO_LABEL, SW1_GPIO_PIN);


/////////////////// Timer init //////////////////////////////

	printk("Hello World Again! \n");
	start_time = k_cycle_get_32();		// Captura del número de ciclos de reloj actuales
	k_timer_init(&my_timer, my_expiry_function, NULL);
	k_timer_start(&my_timer, K_MSEC(sample_time), K_MSEC(sample_time));

/////////////////// Motor init //////////////////////////////
	configure_motor();

///////////////// While infinito ///////////////////////////	

	while (1) {
		gpio_pin_set(dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		k_msleep(SLEEP_TIME);

		//printk("Clock ");
		//printk("%d\n",nanoseconds_spent);
		

		printk("Count %d Pin %d \n ", pulses, test);
		
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

