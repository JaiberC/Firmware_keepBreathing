/*
 * Copyright (c) 2020 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/adc.h>

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define ADC_NUM_CHANNELS	DT_PROP_LEN(DT_PATH(zephyr_user), io_channels)

#if ADC_NUM_CHANNELS > 3
#error "Currently only 1 2 or 3 channels supported in this sample"
#endif

#if ADC_NUM_CHANNELS == 2 && !DT_SAME_NODE( \
	DT_PHANDLE_BY_IDX(DT_PATH(zephyr_user), io_channels, 0), \
	DT_PHANDLE_BY_IDX(DT_PATH(zephyr_user), io_channels, 1))
#error "Channels have to use the same ADC."
#endif

#define ADC_NODE		DT_PHANDLE(DT_PATH(zephyr_user), io_channels)

/* Common settings supported by most ADCs */
#define ADC_RESOLUTION		12
#define ADC_GAIN		ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT

/* Get the numbers of up to three channels */
static uint8_t channel_ids[ADC_NUM_CHANNELS] = {
	DT_IO_CHANNELS_INPUT_BY_IDX(DT_PATH(zephyr_user), 0),
#if ADC_NUM_CHANNELS == 2
	DT_IO_CHANNELS_INPUT_BY_IDX(DT_PATH(zephyr_user), 1)
#endif
#if ADC_NUM_CHANNELS == 3
	DT_IO_CHANNELS_INPUT_BY_IDX(DT_PATH(zephyr_user), 1),
	DT_IO_CHANNELS_INPUT_BY_IDX(DT_PATH(zephyr_user), 2)
#endif
};

static int16_t sample_buffer[ADC_NUM_CHANNELS];

struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	/* channel ID will be overwritten below */
	.channel_id = 0,
	.differential = 0
};

struct adc_sequence sequence = {
	/* individual channels will be added below */
	.channels    = 0,
	.buffer      = sample_buffer,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(sample_buffer),
	.resolution  = ADC_RESOLUTION,
};

void main(void)
{

	int err;
	const struct device *dev_adc = DEVICE_DT_GET(ADC_NODE);

	if (!device_is_ready(dev_adc)) {
		printk("ADC device not found\n");
		return;
	}
	adc_channel_setup(dev_adc, &channel_cfg);


	int32_t adc_vref = adc_ref_internal(dev_adc);
	printk("ADC vref %d.\n", adc_vref);
	
	while (1) {
	

		printk("ADC reading:");
		for (uint8_t i = 0; i < ADC_NUM_CHANNELS; i++) {
			channel_cfg.channel_id = channel_ids[i];
			sequence.channels = BIT(channel_ids[i]);
			err = adc_read(dev_adc, &sequence);
			if (err != 0) {
				printk("ADC reading failed with error %d.\n", err);
			return;
			}
	
			int16_t raw_value = sample_buffer[0];

			printk(" %d", raw_value);
			if (adc_vref > 0) {
				/*
				 * Convert raw reading to millivolts if driver
				 * supports reading of ADC reference voltage
				 */
				int32_t mv_value = raw_value;

				adc_raw_to_millivolts(adc_vref, ADC_GAIN,
					ADC_RESOLUTION, &mv_value);
				printk(" = %d mV  ", mv_value);
			}
		}
		printk("\n");

		k_sleep(K_MSEC(1000));
	}
}
