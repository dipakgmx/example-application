/*
 * Copyright (c) 2025 Dipak Shetty <shetty.dipak@gmx.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>

#define SERVICE_UUID 0xfcd2

#define SERVICE_DATA_TEMP_HUM_LEN  9
#define SERVICE_DATA_PRESS_GAS_LEN 7

#define IDX_TEMPL      4 /* Index of lo byte of temp in service data*/
#define IDX_TEMPH      5 /* Index of hi byte of temp in service data*/
#define IDX_HUMIDL     7 /* Index of lo byte of humidity in service data*/
#define IDX_HUMIDH     8 /* Index of hi byte of humidity in service data*/
#define IDX_PRESS_LOW  4 /* Index of lo byte of pressure in service data*/
#define IDX_PRESS_MID  5 /* Index of mid byte of pressure in service data*/
#define IDX_PRESS_HIGH 6 /* Index of hi byte of pressure in service data*/

#define ADV_PARAM                                                                                  \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY, BT_GAP_ADV_SLOW_INT_MIN,                       \
			BT_GAP_ADV_SLOW_INT_MAX, NULL)

// First packet: Temperature and Humidity
static uint8_t service_data_temp_hum[SERVICE_DATA_TEMP_HUM_LEN] = {
	BT_UUID_16_ENCODE(SERVICE_UUID),
	0x40, // BTHome format - unencrypted, version 2
	0x02, // Temperature ID
	0xc4,
	0x00, // Temperature value placeholder
	0x03, // Humidity ID
	0xbf,
	0x13, // Humidity value placeholder
};

// Second packet: Pressure and Gas resistance
static uint8_t service_data_press_gas[SERVICE_DATA_PRESS_GAS_LEN] = {
	BT_UUID_16_ENCODE(SERVICE_UUID),
	0x40, // BTHome format - unencrypted, version 2
	0x04, // Pressure ID
	0x13,
	0x8A,
	0x01 // Pressure value placeholder
};

// Two advertising datasets
static struct bt_data ad_temp_hum[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA(BT_DATA_SVC_DATA16, service_data_temp_hum, sizeof(service_data_temp_hum))};

static struct bt_data ad_press_gas[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA(BT_DATA_SVC_DATA16, service_data_press_gas, sizeof(service_data_press_gas))};

int main(void)
{
	int err;

	const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(gas_sensor));
	struct sensor_value temp;
	struct sensor_value press;
	struct sensor_value humidity;
	struct sensor_value gas_res;

	printk("Starting BTHome sensor template\n");

	if (!device_is_ready(dev)) {
		printk("sensor: device not ready.\n");
		return 0;
	}

	printf("Device %p name is %s\n", dev, dev->name);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	/* Start advertising */
	err = bt_le_adv_start(ADV_PARAM, ad_temp_hum, ARRAY_SIZE(ad_temp_hum), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

	for (;;) {
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);
		sensor_channel_get(dev, SENSOR_CHAN_GAS_RES, &gas_res);

		printf("T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d\n", temp.val1, temp.val2,
		       press.val1, press.val2, humidity.val1, humidity.val2, gas_res.val1,
		       gas_res.val2);

		// Convert temperature to BTHome format (0.01 degree resolution)
		int16_t temp_scaled = temp.val1 * 100 + temp.val2 / 10000;
		service_data_temp_hum[IDX_TEMPH] = (temp_scaled >> 8) & 0xFF; // High byte
		service_data_temp_hum[IDX_TEMPL] = temp_scaled & 0xFF;        // Low byte

		// Convert humidity to BTHome format (0.01% resolution)
		int16_t humidity_scaled = humidity.val1 * 100 + humidity.val2 / 10000;
		service_data_temp_hum[IDX_HUMIDH] = (humidity_scaled >> 8) & 0xFF;
		service_data_temp_hum[IDX_HUMIDL] = humidity_scaled & 0xFF;

		// Convert pressure to BTHome format (0.01 hPa resolution)
		int32_t pressure_scaled = press.val1 * 1000 + press.val2 / 10000;
		service_data_press_gas[IDX_PRESS_LOW] = pressure_scaled & 0xFF;        // Low byte
		service_data_press_gas[IDX_PRESS_MID] = (pressure_scaled >> 8) & 0xFF; // Mid byte
		service_data_press_gas[IDX_PRESS_HIGH] =
			(pressure_scaled >> 16) & 0xFF; // High byte

		err = bt_le_adv_update_data(ad_temp_hum, ARRAY_SIZE(ad_temp_hum), NULL, 0);
		if (err) {
			printk("Failed to update advertising data (err %d: %s)\n", err,
			       strerror(-err));
		}
		// Wait so this data can be advertised for some time
		k_sleep(K_MSEC(BT_GAP_ADV_SLOW_INT_MIN));

		err = bt_le_adv_update_data(ad_press_gas, ARRAY_SIZE(ad_press_gas), NULL, 0);
		if (err) {
			printk("Failed to update advertising data (err %d: %s)\n", err,
			       strerror(-err));
		}

		k_sleep(K_MINUTES(5));
	}
	return 0;
}
