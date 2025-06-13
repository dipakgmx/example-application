/*
 * Copyright (c) 2025 Dipak Shetty <shetty.dipak@gmx.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <drivers/bme68x_iaq.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include "sensor_handler.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_handler);

#define SERVICE_UUID 0xFCD2

#define SERVICE_DATA_TEMP_HUM_LEN  9
#define SERVICE_DATA_PRESS_GAS_LEN 7
#define SERVICE_DATA_VOC_LEN       9

#define IDX_TEMPL  4 /* Index of lo byte of temp in service data*/
#define IDX_TEMPH  5 /* Index of hi byte of temp in service data*/
#define IDX_HUMIDL 7 /* Index of lo byte of humidity in service data*/
#define IDX_HUMIDH 8 /* Index of hi byte of humidity in service data*/

#define IDX_PRESS_LOW  4 /* Index of lo byte of pressure in service data*/
#define IDX_PRESS_MID  5 /* Index of mid byte of pressure in service data*/
#define IDX_PRESS_HIGH 6 /* Index of hi byte of pressure in service data*/

#define IDX_VOC_LOW  4 /* Index of lo byte of VOC in service data*/
#define IDX_VOC_HIGH 5 /* Index of hi byte of VOC in service data*/
#define IDX_CO2_HIGH 7 /* Index of hi byte of CO2 in service data*/
#define IDX_CO2_LOW  8 /* Index of lo byte of CO2 in service data*/

/* Forward declarations for work handlers */
static void sensor_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(sensor_advertise_work, sensor_work_handler);
#define SENSOR_WORK_DELAY K_SECONDS(3)

/* First packet: Temperature and Humidity */
static uint8_t service_data_temp_hum[SERVICE_DATA_TEMP_HUM_LEN] = {
	BT_UUID_16_ENCODE(SERVICE_UUID),
	0x40, // BTHome format - unencrypted, version 2
	0x02, // Temperature ID
	0x00, // Placeholder for temperature value
	0x00, // Placeholder for temperature value
	0x03, // Humidity ID
	0x00, // Placeholder for humidity value
	0x00, // Placeholder for humidity value
};

/* Second packet: Pressure */
static uint8_t service_data_press[SERVICE_DATA_PRESS_GAS_LEN] = {
	BT_UUID_16_ENCODE(SERVICE_UUID),
	0x40, // BTHome format - unencrypted, version 2
	0x04, // Pressure ID
	0x00, // Placeholder for pressure value
	0x00, // Placeholder for pressure value
	0x00, // Pressure value placeholder
};

/* Third packet: VOC and CO2 */
static uint8_t service_data_voc_co2[SERVICE_DATA_VOC_LEN] = {
	BT_UUID_16_ENCODE(SERVICE_UUID),
	0x40, // BTHome format - unencrypted, version 2
	0x13, // VOC ID
	0x00, // Placeholder for VOC value
	0x00, // Placeholder for VOC value
	0x12, // CO2 ID
	0x00, // Placeholder for CO2 value
	0x00 // Placeholder for CO2 value
};

#define ADV_PARAM                                                                                  \
BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_SLOW_INT_MIN,                       \
BT_GAP_ADV_SLOW_INT_MAX, NULL)

/* Advertising data sets */
static struct bt_data ad_temp_hum[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA(BT_DATA_SVC_DATA16, service_data_temp_hum, sizeof(service_data_temp_hum))
};

static struct bt_data ad_press_gas[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA(BT_DATA_SVC_DATA16, service_data_press, sizeof(service_data_press))
};

static struct bt_data ad_voc[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA(BT_DATA_SVC_DATA16, service_data_voc_co2, sizeof(service_data_voc_co2))
};

/* State variables */
const struct device *sensor_dev = DEVICE_DT_GET(DT_ALIAS(gas_sensor));

/* Initialize sensor device */
int init_sensor(void)
{
	int err;

	sensor_dev = DEVICE_DT_GET(DT_ALIAS(gas_sensor));

	if (!device_is_ready(sensor_dev)) {
		LOG_ERR("Sensor device not ready");
		return -ENODEV;
	}

	LOG_INF("Sensor device %p name is %s", sensor_dev, sensor_dev->name);

	/* Start advertising */
	err = bt_le_adv_start(ADV_PARAM, ad_temp_hum, ARRAY_SIZE(ad_temp_hum), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

	/* Schedule initial sensor reading */
	k_work_schedule(&sensor_advertise_work, K_NO_WAIT);
	return 0;
}

/* Work handler for sensor data collection */
static void sensor_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	int err;
	struct sensor_value temp;
	struct sensor_value press;
	struct sensor_value humidity;
	struct sensor_value iaq;
	struct sensor_value co2;
	struct sensor_value voc;


	/* Fetch and process sensor data */
	err = sensor_sample_fetch(sensor_dev);
	if (err) {
		LOG_ERR("Failed to fetch sensor sample: %d", err);
		/* Schedule next sensor reading */
		k_work_schedule(&sensor_advertise_work, SENSOR_WORK_DELAY);
	}

	sensor_channel_get(sensor_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	sensor_channel_get(sensor_dev, SENSOR_CHAN_PRESS, &press);
	sensor_channel_get(sensor_dev, SENSOR_CHAN_HUMIDITY, &humidity);
	sensor_channel_get(sensor_dev, SENSOR_CHAN_IAQ, &iaq);
	sensor_channel_get(sensor_dev, SENSOR_CHAN_CO2, &co2);
	sensor_channel_get(sensor_dev, SENSOR_CHAN_VOC, &voc);

	LOG_INF("temp: %d.%06d; press: %d.%06d; humidity: %d.%06d; iaq: %d; CO2: %d.%06d; VOC: %d.%06d",
		temp.val1, temp.val2, press.val1, press.val2, humidity.val1, humidity.val2,
		iaq.val1, co2.val1, co2.val2, voc.val1, voc.val2);

	/* Update advertising data with new sensor values */

	/* Temperature and Humidity data */
	int16_t temp_scaled = temp.val1 * 100 + temp.val2 / 10000;
	service_data_temp_hum[IDX_TEMPH] = (temp_scaled >> 8) & 0xFF;
	service_data_temp_hum[IDX_TEMPL] = temp_scaled & 0xFF;

	int16_t humidity_scaled = humidity.val1 * 100 + humidity.val2 / 10000;
	service_data_temp_hum[IDX_HUMIDH] = (humidity_scaled >> 8) & 0xFF;
	service_data_temp_hum[IDX_HUMIDL] = humidity_scaled & 0xFF;

	/* Pressure data */
	int32_t pressure_scaled = (press.val1 / 100) * 100 + (press.val1 % 100 + press.val2 / 1000000) / 10;
	service_data_press[IDX_PRESS_LOW] = pressure_scaled & 0xFF;
	service_data_press[IDX_PRESS_MID] = (pressure_scaled >> 8) & 0xFF;
	service_data_press[IDX_PRESS_HIGH] = (pressure_scaled >> 16) & 0xFF;

	/* VOC and CO2 data */
	uint16_t voc_scaled = voc.val1 * 100 + voc.val2 / 10000;
	service_data_voc_co2[IDX_VOC_LOW] = voc_scaled & 0xFF;
	service_data_voc_co2[IDX_VOC_HIGH] = (voc_scaled >> 8) & 0xFF;

	uint16_t co2_scaled = co2.val1 + (co2.val2 / 1000000);
	service_data_voc_co2[IDX_CO2_HIGH] = (co2_scaled >> 8) & 0xFF;
	service_data_voc_co2[IDX_CO2_LOW] = co2_scaled & 0xFF;

	err = bt_le_adv_update_data(ad_temp_hum, ARRAY_SIZE(ad_temp_hum), NULL, 0);
	if (err) {
		printk("Failed to update ad_temp_hum advertising data (err %d: %s)\n", err,
		       strerror(-err));
	}
	// Wait so this data can be advertised for some time
	k_sleep(K_MSEC(BT_GAP_ADV_SLOW_INT_MIN));

	err = bt_le_adv_update_data(ad_press_gas, ARRAY_SIZE(ad_press_gas), NULL, 0);
	if (err) {
		printk("Failed to update ad_press_gas advertising data (err %d: %s)\n", err,
		       strerror(-err));
	}

	// Wait so this data can be advertised for some time
	k_sleep(K_MSEC(BT_GAP_ADV_SLOW_INT_MIN));
	err = bt_le_adv_update_data(ad_voc, ARRAY_SIZE(ad_voc), NULL, 0);
	if (err) {
		printk("Failed to update ad_voc advertising data (err %d: %s)\n", err,
		       strerror(-err));
	}

	/* Schedule next sensor reading */
	k_work_schedule(&sensor_advertise_work, SENSOR_WORK_DELAY);
}
