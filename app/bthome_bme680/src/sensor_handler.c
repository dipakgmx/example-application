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

/* Maximum supported AD data length, use a value supported by the Controller,
 * Bluetooth Core Specification define minimum of 31 bytes will be supported by
 * all Controllers, can be a maximum of 1650 bytes when supported.
 */
#if defined(CONFIG_BT_CTLR_ADV_DATA_LEN_MAX)
#define BT_AD_DATA_LEN_MAX CONFIG_BT_CTLR_ADV_DATA_LEN_MAX
#else
#define BT_AD_DATA_LEN_MAX 31U
#endif

#define SERVICE_DATA_LEN        9
#define SERVICE_UUID            0xfcd2      /* BTHome service UUID */
#define IDX_TEMPL  4 /* Index of lo byte of temp in service data*/
#define IDX_TEMPH  5 /* Index of hi byte of temp in service data*/

/* Combined service data structure for all sensor values in BTHome format */
static uint8_t service_data_combined[19] = {
	BT_UUID_16_ENCODE(SERVICE_UUID),
	0x40, // BTHome format - unencrypted, version 2
	0x02, // Temperature ID
	0x00,
	0x00,
	0x03, // Humidity ID
	0x00,
	0x00,
	0x04, // Pressure ID
	0x00,
	0x00,
	0x00,
	0x13, // VOC ID
	0x00,
	0x00,
	0x12, // CO2 ID
	0x00,
	0x00 // Placeholder for CO2 value
};

/* Index positions for updating sensor values in service data */
#define IDX_TEMP_L  4  /* Index of lo byte of temp */
#define IDX_TEMP_H  5  /* Index of hi byte of temp */
#define IDX_HUMID_L 7  /* Index of lo byte of humidity */
#define IDX_HUMID_H 8  /* Index of hi byte of humidity */
#define IDX_PRESS_L 10 /* Index of lo byte of pressure */
#define IDX_PRESS_M 11 /* Index of mid byte of pressure */
#define IDX_PRESS_H 12 /* Index of hi byte of pressure */
#define IDX_VOC_L   14 /* Index of lo byte of VOC */
#define IDX_VOC_H   15 /* Index of hi byte of VOC */
#define IDX_CO2_L   17 /* Index of lo byte of CO2 */
#define IDX_CO2_H   18 /* Index of hi byte of CO2 */

/* Forward declarations for work handlers */
static void sensor_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(sensor_advertise_work, sensor_work_handler);
#define SENSOR_WORK_DELAY K_SECONDS(3)

/* Advertising sets for extended advertising */
static struct bt_le_ext_adv *adv_set;

/* Combined advertising data set */
static const struct bt_data ad_combined[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA(BT_DATA_SVC_DATA16, service_data_combined, sizeof(service_data_combined))};

/* State variables */
const struct device *sensor_dev = DEVICE_DT_GET(DT_ALIAS(gas_sensor));

/* Initialize sensor device */
int init_sensor(void)
{
	int err;
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0U, /* Supply unique SID when creating advertising set */
		.secondary_max_skip = 0U,
		.options = BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_CONN,
		.interval_min = BT_GAP_ADV_SLOW_INT_MIN,
		.interval_max = BT_GAP_ADV_SLOW_INT_MAX,
		.peer = NULL,
	};

	sensor_dev = DEVICE_DT_GET(DT_ALIAS(gas_sensor));

	if (!device_is_ready(sensor_dev)) {
		LOG_ERR("Sensor device not ready");
		return -ENODEV;
	}

	LOG_INF("Sensor device %p name is %s", sensor_dev, sensor_dev->name);

	/* Create a non-connectable advertising set */
	err = bt_le_ext_adv_create(&adv_param, NULL, &adv_set);
	if (err) {
		LOG_ERR("Failed to create advertising set (err %d)", err);
		return err;
	}

	/* Set extended advertising data */
	err = bt_le_ext_adv_set_data(adv_set, ad_combined, ARRAY_SIZE(ad_combined), NULL, 0);
	if (err) {
		LOG_ERR("Failed to set advertising data (err %d)", err);
		return err;
	}

	/* Start extended advertising set */
	err = bt_le_ext_adv_start(adv_set, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		LOG_ERR("Failed to start extended advertising (err %d)", err);
		return err;
	}

	LOG_INF("Extended advertising started successfully");

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
		goto reschedule;
	}

	sensor_channel_get(sensor_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	sensor_channel_get(sensor_dev, SENSOR_CHAN_PRESS, &press);
	sensor_channel_get(sensor_dev, SENSOR_CHAN_HUMIDITY, &humidity);
	sensor_channel_get(sensor_dev, SENSOR_CHAN_IAQ, &iaq);
	sensor_channel_get(sensor_dev, SENSOR_CHAN_CO2, &co2);
	sensor_channel_get(sensor_dev, SENSOR_CHAN_VOC, &voc);

	if (iaq.val1 == 0 || co2.val1 < 0 || voc.val1 < 0) {
		LOG_ERR("Invalid sensor values received, skipping update");
		goto reschedule;
	}

	LOG_INF("temp: %d.%06d; press: %d.%06d; humidity: %d.%06d; iaq: %d; CO2: %d.%06d; VOC: "
		"%d.%06d",
		temp.val1, temp.val2, press.val1, press.val2, humidity.val1, humidity.val2,
		iaq.val1, co2.val1, co2.val2, voc.val1, voc.val2);

	/* Update all sensor data in the combined structure */

	/* Temperature */
	int16_t temp_scaled = temp.val1 * 100 + temp.val2 / 10000;
	service_data_combined[IDX_TEMP_H] = (temp_scaled >> 8) & 0xFF;
	service_data_combined[IDX_TEMP_L] = temp_scaled & 0xFF;

	/* Humidity */
	int16_t humidity_scaled = humidity.val1 * 100 + humidity.val2 / 10000;
	service_data_combined[IDX_HUMID_H] = (humidity_scaled >> 8) & 0xFF;
	service_data_combined[IDX_HUMID_L] = humidity_scaled & 0xFF;

	/* Pressure */
	int32_t pressure_scaled =
		(press.val1 / 100) * 100 + (press.val1 % 100 + press.val2 / 1000000) / 10;
	service_data_combined[IDX_PRESS_L] = pressure_scaled & 0xFF;
	service_data_combined[IDX_PRESS_M] = (pressure_scaled >> 8) & 0xFF;
	service_data_combined[IDX_PRESS_H] = (pressure_scaled >> 16) & 0xFF;

	/* VOC */
	uint16_t voc_scaled = voc.val1 * 100 + voc.val2 / 10000;
	service_data_combined[IDX_VOC_L] = voc_scaled & 0xFF;
	service_data_combined[IDX_VOC_H] = (voc_scaled >> 8) & 0xFF;

	/* CO2 */
	uint16_t co2_scaled = co2.val1 + (co2.val2 / 1000000);
	service_data_combined[IDX_CO2_H] = (co2_scaled >> 8) & 0xFF;
	service_data_combined[IDX_CO2_L] = co2_scaled & 0xFF;

	/* Update the extended advertising data with all sensor values at once */
	err = bt_le_ext_adv_set_data(adv_set, ad_combined, ARRAY_SIZE(ad_combined), NULL, 0);
	if (err) {
		LOG_ERR("Failed to update extended advertising data (err %d)", err);
	}

reschedule:
	/* Schedule next sensor reading */
	k_work_schedule(&sensor_advertise_work, SENSOR_WORK_DELAY);
}
