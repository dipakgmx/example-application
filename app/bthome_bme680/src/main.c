/*
 * Copyright (c) 2025 Dipak Shetty <shetty.dipak@gmx.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "sensor_handler.h"

#include <zephyr/bluetooth/bluetooth.h>

int main(void)
{
	int err;

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized successfully\n");
	err = init_sensor();

	if (err) {
		printk("Sensor initialization failed (err %d)\n", err);
		return 0;
	}
	return 0;
}
