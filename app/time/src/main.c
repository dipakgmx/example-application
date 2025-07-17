/*
 * Copyright (c) 2025, Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/cts.h>

#include <app_version.h>

const struct device *const rtc = DEVICE_DT_GET(DT_ALIAS(rtc));

static bool cts_ntf_enabled;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_CTS_VAL)),
#if defined(CONFIG_BT_EXT_ADV)
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
#endif /* CONFIG_BT_EXT_ADV */
};

#if !defined(CONFIG_BT_EXT_ADV)
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};
#endif /* !CONFIG_BT_EXT_ADV */

enum {
	STATE_CONNECTED,
	STATE_DISCONNECTED,
	STATE_BITS,
};

static ATOMIC_DEFINE(state, STATE_BITS);

static void adv_work_handler(struct k_work *work)
{
	int err;
#if !defined(CONFIG_BT_EXT_ADV)
	printk("Starting Legacy Advertising (connectable and scannable)\n");
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

#else  /* CONFIG_BT_EXT_ADV */
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0U,
		.secondary_max_skip = 0U,
		.options = (BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_CODED),
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
		.peer = NULL,
	};
	struct bt_le_ext_adv *adv;

	printk("Creating a Coded PHY connectable non-scannable advertising set\n");
	err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
	if (err) {
		printk("Failed to create Coded PHY extended advertising set (err %d)\n", err);

		printk("Creating a non-Coded PHY connectable non-scannable advertising set\n");
		adv_param.options &= ~BT_LE_ADV_OPT_CODED;
		err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
		if (err) {
			printk("Failed to create extended advertising set (err %d)\n", err);
			return 0;
		}
	}

	printk("Setting extended advertising data\n");
	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set extended advertising data (err %d)\n", err);
		return 0;
	}

	printk("Starting Extended Advertising (connectable non-scannable)\n");
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising set (err %d)\n", err);
		return 0;
	}
#endif /* CONFIG_BT_EXT_ADV */

	printk("Advertising successfully started\n");
}

static K_WORK_DEFINE(adv_work, adv_work_handler);

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	} else {
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
		printk("Connected %s\n", addr);

		(void)atomic_set_bit(state, STATE_CONNECTED);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Disconnected from %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

	(void)atomic_set_bit(state, STATE_DISCONNECTED);
}

static void recycled_cb(void)
{
	printk("Connection object available from previous conn. Disconnect is complete!\n");
	k_work_submit(&adv_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled_cb,
};

void notify_change(bool enabled)
{
	cts_ntf_enabled = enabled;

	if (cts_ntf_enabled) {
		printk("Time change notifications enabled\n");
	} else {
		printk("Time change notifications disabled\n");
	}
}

int cts_time_write(struct bt_cts_time_format *cts_time)
{
	struct rtc_time tm = {
		.tm_year = cts_time->year - 1900,
		.tm_mon = cts_time->mon - 1,
		.tm_mday = cts_time->mday,
		.tm_hour = cts_time->hours,
		.tm_min = cts_time->min,
		.tm_sec = cts_time->sec,
	};

	printf("Writing time: %04d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1,
	       tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	return rtc_set_time(rtc, &tm);
}

int cts_time_read(struct bt_cts_time_format *cts_time)
{
	int ret = 0;
	struct rtc_time tm;

	ret = rtc_get_time(rtc, &tm);
	if (ret < 0) {
		printk("Cannot read date time: %d\n", ret);
		return ret;
	}
	printk("Current time: %04d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1,
	       tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	cts_time->year = tm.tm_year + 1900;
	cts_time->mon = tm.tm_mon + 1; /* months starting from 1 */
	cts_time->mday = tm.tm_mday;   /* Day of month */
	cts_time->hours = tm.tm_hour;  /* hours */
	cts_time->min = tm.tm_min;     /* minutes */
	cts_time->sec = tm.tm_sec;     /* seconds */
	/* day of week starting from 1-monday, 7-sunday */
	cts_time->wday = (tm.tm_wday + 7) % 7 + 1; // Convert to 1-7 range (1=Monday, 7=Sunday)
	return 0;
}

static struct bt_cts_cb bt_cts = {
	&notify_change,
	&cts_time_write,
	&cts_time_read,
};

int main(void)
{
	int err;
	bt_addr_le_t addr;
	char addr_str[BT_ADDR_LE_STR_LEN];

	printk("Zephyr CTS Application %s\n", APP_VERSION_STRING);

	/* Check if the RTC is ready */
	if (!device_is_ready(rtc)) {
		printk("Device is not ready\n");
		return 0;
	}

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	printk("Bluetooth initialized\n");

	/* Get and print device address */
	bt_id_get(&addr, NULL);
	bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
	printk("Bluetooth device address: %s\n", addr_str);

	bt_cts_init(&bt_cts);
	k_work_submit(&adv_work);
	return 0;
}
