/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// load in the Zephyr library
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <nrfx_power.h>

#include "bt.h"

LOG_MODULE_REGISTER(finalproject,LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define ONE_HZ 500
#define FIVE_HZ 100
#define ONE_TO_FIVE_DIFF_HZ 400
#define FIVE_TO_TEN_DIFF_HZ 50
#define FIFTY_MV 50
#define ONE_FIFTY_MV 150
#define NUM_MEASUREMENT_PTS 5
#define WINDOW_SIZE 20

#define ADC_DT_SPEC_GET_BY_ALIAS(node_id)                    \
{                                                            \
  .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(node_id))),        \
  .channel_id = DT_REG_ADDR(DT_ALIAS(node_id)),              \
  ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(node_id))            \
}                                                            \

#define DT_SPEC_AND_COMMA(node_id, prop, idx)                \
	      ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

// define structs based on DT aliases

/* Declarations 


void fixed_hb(struct k_timer *heartbeats);
void button0_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
int check_vbus(void); */
void on_data_rx(struct bt_conn *conn, const uint8_t *const data, uint16_t len);
void on_connected(struct bt_conn *conn, uint8_t ret);
void on_disconnected(struct bt_conn *conn, uint8_t reason);
void on_notif_changed(enum bt_data_notifications_enabled status);

/* ADC channels (specified in DT overlay) */
static const struct adc_dt_spec adc_vslow = ADC_DT_SPEC_GET_BY_ALIAS(vslow);
static const struct adc_dt_spec adc_vfast = ADC_DT_SPEC_GET_BY_ALIAS(vfast);

/* Bluetooth */
static struct bt_conn *current_conn;

struct bt_conn_cb bluetooth_callbacks = {
  .connected = on_connected,
  .disconnected = on_disconnected,
};
struct bt_remote_srv_cb remote_service_callbacks = {
  .notif_changed = on_notif_changed,
  .data_rx = on_data_rx,
};

/* LEDs */
#define HB_NODE	DT_ALIAS(heartbeat)
static const struct gpio_dt_spec hb = GPIO_DT_SPEC_GET(HB_NODE, gpios);
#define BZ_NODE	DT_ALIAS(buzzer)
static const struct gpio_dt_spec bz = GPIO_DT_SPEC_GET(BZ_NODE, gpios);
#define IV_NODE	DT_ALIAS(ivdrip)
static const struct gpio_dt_spec iv = GPIO_DT_SPEC_GET(IV_NODE, gpios);

/* Buttons */
#define BUTTON0_NODE	DT_ALIAS(button0) 
static const struct gpio_dt_spec butt0 = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);
#define BUTTON1_NODE	DT_ALIAS(button1) 
static const struct gpio_dt_spec butt1 = GPIO_DT_SPEC_GET(BUTTON1_NODE, gpios);

/* PWM */
static const struct pwm_dt_spec mtr_drv1 = PWM_DT_SPEC_GET(DT_ALIAS(drv1));
static const struct pwm_dt_spec mtr_drv2 = PWM_DT_SPEC_GET(DT_ALIAS(drv2));

//global variables
static int32_t val_mv_slow = 0;
static int32_t val_mv_fast = 0;
static bool usbregstatus;
static uint8_t slow_data[NUM_MEASUREMENT_PTS] = {0};
static uint8_t fast_data[NUM_MEASUREMENT_PTS] = {0};
static uint8_t slow_queue[WINDOW_SIZE] = {0};
static uint8_t fast_queue[WINDOW_SIZE] = {0};
float slow_rms_window = 0;
float fast_rms_window = 0;

/* Timers */
//timer for heartbeat
void fixed_hb(struct k_timer *heartbeats)
{
  gpio_pin_toggle_dt(&iv);
}
K_TIMER_DEFINE(heartbeats, fixed_hb, NULL);

/* Callbacks */
/* Save 5 sec worth of data for each sinusoidal input*/
void button0_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
}
static struct gpio_callback butt0_cb_data;

/* Bluetooh notification that the two data arrays are ready to cellphone*/
void button1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
}
static struct gpio_callback butt1_cb_data;

//bluetooth callbacks
void on_data_rx(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
  uint8_t temp_str[len+1];
  memcpy(temp_str, data, len);
  temp_str[len] = 0x00; // manually append NULL character at the end
  LOG_INF("BT received data on conn %p. Len: %d", (void *)conn, len);
  LOG_INF("Data: %s", temp_str);
}
void on_connected(struct bt_conn *conn, uint8_t ret)
{
  if (ret) { LOG_ERR("Connection error: %d", ret); }
  LOG_INF("BT connected");
  current_conn = bt_conn_ref(conn);
}
void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
  LOG_INF("BT disconnected (reason: %d)", reason);
  if (current_conn) {
    bt_conn_unref(current_conn);
    current_conn = NULL;
  }
}
void on_notif_changed(enum bt_data_notifications_enabled status)
{
  if (status == BT_DATA_NOTIFICATIONS_ENABLED) {
    LOG_INF("BT notifications enabled");
  }
  else {
    LOG_INF("BT notifications disabled");
  }
}

/* check VBUS*/
int check_vbus() {
  /* check for voltage on VBUS (USB-C charging cable attached)
  Returns:
  0 - VBUS not detected
  -1 - VBUS detected (need to kill device function)
  */
  usbregstatus = nrf_power_usbregstatus_vbusdet_get(NRF_POWER);;
  if (usbregstatus) {
    LOG_ERR("VBUS voltage detected. Device cannot be operated while charging.");
    return -1;
  }
  else {
    LOG_DBG("VBUS voltage checked and not detected.");
  }
  return 0;
}

/* Read ADCs */
int32_t read_adc_val_slow()
{
  /* Read adc channel 0*/
  int16_t buf;
  struct adc_sequence sequence = {
    .buffer = &buf,
    .buffer_size = sizeof(buf), // bytes
  };
  //LOG_INF("Measuring %s (channel %d)... ", adc_vslow.dev->name, adc_vslow.channel_id);
  (void)adc_sequence_init_dt(&adc_vslow, &sequence);
  int ret;
  ret = adc_read(adc_vslow.dev, &sequence);
  if (ret < 0) {
    LOG_ERR("Could not read (%d)", ret);
  } else {
    //LOG_DBG("Raw ADC Buffer: %d", buf);
  }
  int32_t val_mv;
  val_mv = buf;
  ret = adc_raw_to_millivolts_dt(&adc_vslow, &val_mv);
  if (ret < 0) {
    LOG_ERR("Buffer cannot be converted to mV; returning raw buffer value.");
  } else {
    //LOG_INF("AIN0 ADC Value (mV): %d", val_mv);
  }
  return val_mv;
}

int32_t read_adc_val_fast()
{
  /* Read adc channel 1*/
  int16_t buf;
  struct adc_sequence sequence = {
    .buffer = &buf,
    .buffer_size = sizeof(buf), // bytes
  };
  //LOG_INF("Measuring %s (channel %d)... ", adc_vfast.dev->name, adc_vfast.channel_id);
  (void)adc_sequence_init_dt(&adc_vfast, &sequence);
  int ret;
  ret = adc_read(adc_vfast.dev, &sequence);
  if (ret < 0) {
    LOG_ERR("Could not read (%d)", ret);
  } else {
    //LOG_DBG("Raw ADC Buffer: %d", buf);
  }
  int32_t val_mv;
  val_mv = buf;
  ret = adc_raw_to_millivolts_dt(&adc_vfast, &val_mv);
  if (ret < 0) {
    LOG_ERR("Buffer cannot be converted to mV; returning raw buffer value.");
  } else {
    //LOG_INF("AIN1 ADC Value (mV): %d", val_mv);
  }
  return val_mv;
}

void main(void)
{
  int err;
  /**capture exit or error codes**/
  err = !device_is_ready(adc_vslow.dev);
  if (err){
    LOG_ERR("ADC controller device(s) not ready");
    return -1;
  }

  err = !device_is_ready(butt0.port);
  if (err) {
    LOG_ERR("gpio0 interface not ready.");
    return;
  }

  if (!device_is_ready(mtr_drv1.dev)) {
    LOG_ERR("PWM device %s is not ready.", mtr_drv1.dev->name);
    return -1;
  }

  /* Setup ADCs */
  err = adc_channel_setup_dt(&adc_vslow);
  if (err < 0) {
    LOG_ERR("Could not setup vslow ADC channel (%d)", err);
    return err;
  }

  err = adc_channel_setup_dt(&adc_vfast);
  if (err < 0) {
    LOG_ERR("Could not setup vfast ADC channel (%d)", err);
    return err;
  }

  /* Setup LEDs */
  err = gpio_pin_configure_dt(&hb, GPIO_OUTPUT_LOW);
	if (err < 0) {
		LOG_ERR("Cannot configure heartbeat led");
    return;
	}

  err = gpio_pin_configure_dt(&bz, GPIO_OUTPUT_LOW);
	if (err < 0) {
		LOG_ERR("Cannot configure buzzer led");
    return;
	}

  err = gpio_pin_configure_dt(&iv, GPIO_OUTPUT_LOW);
	if (err < 0) {
		LOG_ERR("Cannot configure ivdrip led");
    return;
	}

  /* Setup buttons */
  err = gpio_pin_configure_dt(&butt0, GPIO_INPUT);
	if (err < 0) {
    LOG_ERR("Cannot configure button0");
		return;
	}

  err = gpio_pin_configure_dt(&butt1, GPIO_INPUT);
	if (err < 0) {
    LOG_ERR("Cannot configure button3");
		return;
	}

  /* Initialize Bluetooth */
  err = bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);
  if (err) {
    LOG_ERR("BT init failed (err = %d)", err);
  }

  /* Setup callbacks */
  err = gpio_pin_interrupt_configure_dt(&butt0, GPIO_INT_EDGE_TO_ACTIVE );
  if (err < 0) {
    LOG_ERR("Cannot configure button0");
		return;
	}
  gpio_init_callback(&butt0_cb_data, button0_pressed, BIT(butt0.pin));
	gpio_add_callback(butt0.port, &butt0_cb_data);

  err = gpio_pin_interrupt_configure_dt(&butt1, GPIO_INT_EDGE_TO_ACTIVE );
  if (err < 0) {
    LOG_ERR("Cannot configure button1");
		return;
	}
  gpio_init_callback(&butt1_cb_data, button1_pressed, BIT(butt1.pin));
	gpio_add_callback(butt1.port, &butt1_cb_data);

  while (1) {
    //check for vbus
    int err ;
    err = check_vbus();
    if (err) {
      gpio_pin_set_dt(&hb, 0);
      gpio_pin_set_dt(&bz, 0);
      k_timer_start(&heartbeats, K_MSEC(ONE_HZ), K_MSEC(ONE_HZ));
      k_msleep(3*1000); //3 second delay
      k_timer_stop(&heartbeats);
    }
    else{
      /* loop to sample voltage reading and act accordingly */
      //read voltage of channel 0 and 1
      val_mv_slow = read_adc_val_slow();
      val_mv_fast = read_adc_val_fast();
      
      /* implement RMS sliding window */
      int rms_window_count = 0;
      int data_count = 0;
      bool full = 0;
      int full_inc = 0;

      if (rms_window_count > 10) { full = 1; }
      if (full == 0){
        if (data_count > 4) { data_count = 0; }
        slow_rms_window += val_mv_slow * val_mv_slow;
        fast_rms_window += val_mv_fast * val_mv_fast;
        slow_queue[rms_window_count] = val_mv_slow;
        fast_queue[rms_window_count] = val_mv_fast;
        rms_window_count += 1;
        slow_data[data_count] == sqrt(slow_rms_window / rms_window_count);
        fast_data[data_count] == sqrt(fast_rms_window / rms_window_count);
        data_count += 1;
      }
      else{
        if (data_count > 4) { data_count = 0; }
        if (full_inc > 19) { full_inc = 0; }
        slow_rms_window -= slow_queue[full_inc];
        fast_rms_window -= fast_queue[full_inc];
        slow_queue[full_inc] = val_mv_slow;
        fast_queue[full_inc] = val_mv_fast;
        slow_data[data_count] == sqrt(slow_rms_window / 20);
        fast_data[data_count] == sqrt(fast_rms_window / 20);
        data_count += 1;
        full_inc += 1;
      }
      LOG_ERR();
      LOG_ERR("Could not set motor driver 1 (PWM0)");

      //modulate LED1 brightness based on adc channel 0 voltage
      err = pwm_set_pulse_dt(&mtr_drv1, mtr_drv1.period * (float) val_mv_slow / FIFTY_MV); // % duty cycle based on ADC0 reading
      if (err) {
        LOG_ERR("Could not set motor driver 1 (PWM0)");
      }

      //modulate LED2 brightness based on adc channel 1 voltage
      err = pwm_set_pulse_dt(&mtr_drv2, mtr_drv2.period * (float) val_mv_fast / ONE_FIFTY_MV); // % duty cycle based on ADC1 reading
      if (err) {
        LOG_ERR("Could not set motor driver 2 (PWM1)");
      }

      // send a notification that "data" is ready to be read...
      err = send_data_notification(current_conn, slow_data, 1);
      if (err) {
        LOG_ERR("Could not send BT notification (err: %d)", err);
      }
      else {
        LOG_INF("BT data transmitted.");
      }

      k_msleep(3*1000); //3 second delay
    }
	}
}