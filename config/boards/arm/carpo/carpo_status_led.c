/*
*
* Copyright (c) 2024 The YMCA Contributors
* SPDX-License-Identifier: MIT
*
*/

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

#include <zephyr/bluetooth/services/bas.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/ble.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/split_peripheral_status_changed.h>
#include <zmk/endpoints.h>

#define STATUSLED_NODE DT_ALIAS(statusledgpio)

#define STEPS 100U
#define SLEEP_DELTA 20U

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(statusledpwm));

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL)

struct output_status_data {
    struct zmk_endpoint_instance selected_endpoint;
    bool active_profile_connected;
    bool active_profile_bonded;
    uint8_t active_profile_index;
};

static struct output_status_data get_state() {
    return (struct output_status_data) {
        .selected_endpoint = zmk_endpoints_selected(),
        .active_profile_connected = zmk_ble_active_profile_is_connected(),
        .active_profile_bonded = !zmk_ble_active_profile_is_open(),
        .active_profile_index = zmk_ble_active_profile_index()
    };
}

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL) */

static int led_init() {
    if (!pwm_is_ready_dt(&pwm_led0)) {
        LOG_ERR("Error: pwm device %s is not ready\n", pwm_led0.dev->name);
        return -ENOTSUP;
    }
    return 1;
}

SYS_INIT(led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

struct pwm_led_state_t {
    bool increasing;
    uint32_t steps_taken;
    uint32_t pulse_width;
    uint32_t pulse_width_delta;
};

struct pwm_led_state_t pwm_led_state = {
    .increasing = true,
    .steps_taken = 0U,
    .pulse_width = 0U,
    .pulse_width_delta = pwm_led0.period / STEPS
};

void conn_status_timer_handler(struct k_timer *timer_info) {
    int ret;

    if (pwm_led_state.increasing) {
        if (pwm_led_state.steps_taken < STEPS) {
            ret = pwm_set_pulse_dt(&pwm_led0, pwm_led_state.pulse_width);
            pwm_led_state.steps_taken++;
            pwm_led_state.pulse_width += pwm_led_state.pulse_width_delta;
        } else {
          pwm_led_state.increasing = false;
          pwm_led_state.steps_taken--;
          pwm_led_state.pulse_width -= pwm_led_state.pulse_width_delta;
        }
    } else {
        if (pwm_led_state.steps_taken > 0) {
            ret = pwm_set_pulse_dt(&pwm_led0, pwm_led_state.pulse_width);
            pwm_led_state.steps_taken--;
            pwm_led_state.pulse_width -= pwm_led_state.pulse_width_delta;
        } else {
            pwm_led_state.increasing = true;
            pwm_led_state.steps_taken++;
            pwm_led_state.pulse_width += pwm_led_state.pulse_width_delta;
        }
    }

}

K_TIMER_DEFINE(conn_status_timer, conn_status_timer_handler, NULL);

void blink(uint8_t times, uint8_t sleep_time_ms) {
    int ret;

    for (int i = 0; i < times; i++) {
        ret = pwm_set_pulse_dt(&pwm_led0, pwm_led0.period);
        if (ret < 0) {
            LOG_DBG("Error: %d: failed to set pulse width", ret);
            return;
        }
        k_msleep(sleep_time_ms);
        ret = pwm_set_pulse_dt(&pwm_led0, 0);
        if (ret < 0) {
            LOG_DBG("Error: %d: failed to set pulse width", ret);
            return;
        }
        k_msleep(sleep_time_ms);
    }
}

void reset_led() {
    int ret;

    pwm_led_state = (struct pwm_led_state_t) {
        .increasing = true,
        .steps_taken = 0U,
        .pulse_width = 0U,
        .pulse_width_delta = pwm_led0.period / STEPS
    };

    ret = pwm_set_pulse_dt(&pwm_led0, 0);

    if (ret < 0) {
        LOG_DBG("Error: %d: failed to set pulse width", ret);
    }
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL)

int ble_event_listener(const zmk_event_t *eh) {
    struct output_status_data state;
    state = get_state();
    struct zmk_endpoint_changed *ev;

    switch (state.selected_endpoint.transport) {
    case ZMK_TRANSPORT_BLE:
        if (state.active_profile_connected) {
            k_timer_stop(&conn_status_timer);
            reset_led();
            ev = as_zmk_endpoint_changed(eh);
            if (ev == NULL) {
                blink(state.active_profile_index + 1, 125);
            }
        } else {
            k_timer_start(&conn_status_timer, K_MSEC(SLEEP_DELTA), K_MSEC(SLEEP_DELTA));
        }
        break;
    case ZMK_TRANSPORT_USB:
        k_timer_stop(&conn_status_timer);
        reset_led();
        break;
    }
    return 0;
}

ZMK_LISTENER(ble, ble_event_listener);
ZMK_SUBSCRIPTION(ble, zmk_ble_active_profile_changed);
ZMK_SUBSCRIPTION(ble, zmk_endpoint_changed);

#else

int peripheral_event_listener(const zmk_event_t *eh) {
    struct zmk_split_peripheral_status_changed *ev;

    ev = as_zmk_split_peripheral_status_changed(eh);

    if (ev->connected) {
        k_timer_stop(&conn_status_timer);
        reset_led();
        blink(3, 125);
    } else {
        k_timer_start(&conn_status_timer, K_MSEC(SLEEP_DELTA), K_MSEC(SLEEP_DELTA));
    }
    return 0;
}

ZMK_LISTENER(peripheral, peripheral_event_listener);
ZMK_SUBSCRIPTION(peripheral, zmk_split_peripheral_status_changed);

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL) */
