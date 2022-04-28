#pragma once

esp_err_t my_ble_init(float* measurement_value);
void my_ble_clear_persistent(void);
void my_ble_notify_value_changed(void);