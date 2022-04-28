#include "user.h"
#include "ble.h"
#include "usb.h"
#include "led.h"

static const char* TAG = "MY_INIT";
static float measurement_dummy = 0xADAD;

void crashpad(uint8_t sequence)
{
    led_configure(sequence);
    int i = 20;
    while (i--)
    {
        led_hearbeat();
        vTaskDelay(500);
    }
    esp_restart();
}

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    led_init();

    ESP_LOGI(TAG, "USB init...");
    if (my_usb_init(false) != ESP_OK) crashpad(1);

    ESP_LOGI(TAG, "BT init...");
    if (my_ble_init(&measurement_dummy) != ESP_OK) crashpad(2);
    my_ble_clear_persistent(); //For testing purposes

    ESP_LOGI(TAG, "--- INIT FINISHED ---");

    while (true)
    {
        ESP_LOGI(TAG, "%f\n", measurement_dummy);
        my_usb_print("%f\n", measurement_dummy++);
        my_ble_notify_value_changed();
        led_hearbeat();
        vTaskDelay(1000);
    }
    
}