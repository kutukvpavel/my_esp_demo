#include "user.h"
#include "ble.h"

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

    if (my_ble_init() != ESP_OK)
    {
        //TODO: failed to init BT, reported the error already
        //Need some indication of the problem for the user, besides debug UART
    }

    
}