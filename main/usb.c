/***   
 *  Based on ESP32-IDF tusb_console example code.
 */

#include "user.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "vfs_tinyusb.h"

#include "usb.h"

#define MY_USB_ACM TINYUSB_CDC_ACM_0
#define MY_USB_FILE "/dev/usb_acm0"

static FILE* input_stream;
static FILE* output_stream;

static tinyusb_config_t tusb_cfg = { 0 }; // the configuration uses default values
tinyusb_config_cdcacm_t amc_cfg = {
    .usb_dev = TINYUSB_USBDEV_0,
    .cdc_port = TINYUSB_CDC_ACM_0
 }; // the configuration uses default values
static const char* TAG = "TINY_USB_DEMO";

esp_err_t my_usb_init(bool enable_console)
{
    esp_err_t ret;

    if ( (ret = tinyusb_driver_install(&tusb_cfg)) )
    {
        ESP_LOGE(TAG, "%s: failed to install TinyUSB driver, %s.", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ( (ret = tusb_cdc_acm_init(&amc_cfg)) )
    {
        ESP_LOGE(TAG, "%s: failed to init CDC device, %s.", __func__, esp_err_to_name(ret));
        return ret;
    }

    if (enable_console)
    {
        if ((ret = esp_tusb_init_console(MY_USB_ACM)))
        {
            ESP_LOGE(TAG, "%s: failed to init USB CDC console, %s.", __func__, esp_err_to_name(ret));
            return ret;
        }
        if (enable_console)
            fprintf(stdout, "Goodnight Moon!");
    }
    else    
    {
        if ( (ret = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_vfs_tusb_cdc_register(MY_USB_ACM, MY_USB_FILE))) ) return ret;
        input_stream = fopen(MY_USB_FILE, "r");
        if (!input_stream) return ESP_FAIL;
        output_stream = fopen(MY_USB_FILE, "w");
        if (!output_stream) return ESP_FAIL;
    }

    return ESP_OK;
}

void my_usb_write(char* src, size_t len)
{
    fwrite(src, 1, len, output_stream);
}

void my_usb_print(const char* fmt, ...)
{
    va_list(args);
    va_start(args, fmt);
    vfprintf(output_stream, fmt, args);
}