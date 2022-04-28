/***
 * Based on ESP32-IDF blink example code
 */

#include "user.h"
#include "led_strip.h"

#include "led.h"

#define MY_LED_PIN 48
#define MY_LED_RMT_CHANNEL 0
#define MY_LED_BRIGHTNESS 1

static uint8_t led_sequence; //Zero = 3 colors
static uint8_t led_state;
static led_strip_t *pStrip_a;
static const uint8_t crash_indications[][2] = { {3, 0}, //Default = OK
    { 1, 0 }, { 1, 1 }, { 1, 2 }, {2, 0}, {2, 1}, {2, 2},
    {1, 3} }; //Last = LED sequence out of range

void led_hearbeat(void)
{
    assert(pStrip_a);
    led_state = (led_state + 1) % crash_indications[led_sequence][0] + crash_indications[led_sequence][1];
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    pStrip_a->set_pixel(pStrip_a, 0,
        led_state == 0 ? MY_LED_BRIGHTNESS : 0,
        led_state == 1 ? MY_LED_BRIGHTNESS : 0,
        led_state == 2 ? MY_LED_BRIGHTNESS : 0);
    /* Refresh the strip to send data */
    pStrip_a->refresh(pStrip_a, 100);
}

void led_init(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    pStrip_a = led_strip_init(MY_LED_RMT_CHANNEL, MY_LED_PIN, 1);
    /* Set all LED off to clear all pixels */
    pStrip_a->clear(pStrip_a, 50);
}

void led_configure(uint8_t sequence)
{
    const size_t s = sizeof(crash_indications) / sizeof(uint8_t*);
    if (led_sequence < s) led_sequence = sequence;
    else {
        led_sequence = s - 1;
        ESP_LOGE("LED", "Led sequence index is out of range.");
    }
}