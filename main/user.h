#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#ifndef _LB
    #define _LB(val) ( (uint8_t)((val) & 0xFF) )
#endif
#ifndef _HB
    #define _HB(val) ( (uint8_t)(((val) >> (8 * (sizeof(val) - 1))) & 0xFF) )
#endif