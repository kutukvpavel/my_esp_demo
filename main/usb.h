#pragma once

#include <stdarg.h>

#define MY_USB_WRITE(str) my_usb_write((str), sizeof(str))

esp_err_t my_usb_init(bool enable_console);
void my_usb_write(char* src, size_t len);
void my_usb_print(const char* fmt, ...);