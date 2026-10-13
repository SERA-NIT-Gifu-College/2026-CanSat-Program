#ifndef __BUTTON_H__
#define __BUTTON_H__

/**
 * @file button.h
 * @brief Interfacing function to BOOTSEL button, taken from pico-examples/picoboard/button
 */

#include <pico/stdlib.h>

/**
 * Get bootsel button state
 */
bool __no_inline_not_in_flash_func(getBootselBtn)();

#endif // __BUTTON_H__
