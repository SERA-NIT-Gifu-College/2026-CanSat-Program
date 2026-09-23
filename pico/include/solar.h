#ifndef __SOLAR_H__
#define __SOLAR_H__

/**
 * @file solar.h
 * @brief Helper functions for monitoring solar pannel voltage
 */

#include <hardware/adc.h>
#include <stdint.h>

#define SOLAR_ADC_NUM 0 //!< ADC index for voltage divided solar pannel

/**
 * States of Sun Search Algorithm
 */
typedef enum {
    SUNSEARCH_IDLE,
    SUNSEARCH_SCAN_HEADING,
    SUNSEARCH_ROTATE_BRIGHTEST,
    SUNSEARCH_SCAN_PITCH,
    SUNSEARCH_SET_PITCH,
    NUM_SUN_SEARCH_STATES
} SunSearchStates;

/**
 * Sun Search Algorithm State Machine
 */
typedef struct {
    SunSearchStates state;
    uint16_t heading;
    uint16_t max_brightness_heading;
    uint16_t pitch;
    uint16_t max_brightness_pitch;
    uint16_t brightness;
    bool brightness_read;
    bool transition;
} SunSearchSM;

/**
 * Initialize ADC for solar pannel output voltage monitoring, setup as default 12 bits conversion
 */
void setupSolarMon(void);
/** 
 * Read raw value of voltage divided solar pannel output
 * @return raw value from ADC
 */
uint16_t getGen(void);

#endif // __SOLAR_H__
