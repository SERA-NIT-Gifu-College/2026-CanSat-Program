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
 * Initialize ADC for solar pannel output voltage monitoring, setup as default 12 bits conversion
 */
void setupSolarMon(void);
/** 
 * Read raw value of voltage divided solar pannel output
 * @return raw value from ADC
 */
uint16_t getGen(void);

#endif // __SOLAR_H__
