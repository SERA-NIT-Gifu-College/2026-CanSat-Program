#ifndef __LIGHT_SENSOR_H__
#define __LIGHT_SENSOR_H__

/**
 * @file light_sensor.h
 * @brief Helper functions for reading photo-transistor sensor
 */

#include <hardware/adc.h>
#include <stdint.h>

#define LIGHT_SENSOR_ADC_NUM 0 //!< ADC index for photo-transistor
#define LIGHT_SENSOR_SAMPLING_NUMBER 5 //!< Sampling number per one readAVG call @see readAVG()
#define LIGHT_SENSOR_SAMPLING_INTERVAL 10 //!< Sampling interval in milliseconds @see readAVG()

/**
 * Initialize ADC for photo-transistor, setup as default 12 bits conversion
 */
void setupLightSensor(void);
/** 
 * Read averaged value of photo-transistor
 * @see LIGHT_SENSOR_SAMPLING_NUMBER
 * @see LIGHT_SENSOR_SAMPLING_INTERVAL
 * @return Averaged raw value from ADC
 */
uint16_t readAVG(void);

#endif // __LIGHT_SENSOR_H__
