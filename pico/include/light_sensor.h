#ifndef __LIGHT_SENSOR_H__
#define __LIGHT_SENSOR_H__

#include <hardware/adc.h>
#include <stdint.h>

#define LIGHT_SENSOR_ADC_NUM 0
#define LIGHT_SENSOR_SAMPLING_NUMBER 5
#define LIGHT_SENSOR_SAMPLING_INTERVAL 10 // ms

void setupLightSensor(void);
uint16_t readAVG(void);

#endif // __LIGHT_SENSOR_H__
