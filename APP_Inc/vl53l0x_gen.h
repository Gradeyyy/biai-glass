#ifndef __VL53L0X_GEN_H
#define __VL53L0X_GEN_H

#include "vl53l0x.h"

/**
 *  vl53l0x_gen.h
 *
 *  Created on: 2025年02月18日
 *  Author: lang
 */

#define KEY0_PRES 	1
#define KEY1_PRES	2
#define KEY2_PRES	3
#define WKUP_PRES   4

extern VL53L0X_RangingMeasurementData_t vl53l0x_data;

VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,uint8_t mode);
void vl53l0x_general_test(VL53L0X_Dev_t *dev);
void vl53l0x_general_start(VL53L0X_Dev_t *dev, uint8_t mode);

#endif


