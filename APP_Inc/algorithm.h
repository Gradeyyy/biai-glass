/*
 * algorithm.h
 *
 *  Created on: 2025/2/18
 *      Author: lang
 */
#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

#include "main.h"

void Acc_algorithm_init(void);
uint8_t Algorithm_Input(void);
uint8_t Acc_algorithm_result(int16_t gyro_x_in, int16_t gyro_y_in, int16_t gyro_z_in);



#endif /* INC_ALGORITHM_H_ */
