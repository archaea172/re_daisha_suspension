/*
 * t_semi_can_lib.h
 *
 *  Created on: Feb 19, 2025
 *      Author: akiyu
 */

#ifndef INC_T_SEMI_CAN_LIB_H_
#define INC_T_SEMI_CAN_LIB_H_


#include "stm32g4xx_hal.h"


typedef float float32_t;
typedef uint16_t float16_t;


float16_t convert_f32_f16(float32_t);
float32_t convert_f16_f32(float16_t);
void convert_f32_u8(float32_t*, uint8_t*);
void convert_u8_f32(uint8_t*, float32_t*);
int CAN_SEND(uint32_t, uint8_t*, FDCAN_HandleTypeDef*, FDCAN_TxHeaderTypeDef*);
void convert_can_message(float32_t*, uint8_t*);
int CAN_RxTxSettings(FDCAN_HandleTypeDef*, FDCAN_FilterTypeDef*, FDCAN_TxHeaderTypeDef*, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
int interboard_comms_CAN_RxTxSettings_nhk2025(FDCAN_HandleTypeDef*, FDCAN_FilterTypeDef*, FDCAN_TxHeaderTypeDef*);
int robomas_CAN_RxTxSettings_nhk2025(FDCAN_HandleTypeDef*, FDCAN_FilterTypeDef*, FDCAN_TxHeaderTypeDef*);

#endif /* INC_T_SEMI_CAN_LIB_H_ */
