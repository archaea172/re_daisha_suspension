/*
 * t_semi_can_lib.c
 *
 *  Created on: Feb 19, 2025
 *      Author: akiyu
 */
#include <stdint.h>


#include "t_semi_can_lib.h"
#include "stm32g4xx_hal.h"


float16_t convert_f32_f16(float32_t fp32){
    uint32_t fp32_bits = *(uint32_t*)&fp32;
    uint16_t sign = (fp32_bits >> 31) & 1;
    uint16_t exponent = (fp32_bits >> 23) & 0xff;
    uint32_t mantissa = fp32_bits & 0x7fffff;
    uint16_t fp16_bits;

    if(exponent == 0xFF){
        if(mantissa == 0){
            //NaN
            fp16_bits = 0x1F << 10 | 1;
        }else{
            //+inf/-inf
            fp16_bits = sign << 15 | 0x1F << 10;
        }
    }else{
        if(exponent > 16 + 127){
            //Overflow
            fp16_bits = sign << 15 | 0x1F << 10;
        }else if(exponent < - 15 + 127){
            //Underflow
            return sign << 15;
        }else{
            fp16_bits = sign << 15 | (exponent + 15 - 127) << 10 | mantissa >> 14;
        }
    }
    return *(float16_t*)&fp16_bits;
}


float32_t convert_f16_f32(float16_t fp16){
    uint16_t fp16_bits = *(uint16_t*)&fp16;
    uint32_t sign = (fp16_bits >> 15) & 1;
    uint32_t exponent = (fp16_bits >> 10) & 0x1F;
    uint32_t mantissa = fp16_bits & 0x3FF;
    uint32_t fp32_bits = 0;

    if(exponent == 0x1F){
        if(mantissa == 0){
            //NaN
            fp32_bits = 0xFF << 23 | 1;
        }else{
            //+inf/-inf
            fp32_bits = sign << 31 | 0xFF << 23;
        }
    }else{
        if(exponent == 0){
            fp32_bits = sign << 31;
        }else{
            fp32_bits = sign << 31 | (exponent - 15 + 127) << 23 | mantissa << 14;
        }
    }

    return *(float32_t*)&fp32_bits;
}

void convert_f32_u8(float32_t *f32, uint8_t *txdata) {
    for (int i = 0; i < 16; i++) {
		union IntAndFloat {
			uint32_t ival;
			float fval;
		};
		union IntAndFloat target;
		target.fval = f32[i];
		uint16_t a_16[2] = {};
		a_16[0] = (uint16_t)(target.ival >> 16);
		a_16[1] = (uint16_t)(target.ival & 0xFFFF);
		txdata[i*4]   = (uint8_t)(a_16[0] >> 8);
		txdata[i*4+1] = (uint8_t)(a_16[0] & 0xFF);
		txdata[i*4+2] = (uint8_t)(a_16[1] >> 8);
		txdata[i*4+3] = (uint8_t)(a_16[1] & 0xFF);
    }
}

void convert_u8_f32(uint8_t *txdata, float32_t *f32) {
	for (int i = 0; i < 16; i++) {
		uint32_t f32_u32 = ((txdata[i*4] << 24) | (txdata[i*4+1] << 16) | (txdata[i*4+2] << 8) | (txdata[i*4+3]));
        union IntAndFloat {
			uint32_t ival;
			float fval;
		};
        union IntAndFloat target;
        target.ival = f32_u32;
        f32[i] = target.fval;
	}
}


int CAN_SEND(uint32_t CANID, uint8_t *txdata, FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *htxheader) {
	htxheader->Identifier = CANID;
	if (HAL_OK != HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, htxheader, txdata)){
		return -1;
	}
	return 0;
}


void convert_can_message(float32_t *txdata_f32, uint8_t *txdata) {
	float16_t txdata_f16[32] = {};
	for (int i = 0; i < 32; i++) {
		txdata_f16[i] = convert_f32_f16(txdata_f32[i]);
		txdata[i*2] = txdata_f16[i] >> 8;
		txdata[i*2+1] = (uint8_t)(txdata_f16[i] & 0xff);
	}
}


int CAN_RxTxSettings(FDCAN_HandleTypeDef *hfdcan,
		FDCAN_FilterTypeDef *hfdcan_filter_settings,
		FDCAN_TxHeaderTypeDef *htxheader,
		uint32_t fdformat,
		uint32_t fifox_f,
		uint32_t fifox_n,
		uint32_t datalength,
		uint32_t initial_id,
		uint32_t id1_f,
		uint32_t id2_f) {


	hfdcan_filter_settings->IdType = FDCAN_STANDARD_ID;
	hfdcan_filter_settings->FilterIndex = 0;
	hfdcan_filter_settings->FilterType = FDCAN_FILTER_RANGE;
	hfdcan_filter_settings->FilterConfig = fifox_f;
	hfdcan_filter_settings->FilterID1 = id1_f;
	hfdcan_filter_settings->FilterID2 = id2_f;


	htxheader->Identifier = initial_id;
	htxheader->IdType = FDCAN_STANDARD_ID;
	htxheader->TxFrameType = FDCAN_DATA_FRAME;
	htxheader->DataLength = datalength;
	htxheader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	htxheader->FDFormat = fdformat;
	if (FDCAN_FD_CAN == htxheader->FDFormat) htxheader->BitRateSwitch = FDCAN_BRS_ON;
	else htxheader->BitRateSwitch = FDCAN_BRS_OFF;
	htxheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	htxheader->MessageMarker = 0;


	if (HAL_OK != HAL_FDCAN_ConfigFilter(hfdcan, hfdcan_filter_settings)) return -1;
	if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_FILTER_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)) return -2;
	if (HAL_OK != HAL_FDCAN_Start(hfdcan)) return -3;
	if (HAL_OK != HAL_FDCAN_ActivateNotification(hfdcan, fifox_n, 0)) return -4;


	return 0;
}


int interboard_comms_CAN_RxTxSettings_nhk2025(FDCAN_HandleTypeDef *hfdcan, FDCAN_FilterTypeDef *hfdcan_filter_settings, FDCAN_TxHeaderTypeDef *htxheader) {


	uint32_t FDformat = FDCAN_FD_CAN;
	uint32_t FIFOx_f = FDCAN_FILTER_TO_RXFIFO1;
	uint32_t FIFOx_n = FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
	uint32_t DATAlength = FDCAN_DLC_BYTES_64;
	uint32_t initial_ID = 0x000;
	uint32_t ID1_f = 0;
	uint32_t ID2_f = 0x7FF;


	int return_value = CAN_RxTxSettings(hfdcan, hfdcan_filter_settings, htxheader, FDformat, FIFOx_f, FIFOx_n, DATAlength, initial_ID, ID1_f, ID2_f);


	return return_value;
}


int robomas_CAN_RxTxSettings_nhk2025(FDCAN_HandleTypeDef *hfdcan, FDCAN_FilterTypeDef *hfdcan_filter_settings, FDCAN_TxHeaderTypeDef *htxheader) {


	uint32_t FDformat = FDCAN_CLASSIC_CAN;
	uint32_t FIFOx_f = FDCAN_FILTER_TO_RXFIFO0;
	uint32_t FIFOx_n = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
	uint32_t DATAlength = FDCAN_DLC_BYTES_8;
	uint32_t initial_ID = 0x200;
	uint32_t ID1_f = 0x200;
	uint32_t ID2_f = 0x410;


	int return_value = CAN_RxTxSettings(hfdcan, hfdcan_filter_settings, htxheader, FDformat, FIFOx_f, FIFOx_n, DATAlength, initial_ID, ID1_f, ID2_f);


	return return_value;
}
