/*
 * PSX_controller.c
 *
 *  Created on: Feb 21, 2024
 *      Author: Loren
 */
#include "PSX_Controller.h"


/**
  * @brief  Buffer used in the comunication
  */
uint8_t PSX_Tx_Buffer[9] = {0x01, 0x42};
uint8_t PSX_Rx_Buffer[9] = {};
uint8_t cur_mod;

/**
  * @brief  Initialization for the PSX Controller: set the ATTENTION pin HIGH.
  * @param  None
  * @retval None
  */
void PSX_Init(){
	PSX.PSX_AD = 128;
	PSX.speed = 0.0;
	PSX.PSX_MOD = 0;
	HAL_GPIO_WritePin(ATT_GPIO_Port, ATT_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

/**
  * @brief  Send a request to the controller, witch reply with a set of data
  * @param  *TX_Buffer uint8_t, *RX_Buffer uint8_t
  * @retval None
  */
void GET_PSX_DATA(uint8_t *TX_Buffer, uint8_t *RX_Buffer){

	HAL_GPIO_WritePin(ATT_GPIO_Port, ATT_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspi1, PSX_Tx_Buffer, PSX_Rx_Buffer, 9, 1);
	HAL_GPIO_WritePin(ATT_GPIO_Port, ATT_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}


/**
  * @brief  Interprets data received from the PSX Controller
  * @param  *PSX_Rx_Buffer uint8_t
  * @retval None
  */
void DATA_DIGEST(uint8_t *PSX_Rx_Buffer){
	GET_PSX_DATA(PSX_Tx_Buffer, PSX_Rx_Buffer);

	if(PSX_Rx_Buffer[0] == 0xFF && PSX_Rx_Buffer[1] == 0x73 && PSX_Rx_Buffer[2] == 0x5A){

		PSX_Rx_Buffer[6] = ~PSX_Rx_Buffer[6];

		PSX.PSX_AD = PSX_Rx_Buffer[6];

		PSX.speed = ((PSX.PSX_AD*320)/255)-160;

		cur_mod = PSX.PSX_MOD;
		if(PSX_Rx_Buffer[3] == 0x7F){
			PSX.PSX_MOD = 0;
		}else if(PSX_Rx_Buffer[3] == 0xDF) {
			PSX.PSX_MOD = 1;
		}else if(PSX_Rx_Buffer[3] == 0xFF){
			PSX.PSX_MOD = cur_mod;
		}
	}
}
