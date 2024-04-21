/*
 * communication.c
 *
 *  Created on: Mar 16, 2024
 *      Author: patry
 */

#include "communication.h"



int _write(int file, uint8_t *buf, int nbytes){
  uint8_t num_of_byte = 0;
  while (num_of_byte <= nbytes - 1){
    while (!LL_USART_IsActiveFlag_TXE(USART2));
    LL_USART_TransmitData8(USART2, buf[num_of_byte++]);
  }
  while (!LL_USART_IsActiveFlag_TC(USART2));
  return nbytes;
}



void USART2_IRQHandler(void){
	uint8_t received_byte;
	if(LL_USART_IsActiveFlag_RXNE(USART2)){
	  received_byte = LL_USART_ReceiveData8(USART2);
	  usart_append(received_byte);
	}
}

void usart_transmit(USART_TypeDef *USARTx, uint8_t text[]){
	uint8_t num_of_byte = 0;
	while (num_of_byte < (strlen((const char*)text))){
		//waiting until the Transmit Empty flag is set
		while (!LL_USART_IsActiveFlag_TXE(USARTx)){
			uint16_t counter = 0;
			if(counter++ > 65000) return;
		}
		LL_USART_TransmitData8(USARTx, text[num_of_byte++]);
	}
	while (!LL_USART_IsActiveFlag_TC(USARTx)){
		uint16_t counter = 0;
		if(counter++ > 65000) return;
	}
	num_of_byte = 0;
}


void usart2_init(void){
  LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(USART2_IRQn);

  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  LL_USART_EnableIT_RXNE(USART2);

}
