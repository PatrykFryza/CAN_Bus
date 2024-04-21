/*
 * communication.h
 *
 *  Created on: Mar 21, 2024
 *      Author: patry
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_
#include "main.h"
#include "stdio.h"
#include "string.h"

#define LINE_MAX_LENGTH 50

void usart2_init(void);
void usart_transmit(USART_TypeDef *USARTx, uint8_t text[]);

#endif /* INC_COMMUNICATION_H_ */
