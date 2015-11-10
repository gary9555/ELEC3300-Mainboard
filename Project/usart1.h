#ifndef _UASRT1_H
#define _UASRT1_H

#include "stm32f10x_usart.h"
#include "stm32f10x.h"
#include <stdio.h>

void USART1_Config(void);
void NVIC_Configuration(void);
int fputc(int ch, FILE *f);

#endif

