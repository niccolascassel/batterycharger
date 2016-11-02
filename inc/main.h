#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_nucleo.h"

#define TRUE		1
#define FALSE		0

/* Size of buffer */
#define BUFFERSIZE	10

#define	QNTD_DE_AMOSTRAS_P_MEDIA	10

#define	RESOLUCAO_AD			4096
#define	VREF					3.3
#define	TENSAO_MAXIMA			14.4
#define	CORRENTE_MAXIMA			1
#define	RELACAO_TENSAO         	TENSAO_MAXIMA / VREF
#define	RELACAO_CORRENTE        4
// Tem que ensaiar o LM35
//#define	RELACAO_TEMPERATURA

/* Fases do processo de carga */
#define	FASE_A	0x0A
#define	FASE_B	0x0B
#define	FASE_C	0x0C

#endif /* __MAIN_H */

