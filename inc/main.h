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

#define VREF	3.3

/* Valores referentes à medida de tensão da bateria */
#define	VBAT_RES_ALTO	39000
#define	VBAT_RES_BAIXO	10000
#define	VBAT_RELACAO	VBAT_RES_BAIXO / (VBAT_RES_ALTO + VBAT_RES_BAIXO);

/* Valores referentes à medida de corrente */
#define	CORRENTE_MAXIMA		1

/* Valores referentes à medida de temperatura */
#define	VARIACAO_LM35		0.01

/* Fases do processo de carga */
#define	FASE_A	0x0A
#define	FASE_B	0x0B
#define	FASE_C	0x0C

#endif /* __MAIN_H */

