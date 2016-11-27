#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_nucleo.h"

#define TRUE		1
#define FALSE		0

/* Size of buffer */
#define BUFFERSIZE	50

#define  PERIOD_VALUE       (uint32_t)(16000 - 1)  			/* Período do sinal PWM  */
#define  PULSE_VALUE       (uint32_t)(PERIOD_VALUE/100)     /* Duty Cicle do sinal PWM = 10% */

#define	QNTD_DE_AMOSTRAS_P_MEDIA	10

#define VREF	3.3

/* Valores referentes à medida de tensão da bateria */
#define	VBAT_RES_ALTO	39000
#define	VBAT_RES_BAIXO	8200

/* Valores referentes à medida de corrente */
#define	CORRENTE_MAXIMA_DIGITAL		4096

/* Valores referentes à medida de temperatura */
#define	VARIACAO_LM35		0.01

/* Fases do processo de carga */
#define	FASE_A	'A'
#define	FASE_B	'B'
#define	FASE_C	'C'

#define	TEMPO_SOBREAQUECIMENTO		10		// Tempo para sinalização de alarme de temperatura 

#endif /* __MAIN_H */

