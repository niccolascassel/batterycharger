#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_nucleo.h"

#define TRUE		1
#define FALSE		0

/* Size of buffer */
#define BUFFERSIZE	50

#define  PERIOD_VALUE       (uint32_t)(16000 - 1)  			/* Per�odo do sinal PWM  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE/10)     /* Duty Cicle do sinal PWM = 10% */

#define	QNTD_DE_AMOSTRAS_P_MEDIA	10

#define VREF	3.3

/* Valores referentes � medida de tens�o da bateria */
//#define	VBAT_RES_ALTO	39000
//#define	VBAT_RES_BAIXO	8200
//#define	VBAT_RELACAO	VBAT_RES_BAIXO / (VBAT_RES_ALTO + VBAT_RES_BAIXO);

/* Valores referentes � medida de corrente */
#define	CORRENTE_MAXIMA_DIGITAL		4096

/* Valores referentes � medida de temperatura */
#define	VARIACAO_LM35		0.01

/* Fases do processo de carga */
#define	FASE_A	0x0A
#define	FASE_B	0x0B
#define	FASE_C	0x0C

#define	TEMPO_SOBREAQUECIMENTO		10		// Tempo para sinaliza��o de alarme de temperatura
#define	FUNDO_ESCALA_TENSAO			19.00   // Tens�o m�xima poss�vel sobre a carga
#define	FUNDO_ESCALA_CORRENTE		1.00    // Corrente m�xima que pode ser medida  

#endif /* __MAIN_H */

