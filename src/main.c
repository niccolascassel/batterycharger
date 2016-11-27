/*
  ******************************************************************************
  Circuitos Microprocessados
  Níccolas F. Cassel
  Relogio com Timer como base de tempo
  ******************************************************************************
 */
 
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "main.h"
#include "strings.h"

/* Struct de configuração da comunicação serial definido em  stm32f0xx_hal_uart.h*/
UART_HandleTypeDef huart2;

/* Struct definida em  stm32f0xx_hal_tim.h para configrar o timer*/
TIM_HandleTypeDef TimHandle;

/* Struct definida em  stm32f0xx_hal_tim.h para configrar o pwm*/
TIM_HandleTypeDef pwmTimHandle;

/* Struct definida em  stm32f0xx_hal_gpio.h para configrar o port*/
static GPIO_InitTypeDef GPIO_InitStruct;

/* ADC configuration structure declaration */
ADC_HandleTypeDef AdcHandle;

/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef adcConfig;

/* Struct de configuração da saida do Timer */
TIM_OC_InitTypeDef pwmConfig;

/* Variáveis para armazenamento da versão */
unsigned char vMajor = 1;
unsigned char vMinor = 0;
unsigned char vBeta = 0;

/* Variáveis pra controle da base de tempo */
volatile unsigned char dezMiliSeg = 0;
volatile unsigned char quinhentosMiliseg = 0;
volatile unsigned char segundos = 0;
volatile unsigned char minutos = 0;

/* Variáveis de controle do processo de carga */
volatile unsigned char leAD = FALSE;
unsigned char contLeAD = 0;
volatile unsigned char avisoTemperatura = FALSE;
volatile unsigned char alarmeTemperatura = FALSE;
volatile char fase = FASE_A;
double tensaoCarga = 0;
double tensaoFlutuacao = 0;
double correnteCarga = 0;
double correnteFlutuacao = 0;
volatile double temperaturaCarga = 0;
unsigned char aumentouDutyCicle = FALSE;
volatile unsigned char verificaFase = TRUE;

/* Variáveis para armazenamento dos valores lidos pelo A/D */
unsigned int tensaoInstantanea = 0;
unsigned int tensaoMedia = 0;
unsigned int correnteInstantanea = 0;
unsigned int correnteMedia = 0;
unsigned int temperaturaInstantanea = 0;
volatile unsigned int temperaturaMedia = 0;
unsigned char dadosBateriaComputados = FALSE;

/* Variáveis para amostragem dos valores médios da grandezas monitoradas */
double tensao = 0;
double corrente = 0;
double temperatura = 0;

/* Buffer da comunicacao serial */
char Buffer[BUFFERSIZE];
volatile uint8_t  pacoteSerial[11];
volatile unsigned char pacoteRecebido = FALSE;

/* Declaração da variável Prescaler */
uint32_t uwPrescalerValue = 0;

static void SystemClock_Config(void);
static void Error_Handler(void);

int main(void) 
{
	/* Inicializa as bibliotecas HAL */
	HAL_Init();

	/* Configura o clock do sistema */
	SystemClock_Config();
	
	/* Habilita o Clock no port do led,  função definida em stm32f0xx_hal_rcc.h*/
	__GPIOA_CLK_ENABLE();
	
	//    PA0     ------> ADC CH0
	//    PA1     ------> ADC CH1
	//    PA4     ------> ADC CH4
	/* Configura os pinos GPIOs  para a função alternativa de canais de A/D - CH0, CH1 e CH4 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//    PA2     ------> USART2_TX
	//    PA3     ------> USART2_RX
    /* Configura os pinos GPIOs  para a função alternativa de RX e TX */
	GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/* Configura  o pino do led como output push-pull */
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pin   = GPIO_PIN_5;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/* Habilita o Clock no port C,  função definida em stm32f0xx_hal_rcc.h*/
    __GPIOB_CLK_ENABLE();

	//    PB4     ------> TIM3_CH1
    /* Configura os pinos GPIOs  para a função alternativa de saída PWM - TIM3_CH1 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    GPIO_InitStruct.Pin = GPIO_PIN_4;
	
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/* habilita o clock da  USART2 */
	__USART2_CLK_ENABLE();
    
	/*Configuração do Periférico USART
   	  - modo assincrono (UART Mode)
      - Word  = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
	huart2.Instance        = USART2;
	huart2.Init.BaudRate   = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits   = UART_STOPBITS_1;
	huart2.Init.Parity     = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	huart2.Init.Mode       = UART_MODE_TX_RX;
    
	if(HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	
	/* Habilita e configura a priodidade da interrupção de transmissão da USART2 */
    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
	
	/* Habilita o clock do ADC 1*/
	__ADC1_CLK_ENABLE();

	/* Configuração  do periférico ADC */
		/*
		*  Instance                  = ADC1.
		*  ClockPrescaler            = PCLK divided by 4.
		*  LowPowerAutoWait          = Disabled
		*  LowPowerAutoPowerOff      = Disabled
		*  Resolution                = 12 bit (increased to 16 bit with oversampler)
		*  ScanConvMode              = ADC_SCAN_ENABLE
		*  DataAlign                 = Right
		*  ContinuousConvMode        = Enabled
		*  DiscontinuousConvMode     = Enabled
		*  ExternalTrigConv          = ADC_SOFTWARE_START
		*  ExternalTrigConvEdge      = None (Software start)
		*  EOCSelection              = End Of Conversion event
		*  DMAContinuousRequests     = Disabled
		*/

	AdcHandle.Instance = ADC1;
    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC;
    AdcHandle.Init.LowPowerAutoWait      = DISABLE;
    AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
    AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;
    AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle.Init.ContinuousConvMode    = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = ENABLE;
    AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    AdcHandle.Init.EOCSelection          = EOC_SINGLE_CONV;
    AdcHandle.Init.DMAContinuousRequests = DISABLE;
    AdcHandle.Init.Overrun               = OVR_DATA_OVERWRITTEN;
    AdcHandle.NbrOfConversionRank = 3;

    /* Inicilaiza  o ADC  com as configurações*/
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
    {
		Error_Handler();
    }

    /*Calibra o ADC com as configurações */
    if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
	{
		Error_Handler();
	}

    /* Seleciona o canal analogico (Channel 0) */
    adcConfig.Channel      = ADC_CHANNEL_0;
    adcConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
    adcConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;  // AUMENTA O TEMPO DE AMOSTRAGEM, RESOLVE O PROBLEMA DE INSTABILIDADE

    if(HAL_ADC_ConfigChannel(&AdcHandle, &adcConfig) != HAL_OK)
    {
		Error_Handler();
    }

    /* Seleciona o canal analogico (Channel 1) */
    adcConfig.Channel      =  ADC_CHANNEL_1;
    adcConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
    adcConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    
	if(HAL_ADC_ConfigChannel(&AdcHandle, &adcConfig) != HAL_OK)
    {
		Error_Handler();
    }

    /* Seleciona o canal analogico (Channel 4) */
    adcConfig.Channel      =  ADC_CHANNEL_4;
    adcConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
    adcConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    
	if(HAL_ADC_ConfigChannel(&AdcHandle, &adcConfig) != HAL_OK)
    {
		Error_Handler();
    }
	
	/* Habilita o clock do Timer 3*/
    __TIM3_CLK_ENABLE();
	
	/* Compute the prescaler value to have TIM3 counter clock equal to 16000000 Hz */
    uwPrescalerValue = (uint32_t)(SystemCoreClock / 16000000) - 1;


	/* Configuração  do periférico TIM3 como PWM
        + Prescaler = (SystemCoreClock / 16000000) - 1
        + Period = (666 - 1)
        + ClockDivision = 0
        + Counter direction = Up
	*/
	pwmTimHandle.Instance = TIM3;
	pwmTimHandle.Init.Prescaler         = uwPrescalerValue;
	pwmTimHandle.Init.Period            = PERIOD_VALUE;
	pwmTimHandle.Init.ClockDivision     = 0;
	pwmTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	pwmTimHandle.Init.RepetitionCounter = 0;
	
	HAL_TIM_PWM_Init(&pwmTimHandle);

	/* Configure the PWM channels*/
	pwmConfig.OCMode       = TIM_OCMODE_PWM1;
	pwmConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	pwmConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	pwmConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	pwmConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
	pwmConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	/* Set the pulse value for channel 1  PB4*/
	pwmConfig.Pulse = PULSE_VALUE;
	HAL_TIM_PWM_ConfigChannel(&pwmTimHandle, &pwmConfig, TIM_CHANNEL_1);
	
	/* Start PWM ignal */
	HAL_TIM_PWM_Start(&pwmTimHandle, TIM_CHANNEL_1);
   
	/* Peripheral clock enable */
	__TIM2_CLK_ENABLE();
	
	/* Configura a variável prescaler com valor de contagem  para 10000 Hz */
	uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

	/* Configura TIM1 */
	TimHandle.Instance = TIM2;
	TimHandle.Init.Period            = 100 - 1;
	TimHandle.Init.Prescaler         = uwPrescalerValue;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&TimHandle);

	/* Configura a geração de interrupção para o timer 1 */
	HAL_TIM_Base_Start_IT(&TimHandle);
	
	/* Peripheral TIMER 2 interrupt init*/    
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
	
	/* Apresentação do projeto */
	HAL_UART_Transmit(&huart2, (uint8_t*)HEADER1, sizeof(HEADER1), 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)HEADER2, sizeof(HEADER2), 100);
	
	if (vBeta > 0)
		sprintf(Buffer, "\n\r Carregador de Bateria v%d.%02d Beta %02d", vMajor, vMinor, vBeta);
	else
		sprintf(Buffer, "\n\r Carregador de Bateria v%d.%02d", vMajor, vMinor);
	
	HAL_UART_Transmit(&huart2, (uint8_t*)Buffer, 50, 100);
	
	for(unsigned char i = 0; i < sizeof(Buffer)/sizeof(Buffer[0]); i++)
		Buffer[i] = '\0';
	
	HAL_UART_Transmit(&huart2, (uint8_t*)HEADER3, sizeof(HEADER3), 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)HEADER4, sizeof(HEADER4), 100);
	
	HAL_UART_Transmit(&huart2, (uint8_t*)SOLICITA_CONFIGURACAO, sizeof(SOLICITA_CONFIGURACAO), 100);
      
	/* Infinite loop */
    while (1)
	{
		if (dadosBateriaComputados)
		{
			if (!alarmeTemperatura)
			{
				if (leAD)
				{
					// Le o valor do primeiro canal - Tensao
					HAL_ADC_Start(&AdcHandle); // Inicia a amostragem do primeiro canal
					if (HAL_ADC_PollForConversion(&AdcHandle, 1000000) == HAL_OK)
					{    		 
						tensaoInstantanea += HAL_ADC_GetValue(&AdcHandle);
					}

					// Le o valor do segundo canal - Corrente
					HAL_ADC_Start(&AdcHandle); // Inicia a amostragem do segundo canal
					if (HAL_ADC_PollForConversion(&AdcHandle, 1000000) == HAL_OK)
					{						 
						correnteInstantanea += HAL_ADC_GetValue(&AdcHandle);
					}

					// Le o valor do terceiro canal - Temperatura
					HAL_ADC_Start(&AdcHandle); // Inicia a amostragem do teceiro canal
					if (HAL_ADC_PollForConversion(&AdcHandle, 1000000) == HAL_OK)
					{
						temperaturaInstantanea += HAL_ADC_GetValue(&AdcHandle);
					}

					HAL_ADC_Stop(&AdcHandle);
		 
					contLeAD++;
					
					// Se atingiu a quantidade de amostras, calcula a media
					if (contLeAD == QNTD_DE_AMOSTRAS_P_MEDIA)
					{
						tensaoMedia = tensaoInstantanea / QNTD_DE_AMOSTRAS_P_MEDIA;						
						correnteMedia = correnteInstantanea / QNTD_DE_AMOSTRAS_P_MEDIA;
						temperaturaMedia = temperaturaInstantanea / QNTD_DE_AMOSTRAS_P_MEDIA;
						
						tensaoInstantanea = 0;
						correnteInstantanea = 0;
						temperaturaInstantanea = 0;
						
						contLeAD = 0;
						
						tensao = (((double)VREF * tensaoMedia) / 4096) / ((double)VBAT_RES_BAIXO / (VBAT_RES_ALTO + VBAT_RES_BAIXO));
						corrente = (double)correnteMedia / 4096;
						temperatura = (double)(((VREF / 4096) * (double)temperaturaMedia) / VARIACAO_LM35);
						                                                    
						sprintf(Buffer, "\n\r%.02lfV\t%.02lfA\t%.02lfºC\t FASE %c", tensao, corrente, temperatura, fase);  
						
						HAL_UART_Transmit(&huart2, (uint8_t*)Buffer, sizeof(Buffer), 100);
					}
					
					leAD = FALSE;	// Desabilita leitura dos canais do A/D
				}
				
				if (verificaFase)
				{
					switch(fase)
					{
						case FASE_A:
							if (tensaoMedia < tensaoCarga)
							{
								if(correnteMedia < correnteCarga)
								{
									// Não permite que aumente o duty cicle se atingiu corrente máxima
									if (correnteMedia < CORRENTE_MAXIMA_DIGITAL) 
									{
										if ((pwmConfig.Pulse + PULSE_VALUE) < PERIOD_VALUE)
										{
											// Incrementa duty cicle em 1%
											pwmConfig.Pulse += PULSE_VALUE;
											HAL_TIM_PWM_ConfigChannel(&pwmTimHandle, &pwmConfig, TIM_CHANNEL_1);
											HAL_TIM_PWM_Start(&pwmTimHandle, TIM_CHANNEL_1);
											aumentouDutyCicle = TRUE;
											verificaFase = FALSE;
										}
									}
								}
								else if (correnteMedia > correnteCarga)
								{
									if ((pwmConfig.Pulse - PULSE_VALUE) > 1)
									{
										//Decrementa duty cicle em 1%
										pwmConfig.Pulse -= PULSE_VALUE;
										HAL_TIM_PWM_ConfigChannel(&pwmTimHandle, &pwmConfig, TIM_CHANNEL_1);
										HAL_TIM_PWM_Start(&pwmTimHandle, TIM_CHANNEL_1);
										aumentouDutyCicle = FALSE;
										verificaFase = FALSE;
									}
								}
							}
							else
							{
								fase = FASE_B;
							}
							break;
						
						case FASE_B:
							if (correnteMedia > correnteFlutuacao)
							{	
								if (tensaoMedia < tensaoCarga)
								{
									// Não permite que aumente o duty cicle se atingiu corrente máxima
									if (correnteMedia < CORRENTE_MAXIMA_DIGITAL)
									{
										if ((pwmConfig.Pulse + PULSE_VALUE) < PERIOD_VALUE)
										{
											// Incrementa duty cicle em 1%
											pwmConfig.Pulse += PULSE_VALUE;
											HAL_TIM_PWM_ConfigChannel(&pwmTimHandle, &pwmConfig, TIM_CHANNEL_1);
											HAL_TIM_PWM_Start(&pwmTimHandle, TIM_CHANNEL_1);
											aumentouDutyCicle = TRUE;
											verificaFase = FALSE;
										}
									}
								}
								else if (tensaoMedia > tensaoCarga)
								{
									if ((pwmConfig.Pulse - PULSE_VALUE) > 1)
									{
										//Decrementa duty cicle em 1%
										pwmConfig.Pulse -= PULSE_VALUE;
										HAL_TIM_PWM_ConfigChannel(&pwmTimHandle, &pwmConfig, TIM_CHANNEL_1);
										HAL_TIM_PWM_Start(&pwmTimHandle, TIM_CHANNEL_1);
										aumentouDutyCicle = FALSE;
										verificaFase = FALSE;
									}
								}
							}
							else
							{
								fase = FASE_C;
							}
							break;
						
						case FASE_C:
							if (tensaoMedia < tensaoFlutuacao)
							{
								// Não permite que aumente o duty cicle se atingiu corrente máxima
								if (correnteMedia < CORRENTE_MAXIMA_DIGITAL)
								{
									if ((pwmConfig.Pulse + PULSE_VALUE) < PERIOD_VALUE)
									{
										// Incrementa duty cicle em 1%
										pwmConfig.Pulse += PULSE_VALUE;
										HAL_TIM_PWM_ConfigChannel(&pwmTimHandle, &pwmConfig, TIM_CHANNEL_1);
										HAL_TIM_PWM_Start(&pwmTimHandle, TIM_CHANNEL_1);
										aumentouDutyCicle = TRUE;
										verificaFase = FALSE;
									}
								}
							}
							else if (tensaoMedia > tensaoFlutuacao)
							{
								if ((pwmConfig.Pulse - PULSE_VALUE) > 1)
								{
									//Decrementa duty cicle em 1%
									pwmConfig.Pulse -= PULSE_VALUE;
									HAL_TIM_PWM_ConfigChannel(&pwmTimHandle, &pwmConfig, TIM_CHANNEL_1);
									HAL_TIM_PWM_Start(&pwmTimHandle, TIM_CHANNEL_1);
									aumentouDutyCicle = FALSE;
									verificaFase = FALSE;
								}
							}
							break;
					}
				}
			}
		}
		else
		{	
			// Se recebeu um pacote pela serial, verifica o que foi que recebeu
			if (pacoteRecebido)
			{
				char cmd[6];
				char value[5];
				
				// Copia as posições referentes ao comando
				for(unsigned char i = 0; i < sizeof(cmd)/sizeof(cmd[0]); i++)
					cmd[i] = pacoteSerial[i];
				
				// Copia as posições referentes ao valor
				for(unsigned char i = 0; i < sizeof(value)/sizeof(value[0]); i++)
					value[i] = pacoteSerial[i + sizeof(cmd)/sizeof(cmd[0])];
										
				// Verifica se o comando recebido é de configuracao de tensao de flutuação
				if (!strcmp(cmd, TENSAO_FLUTUACAO)) 
				{
					// Converte e copia o valor para variável referente
					sscanf(value, "%lf", &tensaoFlutuacao);
					
					// Se o valor enviado pela serial para tensão de flutuação for zero. Não armazena.
					if (tensaoFlutuacao == 0)
						HAL_UART_Transmit(&huart2, (uint8_t*)VALOR_ZERO, sizeof(VALOR_ZERO), 100);
					else
					{						
						HAL_UART_Transmit(&huart2, (uint8_t*)TENSAO_FLUTUACAO_CONFIGURADA, sizeof(TENSAO_FLUTUACAO_CONFIGURADA), 100);
						
						sprintf(Buffer, "\n\r%.02lfV", tensaoFlutuacao); 
						HAL_UART_Transmit(&huart2, (uint8_t*)Buffer, sizeof(Buffer), 100);
						
						tensaoFlutuacao = (4096 / VREF) * (tensaoFlutuacao * ((double)VBAT_RES_BAIXO / (VBAT_RES_ALTO + VBAT_RES_BAIXO)));
					}
				}
				// Se não, verifica se o comando recebido é de configuracao de tensao de carga
				else if (!strcmp(cmd, TENSAO_CARGA))
				{
					// Converte e copia o valor para variável referente
					sscanf(value, "%lf", &tensaoCarga);
					
					// Se o valor enviado pela serial para tensão de carga for zero. Não armazena.
					if (tensaoCarga == 0)
						HAL_UART_Transmit(&huart2, (uint8_t*)VALOR_ZERO, sizeof(VALOR_ZERO), 100);
					else
					{
						HAL_UART_Transmit(&huart2, (uint8_t*)TENSAO_CARGA_CONFIGURADA, sizeof(TENSAO_CARGA_CONFIGURADA), 100);
						
						sprintf(Buffer, "\n\r%.02lfV", tensaoCarga); 
						HAL_UART_Transmit(&huart2, (uint8_t*)Buffer, sizeof(Buffer), 100);
						
						tensaoCarga = (4096 / VREF) * (tensaoCarga * ((double)VBAT_RES_BAIXO / (VBAT_RES_ALTO + VBAT_RES_BAIXO)));
					}
				}
				// Se não, verifica se o comando recebido é de configuracao de corrente de carga.
				else if (!strcmp(cmd, CORRENTE_CARGA))
				{
					sscanf(value, "%lf", &correnteCarga);
					
					// Se o valor enviado pela serial para temperatura de aviso de sobreaquecimento for zero. Não armazena.
					if (correnteCarga == 0)
						HAL_UART_Transmit(&huart2, (uint8_t*)VALOR_ZERO, sizeof(VALOR_ZERO), 100);
					else
					{
						HAL_UART_Transmit(&huart2, (uint8_t*)CORRENTE_CARGA_CONFIGURADA, sizeof(CORRENTE_CARGA_CONFIGURADA), 100);
						
						sprintf(Buffer, "\n\r%.02lfA", correnteCarga); 
						HAL_UART_Transmit(&huart2, (uint8_t*)Buffer, sizeof(Buffer), 100);
						
						correnteCarga = 4096 * correnteCarga;
					}
				}
				// Se não, verifica se o comando recebido é de configuracao de corrente de flutuação.
				else if (!strcmp(cmd, CORRENTE_FLUTUACAO))
				{
					// Armazenamento da temperatura de alarme
					//HAL_UART_Transmit(&huart2, (uint8_t*)TEMP_ALARME, sizeof(TEMP_ALARME), 100);	
					// Converte e copia o valor para variável referente					
					sscanf(value, "%lf", &correnteFlutuacao);
					
					// Se o valor enviado pela serial para temperatura de alarme de sobreaquecimentolutuação for zero. Não armazena.
					if (correnteFlutuacao == 0)
						HAL_UART_Transmit(&huart2, (uint8_t*)VALOR_ZERO, sizeof(VALOR_ZERO), 100);
					else
					{
						HAL_UART_Transmit(&huart2, (uint8_t*)CORRENTE_FLUTUACAO_CONFIGURADA, sizeof(CORRENTE_FLUTUACAO_CONFIGURADA), 100);
						
						sprintf(Buffer, "\n\r%.02lfA", correnteFlutuacao); 
						HAL_UART_Transmit(&huart2, (uint8_t*)Buffer, sizeof(Buffer), 100);
						
						correnteFlutuacao = 4096 * correnteFlutuacao;
					}
				}
				// Se não, verifica se o comando recebido é de configuracao do temperatura de carga.
				else if (!strcmp(cmd, TEMP_CARGA))
				{
					// Armazenamento do tempo de sobreaquecimento
					//HAL_UART_Transmit(&huart2, (uint8_t*)SOBREAQ_TIME, sizeof(SOBREAQ_TIME), 100);
					// Converte e copia o valor para variável referente
					sscanf(value, "%lf", &temperaturaCarga);
					
					// Se o valor enviado pela serial para tempo de sobreaquecimento for zero. Não armazena.
					if (temperaturaCarga == 0)
						HAL_UART_Transmit(&huart2, (uint8_t*)VALOR_ZERO, sizeof(VALOR_ZERO), 100);
					else
					{
						HAL_UART_Transmit(&huart2, (uint8_t*)TEMP_CARGA_CONFIGURADA, sizeof(TEMP_CARGA_CONFIGURADA), 100);
						
						sprintf(Buffer, "\n\r%.02lfºC", temperaturaCarga); 
						HAL_UART_Transmit(&huart2, (uint8_t*)Buffer, sizeof(Buffer), 100);
						
						temperaturaCarga = (4096 / (VREF / VARIACAO_LM35)) * temperaturaCarga;
					}
				}
				else // Se não, indica que o comando é inválido
				{
					// Comando inválido
					HAL_UART_Transmit(&huart2, (uint8_t*)COMANDO_INVALIDO, sizeof(COMANDO_INVALIDO), 100);
				}
				
				// Limpa o buffer
				for(unsigned char i = 0; i < sizeof(Buffer)/sizeof(Buffer[0]); i++)
					Buffer[i] = '\0';
							
				pacoteRecebido = FALSE;
			}
			
			// Se todos os valores foram computados, libera inicio do controle
			if (tensaoFlutuacao > 0 && tensaoCarga > 0 && correnteCarga > 0 && correnteFlutuacao > 0 && temperaturaCarga > 0)
			{
				HAL_UART_Transmit(&huart2, (uint8_t*)DADOS_COMPUTADOS, sizeof(DADOS_COMPUTADOS), 100);
				dadosBateriaComputados = TRUE;
			}
		}
		
		HAL_UART_Receive_IT(&huart2, (uint8_t*)pacoteSerial, sizeof(pacoteSerial));
	}

    return 0;
}

// Declaração Função de tratamento da Interrupção de Timer 
// Esta função vai ser chamada quando ocorrer o evento de interrupção do Timer 1
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	leAD = TRUE;	// Habilita leitura dos canais do A/D
	dezMiliSeg++;
	
	// Base de tempo de 1 segundo
	if (dezMiliSeg == 50)
	{
		dezMiliSeg = 0;
		quinhentosMiliseg++;
		
		verificaFase = TRUE;
		
		if (alarmeTemperatura)
		{
			// Pica o LED Verde para sinalizar alarme de temperatura elevada
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		}
		
		if (quinhentosMiliseg == 2)
		{
		    quinhentosMiliseg = 0;
			segundos++;
		
			// Base de tempo de 1 minuto
			if (segundos == 60)
			{
				segundos = 0;

				if (fase == FASE_C)
					fase = FASE_B;
			
				if (avisoTemperatura)
				{
					minutos++;
				
					if (minutos >= TEMPO_SOBREAQUECIMENTO)
					{
						minutos = 0;
						alarmeTemperatura = TRUE;
					}
				}
			}
		}
		
		if (dadosBateriaComputados)
		{
			if (temperaturaMedia >= (temperaturaCarga * 1.1))
				alarmeTemperatura = TRUE;
			else if (temperaturaMedia >= temperaturaCarga)
			{
				if (!alarmeTemperatura)
				{
					avisoTemperatura = TRUE;
				
					// Ascende o LED Verde para sinalizar sobreaquecimento
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, TRUE);
				}
			}
			else
			{
				avisoTemperatura = FALSE;
				alarmeTemperatura = FALSE;
				minutos = 0;
				
				// Apaga o LED Verde
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, FALSE);
			}
		}
	}
}

// Declaração Função de tratamento da Interrupção de Recepção da USART2 
// Esta função vai ser chamada quando ocorrer o evento de interrupção do canal de recepção da porta serial 2
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
	pacoteRecebido = TRUE;
}

/* Função de configuração do clock */
static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* No HSE Oscillator on Nucleo, Activate PLL with HSI/2 as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
	{	
		Error_Handler();
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
	{
		Error_Handler();
	}
}

/* Função chamada no caso de erro na configuração */
static void Error_Handler(void)
{
	while(1)
	{		
		/* Pisca o LED Verde 2 vezes a cada segundo */
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(1000);
	}
}


