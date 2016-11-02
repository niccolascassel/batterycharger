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
unsigned char vBeta = 2;

/* Variáveis pra controle da base de tempo */
unsigned char dezMiliSeg = 0;
unsigned char segundos = 0;
unsigned char minutos = 0;

/* Variáveis de controle do processo de carga */
unsigned char leAD = FALSE;
unsigned char contLeAD = 0;
unsigned char avisoTemperatura = FALSE;
unsigned char alarmeTemperatura = FALSE;
unsigned char fase = FASE_A;
unsigned char tensaoCarga = 0;
unsigned char tensaoFlutuacao = 0;
unsigned char correnteCarga = 0;
unsigned char temperaturaAviso = 0;
unsigned char temperaturaAlarme = 0;
unsigned char pwmStartado = FALSE;

/* Variáveis para armazenamento dos valores pelo A/D */
unsigned char tensaoInstantanea = 0;
unsigned char tensaoMedia = 0;
unsigned char correnteInstantanea = 0;
unsigned char correnteMedia = 0;
unsigned char temperaturaInstantanea = 0;
unsigned char temperaturaMedia = 0;
unsigned char dadosBateriaComputados = TRUE;
unsigned char tempoSobreaquecimento = 0;

/* Variáveis para amostragem dos valores médios da grandezas monitoradas */
double tensao = 0;
double corrente = 0;
double temperatura = 0;

/* Cabeçalho de apresentação do projeto */
char Handler1[BUFFERSIZE] = "\n\r**********************************************\0";
char Handler2[BUFFERSIZE] = "\n\r Projeto GB - Circuitos Microprocessados\0";
char Handler3[BUFFERSIZE];
char Handler4[BUFFERSIZE] = "\n\r Autores: Niccolas F. Cassel e Ingirdt Ayres\0";
char Handler5[BUFFERSIZE] = "\n\r**********************************************\0";

/* Buffer da comunicacao serial */
char Buffer[BUFFERSIZE];

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
	
	/* ADC1 Channels 0, 1 and 4 GPIO pin configuration */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//    PA2     ------> USART2_TX
	//    PA3     ------> USART2_RX
    /*Configura os pinos GPIOs  para a função alternativa de RX e TX */
	GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Habilita o Clock no port do led,  função definida em stm32f0xx_hal_rcc.h*/
	__GPIOA_CLK_ENABLE();

	/* Configura  o pino do led como output push-pull */
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pin   = GPIO_PIN_5;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/* habilita o clock da  USART2 */
	__USART2_CLK_ENABLE();
    
	/*Configuração do Periférico USART
   	  - modo assincrono (UART Mode)
      - Word  = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
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
	
	/* Peripheral clock enable */
	__TIM3_CLK_ENABLE();
	
	/* Configura a variável prescaler com valor de contagem  para 10000 Hz */
	uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

	/* Configura TIM1 */
	TimHandle.Instance = TIM3;
	TimHandle.Init.Period            = 100 - 1;
	TimHandle.Init.Prescaler         = uwPrescalerValue;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&TimHandle);

	/* Configura a geração de interrupção para o timer 1 */
	HAL_TIM_Base_Start_IT(&TimHandle);
	
	/* Peripheral TIMER 2 interrupt init*/    
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
	
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

    /* Seleciona o canal analogico (Channel 0) */
    adcConfig.Channel      =  ADC_CHANNEL_1;
    adcConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
    adcConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    
	if(HAL_ADC_ConfigChannel(&AdcHandle, &adcConfig) != HAL_OK)
    {
		Error_Handler();
    }

    /* Seleciona o canal analogico (Channel 0) */
    adcConfig.Channel      =  ADC_CHANNEL_4;
    adcConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
    adcConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    
	if(HAL_ADC_ConfigChannel(&AdcHandle, &adcConfig) != HAL_OK)
    {
		Error_Handler();
    }
	
	/* Compute the prescaler value to have TIM3 counter clock equal to 16000000 Hz */
    uwPrescalerValue = (uint32_t)(SystemCoreClock / 16000000) - 1;
     
	/* Configuração  do periférico TIM3 como PWM
        + Prescaler = (SystemCoreClock / 16000000) - 1
        + Period = (666 - 1)
        + ClockDivision = 0
        + Counter direction = Up
	*/
	pwmTimHandle.Instance = TIM2;

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
	pwmConfig.Pulse = PULSE1_VALUE;
	HAL_TIM_PWM_ConfigChannel(&pwmTimHandle, &pwmConfig, TIM_CHANNEL_1);
	
	/* Apresentação do projeto */
	HAL_UART_Transmit(&huart2, (uint8_t*)Handler1, 50, 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)Handler2, 50, 100);
	
	if (vBeta > 0)
		sprintf(Handler3, "\n\r Carregador de Bateria v%d.%02d Beta %02d", vMajor, vMinor, vBeta);
	else
		sprintf(Handler3, "\n\r Carregador de Bateria v%d.%02d", vMajor, vMinor);
	
	HAL_UART_Transmit(&huart2, (uint8_t*)Handler3, 50, 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)Handler4, 50, 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)Handler5, 50, 100);
	
	sprintf(Buffer, "\n\n\n\r Digite os dados da bateria:\n\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*)Buffer, 50, 100);
      
	/* Infinite loop */
    while (1)
	{
		if (dadosBateriaComputados)
		{
			if (!pwmStartado)
			{
				/* Start PWM ignal */
				HAL_TIM_PWM_Start(&pwmTimHandle, TIM_CHANNEL_1);
				
				pwmStartado = TRUE;
			}
			
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
					}
					
					leAD = FALSE;	// Desabilita leitura dos canais do A/D
				}
				
				switch(fase)
				{
					case FASE_A:
						if (tensaoMedia < tensaoCarga)
						{
							if(correnteMedia < correnteCarga)
							{
								//Aumenta duty cicle
							}
							else if (correnteMedia > correnteCarga)
							{
								//Diminui duty cicle
							}
						}
						else
							fase = FASE_B;
						break;
					
					case FASE_B:
						if (correnteMedia > correnteCarga)
						{
							if (tensaoMedia < tensaoCarga)
							{
								// Aumenta duty cicle
							}
							else if (tensaoMedia > tensaoCarga)
							{
								// Diminui duty cicle
							}
						}
						break;
					
					case FASE_C:
						// Diminui duty cicle para 20%
						break;
				}
			}
		}
		else
		{
			HAL_UART_Receive(&huart2, (uint8_t*)Buffer, 50, 1000);
		}
	}

    return 0;
}

// Declaração Função de tratamento da Interrupção de Timer 
// Esta função vai ser chamada quando ocorrer o evento de interrupção do Timer 3
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	leAD = TRUE;	// Habilita leitura dos canais do A/D
	dezMiliSeg++;
	
	// Base de tempo de 1 segundo
	if (dezMiliSeg == 100)
	{
		dezMiliSeg = 0;
		
		// Pisca o led (Debug)
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		
		// Conta base de tempo do relógio
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
				
				if (minutos >= tempoSobreaquecimento)
				{
					minutos = 0;
					alarmeTemperatura = TRUE;
				}
			}
		}
		
		if (temperaturaMedia >= temperaturaAlarme)
			alarmeTemperatura = TRUE;
		else if (temperaturaMedia >= temperaturaAviso)
			avisoTemperatura = TRUE;
		else
		{
			avisoTemperatura = FALSE;
			alarmeTemperatura = FALSE;
			minutos = 0;
		}
	}
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


