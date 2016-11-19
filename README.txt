# batterycharger
Projeto para o Grau B da cadeira de Circuito Microprocessados I

OBS.: Este projeto foi criado no ambiente do compilador Keil uVision 5

Autores: Níccolas F. Cassel e Ingridt Ayres
Kit de Avaliação da ´serie STM32 - NUCLEO F072RB

Histósico de Versões:

	v1.00 Beta 01
		- Criação do Projeto no Keil
		- Controle da leitura dos canais do A/D
		- Controle de temperatura
		
	v1.00 Beta 02
		- Apresentação do projeto através da USART2
		- Configuração de saída PWM utilizando o Timer 2 e o pino PB4
		- Implementada a lógica de controle da carga da bateria
		
	v1.00 Beta 03
		- Configuração do TIMER 3 para PWM e TIMER 2 paa base de tempo.
		- Implementação do protocolo de comunicação e tratamento dos comandos de configuração das variáveis de controle.
		
	v1.00 Beta 04
		- Correção das informações que devem ser inseridas pelo usuário para controle do processo de carga
		- Implementação do controle de PWM de acordo com a fase do processo
		- Melhorias no monitoramento de temperatura
