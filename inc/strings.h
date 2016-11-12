#ifndef __STRING_TABLE_H
#define __STRING_TABLE_H

/* Cabeçalho de apresentação do projeto */
#define	HEADER1					"\n\r**********************************************\0"
#define	HEADER2					"\n\r Projeto GB - Circuitos Microprocessados\0"
#define	HEADER3					"\n\r Autores: Niccolas F. Cassel e Ingirdt Ayres\0"
#define	HEADER4					"\n\r**********************************************\0"
#define	SOLICITA_CONFIGURACAO	"\n\n\n\r Digite os dados da bateria:\n\n\r\0"

/* Comandos para configuração das grandezas utilizadas no controle do processo de carregamento */
#define	TENSAO_FLUTUACAO		"TNSFLT"
#define	TENSAO_CARGA			"TNSCRG"
#define	TEMP_SOBREAQ			"TEMPSB"
#define	TEMP_ALARME				"TEMPAL"
#define	SOBREAQ_TIME			"TIMESB"
#define	INVALIDO				"INVCMD"

/* Textos utilizados para interface com usuário */
#define	TENSAO_FLUTUACAO_CONFIGURADA	"\n\rTensao de flutuacao configurada:\0"
#define	TENSAO_CARGA_CONFIGURADA		"\n\rTensao de Carga configurada:\0" 
#define	TEMP_SOBREAQ_CONFIGURADA		"\n\rTemperatura de Sobreaquecimento configurada:\0"
#define	TEMP_ALARME_CONFIGURADA			"\n\rTemperatura Maxima configurada:\0"
#define	SOBREAQ_TIME_CONFIGURADO		"\n\rTempo de Sobreaquecimento configurado em minutos:\0"
#define	COMANDO_INVALIDO				"\n\rComando Invalido.\0"
#define	VALOR_ZERO						"\n\rO valor configurado nao pode ser zero.\0"
#define	DADOS_COMPUTADOS				"\n\rTodos os dados necessarios para controle do processo de carga."

#endif /* __STRING_TABLE_H */

