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
#define	TEMP_CARGA				"TEMPCR"
#define	CORRENTE_CARGA			"CORCRG"
#define	CORRENTE_FLUTUACAO		"CORFLT"
#define	INVALIDO				"INVCMD"

/* Textos utilizados para interface com usuário */
#define	TENSAO_FLUTUACAO_CONFIGURADA	"\n\rTensao de Flutuacao configurada:\0"
#define	TENSAO_CARGA_CONFIGURADA		"\n\rTensao de Carga configurada:\0" 
#define	TEMP_CARGA_CONFIGURADA			"\n\rTemperatura de Carga configurada:\0"
#define	CORRENTE_CARGA_CONFIGURADA		"\n\rCorrente de Carga configurada:\0"
#define	CORRENTE_FLUTUACAO_CONFIGURADA	"\n\rCorrente de Flutuacao configurada:\0"
#define	COMANDO_INVALIDO				"\n\rComando Invalido.\0"
#define	VALOR_ZERO						"\n\rO valor configurado nao pode ser zero.\0"
#define	DADOS_COMPUTADOS				"\n\rTodos os dados necessarios para controle do processo de carga foram computados."

#endif /* __STRING_TABLE_H */

