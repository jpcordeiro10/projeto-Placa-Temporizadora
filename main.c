/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include <stdio.h>
#include "stdlib.h"
#include <string.h>
#include "fonts.h"
#include "ssd1306.h"
#include "test.h"
#include "logoRossi.h"


#define		Off											0
#define		On											1
#define		HIGH										0
#define		LOW											1
#define 	TempoReleLigado					500 // Tempo q rele permanece ligado [ms]
#define		debounceDelay						200
#define 	TempoPortaoPassarReed		500
#define 	DelayAjuste 						5000
#define		TempoTermico 						5000
#define 	FLASH_USER_START_ADDR   ((uint32_t)0x08008000)   /*0x08008000 Start @ of user Flash area */
#define 	FLASH_TYPEPROGRAM_HALFWORD   	0x00000001U
#define 	FLASH_USER_END_ADDR (FLASH_USER_START_ADDR-1)
#define FLASH_ADDRESS 0x08008000
//#define Address1	 ((uint32_t)0x08008000)
//#define Address2 ((uint32_t)0x0800801D)
//#define Address3 ((uint32_t)0x0800803A)
#define delayGravacao 	4000
//uint32_t Address;
uint32_t AddressLeitura;
int tempoMudou = 0;
char TempoDisplay[10];
char ReedDisplay[10];
float auxTempoSetado = 0;
float auxTempoReleLigado = 0;
int EstadoRele = Off;
int Tempo;
int lock, lock1, lock2, lock3, lock6, lock7 = 0;
int botaomais, botaomenos = 1;
uint32_t lastDebounceTime = 0; // a ultima vez que o pino de saida foi modificado (toggled)
uint32_t millis = 0;
uint32_t millisReset = 0;
uint32_t millisAtual = 0;
uint32_t millisRele = 0;
uint32_t VariavelDebounceMais, VariavelDebounceMenos = 0;
uint32_t delayReleSetado = 0;
uint32_t timeCountReset = 5000;
uint32_t millisAnterior = 0;
uint32_t DebounceReed = 0;
uint32_t Cabecalho = 0;
uint32_t DelayComecarCiclagem = 0; 
uint32_t paradaTermica = 0;
int TempoAnterior = 0;
int FlagLimparTela = 0;
int TravaLimpaTela = 0;
int TravaContagem = 0;
int flagTermico = 0;
int cadeadoAjuste = 0;
int flagCabecalho = 1;
int k = 0;
int flagReset = 0;
int Reed = 0;
int contaReed;
int ciclosPortao = 0;
int cadeadoCabecalho = 0;
int cadeadoReed = 0;
float millisDebounce = 0;
int buttonState;
int lastButtonState;
int ReadingBotaoMenos, ReadingBotaoMais;
int cont;
char contChar[10];
char auxChar[10] = {'0','1','2','3','4','5','6','7','8','9'};
uint32_t delayEprom = 0;
uint8_t TempoEprom;

// Eemprom
uint32_t address;
uint64_t data;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);

static FLASH_EraseInitTypeDef EraseInitStruct = {0};

static uint32_t GetPage(uint32_t Address);

uint32_t FirstPage = 0, NbOfPages = 0;
uint32_t Address = 0; 
uint32_t PageError = 0;
uint32_t Address2, Address3;
	__IO uint32_t MemoryProgramStatus = 0;
	__IO uint32_t data32 = 0;
uint32_t dataCont, dataReed, dataTempo = 0;
uint64_t dataEprom[3] = {0};
/*
posicao 0 -> tempo
posicao 1 -> rele
posicao 2 -> reed
*/

/**
  * @brief  The application entry point.
  * @retval int
  */
static uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}
	
int main(void) {
  
  HAL_Init();

  
  SystemClock_Config();

  
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
	
	HAL_Delay(100); // Delay necessário pois a função de inicialização do display não estava fncionando corretamente
	SSD1306_Init(); // initialize the display 
	/*
	for (uint32_t i = 0; i < 3; i++)
  {
    dataEprom[i] = *(uint32_t *)(FLASH_ADDRESS + (i * sizeof(uint32_t)));
  }
	Dava pra usar for mas eu preferi fazer na mao pq sao so tres lacos
	*/
	
	AddressLeitura = FLASH_ADDRESS;
	dataEprom[0] = *(uint32_t*)(AddressLeitura);
	AddressLeitura += 8;
	dataEprom[1] = *(uint32_t*)(AddressLeitura);
	AddressLeitura += 8;
	dataEprom[2] = *(uint32_t*)(AddressLeitura);
	
	Tempo = dataEprom[0];
	cont = dataEprom[1];
	contaReed = dataEprom[2];
	
	if(Tempo < 0){
		Tempo = 0;
	}
	if(cont < 0){
		cont = 0;
	}
	if(contaReed < 0){
		contaReed = 0;
	}
	
	SSD1306_DrawBitmap(0,0,epd_bitmap_logoRossi128x64,128,64,1);
	SSD1306_UpdateScreen(); // update screen
	HAL_Delay(5000);
	SSD1306_Clear();
	/* Se quiser apresentar o coracao na inicializacao do display so descomentar */
	SSD1306_DrawBitmap(0,0,epd_bitmap_coracao,128,64,1);
	SSD1306_UpdateScreen(); // update screen
	HAL_Delay(5000);
	SSD1306_Clear();
  
	//epd_bitmap_coracao
  while (1)
  {
    millis = HAL_GetTick(); // Pega o valor em milissegundos do tempo de funcionamento do processador (estoura com 49 dias)
	
		ReadingBotaoMais = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10); // Lê o botão de incremento do tempo
		
		TempoAnterior = Tempo;
		
		if(!ReadingBotaoMais && !lock2){ // Se o botão estiver pressionado e o cadeado liberado entra na condição
			Tempo++; // Incrementa a variavel int do tempo
			VariavelDebounceMais = millis + debounceDelay; // Grava o valor atual do tempo + o tempo de debouncin
			lock2 = 1; // Trava o cadeado
			DelayComecarCiclagem = millis + DelayAjuste;
			cadeadoAjuste = 1;
			tempoMudou = 1;
			delayEprom = millis + delayGravacao;
		}
		
		if(millis >= VariavelDebounceMais){ // Se o valor de tempo do processador atingiu o valor de tempo de debouncing, destrava o cadeado		
			if(ReadingBotaoMais){
			lock2 = 0;
			}
			
		}
		// Realiza a mesma rotina, só que para o botão de decremento do tempo
		ReadingBotaoMenos = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
		if(!ReadingBotaoMenos && !lock3){
			Tempo--;
			VariavelDebounceMenos = millis + debounceDelay;
			lock3 = 1;
			DelayComecarCiclagem = millis + DelayAjuste;
			cadeadoAjuste = 1;
			tempoMudou = 1;
			delayEprom = millis + delayGravacao;
		}
		if(millis >= VariavelDebounceMenos){
			lock3 = 0;
		}
		// -------
		if(millis >= DelayComecarCiclagem){ // Cadeado para o rele aguardar 5 segundos apos mudanca de tempo
			cadeadoAjuste = 0;
		}
	
		Reed = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
//		if(Reed && !lock7 && (millis >= auxTempoSetado) && Tempo != 0 && !cadeadoAjuste){
		if(Reed && !lock7 && Tempo != 0 && !cadeadoAjuste && !cadeadoReed){
			contaReed++;
			DebounceReed = millis + TempoPortaoPassarReed;
			lock7 = 1;
			cadeadoReed = 1;
		}
		if(millis >= DebounceReed){
			lock7 = 0;
		}
		if(Reed && !TravaContagem && Tempo != 0){ // Se reed igual 1 comeca grava o tempo futuro para verificar se o motor parou em cima do ima
			paradaTermica = millis + TempoTermico;
			TravaContagem = 1;
		} else if(!Reed){
			paradaTermica = 0;
			TravaContagem = 0;
		}
		
		if(Reed && millis >= paradaTermica){
			flagTermico = 1;
		}
		
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) && flagTermico && Tempo != 0){
			SSD1306_Clear();
			SSD1306_GotoXY (10,10); // goto 10, 10 
			SSD1306_Puts ("Motor parado em", &Font_7x10, 1); // print Hello 
			SSD1306_GotoXY (10,30); // goto 10, 10 
			SSD1306_Puts ("cima do ima!!", &Font_7x10, 1); // print Hello
			SSD1306_UpdateScreen();	
				if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)){
					flagTermico = 0;
					SSD1306_Clear();
					break;
				}
		}
		// -------------------------------------------------------------------
		
		// O if abaixo trava o range de tempo selecionado de 0 a 30 segundos
		if(Tempo > 30){
			Tempo = 30;
		} else if(Tempo < 0){
			Tempo = 0;
		}
		if(TempoAnterior == 10 && Tempo == 9){ // Remover aquele bug de deixar o '0' apos abaixar o tempo menor que 9
			FlagLimparTela = 1;
		} else {
			FlagLimparTela = 0;
		}
		
		
		delayReleSetado = Tempo * 1000; // Transforma o tempo lido [s] em [ms]
		
		// Rotina para comutar o rele com base no contador interno do microcontrolador
	
		if(EstadoRele == Off && lock == 0){ // Se o rele estiver desligado e o cadeado estiver liberado
			auxTempoSetado = millis + delayReleSetado; // Grava o tempo futuro em que o rele precisa ligar
			lock = 1; //  Trava o cadeado
		}
		if(EstadoRele == On && lock1 == 0){ // Se o rele estiver ligado e o cadeado estiver liberado
			auxTempoReleLigado = millis + TempoReleLigado; // Grava o tempo futuro em que o rele precisava desligar
			lock1 = 1; // Trava o outro cadeado
		}
  	if (millis >= auxTempoSetado && EstadoRele == Off && Tempo != 0 && !cadeadoAjuste){ // Se o valor atual de tempo atingiu o valor futuro com base no tempo escolhido e o rele estiver desligado
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // Liga o rele
			EstadoRele = On; // Variavel de controle de estado do rele
			lock = 0; // Libera o cadeado
			cont++;	// Conta quantas vezes o rele comutou
			}	else if((millis >= auxTempoReleLigado) && EstadoRele == On){ // Mesma rotina do if de cima mas para 500ms [meio segundo]: tempo de pulso do rele
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Desliga o rele apos atingir esse tempo
		EstadoRele = Off; // Muda a variavel de controle
		lock1 = 0; // Libera o cadeado
		cadeadoReed = 0; // Libera o cadeado de contagem do reed
	}
	// -------------------------------------------------------------------
	 
	// Rotina para resetar o circuito
	while(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)){
		millisReset = HAL_GetTick();
		if(millisReset >= (millis + 5000)){
			break;
		}
	}
	if(millisReset >= (millis + 5000)){ // Contar 5 segundos e resetar as variaveis
		cont = 0;
		Tempo = 0;
		ciclosPortao = 0;
		contaReed = 0;
		SSD1306_Clear();
		millisReset = 0;
	} else{
		millisReset = 0;
	}
	// -------------------------------------------------------------------
	// Rotina pra atualizar o display
	// mostrar cont e contaReed
	//ciclosPortao = contaReed;1
	if (FlagLimparTela){
		SSD1306_Clear();
		FlagLimparTela = 0;
	}
	
	sprintf(TempoDisplay, "%d", Tempo); // Joga de int para string
	sprintf(contChar, "%d", cont);
	sprintf(ReedDisplay, "%d", contaReed);
	SSD1306_GotoXY (10,10); // goto 10, 10 
	SSD1306_Puts ("Tempo: ", &Font_7x10, 1); // print Hello 
	SSD1306_Puts (TempoDisplay, &Font_7x10, 1); // print Hello 
	SSD1306_GotoXY (73,10);
	SSD1306_Puts ("s", &Font_7x10, 1); // print Hello
	SSD1306_GotoXY (10, 30); 
	SSD1306_Puts ("Rele: ", &Font_7x10, 1);
	//SSD1306_GotoXY (10, 50);	
	SSD1306_Puts(contChar, &Font_7x10, 1);
	SSD1306_GotoXY (10, 50); 
	SSD1306_Puts ("Reed: ", &Font_7x10, 1);
	SSD1306_Puts(ReedDisplay, &Font_7x10, 1);	
	SSD1306_UpdateScreen(); // update screen
	
	
	dataEprom[0] = Tempo;
	dataEprom[1] = cont;
	dataEprom[2] = contaReed;
	// delayEprom = millis + delayGravacao;
	if(tempoMudou && (millis > delayEprom)){
		// Rotina para gravar na EEPROM
	HAL_FLASH_Unlock();
	FirstPage = GetPage(FLASH_USER_START_ADDR);
	
	// Get the number of pages to erase from 1st page 
	NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;
	
	// Fill EraseInit structure
	//	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = 1;
	uint32_t page_error = 0;
	
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	//Address = FLASH_USER_START_ADDR;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_ADDRESS, dataEprom[0]);
	/*
	  for (uint64_t i = 0; i < 3; i++)
  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_ADDRESS + (i * sizeof(uint64_t)), dataEprom[i]);
  }
  */
	//Address = FLASH_USER_START_ADDR;
	HAL_FLASH_Lock();
	tempoMudou = 0;
	delayEprom = 0;
	}
	/*
	// Rotina para gravar na EEPROM
	HAL_FLASH_Unlock();
	FirstPage = GetPage(FLASH_USER_START_ADDR);
	
	// Get the number of pages to erase from 1st page 
  NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;
	
	// Fill EraseInit structure
	//	FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = 1;
	uint32_t page_error = 0;
	
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	//Address = FLASH_USER_START_ADDR;
	  for (uint64_t i = 0; i < 3; i++)
  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_ADDRESS + (i * sizeof(uint64_t)), dataEprom[i]);
  }
	//Address = FLASH_USER_START_ADDR;
	HAL_FLASH_Lock();
	*/
	// ------------------------------------------------------------
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x0010061A;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1600;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Rele_GPIO_Port, Rele_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Reed_Pin */
  GPIO_InitStruct.Pin = Reed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Reed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Botaomais_Pin */
  GPIO_InitStruct.Pin = Botaomais_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Botaomais_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BotaoMenos_Pin */
  GPIO_InitStruct.Pin = BotaoMenos_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BotaoMenos_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Rele_Pin */
  GPIO_InitStruct.Pin = Rele_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Rele_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
