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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pwm_tables.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct ADC_meas_s{
	uint32_t current;
	uint32_t pot;
	uint32_t voltage;
} ADC_meas_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//volatile uint8_t LED_STATE = 0;

const uint32_t CLK_FREQ = 64000000;
const uint32_t MAX_I_SHTDWN = 1500; /* in mA */
const uint32_t F_MAX = 90; /* in Hz */
const uint32_t F_MIN = 4;
const int32_t DELTA_F_UP = 5; // units of 2^16/65507 (almost 1) Hz/s
const int32_t DELTA_F_DN = -10;
const uint32_t PERIOD_FULL = UINT32_MAX;
const uint32_t PERIOD_ONE_THIRD = PERIOD_FULL / 3;
const uint32_t PERIOD_TWO_THIRD = PERIOD_FULL / 3 * 2;

#define TIM_DIV 977
const uint32_t TIM_TRIG_FREQ = CLK_FREQ / TIM_DIV; // approx 65507 Hz, input 2^16 step for 1 Hz

const uint32_t U_NOM = 195; // HACK
const uint32_t F_NOM = 310;

// 191156419 = sqrt(2) (peak) * 4096 (ADC) * 3300 (R2) * 10 (VDD=34/10)
const uint32_t VF_MAGIC_NUMBER = 589303; //191156419 << 16 * U_NOM / (F_NOM * 34 * 393300);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

ADC_meas_t adc_meas;


/* this is a NCO
 * word structure:
 * [31][30 - 20][19 - 0]
 *  │      │        └ remainder
 *  │      └ pointer to the element in LUT
 *  └ Sign determines if wave is currently in positive or negative half
 */
uint32_t phase_accumulator;

volatile uint32_t f_target; // in (2^-16) Hz
volatile uint32_t vf_coef;

uint8_t *p_current_table;
uint16_t current_gear;

uint32_t i_offset = 0;

uint32_t i_meas;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

void switch_table(int8_t change);
void increment_accumulator(void);
void motor_disconnect(void);
uint32_t min(uint32_t x, uint32_t y);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  LL_GPIO_WriteOutputPort(GPIOB, (1 << 8) | (1 << 6) | (1 << 4) );


  volatile uint32_t count = CLK_FREQ / 5;
  while(count--)
  {
	  __NOP();
  }

  /*read current*/
  LL_ADC_ClearFlag_CCRDY(ADC1);
  LL_ADC_ClearFlag_EOC(ADC1);
  ADC1->CHSELR = (1 << 1);
  while(LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0);
  LL_ADC_ClearFlag_CCRDY(ADC1);
  LL_ADC_Enable(ADC1);
  while(LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
  LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_REG_StartConversion(ADC1);
  while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0);
  LL_ADC_ClearFlag_EOC(ADC1);
  LL_ADC_Disable(ADC1);
  adc_meas.current = LL_ADC_REG_ReadConversionData12(ADC1);

  /*read voltage*/
  ADC1->CHSELR = (1 << 10);
  while(LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0);
  LL_ADC_ClearFlag_CCRDY(ADC1);
  LL_ADC_Enable(ADC1);
  while(LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
  LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_REG_StartConversion(ADC1);
  while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0);
  LL_ADC_ClearFlag_EOC(ADC1);
  LL_ADC_Disable(ADC1);
  adc_meas.voltage = LL_ADC_REG_ReadConversionData12(ADC1);



  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableCounter(TIM1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* convert ADC values
		 *
		 * V_meas = ( Vref * ADC_data / ADC_resolution) * ( (R2 + R1) / R2 )
		 * I_meas = Uref * ADC_data / ( ADC_resolution * R_shunt * A_opamp)
		 */

		/*read current*/
		ADC1->CHSELR = (1 << 1);
		while(LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0);
		LL_ADC_ClearFlag_CCRDY(ADC1);
		LL_ADC_Enable(ADC1);
		while(LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
		LL_ADC_ClearFlag_ADRDY(ADC1);
		LL_ADC_REG_StartConversion(ADC1);
		while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0);
		LL_ADC_ClearFlag_EOC(ADC1);
		LL_ADC_Disable(ADC1);
		adc_meas.current = LL_ADC_REG_ReadConversionData12(ADC1);


		i_meas = ((3300 / 22 * 20) * adc_meas.current) / (1<<12); /* in mA */
		/*check if current is too high */
		if(i_meas > MAX_I_SHTDWN)
		{
			Error_Handler();
		}


		/*read pot*/
		ADC1->CHSELR = (1 << 8);
		while(LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0);
		LL_ADC_ClearFlag_CCRDY(ADC1);
		LL_ADC_Enable(ADC1);
		while(LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
		LL_ADC_ClearFlag_ADRDY(ADC1);
		LL_ADC_REG_StartConversion(ADC1);
		while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0);
		LL_ADC_ClearFlag_EOC(ADC1);
		LL_ADC_Disable(ADC1);
		adc_meas.pot = LL_ADC_REG_ReadConversionData12(ADC1);

		/* calculate target frequency */
		f_target = adc_meas.pot * F_MAX * (1 << 4); // adc goes to 12 bits, 1 Hz is 16 bits
		if(f_target < (F_MIN * (1 << 16))) {
			f_target = F_MIN * (1 << 16);
		}

		/*read voltage*/
		ADC1->CHSELR = (1 << 10);
		while(LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0);
		LL_ADC_ClearFlag_CCRDY(ADC1);
		LL_ADC_Enable(ADC1);
		while(LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
		LL_ADC_ClearFlag_ADRDY(ADC1);
		LL_ADC_REG_StartConversion(ADC1);
		while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0);
		LL_ADC_ClearFlag_EOC(ADC1);
		LL_ADC_Disable(ADC1);
		adc_meas.voltage = LL_ADC_REG_ReadConversionData12(ADC1);


		// calculate the V/f coefficient so the ISR can determine the voltage
//		vf_coef = VF_MAGIC_NUMBER / adc_meas.voltage; // TODO
		vf_coef = 179; // multiplied by 256


//		v_meas = 3300 * adc_meas.voltage * (39120 + 336) / 336 / (1<<12); /* in mV */
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(64000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /**ADC1 GPIO Configuration
  PA1   ------> ADC1_IN1
  PB0   ------> ADC1_IN8
  PB2   ------> ADC1_IN10
  */
  GPIO_InitStruct.Pin = I_MEAS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(I_MEAS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = POT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(POT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = V_MEAS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(V_MEAS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */

   #define ADC_CHANNEL_CONF_RDY_TIMEOUT_MS ( 1U)
   #if (USE_TIMEOUT == 1)
   uint32_t Timeout ; /* Variable used for Timeout management */
   #endif /* USE_TIMEOUT */

  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_FIXED);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_SetTriggerFrequencyMode(ADC1, LL_ADC_CLOCK_FREQ_MODE_HIGH);
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_1|LL_ADC_CHANNEL_8
                              |LL_ADC_CHANNEL_10);

   /* Poll for ADC channel configuration ready */
   #if (USE_TIMEOUT == 1)
   Timeout = ADC_CHANNEL_CONF_RDY_TIMEOUT_MS;
   #endif /* USE_TIMEOUT */
   while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0)
     {
   #if (USE_TIMEOUT == 1)
   /* Check Systick counter flag to decrement the time-out value */
   if (LL_SYSTICK_IsActiveCounterFlag())
     {
   if(Timeout-- == 0)
         {
   Error_Handler();
         }
     }
   #endif /* USE_TIMEOUT */
     }
   /* Clear flag ADC channel configuration ready */
   LL_ADC_ClearFlag_CCRDY(ADC1);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);

   /* Enable ADC internal voltage regulator */
   LL_ADC_EnableInternalRegulator(ADC1);
   /* Delay for ADC internal voltage regulator stabilization. */
   /* Compute number of CPU cycles to wait for, from delay in us. */
   /* Note: Variable divided by 2 to compensate partially */
   /* CPU processing cycles (depends on compilation optimization). */
   /* Note: If system core clock frequency is below 200kHz, wait time */
   /* is only a few CPU processing cycles. */
   uint32_t wait_loop_index;
   wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
   while(wait_loop_index != 0)
     {
   wait_loop_index--;
     }
  /* USER CODE BEGIN ADC1_Init 2 */
  LL_ADC_StartCalibration(ADC1);
  while(LL_ADC_IsCalibrationOnGoing(ADC1))
  {
	  /* wait */
  }

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0);
  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_DOWN;
  TIM_InitStruct.Autoreload = TIM_DIV;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  /* USER CODE BEGIN TIM1_Init 2 */
  TIM1->ARR = TIM_DIV;
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(V_L_GPIO_Port, V_L_Pin);

  /**/
  LL_GPIO_ResetOutputPin(TRIG_GPIO_Port, TRIG_Pin);

  /**/
  LL_GPIO_ResetOutputPin(U_H_GPIO_Port, U_H_Pin);

  /**/
  LL_GPIO_ResetOutputPin(V_H_GPIO_Port, V_H_Pin);

  /**/
  LL_GPIO_ResetOutputPin(W_H_GPIO_Port, W_H_Pin);

  /**/
  LL_GPIO_SetOutputPin(U_L_GPIO_Port, U_L_Pin);

  /**/
  LL_GPIO_SetOutputPin(W_L_GPIO_Port, W_L_Pin);

  /**/
  GPIO_InitStruct.Pin = V_L_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(V_L_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = U_H_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(U_H_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = U_L_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(U_L_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = V_H_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(V_H_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DEAD_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DEAD_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = W_H_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(W_H_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = W_L_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(W_L_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void switch_table(int8_t change)
{
	/* check for border cases outside this function */
	current_gear += change;
	p_current_table = pwm_table_ptrs[current_gear];
}

void increment_accumulator(void)
{
	static uint32_t f_current;
//	LL_GPIO_SetOutputPin(GPIOB, (1<<1));

	/* calculate new frequency */
	/* is frequency increasing or decreasing? */
	const int32_t f_diff = f_target - f_current;

	if(f_diff > DELTA_F_UP)
	{
		f_current += DELTA_F_UP;
	}
	else if (f_diff < DELTA_F_DN)
	{
		f_current += DELTA_F_DN;
	}
	else
	{
		f_current += f_diff;
	}


	/* check if gear switch is needed */
	if( (current_gear != (NUM_OF_TABLES-1)) && (f_current > pwm_max_freq[current_gear]) )
	{
		switch_table(1);
	}
	else if( (current_gear != 0) && (f_current < pwm_max_freq[current_gear-1]) )
	{
		switch_table(-1);
	}


	/* increment phase accumulator */
	phase_accumulator += f_current;


	// determine modulation index
	uint32_t threshold = (vf_coef * f_current) >> 24; // 16b from freq, 8b from vf_coef
	if (threshold < 2) {
		// TODO: base boost?
		threshold = 2; // TODO: calculate this?, change for full voltage
	} else if (threshold > 255) {
		threshold = 255;
	}

	/* extract pointer and set new output values */
	uint32_t gpio_reg_write = 0;

	/* set phase U pins */
	uint32_t sign_and_index = phase_accumulator >> 20;
	if((p_current_table[(sign_and_index & 0x7FF)] > threshold) ^ ((sign_and_index >> 11) & 0x1) )
	{
		gpio_reg_write = U_H_Pin;
	}
	else
	{
		gpio_reg_write = U_L_Pin;
	}

	/* set phase V pins */
	sign_and_index = (phase_accumulator+PERIOD_ONE_THIRD) >> 20;

	if((p_current_table[(sign_and_index & 0x7FF)] > threshold) ^ ((sign_and_index >> 11) & 0x1) )
	{
		gpio_reg_write |= V_H_Pin;
	}
	else
	{
		gpio_reg_write |= V_L_Pin;
	}

	/* set phase W pins */
	sign_and_index = (phase_accumulator+PERIOD_TWO_THIRD) >> 20;
	if((p_current_table[(sign_and_index & 0x7FF)] > threshold) ^ ((sign_and_index >> 11) & 0x1) )
	{
		gpio_reg_write |= W_H_Pin;
	}
	else
	{
		gpio_reg_write |= W_L_Pin;
	}

	gpio_reg_write |= ((sign_and_index >> 11) & 1) << 1; // scope trigger
	 LL_GPIO_WriteOutputPort(GPIOB, gpio_reg_write);
}


void motor_disconnect(void)
{
	/* connect all phases to GND */
	LL_GPIO_WriteOutputPort(GPIOB, (1 << 8) | (1 << 6) | (1 << 4) );
	while(1);

}
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
  motor_disconnect();
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
