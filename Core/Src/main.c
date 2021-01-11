/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_tm1637.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define motor_delay 10
#define fan_mode  1
#define dry_mode  101
#define cool_mode 11
#define heat_mode 110
#define auto_mode 111
#define pipe_temp_limit 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC_Value[5];
uint16_t ADC_temp1_ok[10],adc_count;
uint16_t ADC_temp2_ok[10];
uint32_t data[50];
int Count_Seg=23,Count_Show;
int Flag_IR;
int i,count,debunce_rep=100,debunce=100;
int Power,Timing,Sleeping,flag_Read_IR,count_read;
int cnt=0;
float envirment_temp;
float pipe_temp;
int rec_bits[255];
int temp_setpoint;
int Timer_setpoint;
int comprasor_count,ElecValve_count,swing_time_count,swing_loop_count,swing_dir,on_off_flag,swing_mode;
FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t pageAddress_start=0x0800E000;
uint32_t pageAddress_Set_Point=0x0800E800;
uint32_t PAGEError=0;
int Work_Mode;
int status_flash;
int dir_counter;
struct swing_INFO{
	int dir;
	int n;
	int on_flag;
	int off_time;
	int step;
	int level;
} swing_info;
int air_flow_flag;
int air_flow_loop_count,count_fan;
int adc_temp_avg,ind;
int swing_level;
int level_work;
int indoor_fan_mode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void receive_data (void);
void convert_code (uint32_t code);
void DWT_Delay_us(uint32_t ticks);
void flow_m_on_ccw(int n);
void flow_m_on_cw(int n);
void swing_m_on_ccw(int n);
void swing_m_on_cw(int n);
void fan_indoor(int speed);
float adc2temp(uint32_t adc);
void ir_praser(void);
void Write_To_Flash(uint32_t data,uint32_t Address,uint8_t num_page);
void swing_init(void);
void swing_process();
void swing_off(void);
void swing_stop(void);
int Is_Indoor_Fan_On(void);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM14_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	tm1637Init();
	Power=*(__IO uint32_t *)(pageAddress_start);
	if(Power==3){
		temp_setpoint=*(__IO uint32_t *)(pageAddress_Set_Point);
		tm1637SetBrightness(4);
		Count_Show=(temp_setpoint*1000)+(3*100)+(Timing*10)+Sleeping;
		HAL_Delay(10);
			tm1637DisplayDecimal(Count_Show, 0);
	}
	else
	{
		tm1637SetBrightness(0);	
	}
	//tm1637DisplayString();
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_DMA(&hadc,(uint32_t *)ADC_Value,5);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_RESET);
	//fan_indoor(1);
	//HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_SET);
	//HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(Work_Mode)
	  {
		case auto_mode:
		break;
		case cool_mode:
			
		break;
		case dry_mode:
		break;
		case heat_mode:
			if(level_work==0)
			{
				if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_count=0;
				}
				level_work=1;
			}
			if(Power==3){
				if(swing_level==-1)
					swing_level=0;
				if(level_work==1){
					if(temp_setpoint>envirment_temp+2)
					{
						if(comprasor_count*0.5/60>3)
						{
							HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_SET);
							HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_SET);
							HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_SET);
						}
						if(pipe_temp>pipe_temp_limit)
							fan_indoor(indoor_fan_mode);
					}
					else if((temp_setpoint<envirment_temp-2) && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_SET)
					{
						HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
						comprasor_count=0;
						ElecValve_count=0;
					}
					if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET)
					{
						if(ElecValve_count*0.5/60>1.5)
						{
							fan_indoor(-1);
						}
						if(ElecValve_count*0.5/60>2)
						{
							HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_RESET);
						}
					}

				}

				if(swing_level==0 && swing_loop_count==0)
				{
					swing_info.on_flag=1;
					swing_info.dir=1;
					swing_info.n=170*4;
					swing_info.off_time=0;
					swing_info.step=1;
				}
				else if(swing_level==0 && swing_info.on_flag==0)
				{
					swing_level=1;
					swing_loop_count=0;
				}
				else if(swing_level==1 && swing_loop_count==0)
				{
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=130*4;
					swing_info.off_time=0;
					swing_info.step=1;
				}
				else if(swing_level==1 && swing_info.on_flag==0)
				{
					swing_level=2;
					swing_loop_count=0;
				}
				else if(swing_level==2 && swing_loop_count==0)
				{
					swing_info.on_flag=1;
					swing_info.dir=1;
					swing_info.n=100*4;
					swing_info.off_time=30;
					swing_info.step=(swing_mode!=11);
				}
				else if(swing_level==2 && swing_info.on_flag==0)
				{
					swing_level=3;
					swing_loop_count=0;
				}
				else if(swing_level==3 && swing_loop_count==0)
				{
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=100*4;
					swing_info.off_time=0;
					swing_info.step=(swing_mode!=11);
				}
				else if(swing_level==3 && swing_info.on_flag==0)
				{
					swing_level=(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET)?10:2;
					swing_loop_count=0;
				}
				if(swing_level==10 && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_SET && Is_Indoor_Fan_On())
					swing_level=2;

	  		}
			else if(Power==0)
			{
				if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_count=0;
					ElecValve_count=0;
				}
				if(ElecValve_count*0.5/60>2)
					HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_RESET);

				fan_indoor(-1);
				if(swing_level>=0)
				{
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=170*4;
					swing_loop_count=0;
					swing_level=-1;
				}
				else if(swing_info.on_flag==0)
					swing_loop_count=0;
			}
			break;
		case fan_mode:
			if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
			{
				HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
				comprasor_count=0;
				if(HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin))
					ElecValve_count=0;
			}
			if(HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin))
			{
				if(ElecValve_count*0.5/60>2)
				{
					HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_RESET);
				}
			}
			if(Power==3)
			{
				if(swing_level==10 || swing_level==-1)
					swing_level=0;
				fan_indoor(indoor_fan_mode);
				if(swing_level==0 && swing_loop_count==0)
				{
					swing_info.on_flag=1;
					swing_info.dir=1;
					swing_info.n=170*4;
					swing_info.off_time=0;
					swing_info.step=1;
				}
				else if(swing_level==0 && swing_info.on_flag==0)
				{
					swing_level=1;
					swing_loop_count=0;
				}
				else if(swing_level==1 && swing_loop_count==0)
				{
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=130*4;
					swing_info.off_time=0;
					swing_info.step=1;
				}
				else if(swing_level==1 && swing_info.on_flag==0)
				{
					swing_level=2;
					swing_loop_count=0;
				}
				else if(swing_level==2 && swing_loop_count==0)
				{
					swing_info.on_flag=1;
					swing_info.dir=1;
					swing_info.n=100*4;
					swing_info.off_time=0;
					swing_info.step=(swing_mode!=11);
				}
				else if(swing_level==2 && swing_info.on_flag==0)
				{
					swing_level=3;
					swing_loop_count=0;
				}
				else if(swing_level==3 && swing_loop_count==0)
				{
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=100*4;
					swing_info.off_time=30;
					swing_info.step=(swing_mode!=11);
				}
				else if(swing_level==3 && swing_info.on_flag==0)
				{
					swing_level=2;
					swing_loop_count=0;
				}
			}
			else if(Power==0)
			{
				fan_indoor(-1);
				if(swing_level>=0)
				{
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=170*4;
					swing_loop_count=0;
					swing_level=-1;
				}
				else if(swing_info.on_flag==0){
					swing_loop_count=0;
				}
			}	
		break;
	  }
	  /*
		if(Work_Mode==fan_mode && on_off_flag==1)
		{
			if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
			{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_count=0;
			}
		}
		if(Work_Mode==fan_mode && on_off_flag==2)
			fan_indoor(0);
		if(Work_Mode==cool_mode && on_off_flag==1)
		{
			if(envirment_temp>temp_setpoint+2){
				if(comprasor_count*0.5/60>3)
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_SET);
				}
			}
			else if(envirment_temp<temp_setpoint-2){
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_count=0;
			}
		}
		if(Work_Mode==cool_mode && on_off_flag==2)
		{
			if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
			{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_count=0;
			}
				if(count_fan*0.5>=30 && count_fan*0.5<=32){
					fan_indoor(0);
					swing_info.on_flag=1;
				}
				else if(swing_info.on_flag!=0 && count_fan*0.5>=32 && count_fan*0.5<=34)
				{
					swing_info.n=170*2;
					swing_info.dir=2;
					swing_info.level=1;
					swing_loop_count=0;
			}
		}
		
		if(Work_Mode==heat_mode && on_off_flag==1)
		{
			HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_SET);
			if(envirment_temp<temp_setpoint-2){
				if(comprasor_count*0.5/60>3)
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_SET);
				}
			}
			else if(envirment_temp>temp_setpoint+2){
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_count=0;
			}
		}
		if(Work_Mode==heat_mode && on_off_flag==2)
		{
			fan_indoor(0);
			if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
			{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_count=0;
			}
			if(ElecValve_count*0.5/60>2)
			{
				HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_RESET);
			}
			
		}
		*/
			
		//swing_process(1);
		/*tm1637SetBrightness(4);
		Count_Show=((ADC_Value[4]/100)*1000)+(3*100)+(Timing*10)+Sleeping;
		tm1637DisplayDecimal(Count_Show, 0);
		HAL_Delay(1000);
		Count_Show=(88*1000)+(3*100)+(Timing*10)+Sleeping;
		tm1637DisplayDecimal(Count_Show, 0);
		HAL_Delay(1000);
		Count_Show=(((ADC_Value[4])%100)*1000)+(3*100)+(Timing*10)+Sleeping;
		tm1637DisplayDecimal(Count_Show, 0);
		HAL_Delay(1000);*/
		//fan_indoor();
		/*while(++cnt<500)
			swing_m_on_cw();
		cnt=0;
		while(++cnt<500)
			swing_m_on_ccw();
		cnt=0;*/
		
		//flow_m_on_cw();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Flag_IR==1)
		{
			HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
			//HAL_Delay(1);
			if(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13)==GPIO_PIN_RESET)
			{		
				receive_data();
//			convert_code(data[2]);
				ir_praser();
			}
			Flag_IR=0;
			HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
		}

//		HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_SET);
//		HAL_Delay(2000);
//		HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
//		HAL_Delay(2000);
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim14)
	{
		HAL_GPIO_TogglePin(Heart_GPIO_Port,Heart_Pin);
		comprasor_count++;
		ElecValve_count++;
		count_fan++;
		adc_temp_avg=0;
		for(ind=0;ind<10;ind++)
			adc_temp_avg+=ADC_temp1_ok[ind];
		adc_temp_avg=adc_temp_avg/10;
		envirment_temp= adc2temp(adc_temp_avg);
		adc_temp_avg=0;
		for(ind=0;ind<10;ind++)
			adc_temp_avg+=ADC_temp2_ok[ind];
		adc_temp_avg=adc_temp_avg/10;
		pipe_temp=adc2temp(adc_temp_avg);
//		Count_Seg++;
//		if(Count_Seg>100)
//			Count_Seg=0;
//		Count_Show=(Count_Seg*1000)+(0)+(30)+3;
//		tm1637DisplayDecimal(Count_Show, 0);
	}
		if(htim==&htim3){
			ADC_temp1_ok[adc_count]=ADC_Value[0];
			ADC_temp2_ok[adc_count]=ADC_Value[1];
			adc_count=(adc_count+1)%10;
			if(swing_info.on_flag==1)
			{
				swing_loop_count+=swing_info.step;
				if(swing_loop_count<=swing_info.n){
					HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,(swing_info.dir==1)?swing_loop_count%4==0:swing_loop_count%4==3);
					HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,(swing_info.dir==1)?swing_loop_count%4==1:swing_loop_count%4==2);
					HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,(swing_info.dir==1)?swing_loop_count%4==2:swing_loop_count%4==1);
					HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,(swing_info.dir==1)?swing_loop_count%4==3:swing_loop_count%4==0);
				}
				else if(swing_loop_count>=swing_info.n+(swing_mode==01 && ((Work_Mode==fan_mode && swing_info.dir==2)|| (Work_Mode==heat_mode && swing_info.dir==1)))*swing_info.off_time*50)
				{
					swing_info.on_flag=0;
				}
			}
			/*if(swing_info.on_flag && swing_info.level==1){
				HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,(swing_info.dir==1)?swing_loop_count%4==0:swing_loop_count%4==3);
				HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,(swing_info.dir==1)?swing_loop_count%4==1:swing_loop_count%4==2);
				HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,(swing_info.dir==1)?swing_loop_count%4==2:swing_loop_count%4==1);
				HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,(swing_info.dir==1)?swing_loop_count%4==3:swing_loop_count%4==0);
				swing_loop_count++;
			}
			
			if(swing_info.n<=swing_loop_count && swing_info.level==1){
				swing_info.on_flag=0;
				swing_loop_count=0;
				swing_info.level=2;
				swing_info.dir=(swing_info.dir+1)%2;
				if(Work_Mode==cool_mode && on_off_flag==2 && swing_mode!=01 && count_fan<0)
					swing_mode=01;
				else if(Work_Mode==cool_mode && on_off_flag==2 && count_fan*0.5>=35)
					swing_mode=00;
			}
			
			else if((swing_info.level==0||swing_info.level==2) &&(Work_Mode==cool_mode || on_off_flag==1)){
				switch (swing_mode){
					case (01) :
						HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,(swing_info.dir==1)?swing_loop_count%4==0:swing_loop_count%4==3);
						HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,(swing_info.dir==1)?swing_loop_count%4==1:swing_loop_count%4==2);
						HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,(swing_info.dir==1)?swing_loop_count%4==2:swing_loop_count%4==1);
						HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,(swing_info.dir==1)?swing_loop_count%4==3:swing_loop_count%4==0);
						swing_loop_count++;
						if((swing_info.level==2 && swing_loop_count>=150*3) || (swing_info.level==0 && swing_loop_count>=120*3)){
							swing_info.level=0;
							swing_loop_count=0;
							swing_info.dir=(swing_info.dir+1)%2;
							if(Work_Mode==cool_mode && on_off_flag==2)
								swing_mode=0;
								count_fan=0;
						}
					break;
				case (10) :
						swing_loop_count++;
					if(swing_loop_count<=120*3){
						HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,(swing_info.dir==1)?swing_loop_count%4==0:swing_loop_count%4==3);
						HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,(swing_info.dir==1)?swing_loop_count%4==1:swing_loop_count%4==2);
						HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,(swing_info.dir==1)?swing_loop_count%4==2:swing_loop_count%4==1);
						HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,(swing_info.dir==1)?swing_loop_count%4==3:swing_loop_count%4==0);}
						else if((swing_loop_count>=170*3 && swing_info.dir==1)|| swing_info.dir%2==0){
							swing_loop_count=0;
							swing_info.dir=(swing_info.dir+1)%2;
						}
					
					
				}
				
			}*/
			
			if(air_flow_flag && Power==3)
			{
				HAL_GPIO_WritePin(Flow_M_1_GPIO_Port,Flow_M_1_Pin,(air_flow_loop_count<300*3)?(air_flow_loop_count%4==0):air_flow_loop_count%4==3);
				HAL_GPIO_WritePin(Flow_M_2_GPIO_Port,Flow_M_2_Pin,(air_flow_loop_count<300*3)?(air_flow_loop_count%4==1):air_flow_loop_count%4==2);
				HAL_GPIO_WritePin(Flow_M_3_GPIO_Port,Flow_M_3_Pin,(air_flow_loop_count<300*3)?(air_flow_loop_count%4==2):air_flow_loop_count%4==1);
				HAL_GPIO_WritePin(Flow_M_4_GPIO_Port,Flow_M_4_Pin,(air_flow_loop_count<300*3)?(air_flow_loop_count%4==3):air_flow_loop_count%4==0);
				air_flow_loop_count++;
				if(air_flow_loop_count>=300*6)
					air_flow_loop_count=0;
			}
		}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin==GPIO_PIN_13)
	{
		Flag_IR=1;
	}
}
void receive_data (void)
{
	
		  /* The START Sequence begin here
	   * there will be a pulse of 9ms LOW and
	   * than 4.5 ms space (HIGH)
	   */
	  while (!(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13)));  // wait for the pin to go high.. 9ms LOW

	  while ((HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13)));  // wait for the pin to go low.. 4.5ms HIGH

	  /* START of FRAME ends here*/

	  /* DATA Reception
	   * We are only going to check the SPACE after 562.5us pulse
	   * if the space is 562.5us, the bit indicates '0'
	   * if the space is around 1.6ms, the bit is '1'
	   */

	  for (i=0; i<96; i++)
	  {
		  count=0;

		  while (!(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13))); // wait for pin to go high.. this is 562.5us LOW

		  while ((HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13)))  // count the space length while the pin is high
		  {
			  count++;
			  DWT_Delay_us(debunce);
				if(count>200)
					break;
		  }

		  if (count > 12) // if the space is more than 1.2 ms
		  {
			  //code |= (1UL << (31-i));   // write 1
				rec_bits[i]=1UL;
		  }

		  else {//code &= ~(1UL << (31-i));  // write 0
				rec_bits[i]=(0UL);
				}
	  }
		HAL_Delay(debunce_rep);
}
void ir_praser()
{
	//on/off
	int p=Power;
	if(rec_bits[49]!=rec_bits[57]){
		Power=3*rec_bits[57];
	}
	/*if(Power==0 && p==3){
			on_off_flag=2;
			swing_info.n=170*4;
		if(Work_Mode==cool_mode)
			swing_info.dir=1;
		else
			swing_info.dir=2;
		swing_info.on_flag=1;
		swing_info.level=1;
		swing_loop_count=0;
		count_fan=-10000;
		if(Work_Mode==heat_mode)
				ElecValve_count=0;
	}
	if(Power==3 && p==0)
	{
		on_off_flag=1;
			swing_info.n=170*4;
			swing_info.dir=1;
			swing_info.on_flag=1;
			swing_info.level=1;
			swing_loop_count=0;
	}
	*/
	//mode
	if(rec_bits[69]!=rec_bits[77] && rec_bits[70]!=rec_bits[78] && rec_bits[71]!=rec_bits[79])
	{
		Work_Mode=rec_bits[69]*100+rec_bits[70]*10+rec_bits[71];
	}
	
	//Temp
	if(rec_bits[72]!=rec_bits[64] && rec_bits[73]!=rec_bits[65] && rec_bits[74]!=rec_bits[66] && rec_bits[75]!=rec_bits[67] && rec_bits[76]!=rec_bits[68])
		temp_setpoint=rec_bits[72]+rec_bits[73]*2+rec_bits[74]*4+rec_bits[75]*8+rec_bits[76]*16+16;	
		
	//fan speed
	if(rec_bits[53]!=rec_bits[61] && rec_bits[54]!=rec_bits[62]){
		flag_Read_IR=1;
		indoor_fan_mode=rec_bits[53]*10+rec_bits[54];
	}
	//swing
	if(rec_bits[50]!=rec_bits[58] && rec_bits[51]!=rec_bits[59]){
		swing_mode=rec_bits[50]*10+rec_bits[51];
		swing_info.step=(swing_mode==11)?0:1;
			/*case (11) :
				swing_mode=11
			break;
			case (01) :
				swing_process();
			break;
			case (10) :
				swing_process();
			break;*/
	}	

	//air flow
	if(rec_bits[52]!=rec_bits[60]){
		air_flow_flag=rec_bits[52];
	}	
	
	//timer
	if(rec_bits[8]!=rec_bits[0] && rec_bits[9]!=rec_bits[1] && rec_bits[10]!=rec_bits[2] && rec_bits[11]!=rec_bits[3] && rec_bits[12]!=rec_bits[4])
	{
		Timing=3*rec_bits[15];
		Timer_setpoint=rec_bits[8]+rec_bits[9]*2+rec_bits[10]*4+rec_bits[11]*8+rec_bits[12]*16;
	}
	
	//hold
	if(rec_bits[18]!=rec_bits[26]){
	}
	
	//sleep
	if(rec_bits[48]!=rec_bits[56]){
		Sleeping=3*rec_bits[56];
	}
	//turbo
	if(rec_bits[19]!=rec_bits[27]){
	}
	
	if(Power!=0)
	{
		tm1637SetBrightness(4);
		if(Timing){
			Count_Show=(Timer_setpoint*1000)+(Power*100)+(Timing*10)+Sleeping;
			tm1637DisplayDecimal(Count_Show, 0);
			HAL_Delay(2000);
		}
			Count_Show=(temp_setpoint*1000)+(Power*100)+(Timing*10)+Sleeping;
			tm1637DisplayDecimal(Count_Show, 0);
			
	}
	else{
		tm1637SetBrightness(0);}
	//lamp
	if(rec_bits[16]!=rec_bits[24] && Power){
				tm1637SetBrightness(4*rec_bits[16]);
	}
	
	Write_To_Flash(Power,pageAddress_start,2);
	Write_To_Flash(temp_setpoint,pageAddress_Set_Point,3);
 /*switch (code)//
 {
	 	case (0x2AFFFFFF):
			flag_Read_IR=1;
			Power=3;		
			tm1637SetBrightness(4);		
			Count_Show=(Count_Seg*1000)+(Power*100)+(Timing*10)+Sleeping;
			tm1637DisplayDecimal(Count_Show, 0);
		break;
		case (0x2AFFFFF3):
				flag_Read_IR=1;
				Power=0;
				tm1637SetBrightness(0);
		break;
	case (0x807FC03F):
		flag_Read_IR=1;
		HAL_GPIO_TogglePin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin);
		break;
	case (0x807FA05F):
		flag_Read_IR=1;
		HAL_GPIO_TogglePin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin);
		break;
	case (0x807FE01F):
		flag_Read_IR=1;
		HAL_GPIO_TogglePin(Fan_Indoor_3_GPIO_Port,Fan_Indoor_3_Pin);
		break;
	case (0x807F906F):
		flag_Read_IR=1;
		HAL_GPIO_TogglePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin);
		break;
	case (0x807FD02F):
		flag_Read_IR=1;
		HAL_GPIO_TogglePin(Compressor_GPIO_Port,Compressor_Pin);
		break;//
	case (0x807FB04F):
		flag_Read_IR=1;
		HAL_GPIO_TogglePin(ElecValve_GPIO_Port,ElecValve_Pin);
		break;
	case (0x807FE817):
		flag_Read_IR=1;
		Count_Seg++;
		Count_Show=(Count_Seg*1000)+300;
		tm1637DisplayDecimal(Count_Show, 0);
		break;
	case (0x807FB847):
		flag_Read_IR=1;
		Count_Seg--;
		Count_Show=(Count_Seg*1000)+300;
		tm1637DisplayDecimal(Count_Show, 0);
		break;
	case (0x807FB44B):
		flag_Read_IR=1;
		if(Timing==0)
			Timing=3;
		else if(Timing==3)
			Timing=0;
		Count_Show=(Count_Seg*1000)+(Power*100)+(Timing*10)+Sleeping;
		tm1637DisplayDecimal(Count_Show, 0);
		break;
	case (0x807FFC03):
		flag_Read_IR=1;
		if(Sleeping==0)
			Sleeping=3;
		else if(Sleeping==3)
			Sleeping=0;
		Count_Show=(Count_Seg*1000)+(Power*100)+(Timing*10)+Sleeping;
		tm1637DisplayDecimal(Count_Show, 0);
		break;
	}*/
 if(flag_Read_IR==1)
 {
	 flag_Read_IR=0;
	 HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_RESET);
 }
 }
void DWT_Delay_us(uint32_t ticks)
{
//	while(ticks--);
	TIM2->CNT=0;
	while(TIM2->CNT<ticks);
}
int Is_Indoor_Fan_On()
{
	return HAL_GPIO_ReadPin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin) || HAL_GPIO_ReadPin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin) || HAL_GPIO_ReadPin(Fan_Indoor_3_GPIO_Port,Fan_Indoor_3_Pin);
}
void flow_m_on_ccw(int n){
int j=0;
	while(j<n){
	HAL_GPIO_WritePin(Flow_M_1_GPIO_Port,Flow_M_1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Flow_M_2_GPIO_Port,Flow_M_2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_3_GPIO_Port,Flow_M_3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_4_GPIO_Port,Flow_M_4_Pin,GPIO_PIN_RESET);
	HAL_Delay(motor_delay);
	HAL_GPIO_WritePin(Flow_M_1_GPIO_Port,Flow_M_1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_2_GPIO_Port,Flow_M_2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Flow_M_3_GPIO_Port,Flow_M_3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_4_GPIO_Port,Flow_M_4_Pin,GPIO_PIN_RESET);
	HAL_Delay(motor_delay);
	HAL_GPIO_WritePin(Flow_M_1_GPIO_Port,Flow_M_1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_2_GPIO_Port,Flow_M_2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_3_GPIO_Port,Flow_M_3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Flow_M_4_GPIO_Port,Flow_M_4_Pin,GPIO_PIN_RESET);
	HAL_Delay(motor_delay);
	HAL_GPIO_WritePin(Flow_M_1_GPIO_Port,Flow_M_1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_2_GPIO_Port,Flow_M_2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_3_GPIO_Port,Flow_M_3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_4_GPIO_Port,Flow_M_4_Pin,GPIO_PIN_SET);
	HAL_Delay(motor_delay);
	j++;
	}
}
void flow_m_on_cw(int n){
	int j=0;
	while(j<n){
	HAL_GPIO_WritePin(Flow_M_1_GPIO_Port,Flow_M_1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_2_GPIO_Port,Flow_M_2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_3_GPIO_Port,Flow_M_3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_4_GPIO_Port,Flow_M_4_Pin,GPIO_PIN_SET);
	HAL_Delay(motor_delay);
	HAL_GPIO_WritePin(Flow_M_1_GPIO_Port,Flow_M_1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_2_GPIO_Port,Flow_M_2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_3_GPIO_Port,Flow_M_3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Flow_M_4_GPIO_Port,Flow_M_4_Pin,GPIO_PIN_RESET);
	HAL_Delay(motor_delay);
	HAL_GPIO_WritePin(Flow_M_1_GPIO_Port,Flow_M_1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_2_GPIO_Port,Flow_M_2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Flow_M_3_GPIO_Port,Flow_M_3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_4_GPIO_Port,Flow_M_4_Pin,GPIO_PIN_RESET);
	HAL_Delay(motor_delay);
	HAL_GPIO_WritePin(Flow_M_1_GPIO_Port,Flow_M_1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Flow_M_2_GPIO_Port,Flow_M_2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_3_GPIO_Port,Flow_M_3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Flow_M_4_GPIO_Port,Flow_M_4_Pin,GPIO_PIN_RESET);
	HAL_Delay(motor_delay);
	j++;
	}
}
void swing_m_on_cw(int n){
	if(swing_dir==0){
		swing_dir=1;
		swing_time_count=0;
	}
	if( swing_dir==1){
		if(swing_loop_count<n ){
			if(swing_time_count<motor_delay)
			{	HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,GPIO_PIN_RESET);}
			else if(swing_time_count<2*motor_delay)
			{	HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,GPIO_PIN_RESET);}
			else if(swing_time_count<3*motor_delay)
			{	HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,GPIO_PIN_RESET);}
			else if(swing_time_count<4*motor_delay)
			{	HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,GPIO_PIN_SET);}
			else{
				swing_time_count=0;
				swing_loop_count++;
			}
		}
		else{
			swing_loop_count=0;
			swing_dir=0;
		}
	}
}
void swing_m_on_ccw(int n){
	if(swing_dir==0){
		swing_dir=2;
		swing_time_count=0;
	}
	if( swing_dir==2){
		if(swing_loop_count<n){
			if(swing_time_count<motor_delay)
			{	HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,GPIO_PIN_SET);}
			else if(swing_time_count<2*motor_delay)
			{	HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,GPIO_PIN_RESET);}
			else if(swing_time_count<3*motor_delay)
			{	HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,GPIO_PIN_RESET);}
			else if(swing_time_count<4*motor_delay)
			{	HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,GPIO_PIN_RESET);}
			else{
				swing_time_count=0;
				swing_loop_count++;
			}
		}
		else{
			swing_dir=0;
			swing_loop_count=0;
		}
	}
}
void fan_indoor(int speed){
	switch(speed)
	{
		case 00:
			if(HAL_GPIO_ReadPin(Fan_Indoor_3_GPIO_Port,Fan_Indoor_3_Pin) || HAL_GPIO_ReadPin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin)){
				HAL_GPIO_WritePin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Fan_Indoor_3_GPIO_Port,Fan_Indoor_3_Pin,GPIO_PIN_RESET);
				HAL_Delay(200);
			}
			HAL_GPIO_WritePin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin,GPIO_PIN_SET);
		break;
		case 10:
			if(HAL_GPIO_ReadPin(Fan_Indoor_3_GPIO_Port,Fan_Indoor_3_Pin) || HAL_GPIO_ReadPin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin)){
				HAL_GPIO_WritePin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Fan_Indoor_3_GPIO_Port,Fan_Indoor_3_Pin,GPIO_PIN_RESET);
				HAL_Delay(200);
			}
			HAL_GPIO_WritePin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin,GPIO_PIN_SET);
		break;
		case 01:
			if(HAL_GPIO_ReadPin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin) || HAL_GPIO_ReadPin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin)){
				HAL_GPIO_WritePin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin,GPIO_PIN_RESET);
				HAL_Delay(200);
			}
			HAL_GPIO_WritePin(Fan_Indoor_3_GPIO_Port,Fan_Indoor_3_Pin,GPIO_PIN_SET);
		break;
		case (-1):
			HAL_GPIO_WritePin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Fan_Indoor_3_GPIO_Port,Fan_Indoor_3_Pin,GPIO_PIN_RESET);
		break;
		
	}
}

float adc2temp(uint32_t adc){
	//y = -2E-07x3 + 0.0003x2 - 0.2577x + 82.567
	return -2.102e-7*(adc*adc*adc)+0.0003356*(adc*adc)-0.2578*adc+82.67+16.7;
}

void heat_mode_process(){
	
}
void Write_To_Flash(uint32_t data,uint32_t Address,uint8_t num_page){
	HAL_FLASH_Unlock();
	EraseInitStruct.NbPages=num_page;
	EraseInitStruct.PageAddress=Address;
	EraseInitStruct.TypeErase=FLASH_TYPEERASE_PAGES;
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError)!=HAL_OK){
	//
	}
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Address,data)==HAL_OK)
	{
		status_flash=1;
	}
	HAL_FLASH_Lock();
}

void swing_init(){
	if(swing_dir==3)
		swing_dir=0;
	swing_m_on_cw(170);
	swing_m_on_ccw(120);
}
void swing_process(){
	if(swing_dir==3)
		swing_dir=0;
	swing_m_on_cw(100);
	swing_m_on_ccw(100);
}
void swing_off(){
	if(swing_dir==3)
		swing_dir=0;
	swing_m_on_ccw(170);
}
void swing_stop(){
		swing_dir=3;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
