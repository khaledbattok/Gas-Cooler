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
#define fan_mode  1 //1
#define dry_mode  2 //101
#define cool_mode 3 //11
#define heat_mode 4 //110
#define auto_mode 5 //111
#define pipe_temp_limit 30
#define max_comprossor_current 12
#define min_comprossor_current 3
#define initial_speed 2499
#define default_speed 4999
#define indoor_defroast_temp 5
#define indoor_defroast_diff_temp 7
#define indoor_defroast_min_period 2
#define fan_comprasor_temp_limit 45
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC_Value[3];
uint16_t ADC_temp1_ok[100],adc_count;
uint16_t ADC_temp2_ok[100];
uint16_t ADC_current_ok[100];
int Count_Show;
int Flag_IR;
int i,count,debunce_rep=0,debunce=100;
int Power,Timing,Sleeping,flag_Read_IR;
int sleep_count=0,timer_count;
float envirment_temp;
float pipe_temp;
float comprasor_current;
int rec_bits[96];
int temp_setpoint;
int Timer_setpoint;
int comprasor_off_count,ElecValve_count,swing_loop_count,swing_mode;
int comprasor_on_count;
FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t pageAddress_1=0x0800E000;
uint32_t pageAddress_2=0x0800E400;
uint32_t pageAddress_3=0x0800E800;
uint32_t PAGEError=0;
int Work_Mode;
int auto_mode_selected;
int status_flash;
struct swing_INFO{
	int dir;
	int n;
	int on_flag;
	int off_time;
	int step;
} swing_info;
int air_flow_flag;
int air_flow_loop_count,count_fan;
float adc_temp_avg;
int ind;
int swing_level;
int level_work;
int indoor_fan_mode;
int comprasor_dry_count;
int turbo_flag;
uint32_t ir_32bit_arr[3];
int temp1_offset;
int temp2_offset;
int tim14_flag,tim3_flag;
int Error;
int auto_fan_flag;
int swing_pole;
int indoor_defroast_flag,indoor_defroast_count=0;
int ir_sum;
int timer_blink;
int ans;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void receive_data (void);
void DWT_Delay_us(uint32_t ticks);
void fan_indoor(int speed);
float adc2temp(float adc);
void ir_praser(void);
void Write_To_Flash(uint32_t data,uint32_t Address,uint8_t num_page);
int Is_Indoor_Fan_On(void);
int Is_Ir_Valid(void);
void ir_bits_to_32_bit_arr(void);
void _32_bit_arr_to_ir_bits(void);
float adc2current(float adc);
void process_work(void);
void tim14_process(void);
void tim3_process(void);
void config(void);
void ir_process(void);
void auto_fan_fcn(void);
void indoor_defroast_process(void);
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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	config();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(1/*Error==0*/){
				if(tim14_flag)
				{
					tim14_process();
					tim14_flag=0;
				}
				if(tim3_flag)
				{
					tim3_process();
					tim3_flag=0;
				}
				process_work();
				if(auto_fan_flag==1)
					auto_fan_fcn();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Flag_IR==1)
			{
				ir_process();
			}
  }
		else
		{
			tm1637SetBrightness(4);
			tm1637DisplayError(Error,0);
			HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_RESET);
			fan_indoor(-1);
			HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_RESET);
			HAL_Delay(1000);			
		}
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
		tim14_flag=1;
	}
		if(htim==&htim3){
			tim3_flag=1;
		}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin==GPIO_PIN_13)
	{
		Flag_IR=1;
	}
}
void config(){
	tm1637Init();
	ir_32bit_arr[0]=*(__IO uint32_t *)(pageAddress_1);
	ir_32bit_arr[1]=*(__IO uint32_t *)(pageAddress_2);
	ir_32bit_arr[2]=*(__IO uint32_t *)(pageAddress_3);
	_32_bit_arr_to_ir_bits();
	ir_praser();
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_DMA(&hadc,(uint32_t *)ADC_Value,3);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_RESET);
}
void tim14_process()
{
	comprasor_off_count+=!HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin);
	comprasor_on_count+=HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin);
	ElecValve_count+=HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin);
	count_fan++;
	comprasor_dry_count++;
	sleep_count++;
	timer_count++;
	timer_blink++;
	indoor_defroast_count++;
	if(((Work_Mode==heat_mode && !Is_Indoor_Fan_On())||(((Work_Mode==cool_mode || Work_Mode==dry_mode ) && HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin)))))
	{
		if((!Timing || (Timing && timer_blink>15)) && Sleeping==0)
		{
			Count_Show=(temp_setpoint*1000)+(((sleep_count%2)*3)*100)+(Timing*10)+Sleeping;
		if(Work_Mode==dry_mode)
			tm1637Display_null(Count_Show, 0);
		else
			tm1637DisplayDecimal(Count_Show, 0);
		}
	}
	else if((Work_Mode==heat_mode || Work_Mode==cool_mode || Work_Mode==dry_mode) && (!Timing || (Timing && timer_blink>15)) && Sleeping==0) 
	{
			Count_Show=(temp_setpoint*1000)+(Power*100)+(Timing*10)+Sleeping;
		if(Work_Mode==dry_mode)
			tm1637Display_null(Count_Show, 0);
		else
			tm1637DisplayDecimal(Count_Show, 0);
	}

		if(Timing)
		{
			if(timer_blink<10)
			{
				tm1637SetBrightness(4*(timer_blink%2==0));
				Count_Show=(Timer_setpoint*1000)+(Power*100)+(Timing*10)+Sleeping;
				tm1637DisplayDecimal(Count_Show, 0);
			}
			else if(timer_blink<15)
			{
				tm1637SetBrightness((Sleeping && sleep_count*0.5>10)?0:4);
				Count_Show=(temp_setpoint*1000)+(Power*100)+(Timing*10)+Sleeping;
				if((Work_Mode==fan_mode || Work_Mode==dry_mode) && (auto_mode_selected==0))
					tm1637Display_null(Count_Show, 0);
				else
					tm1637DisplayDecimal(Count_Show, 0);
			}
			if(timer_count*0.5/3600>=Timer_setpoint)
			{
				if(flag_Read_IR)
				{
				rec_bits[15]=0;
				rec_bits[57]=0;
				rec_bits[49]=1;
				ir_praser();
				}
			}
		}
		if(Sleeping && sleep_count*0.5>10){
			tm1637DisplaySleep();
		}
		adc_temp_avg=0;
		for(ind=0;ind<100;ind++)
			adc_temp_avg+=ADC_temp1_ok[ind];
		adc_temp_avg=adc_temp_avg/100;
		envirment_temp= adc2temp(adc_temp_avg)+temp1_offset;
		adc_temp_avg=0;
		for(ind=0;ind<100;ind++)
			adc_temp_avg+=ADC_temp2_ok[ind];
		adc_temp_avg=adc_temp_avg/100;
		pipe_temp=adc2temp(adc_temp_avg)+temp2_offset;
		adc_temp_avg=0;
		for(ind=0;ind<100;ind++)
			adc_temp_avg+=ADC_current_ok[ind];
		adc_temp_avg=adc_temp_avg/100;
		comprasor_current=adc2current(adc_temp_avg);
		
		//overcurrent and undercurrent
		if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin) && comprasor_on_count*0.5>30)
		{
			if(comprasor_current>max_comprossor_current)
			{
				Error=1;
			}
			else if(comprasor_current<min_comprossor_current)
			{
				Error=2;
			}
		}

		if(Work_Mode==cool_mode || Work_Mode==dry_mode){
			if(pipe_temp<indoor_defroast_temp && indoor_defroast_flag==0)
			{
				if(indoor_defroast_count*0.5/3600>indoor_defroast_min_period)
					indoor_defroast_flag=1;
			}
			else if((pipe_temp>(indoor_defroast_temp+indoor_defroast_diff_temp)) && indoor_defroast_flag==1){
				indoor_defroast_flag=0;
				indoor_defroast_count=0;
			}
	}
		
}

void tim3_process(){
	ADC_temp1_ok[adc_count]=ADC_Value[0];
	ADC_temp2_ok[adc_count]=ADC_Value[1];
	ADC_current_ok[adc_count]=ADC_Value[2];
	adc_count=(adc_count+1)%100;

	if(swing_info.on_flag==1)
	{
		swing_pole=(swing_pole+1)%4;
		swing_loop_count+=(swing_info.step && swing_pole==3);
		if(swing_loop_count<=swing_info.n){
			HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,(swing_info.dir==1)?swing_loop_count%4==0:swing_loop_count%4==3);
			HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,(swing_info.dir==1)?swing_loop_count%4==1:swing_loop_count%4==2);
			HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,(swing_info.dir==1)?swing_loop_count%4==2:swing_loop_count%4==1);
			HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,(swing_info.dir==1)?swing_loop_count%4==3:swing_loop_count%4==0);
			if(HAL_GPIO_ReadPin(Swing_M_1_GPIO_Port,Swing_M_1_Pin))
			{
			if(swing_pole==0)
				{
				if(swing_info.dir==1)
					HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,GPIO_PIN_SET);
				else 
					HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,GPIO_PIN_SET);
				}
			}
			else if(HAL_GPIO_ReadPin(Swing_M_2_GPIO_Port,Swing_M_2_Pin))
			{
				if(swing_pole==0){
				if(swing_info.dir==1)
					HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,GPIO_PIN_SET);
				else 
					HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,GPIO_PIN_SET);
				}
			}
			else if(HAL_GPIO_ReadPin(Swing_M_3_GPIO_Port,Swing_M_3_Pin))
			{
				if(swing_pole==0){
				if(swing_info.dir==1)
					HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,GPIO_PIN_SET);
				else 
					HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,GPIO_PIN_SET);
				}
			}
			else if(HAL_GPIO_ReadPin(Swing_M_4_GPIO_Port,Swing_M_4_Pin))
			{
				if(swing_pole==0){
				if(swing_info.dir==1)
					HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,GPIO_PIN_SET);
				else 
					HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,GPIO_PIN_SET);
				}
			}
		}
		else if(swing_loop_count>=swing_info.n+(swing_mode==01 && (((Work_Mode==fan_mode|| Work_Mode==cool_mode || Work_Mode==dry_mode) && swing_info.dir==2)|| (Work_Mode==heat_mode && swing_info.dir==1)))*swing_info.off_time*50)
		{
			swing_info.on_flag=0;
			HAL_GPIO_WritePin(Swing_M_1_GPIO_Port,Swing_M_1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Swing_M_2_GPIO_Port,Swing_M_2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Swing_M_3_GPIO_Port,Swing_M_3_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Swing_M_4_GPIO_Port,Swing_M_4_Pin,GPIO_PIN_RESET);
		}
	}
	
	if(air_flow_flag && Power==3 && Is_Indoor_Fan_On())
	{
		if(air_flow_loop_count%4==0){
		HAL_GPIO_WritePin(Flow_M_1_GPIO_Port,Flow_M_1_Pin,(air_flow_loop_count<300*12)?(air_flow_loop_count%16==0):air_flow_loop_count%16==12);
		HAL_GPIO_WritePin(Flow_M_2_GPIO_Port,Flow_M_2_Pin,(air_flow_loop_count<300*12)?(air_flow_loop_count%16==4):air_flow_loop_count%16==8);
		HAL_GPIO_WritePin(Flow_M_3_GPIO_Port,Flow_M_3_Pin,(air_flow_loop_count<300*12)?(air_flow_loop_count%16==8):air_flow_loop_count%16==4);
		HAL_GPIO_WritePin(Flow_M_4_GPIO_Port,Flow_M_4_Pin,(air_flow_loop_count<300*12)?(air_flow_loop_count%16==12):air_flow_loop_count%16==0);}
		air_flow_loop_count++;
		if(air_flow_loop_count>=300*24)
			air_flow_loop_count=0;
	}
	else 
	{
				HAL_GPIO_WritePin(Flow_M_1_GPIO_Port,Flow_M_1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Flow_M_2_GPIO_Port,Flow_M_2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Flow_M_3_GPIO_Port,Flow_M_3_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Flow_M_4_GPIO_Port,Flow_M_4_Pin,GPIO_PIN_RESET);
			}
}
void process_work(){
		if(auto_mode_selected)
	  {
			if(envirment_temp>27 && Work_Mode!=cool_mode){
				Work_Mode=cool_mode;
				level_work=0;
				swing_level=0;
				swing_loop_count=0;
				swing_info.on_flag=0;
			}
			else if(envirment_temp<23 && Work_Mode!=heat_mode){
				Work_Mode=heat_mode;
				level_work=0;
				swing_level=0;
				swing_loop_count=0;
				swing_info.on_flag=0;
			}
			else if(((Work_Mode==heat_mode && envirment_temp>25) || (Work_Mode==cool_mode && envirment_temp<25)) && (Work_Mode!=fan_mode) ){
				Work_Mode=fan_mode;
				level_work=0;
				swing_level=0;
				swing_loop_count=0;
				swing_info.on_flag=0;
			}
			else if(((Work_Mode!=heat_mode) && (envirment_temp>23)) && (Work_Mode!=fan_mode) ){
				if(((Work_Mode!=cool_mode) && (envirment_temp<27))){
				Work_Mode=fan_mode;
				level_work=0;
				swing_level=0;
				swing_loop_count=0;
				swing_info.on_flag=0;
				}
			}
	  }
	  switch(Work_Mode)
	  {
		case cool_mode:
			if(level_work==0)
			{
				if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_off_count=0;
					ElecValve_count=0;
				}
				else if(HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin) && ElecValve_count*0.5/60>2 && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET)
				{
					HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_RESET);
				}
				else if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET && HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin)==GPIO_PIN_RESET)
					level_work=1;
			}
			if(Power==3 && level_work==1)
			{
				
				if(swing_level<0 || swing_level==10){
					swing_level=0;
					swing_loop_count=0;
				}
					
				if((((envirment_temp-temp_setpoint)>2) || turbo_flag) && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET && indoor_defroast_flag==0)
					{
						if(comprasor_off_count*0.5/60>3)
						{
							HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_SET);
							HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_SET);
							comprasor_on_count=0;
						}
					}
				else if((((envirment_temp-temp_setpoint)<-2*(auto_mode_selected==0))  && turbo_flag==0)|| indoor_defroast_flag)
				{
					if(comprasor_on_count*0.5/60>3 && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_SET){
						HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
						comprasor_off_count=0;
					}
				}
				fan_indoor(indoor_fan_mode);
				if(swing_level==0 && swing_loop_count==0)
				{
					TIM3->ARR=initial_speed;
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
					TIM3->ARR=initial_speed;
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
					TIM3->ARR=default_speed;
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
					TIM3->ARR=default_speed;
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
			else if(Power==0 && level_work==1)
			{
				if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_off_count=0;
				}
				if(swing_level>=0)
				{
					TIM3->ARR=initial_speed;
					swing_info.on_flag=1;
					swing_info.dir=1;
					swing_info.n=170*4;
					swing_loop_count=0;
					swing_info.off_time=0;
					swing_level=-1;
				}
				else if(swing_info.on_flag==0 && swing_level==-1)
				{
					TIM3->ARR=initial_speed;
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=130*4;
					swing_loop_count=0;
					swing_info.off_time=30*(Is_Indoor_Fan_On()!=0);
					count_fan=0;
					swing_mode=01;
					swing_level=-2;
				}
				
				else if(swing_info.on_flag==0 && swing_level==-2){
					swing_loop_count=0;
					if(count_fan*0.5>30 || Is_Indoor_Fan_On()==0)
					{
						TIM3->ARR=initial_speed;
						fan_indoor(-1);
						swing_level=-3;
						swing_info.on_flag=1;
						swing_info.dir=2;
						swing_info.n=60*4;
						swing_loop_count=0;
						swing_info.off_time=0;
					}
				}
				else if(swing_info.on_flag==0 && swing_level==-3)
				{
					swing_loop_count=0;
					//level_work=0;
					//Work_Mode=0;
				}
			}

		break;
		case dry_mode:
			if(level_work==0)
			{
				if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_off_count=0;
					ElecValve_count=0;
				}
				else if(HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin) && ElecValve_count*0.5/60>2)
				{
					HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_RESET);
				}
				else if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET && HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin)==GPIO_PIN_RESET)
					{level_work=1;comprasor_dry_count=1200;count_fan=0;}
				if(Is_Indoor_Fan_On()){
					fan_indoor(-1);
				}
			}
			if(Power==3 && level_work==1)
			{
				if(swing_level<0 || swing_level==10){
					swing_level=0;
					swing_loop_count=0;
				}
				if(comprasor_off_count*0.5/60>3 && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET && comprasor_dry_count*0.5/60>7)
				{
					TIM3->ARR=initial_speed;
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_SET);
					comprasor_dry_count=0;
				}
				else if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_SET && comprasor_dry_count*0.5/60>5)
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_off_count=0;
					comprasor_dry_count=0;
				}
				fan_indoor(indoor_fan_mode);
				if(swing_level==0 && swing_loop_count==0)
				{
					TIM3->ARR=initial_speed;
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
					TIM3->ARR=initial_speed;
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=90*4;
					swing_info.off_time=0;
					swing_info.step=1;
				}
				else if(swing_level==1 && swing_info.on_flag==0)
				{
					swing_level=2;
					swing_loop_count=0;
				}
			}
			else if(Power==0 && level_work==1)
			{
				if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_off_count=0;
				}
				if(swing_level>=0)
				{
					TIM3->ARR=initial_speed;
					swing_info.on_flag=1;
					swing_info.dir=1;
					swing_info.n=170*4;
					swing_loop_count=0;
					swing_info.off_time=0;
					swing_level=-1;
				}
				else if(swing_info.on_flag==0 && swing_level==-1)
				{
					TIM3->ARR=initial_speed;
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=130*4;
					swing_loop_count=0;
					swing_info.off_time=30*(Is_Indoor_Fan_On()!=0);
					swing_mode=01;
					swing_level=-2;
				}
				
				else if(swing_info.on_flag==0 && swing_level==-2){
					swing_loop_count=0;
					count_fan=0;
					swing_level=-3;
				}
				else if(swing_info.on_flag==0 && swing_level==-3){
				if(count_fan*0.5>30 || Is_Indoor_Fan_On()==0)
					{
						TIM3->ARR=initial_speed;
						fan_indoor(-1);
						swing_level=-4;
						swing_info.on_flag=1;
						swing_info.dir=2;
						swing_info.n=60*4;
						swing_loop_count=0;
						swing_info.off_time=0;
					}
				}
				else if(swing_info.on_flag==0 && swing_level==-4)
				{
					swing_loop_count=0;
				}
			}	

		break;
		case heat_mode:
			if(level_work==0)
			{
				if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_off_count=0;
					ElecValve_count=0;
				}
				if(HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin) && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET){
					if(ElecValve_count*0.5/60>2)
					{
						HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_RESET);
					}
				}
				else if(((ElecValve_count*0.5/60>2.5) ||((ElecValve_count*0.5/60<2) && (HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin)==GPIO_PIN_RESET))) && Is_Indoor_Fan_On() && auto_mode_selected==0){
						fan_indoor(-1);
						level_work=1;
				}
				else if(!Is_Indoor_Fan_On() || auto_mode_selected==1)
					level_work=1;
			}
			if(Power==3 && level_work==1){
				if(swing_level<0){
					swing_level=0;
					swing_loop_count=0;
				}
				if((pipe_temp>pipe_temp_limit) && (HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_SET))
					fan_indoor(indoor_fan_mode);
				if(level_work==1){
					if((((envirment_temp-temp_setpoint)<-2)|| turbo_flag) && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET)
					{
						if(comprasor_off_count*0.5/60>3 )
						{
							HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_SET);
							HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_SET);
							HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_SET);
							comprasor_on_count=0;
						}

					}
					else if(((envirment_temp-temp_setpoint)>2*(auto_mode_selected==0)) && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_SET && turbo_flag==0 && auto_mode_selected==0)
					{
						if(comprasor_on_count*0.5/60>3){
							HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
							HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
							comprasor_off_count=0;
							ElecValve_count=0;
						}
					}
					if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET && HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin)==GPIO_PIN_SET && auto_mode_selected==0)
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
					if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_SET)
					{
						if(pipe_temp>fan_comprasor_temp_limit+1)
							HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
						else if(pipe_temp<fan_comprasor_temp_limit-1)
							HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_SET);
					}

				}

				if(swing_level==0 && swing_loop_count==0)
				{
					TIM3->ARR=initial_speed;
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
					TIM3->ARR=initial_speed;
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=110*4;
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
					TIM3->ARR=default_speed;
					swing_info.on_flag=1;
					swing_info.dir=1;
					swing_info.n=80*4;
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
					TIM3->ARR=default_speed;
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=80*4;
					swing_info.off_time=0;
					swing_info.step=(swing_mode!=11);
				}
				else if(swing_level==3 && swing_info.on_flag==0)
				{
					swing_level=(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET || !Is_Indoor_Fan_On())?10:2;
					swing_loop_count=0;
				}
				if(swing_level==10 && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_SET && Is_Indoor_Fan_On())
					swing_level=2;

	  		}
			else if(Power==0 && level_work==1)
			{
				if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_off_count=0;
					ElecValve_count=0;
				}
				if(ElecValve_count*0.5/60>2)
					HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_RESET);

				fan_indoor(-1);
				if(swing_level>=0)
				{
					TIM3->ARR=initial_speed;
					swing_info.on_flag=1;
					swing_info.dir=1;
					swing_info.n=170*4;
					swing_loop_count=0;
					swing_info.off_time=0;
					swing_level=-1;
				}
				else if(swing_info.on_flag==0 && swing_level==-1)
				{
					TIM3->ARR=initial_speed;
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=185*4;
					swing_loop_count=0;
					swing_info.off_time=30;
					swing_mode=01;
					swing_level=-2;
				}
				
				else if(swing_info.on_flag==0 && swing_level==-2){
					swing_loop_count=0;
					count_fan=0;
					swing_level=-3;
				}
			}
		break;
		case fan_mode:
			if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
			{
				HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
				comprasor_off_count=0;
				if(HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin))
					ElecValve_count=0;
				}
			if(HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin) && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET)
			{
				if(ElecValve_count*0.5/60>2)
				{
					HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_RESET);
				}
			}
			if(Power==3)
			{
				if(swing_level==10 || swing_level<0){
					swing_level=0;
					swing_loop_count=0;
				}
				fan_indoor(indoor_fan_mode);
				if(swing_level==0 && swing_loop_count==0)
				{
					TIM3->ARR=initial_speed;
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
					TIM3->ARR=initial_speed;
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
					TIM3->ARR=default_speed;
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
					TIM3->ARR=default_speed;
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
					TIM3->ARR=initial_speed;
					swing_info.on_flag=1;
					swing_info.dir=1;
					swing_info.n=170*4;
					swing_loop_count=0;
					swing_info.off_time=0;
					swing_level=-1;
				}
				else if(swing_info.on_flag==0 && swing_level==-1)
				{
					TIM3->ARR=initial_speed;
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=185*4;
					swing_loop_count=0;
					swing_info.off_time=30;
					swing_mode=01;
					swing_level=-2;
				}
				
				else if(swing_info.on_flag==0 && swing_level==-2){
					swing_loop_count=0;
					count_fan=0;
					swing_level=-3;
				}
			}	
		break;
	  }
}
void auto_fan_fcn(){
	if(Work_Mode==fan_mode)
		indoor_fan_mode=1;
	else if(Work_Mode==cool_mode){
		if((-temp_setpoint+envirment_temp)<2)
			indoor_fan_mode=0;
		else if((-temp_setpoint+envirment_temp)>3 && (-temp_setpoint+envirment_temp)<6)
			indoor_fan_mode=10;
		else if((-temp_setpoint+envirment_temp)>7)
			indoor_fan_mode=1;
	}
	else if(Work_Mode==heat_mode){
		if(((temp_setpoint-envirment_temp)<2))
			indoor_fan_mode=0;
		else if((temp_setpoint-envirment_temp)>3 && (temp_setpoint-envirment_temp)<6)
		{
			if(pipe_temp<pipe_temp_limit+4)
				indoor_fan_mode=0;
			else
				indoor_fan_mode=10;
		}
		else if((temp_setpoint-envirment_temp)>7){
			if(pipe_temp<pipe_temp_limit+7)
				indoor_fan_mode=10;
		else
			indoor_fan_mode=1;
		}
	}
}
void ir_process(){
	HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
	if(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13)==GPIO_PIN_RESET)
	{		
		receive_data();
		ir_praser();
	}
	Flag_IR=0;
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
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
				if(count>50)
					break;
		  }

		  if (count*debunce> 1200) // if the space is more than 1.2 ms
		  {
			  //code |= (1UL << (31-i));   // write 1
				rec_bits[i]=1;
		  }

		  else {//code &= ~(1UL << (31-i));  // write 0
				rec_bits[i]=0;
			}
	  }
		HAL_Delay(debunce_rep);
}
void ir_praser()
{
	flag_Read_IR=Is_Ir_Valid();
	
	if(flag_Read_IR)
	{
		flag_Read_IR=0;
		//on/off
		if(Power!=3*rec_bits[57]){
			if(Power==0)
			{
				level_work=0;
				swing_level=0;
				swing_loop_count=0;
				swing_info.on_flag=0;
			}	
		}
		Power=3*rec_bits[57];
	//mode
		int pre_mode=Work_Mode;
		switch((rec_bits[69]*100+rec_bits[70]*10+rec_bits[71])){
			case 1:
				Work_Mode=fan_mode;
				auto_mode_selected=0;
			break;
			case 101:
				Work_Mode=dry_mode;
				auto_mode_selected=0;
			break;
			case 11:
				Work_Mode=cool_mode;
				auto_mode_selected=0;
			break;
			case 110:
				Work_Mode=heat_mode;
				auto_mode_selected=0;
			break;
			case 111:
				auto_mode_selected=1;
			break;
		}

		if(pre_mode!=Work_Mode && auto_mode_selected==0){
			level_work=0;
			swing_level=0;
			swing_loop_count=0;
			swing_info.on_flag=0;
		}
	
	//Temp
		temp_setpoint=rec_bits[72]+rec_bits[73]*2+rec_bits[74]*4+rec_bits[75]*8+rec_bits[76]*16+16;	
		
	//fan speed
		if(rec_bits[53]*10+rec_bits[54]==11)
			auto_fan_flag=1;
		else
		{
			indoor_fan_mode=rec_bits[53]*10+rec_bits[54];
			auto_fan_flag=0;
		}
	//swing
		swing_mode=rec_bits[50]*10+rec_bits[51];
		swing_info.step=(swing_mode==11)?0:1;

	//air flow
		air_flow_flag=rec_bits[52];
	
	//timer
		if((Timing==0 && rec_bits[15]==1))
			timer_count=0;
		if(Timer_setpoint!=rec_bits[8]+rec_bits[9]*2+rec_bits[10]*4+rec_bits[11]*8+rec_bits[12]*16)
			timer_blink=0;
		Timing=3*rec_bits[15];
		Timer_setpoint=(Timing)?rec_bits[8]+rec_bits[9]*2+rec_bits[10]*4+rec_bits[11]*8+rec_bits[12]*16:10000;
	
	//hold
	
	//sleep
		Sleeping=3*rec_bits[56];
		if(Sleeping)
			sleep_count=0;
	//turbo
		if(rec_bits[27]==1)
		{
			turbo_flag=1;
			indoor_fan_mode=1;
		}
		else if(rec_bits[27]==0)
		{
			turbo_flag=0;
		}

	if(Power!=0)
	{
		tm1637SetBrightness(4);
		Count_Show=(temp_setpoint*1000)+(Power*100)+(Timing*10)+Sleeping;
		if((Work_Mode==fan_mode || Work_Mode==dry_mode) && auto_mode_selected==0)
			tm1637Display_null(Count_Show, 0);
		else
			tm1637DisplayDecimal(Count_Show, 0);
			
	}
	else{
		tm1637SetBrightness(0);
	}
	//lamp
	if(Power){
		tm1637SetBrightness(4*rec_bits[16]);
	}
	ir_bits_to_32_bit_arr();
	Write_To_Flash(ir_32bit_arr[0],pageAddress_1,2);
	Write_To_Flash(ir_32bit_arr[1],pageAddress_2,2);
	Write_To_Flash(ir_32bit_arr[2],pageAddress_3,3);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_RESET);
 }
 }
int Is_Ir_Valid(){
	ir_sum=0;
	int j;
	for(j=0;j<96;j++)
		ir_sum+=rec_bits[j];
	ans=(ir_sum==48)?1:0;
	ans=ans && (rec_bits[49]!=rec_bits[57]);
	ans=ans && (rec_bits[69]!=rec_bits[77]) && (rec_bits[70]!=rec_bits[78]) && (rec_bits[71]!=rec_bits[79]);
	ans=ans && (rec_bits[72]!=rec_bits[64]) && (rec_bits[73]!=rec_bits[65]) && (rec_bits[74]!=rec_bits[66]) && (rec_bits[75]!=rec_bits[67]) && (rec_bits[76]!=rec_bits[68]);
	ans=ans && ((rec_bits[72]+rec_bits[73]*2+rec_bits[74]*4+rec_bits[75]*8+rec_bits[76]*16)<17);
	ans=ans && (rec_bits[53]!=rec_bits[61]) && (rec_bits[54]!=rec_bits[62]);
	ans=ans && (rec_bits[50]!=rec_bits[58]) && (rec_bits[51]!=rec_bits[59]);
	ans=ans && (rec_bits[52]!=rec_bits[60]);
	ans=ans && (rec_bits[7]!=rec_bits[15]) && (rec_bits[8]!=rec_bits[0]) && (rec_bits[9]!=rec_bits[1]) && (rec_bits[10]!=rec_bits[2]) && (rec_bits[11]!=rec_bits[3]) && (rec_bits[12]!=rec_bits[4]);
	ans=ans && ((((rec_bits[8] || rec_bits[9] || rec_bits[10] || rec_bits[11] || rec_bits[12])==0) && (rec_bits[15]==1))==0);
	ans=ans && (rec_bits[18]!=rec_bits[26]);
	ans=ans && (rec_bits[48]!=rec_bits[56]);
	ans=ans && (rec_bits[19]!=rec_bits[27]);
	ans=ans && (rec_bits[16]!=rec_bits[24]);
	if(ir_sum!=48 && ans)
		return 0;
	return ans;
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
void fan_indoor(int speed){
	switch(speed)
	{
		case 0:
			if(HAL_GPIO_ReadPin(Fan_Indoor_3_GPIO_Port,Fan_Indoor_3_Pin) || HAL_GPIO_ReadPin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin)){
				HAL_GPIO_WritePin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Fan_Indoor_3_GPIO_Port,Fan_Indoor_3_Pin,GPIO_PIN_RESET);
				HAL_Delay(100);
			}
			HAL_GPIO_WritePin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin,GPIO_PIN_SET);
		break;
		case 10:
			if(HAL_GPIO_ReadPin(Fan_Indoor_3_GPIO_Port,Fan_Indoor_3_Pin) || HAL_GPIO_ReadPin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin)){
				HAL_GPIO_WritePin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Fan_Indoor_3_GPIO_Port,Fan_Indoor_3_Pin,GPIO_PIN_RESET);
				HAL_Delay(100);
			}
			HAL_GPIO_WritePin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin,GPIO_PIN_SET);
		break;
		case 1:
			if(HAL_GPIO_ReadPin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin) || HAL_GPIO_ReadPin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin)){
				HAL_GPIO_WritePin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin,GPIO_PIN_RESET);
				HAL_Delay(100);
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

float adc2temp(float adc){
	//y = -0.00000023x3 + 0.00038873x2 - 0.32278204x + 124.02708169
	return  -0.00000023*(adc*adc*adc)+0.00038873*(adc*adc)-0.32278204*adc+124.02708169;
}
float adc2current(float adc){
	return  0.0151*adc + 0.0411;
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

void ir_bits_to_32_bit_arr(){
	int row,col;
	for(row=0;row<3;row++)
	{
		ir_32bit_arr[row]=0;
		for(col=0;col<32;col++)
		{
			ir_32bit_arr[row]|=(rec_bits[32*row+col]<<col);
		}
	}
}
void _32_bit_arr_to_ir_bits()
{
	int row,col;
	for(row=0;row<3;row++)
	{
		for(col=0;col<32;col++)
		{
			rec_bits[32*row+col]=(ir_32bit_arr[row]>>col)%2;
		}
	}
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
