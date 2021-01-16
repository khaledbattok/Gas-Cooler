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
int Power,Timing,Sleeping,flag_Read_IR;
int sleep_count=0,timer_count;
int cnt=0;
float envirment_temp;
float pipe_temp;
int rec_bits[255];
int temp_setpoint;
int Timer_setpoint;
int comprasor_count,ElecValve_count,swing_time_count,swing_loop_count,swing_dir,on_off_flag,swing_mode;
FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t pageAddress_1=0x0800E000;
uint32_t pageAddress_2=0x0800E400;
uint32_t pageAddress_3=0x0800E800;
uint32_t PAGEError=0;
int Work_Mode;
int auto_mode_selected;
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
int comprasor_dry_count;
int turbo_flag;
uint32_t ir_32bit_arr[3];
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
int Is_Indoor_Fan_On(void);
int Is_Ir_Valid(void);
void ir_bits_to_32_bit_arr(void);
void _32_bit_arr_to_ir_bits(void);
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
	ir_32bit_arr[0]=*(__IO uint32_t *)(pageAddress_1);
	ir_32bit_arr[1]=*(__IO uint32_t *)(pageAddress_2);
	ir_32bit_arr[2]=*(__IO uint32_t *)(pageAddress_3);
	_32_bit_arr_to_ir_bits();
	ir_praser();
	/*Power=*(__IO uint32_t *)(pageAddress_start);
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
	}*/
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
			else if((Work_Mode==heat_mode && envirment_temp>25) || (Work_Mode==cool_mode && envirment_temp<25) ){
				Work_Mode=fan_mode;
				level_work=0;
				swing_level=0;
				swing_loop_count=0;
				swing_info.on_flag=0;
			}
			else if(((Work_Mode!=heat_mode) || (Work_Mode!=cool_mode)) && (envirment_temp<27) && (envirment_temp>23)){
				Work_Mode=fan_mode;
				level_work=0;
				swing_level=0;
				swing_loop_count=0;
				swing_info.on_flag=0;
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
					comprasor_count=0;
					ElecValve_count=0;
				}
				else if(HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin) && ElecValve_count*0.5/60>2)
				{
					HAL_GPIO_WritePin(ElecValve_GPIO_Port,ElecValve_Pin,GPIO_PIN_RESET);
				}
				else if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET && HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin)==GPIO_PIN_RESET)
					level_work=1;
				if(Is_Indoor_Fan_On()){
					fan_indoor(-1);
				}
			}
			if(Power==3 && level_work==1)
			{
				if(swing_level<0 || swing_level==10)
					swing_level=0;
				if(temp_setpoint<(envirment_temp+2) || turbo_flag)
					{
						if(comprasor_count*0.5/60>3)
						{
							HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_SET);
							HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_SET);
						}
					}
				else if((temp_setpoint>(envirment_temp-2)) && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_SET && turbo_flag==0 )
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_count=0;
				}
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
			else if(Power==0 && level_work==1)
			{
				if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin))
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_count=0;
				}
				if(swing_level>=0)
				{
					swing_info.on_flag=1;
					swing_info.dir=1;
					swing_info.n=170*4;
					swing_loop_count=0;
					swing_info.off_time=0;
					swing_level=-1;
				}
				else if(swing_info.on_flag==0 && swing_level==-1)
				{
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=130*4;
					swing_loop_count=0;
					swing_info.off_time=30;
					count_fan=0;
					swing_mode=01;
					swing_level=-2;
				}
				
				else if(swing_info.on_flag==0 && swing_level==-2){
					swing_loop_count=0;
					if(count_fan*0.5>30)
					{
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
					comprasor_count=0;
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
				if(swing_level<0 || swing_level==10)
					swing_level=0;
				if(comprasor_count*0.5/60>3 && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET && comprasor_dry_count*0.5/60>7)
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_SET);
					comprasor_dry_count=0;
				}
				else if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_SET && comprasor_dry_count*0.5/60>5)
				{
					HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
					comprasor_count=0;
					comprasor_dry_count=0;
				}
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
					comprasor_count=0;
				}
				if(swing_level>=0)
				{
					swing_info.on_flag=1;
					swing_info.dir=1;
					swing_info.n=170*4;
					swing_loop_count=0;
					swing_info.off_time=0;
					swing_level=-1;
				}
				else if(swing_info.on_flag==0 && swing_level==-1)
				{
					swing_info.on_flag=1;
					swing_info.dir=2;
					swing_info.n=130*4;
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
				else if(swing_info.on_flag==0 && swing_level==-3){
				if(count_fan*0.5>30)
					{
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
					comprasor_count=0;
					ElecValve_count=0;
				}
				level_work=1;
				if(Is_Indoor_Fan_On()){
					fan_indoor(-1);
				}
			}
			if(Power==3 && level_work==1){
				if(swing_level<0)
					swing_level=0;
				if(level_work==1){

					if(temp_setpoint>(envirment_temp+2) || turbo_flag)
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
					else if((temp_setpoint<(envirment_temp-2)) && HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_SET && turbo_flag==0)
					{
						HAL_GPIO_WritePin(Compressor_GPIO_Port,Compressor_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(Fan_Compressor_GPIO_Port,Fan_Compressor_Pin,GPIO_PIN_RESET);
						comprasor_count=0;
						ElecValve_count=0;
					}
					if(HAL_GPIO_ReadPin(Compressor_GPIO_Port,Compressor_Pin)==GPIO_PIN_RESET && HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin)==GPIO_PIN_SET)
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
				else if(swing_info.on_flag==0){
					swing_loop_count=0;
					//level_work=0;
					//Work_Mode=0;
				}
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
				if(swing_level==10 || swing_level<0)
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
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Flag_IR==1)
		{
			HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
			if(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_13)==GPIO_PIN_RESET)
			{		
				receive_data();
				ir_praser();
			}
			Flag_IR=0;
			HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
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
		comprasor_count++;
		ElecValve_count++;
		count_fan++;
		comprasor_dry_count++;
		sleep_count++;
		timer_count++;
		if(((Work_Mode==heat_mode && !Is_Indoor_Fan_On())||(((Work_Mode==cool_mode || Work_Mode==dry_mode ) && HAL_GPIO_ReadPin(ElecValve_GPIO_Port,ElecValve_Pin)))) &&(!Timing || (Timing && timer_count>15)))
		{
			Count_Show=(temp_setpoint*1000)+(((comprasor_count%2)*3)*100)+(Timing*10)+Sleeping;
			tm1637DisplayDecimal(Count_Show, 0);
		}
		else if(Work_Mode==heat_mode &&(!Timing || (Timing && timer_count>15))){
			Count_Show=(temp_setpoint*1000)+(Power*100)+(Timing*10)+Sleeping;
			tm1637DisplayDecimal(Count_Show, 0);
		}

		if(Timing)
		{
			if(timer_count<10)
			{
				tm1637SetBrightness(4*(timer_count%2==0));
				Count_Show=(Timer_setpoint*1000)+(Power*100)+(Timing*10)+Sleeping;
				tm1637DisplayDecimal(Count_Show, 0);
			}
			else if(timer_count<15)
			{
				tm1637SetBrightness((Sleeping && sleep_count*0.5>10)?0:4);
				Count_Show=(temp_setpoint*1000)+(Power*100)+(Timing*10)+Sleeping;
				if(Work_Mode==fan_mode || Work_Mode==dry_mode)
					tm1637Display_null(Count_Show, 0);
				else
					tm1637DisplayDecimal(Count_Show, 0);
			}
			if(timer_count*0.5/3600>=Timer_setpoint)
			{
				Power=0;
				timer_count=0;
				Timing=0;
				tm1637SetBrightness(0);
			}
		}
		if(Sleeping && sleep_count*0.5>10){
			tm1637SetBrightness(0);
		}
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
				else if(swing_loop_count>=swing_info.n+(swing_mode==01 && (((Work_Mode==fan_mode|| Work_Mode==cool_mode || Work_Mode==dry_mode) && swing_info.dir==2)|| (Work_Mode==heat_mode && swing_info.dir==1)))*swing_info.off_time*50)
				{
					swing_info.on_flag=0;
				}
			}
			
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
	flag_Read_IR=Is_Ir_Valid();
	
	if(flag_Read_IR)
	{
		//on/off
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
		indoor_fan_mode=rec_bits[53]*10+rec_bits[54];
	//swing
		swing_mode=rec_bits[50]*10+rec_bits[51];
		swing_info.step=(swing_mode==11)?0:1;

	//air flow
		air_flow_flag=rec_bits[52];
	
	//timer
		if((Timing==0 && rec_bits[15]==1) || Timer_setpoint!=(rec_bits[8]+rec_bits[9]*2+rec_bits[10]*4+rec_bits[11]*8+rec_bits[12]*16))
			timer_count=0;
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
			indoor_fan_mode=11;
		}
		else if(rec_bits[27]==0)
		{
			turbo_flag=0;
		}

	if(Power!=0)
	{
		tm1637SetBrightness(4);
		Count_Show=(temp_setpoint*1000)+(Power*100)+(Timing*10)+Sleeping;
		if(Work_Mode==fan_mode || Work_Mode==dry_mode)
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
	int ans=1;
	ans=ans && (rec_bits[49]!=rec_bits[57]);
	ans=ans && rec_bits[69]!=rec_bits[77] && rec_bits[70]!=rec_bits[78] && rec_bits[71]!=rec_bits[79];
	ans=ans && rec_bits[72]!=rec_bits[64] && rec_bits[73]!=rec_bits[65] && rec_bits[74]!=rec_bits[66] && rec_bits[75]!=rec_bits[67] && rec_bits[76]!=rec_bits[68];
	ans=ans && rec_bits[53]!=rec_bits[61] && rec_bits[54]!=rec_bits[62];
	ans=ans && rec_bits[50]!=rec_bits[58] && rec_bits[51]!=rec_bits[59];
	ans=ans && rec_bits[52]!=rec_bits[60];
	ans=ans && rec_bits[8]!=rec_bits[0] && rec_bits[9]!=rec_bits[1] && rec_bits[10]!=rec_bits[2] && rec_bits[11]!=rec_bits[3] && rec_bits[12]!=rec_bits[4];
	ans=ans && rec_bits[18]!=rec_bits[26];
	ans=ans && rec_bits[48]!=rec_bits[56];
	ans=ans && rec_bits[19]!=rec_bits[27];
	ans=ans && rec_bits[16]!=rec_bits[24];
	
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
		case 1:
			if(HAL_GPIO_ReadPin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin) || HAL_GPIO_ReadPin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin)){
				HAL_GPIO_WritePin(Fan_Indoor_1_GPIO_Port,Fan_Indoor_1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Fan_Indoor_2_GPIO_Port,Fan_Indoor_2_Pin,GPIO_PIN_RESET);
				HAL_Delay(200);
			}
			HAL_GPIO_WritePin(Fan_Indoor_3_GPIO_Port,Fan_Indoor_3_Pin,GPIO_PIN_SET);
		break;
		case 11 :
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
