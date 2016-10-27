/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

/* USER CODE BEGIN 0 */
#include "tim.h"
#include "usart.h"

#define TRANSMIT_IR_CODE

#define IR_ON         0x01
#define IR_MUTE       0x02
#define IR_IN1        0x03
#define IR_IN2        0x04
#define IR_IN3        0x05
#define IR_INMODE     0x06
#define IR_SCL1X			0x11
#define IR_SCL2X			0x12
#define IR_SCL3X			0x13
#define IR_SCLINE			0x14
#define IR_NAVU       0x15
#define IR_NAVD       0x16
#define IR_NAVL       0x21
#define IR_NAVR       0x22
#define IR_NAVOK      0x23
#define IR_NAVBK			0x24
#define IR_PSETA			0x25
#define IR_PSETB			0x26
#define IR_PSETC			0x31
#define IR_PSETD			0x32
#define IR_PSETE			0x33
#define IR_PSETF			0x34
#define IR_PSETG			0x35
#define IR_PSET0			0x36

#define KEY_ON        0x50
#define KEY_MUTE      0x51
#define KEY_IN1       0x52
#define KEY_IN2       0x53
#define KEY_IN3       0x54
#define KEY_INMODE    0x55
#define KEY_SCL1X     0x56
#define KEY_SCL2X     0x57
#define KEY_SCL3X     0x58
#define KEY_SCLINE    0x59
#define KEY_NAVU      0x5A
#define KEY_NAVD      0x5B
#define KEY_NAVL      0x5C
#define KEY_NAVR      0x5D
#define KEY_NAVOK     0x5E
#define KEY_NAVBK     0x5F
#define KEY_PSETA     0x60
#define KEY_PSETB     0x61
#define KEY_PSETC     0x62
#define KEY_PSETD     0x63
#define KEY_PSETE     0x64
#define KEY_PSETF     0x65
#define KEY_PSETG     0x66
#define KEY_PSET0     0x67

#define READ_DIP_SWITCH_0		HAL_GPIO_ReadPin(DIP_SWITCH_BIT_0_GPIO_Port, DIP_SWITCH_BIT_0_Pin)
#define READ_DIP_SWITCH_1		HAL_GPIO_ReadPin(DIP_SWITCH_BIT_1_GPIO_Port, DIP_SWITCH_BIT_1_Pin)


#define IDLE 		0      // mode when we wait for start of next frame
#define RECEIVE 1   	 // mode when we receive the frame

#define CHECK 	0
#define IN 			1

uint8_t last_bit, bit_cnt;
uint8_t mode = 0;
uint8_t last_state = CHECK;
uint8_t correct_data;
uint16_t receive_data;
uint8_t last_edge;
uint8_t key_code;
uint8_t last_control_bit;
uint8_t first_load = 1;

void Transmit_Key_Code(void);

void Transmit_IR_Code(void);

uint8_t Test_Address(void);


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim16;

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line 0 and 1 interrupts.
*/
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */
	HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, GPIO_PIN_RESET);
  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
* @brief This function handles EXTI line 4 to 15 interrupts.
*/
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
	
  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */
	uint16_t tmp;
	
	if(mode == IDLE)
	{
		__HAL_TIM_SET_COUNTER(&htim16, 0);
		HAL_TIM_Base_Start_IT(&htim16);
		
		receive_data = 1;
		bit_cnt = 1;
		last_bit = 1;
		mode = RECEIVE;
		last_state = CHECK;
		last_edge = 0;
	}
	else
	{
		tmp = __HAL_TIM_GET_COUNTER(&htim16);
		__HAL_TIM_SET_COUNTER(&htim16, 0);
		
		if((tmp > 500) && (tmp < 1000)) // Short delay
		{ 
			
			if(bit_cnt == 14)
			{
				last_edge = 1;
			}
			if(last_state == CHECK)
			{
				last_state = IN;			
			}
			else
			{ 
				receive_data = receive_data << 1;
				receive_data |= last_bit;
			  bit_cnt++;
				last_state = CHECK;
			}
			if((bit_cnt == 14 && !last_bit) || (bit_cnt == 14 && last_bit && last_edge))
			{
				if((last_control_bit != ((receive_data >> 11) & 0x01)) || (first_load))
				{
					first_load = 0;
					
					#ifdef TRANSMIT_KEY_CODE
					Transmit_Key_Code();
					#endif
					
					#ifdef TRANSMIT_IR_CODE
					Transmit_IR_Code();
					#endif
					last_control_bit = (receive_data >> 11) & 0x01;
				}
				else
				{
					__HAL_TIM_DISABLE(&htim16);
					mode = IDLE;
				}
			}	
			
		}
		else
		if((tmp > 1000) && (tmp < 2000))
		{
				if(bit_cnt == 14){
					last_edge = 1;
				}
				else{
				receive_data = receive_data << 1;
				last_bit = !last_bit;;
				receive_data |= last_bit;
				bit_cnt++;
				last_state = CHECK;}
			
			if((bit_cnt == 14 && !last_bit) || (bit_cnt == 14 && last_bit && last_edge))
			{	
				if((last_control_bit != ((receive_data >> 11) & 0x01)) || first_load){
					
					if(Test_Address()){
						
						first_load = 0;
						
						#ifdef TRANSMIT_KEY_CODE
						Transmit_Key_Code();
						#endif
						
						#ifdef TRANSMIT_IR_CODE
						Transmit_IR_Code();
						#endif
						
						last_control_bit = (receive_data >> 11) & 0x01;
					}
				}
				else
				{
					__HAL_TIM_DISABLE(&htim16);
					mode = IDLE;
				}
			}
		}
		else
		{
			mode = IDLE;
			__HAL_TIM_DISABLE(&htim16);
		  		
		}
	}

  /* USER CODE BEGIN EXTI4_15_IRQn 0 */


  /* USER CODE END EXTI4_15_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
* @brief This function handles TIM16 global interrupt.
*/
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */
	__HAL_TIM_DISABLE(&htim16);
	mode = IDLE;
  /* USER CODE END TIM16_IRQn 1 */
}

/* USER CODE BEGIN 1 */




void Transmit_Key_Code(void)
{
  uint8_t valid_key;
	
  __HAL_TIM_DISABLE(&htim16);
  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
  __HAL_UART_ENABLE(&huart1);
  mode = IDLE;
	
	correct_data = receive_data & 0x3F;
 
	valid_key = 0x01;
	key_code = 0;
	
  switch (correct_data) {
    case IR_ON:
    {
      key_code = KEY_ON;
    }break;
		
    case IR_MUTE:
    {
      key_code = KEY_MUTE;
    }break;

    case IR_IN1:
    {
      key_code = KEY_IN1;
    }break;

    case IR_IN2:
    {
      key_code = KEY_IN2;
    }break;

    case IR_IN3:
    {
      key_code = KEY_IN3;
    }break;
		
		case IR_INMODE:
		{
			key_code = KEY_INMODE;
		}break;
		
		case IR_SCL1X:
		{
			key_code = KEY_SCL1X ;
		}break;
		
		case IR_SCL2X:
		{
			key_code = KEY_SCL2X ;
		}break;
		
		case IR_SCL3X:
		{
			key_code = KEY_SCL2X;
		}break;
		
		case IR_SCLINE:
		{
			key_code = KEY_SCLINE;
		}break;
		
    case IR_NAVU:
    {
      key_code = KEY_NAVU;
    }break;

    case IR_NAVD:
    {
      key_code = KEY_NAVD;
    }break;

    case IR_NAVR:
    {
      key_code = KEY_NAVR;
    }break;

    case IR_NAVL:
    {
      key_code = KEY_NAVL;
    }break;

    case IR_NAVOK:
    {
      key_code = KEY_NAVOK;
    }break;
		
		case IR_NAVBK:
    {
      key_code = KEY_NAVBK;
    }break;
		
		case IR_PSETA:
    {
      key_code = KEY_PSETA;
    }break;
		
		case IR_PSETB:
    {
      key_code = KEY_PSETB;
    }break;
		
		case IR_PSETC:
    {
      key_code = KEY_PSETC;
    }break;
		
		case IR_PSETD:
    {
      key_code = KEY_PSETD;
    }break;
		
		case IR_PSETE:
    {
      key_code = KEY_PSETE;
    }break;
		
		case IR_PSETF:
    {
      key_code = KEY_PSETF;
    }break;
		
		case IR_PSETG:
    {
      key_code = KEY_PSETG;
    }break;
		
		case IR_PSET0:
    {
      key_code = KEY_PSET0;
    }break;
		
		default:
		{
			valid_key = 0x00;
		} break;
  }
  if(key_code == KEY_ON)
  {
    HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, GPIO_PIN_SET);
  }
  if(valid_key){
		HAL_GPIO_WritePin(IR_VALID_GPIO_Port, IR_VALID_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit(&huart1, &key_code, 1,100);
		HAL_GPIO_WritePin(IR_VALID_GPIO_Port, IR_VALID_Pin, GPIO_PIN_RESET);
  }
  __HAL_UART_DISABLE(&huart1);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}



void Transmit_IR_Code(void)
{
  __HAL_TIM_DISABLE(&htim16);
  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
  __HAL_UART_ENABLE(&huart1);
  mode = IDLE;
	
	correct_data = receive_data & 0x3F;
 
  if(correct_data == IR_ON)
  {
    HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, GPIO_PIN_SET);
  }
  	
	HAL_GPIO_WritePin(IR_VALID_GPIO_Port, IR_VALID_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, &correct_data, 1,100);
	HAL_GPIO_WritePin(IR_VALID_GPIO_Port, IR_VALID_Pin, GPIO_PIN_RESET);
  
	__HAL_UART_DISABLE(&huart1);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

uint8_t Test_Address(void)
{
	uint8_t address;
	
	address = (correct_data >> 6) & 0x1F;
	
	if(address == 24 || address == 25 || address == 27 || address == 28)
	{
		if(!READ_DIP_SWITCH_0 && !READ_DIP_SWITCH_1 && address == 24)
		{
			return 1;
		}
		else
		if(!READ_DIP_SWITCH_0 && READ_DIP_SWITCH_1 && address == 25)
		{
			return 1;
		}
		else
		if(READ_DIP_SWITCH_0 && !READ_DIP_SWITCH_1 && address == 28)
		{
			return 1;
		}
		else
		if(READ_DIP_SWITCH_0 && READ_DIP_SWITCH_1 && address == 27)
		{
			return 1;
		}
		else
			return 0;

	}
	
	
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
