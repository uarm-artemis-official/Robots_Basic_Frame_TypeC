/* USER CODE BEGIN Header */
/*
 ************************************************************************************************
 * 	  	    		2023 UARM ELECTRICAL&CONTROL TEAM ROBOTS BASIC FRAME TTPE-C     		     *
 ************************************************************************************************
 * 							   Project Copyright (c) 2023 UARM Artemis.
 *
 *
 *                                         .]]]@@@@@@@@@@@@\]`.
 *                                   ,]@@@@[[..            .,[\@@@\].
 *                               ,/@@[`                           .[\@@].
 *                         ,/\]@@/.                                    ,\@\`
 *                       ,@@@@/.                                          ,\@\..`
 *                     /@@@/`           .@@^                                 ,@@`\`
 *                   ,@@@/.          \@/                                       ,@@`\`
 *                 ,@@@@.         @@/        ]]]]]@@@@@@@@@@@@@@.                ,@\.\
 *                  /@`        =@@.    ]]]]]]@@@@@@@@@@@/[[[[[[[                  .\@`=`
 *                .@@.       ]]...     @@       =@@@@.=@^                           ,@\/.
 *               .@/        .\@`       @@       ..... =@^                            ,@\
 *              .@/       .@@^         @@@@^          =@^                             ,@\
 *            =[@@.      ,].           [@@@@@@@@@.    =@\...,@@@@@@@@@@@@@@.           ,@^
 *           ,^=@`       \@^           .@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.            =@`
 *           /.@^       /@^            .@@@@@@@@@@@@@@@                     .       .`  .@\
 *          .[\@.       ,[..]]]]]]`    .@@@@@@@@@@@@@@@                   ,@@        @.  =@^
 *            @/           =@\]].     @@@@@@@@@@@@@@@@@@\.=@@@./@@@@\`  ,@@@@        @^  .@^
 *           ,@^           ,[@@@^   =@/[[[[[[[@@@@@@@@@=@\/@@@.@@@@@@^,@@``@@       .@@   @@@^
 *           =@^            /@=@^ ,`=@^       @@\]]]]]]@@@@@@@.@@@@@@^O`/@^@@       /@@   \@@^
 *           =@^           /@.,[` =^=@^       @@@@@/@@@@@@@@^]]]]]]]]]]]]]`@@      /@@@   =@@^
 *           =@^          =@.@@@^ =\=@^       .]/@O.[\@@@@@@/[[[[[[[[[[[[[[[[    .@@@@@   @@@^
 *           .@^         .@^/@@@\]...=@@@@@@\@[]@\`\@=@.      .,]]].             =@@@@^  .@@@`
 *            \@.        .@@@[....,@\.@@/[\@@^@@@@@@=@@@@@@@@@[....,\@.          @@@@@`  =@@@
 *            ,@^        .@^,@`  ,@`,@\@` =@@^\@@@@^/@@@@@@@/.@[. ,\\.@`        =@@@@/   /@@^
 *             \@.       =@ @.=@@^ @.\^   =@^,@\]]/@`,[[[[[@`=^.@@^ =^=^       ,@@@@@.  =@^
 *            \.@\       .@^=\.  .//.@`   =@@@@@@@@@@      \^,@`   ,@.@^      ,@@@@@`  .@/
 *             \,@^       .@\.,[[`.]@`                      \\`,[[[.,@`      =@@@@@`  .@@
 *             .\,@\        .,\@@/[.                          ,[@@@[.      ./@@@@@`  .@@.
 *              .\,@\.   ..                                               /@@@@@/.  ,@/.
 *                [.@@`   ,\.                                          ./@@@@@@`   =@^
 *                   =@\.  .\@].                                     ,@.,@@@@/   .@@`
 *                    .\@`   .\@@@@@@@\`.                        .]@@@@\.=@`   ./@`
 *                      .\@\.   ,@@@@@@@@@@\`.             ..]/@@@@@@@@@/.   ./@[
 *                       ,@@@\`    ,\@@@@@@@@@@@@@@@@@@@@@@@@@  \@@@@[.    ]@@`=`
 *                         [@@@@\`     .[\@@@@@@@@@@@@@@@@@@@@@@/[      ]@@[./`
 *                            [@@@@@\`        .[[[[[[[[[[[`        .]/@@[\/`
 *                               ,\..[@@@\]`                 .]]@@@/`.
 *                                       .,[\@@@@@@@@@@@@@@@@@@@@@^
 *                                                .@@@@@@@[[[.
 *
 *           /@@@@@\     =@@@@@O@@@@@@@@@@@@@@@@@@^@@@@@@@@@@@@@@@@@].,/@@@@@@@@@@@@@@@@@@@]`.
 *           @@@@@@@     =@@@@@ @@@@@@@@@@@@@@@@@@^@@@@@@@@@@@@@@@@@ @@@@@@@@@@@@@@@@@@@@@@@@^
 *           @@@@@@@     =@@@@@ @@@@@@       @@@@@^@@@@*      =@@@@@ @@@@@@[[[[[\@@/[[[[\@@@@^
 *          @@@@@@@     =@@@@@@ @@@@@]]]]]]]@@@@@^@@@@\]]]]]]/@@@@@@ @@@@@     =@@^    =@@@@^
 *           @@@@@@@     =@@@@@ @@@@@@@@@@@@@@@@@@^@@@@@@@@@@@@@@@@@ @@@@@@     =@@^    =@@@@^
 *           @@@@@@@     =@@@@@ @@@@@@@@@@@@@@@@@@^@@@@@@@@@@@@@@@/@ @@@@@@     =@@^    =@@@@^
 *           @@@@@@@@@@@@@@@@@@ @@@@@@       @@@@@^@@@@^      =@@@@@@@@@@@@     =@@^    =@@@@^
 *          @@@@@@@@@@@@@@@@@@@ @@@@@       @@@@@^@@@@^      =@@@@@@@@@@@     =@@^    =@@@@^
 *
 *
 *
 *	@time 		Nov, 2023
 *	@version 	v1.0(Alpha test)
 *
 *
 * @attention
 * Code Frame Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * @attention
 * if you regenerate the code using cubemx, make sure:
 * 	1. set #define INCLUDE_vTaskDelayUntil 1 in the FreeRTOSConfig.h
 * 	2. delete the generated SPI NSS GPIO part in spi.c
 * 	3. delete the generated MX_IWDG_Init().(if you want to deactivate IWDG, just
 * 	   go to main.h and comment #define USE_IWDG 1)
 *
 * 	ENJOY!
 *
 * */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Control_App.h"
#include "Comm_App.h"
#include "motor.h"
#include "stdio.h"
#include "dwt.h"
#include "buzzer.h"
#include "auto_aim_pack.h"
#include "self_check.h"
#include "auto_aim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t shoot_reserve_flag = 0;
uint8_t shoot_reserve_counter = 0;
uint8_t shoot_check_flag = 0;
uint16_t shoot_check_counter = 0;

/* new defined variables*/
uint32_t debugger_signal_counter = 0;//count the idle time
uint32_t debugger_signal_flag = 0; //mark the debugger task
uint8_t ref_rx_frame[256]={0}; //referee temp frame buffer
uint8_t rc_rx_buffer[DBUS_BUFFER_LEN]; //rc temporary buffer
uint16_t chassis_gyro_counter = 0; // used for backup robots without slipring
uint8_t chassis_gyro_flag = 0;	   // used for backup robots without slipring
char pdata[PACKLEN]={0};	//old mini pc vision pack temp buffer

extern Motor motor_data[MOTOR_COUNT]; //MOTOR_COUNT
extern Referee_t referee;
extern CommVision_t vision_pack;
extern UC_Recv_Pack_t uc_rx_pack;
extern Buzzer_t buzzer;
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_TIM13_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C3_Init();
//  MX_IWDG_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
#ifdef USE_IWDG
/*To deactivate IWDG, go to main.h and comment #define USE_IWDG 1 */
  MX_IWDG_Init();//enable IWDG, period 2s
#endif
	HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);// turn off the green led
	if(firmware_and_system_init() != HAL_OK){
	  Error_Handler();
	}
	else
	  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET);// turn on the green led
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.LSIState = RCC_LSI_ON;//RCC_LSI_ON
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;//4
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef firmware_and_system_init(void){
 /* CAN1 & CAN2 Init */
 if( HAL_CAN_Start(&hcan1) != HAL_OK){
	 return HAL_ERROR;
 }
 if( HAL_CAN_Start(&hcan2) != HAL_OK){
 	 return HAL_ERROR;
 }
 /* CAN1 & CAN2 filter Init */
 can_filter_enable(&hcan1);
 can_filter_enable(&hcan2);

 /* Timer 13 IT Init */
 if( HAL_TIM_Base_Start_IT(&htim13) != HAL_OK){
  	 return HAL_ERROR;
 }
 /* Heat PWM signal Init */
 if( HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1) != HAL_OK){
   	 return HAL_ERROR;
 }
 /* Read Board Status */
 if(HAL_GPIO_ReadPin(Board_Status_GPIO_Port, Board_Status_Pin) == GPIO_PIN_RESET)
	  board_status = CHASSIS_BOARD;
 else
	  board_status = GIMBAL_BOARD;

 /* init fb struct of motors */
 for(int i=0;i<MOTOR_COUNT;i++){
	 memset(&(motor_data[i].motor_feedback), 0, sizeof(Motor_Feedback_Data_t));
 }
 /* referee system init*/
 referee_init(&referee);

 /* init buzzer */
 buzzer_init(&buzzer);

 /* init vision pack */
 uc_rx_pack_init(&uc_rx_pack);

 /* DWT init */
 dwt_init();

 return HAL_OK;
}
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if(htim->Instance == TIM13)
	{
		if(debugger_signal_flag == 1)
			++debugger_signal_counter;
		if(shoot_reserve_flag == 1)
			++shoot_reserve_counter;
		if(chassis_gyro_flag ==1)
			++chassis_gyro_counter;
	}
	  /* USER CODE END Callback 0 */
	  if (htim->Instance == TIM5) {
		  HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
/*
 * @brief  DMA Callback function, update all the dma transmission IT here
 * @note   This function is called when：
 * 			 Referee system recv: UART3_DMA1_Stream1
 * 			 Mini PC recv: 		  UART6_DMA2_Stream1
 *
 * 	THIS FUNCTION WAS ABONDONED BC THE HAL_DMA_START_IT NEED US TO CALL A UINT32_T ADDR WHICH IS
 * 	CONFLICT WITH REFEREE SYSTEM REQUIREMENT
 *
 * */
#ifdef ABONDONED_FUNC
void XferCpltCallback(DMA_HandleTypeDef *hdma){
//	HAL_GPIO_TogglePin(LD_G_GPIO_Port,LD_G_Pin);
	if(hdma->Instance == DMA1_Stream1){
		huart3.Instance->CR3&=~USART_CR3_DMAT;//disable the uart3 dma using register
		/*read data*/
		referee_read_data(&referee, ref_rx_frame);
		/* re-activate DMA */
//		HAL_UART_Receive_DMA(&huart3, ref_rx_frame, sizeof(ref_rx_frame));
		huart6.Instance->CR3|=USART_CR3_DMAT;
	}
	if(hdma->Instance == DMA2_Stream1 && board_status == CHASSIS_BOARD){
		huart6.Instance->CR3&=~USART_CR3_DMAT;//disable the uart3 dma using register
		/*read data from mini pc pack*/
		comm_pack.vision = parse_all(pdata);
		/* re-activate DMA */
//		HAL_UART_Receive_DMA(&huart6, pdata, PACKLEN);
		huart6.Instance->CR3|=USART_CR3_DMAT;
		}

//	HAL_GPIO_TogglePin(LD_G_GPIO_Port,LD_G_Pin);
}
#elif defined(USE_UART_DMA)
/*
 * @brief  UART DMA Callback function, update all the dma transmission IT here
 * @note   This function is called when：
 * 			 Referee system recv: UART3_DMA1_Stream1
 * 			 Mini PC recv: 		  UART6_DMA2_Stream1
 *
 * */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart == &huart2 && board_status == CHASSIS_BOARD){
	 /*read data*/
	 referee_read_data(&referee, ref_rx_frame);
	 /* re-activate DMA */
	 HAL_UART_Receive_DMA(&huart2, ref_rx_frame, sizeof(ref_rx_frame));
  } else if (huart == &huart1 && board_status == GIMBAL_BOARD) {
		UC_Response_Pack_t uc_response_pack;
		uc_response_pack_init(&uc_response_pack);

		uint32_t data_checksum = calculate_checksum(uc_input_buffer, UC_RECV_PACK_SIZE);
		uint32_t sent_checksum = *((uint32_t*) uc_input_buffer + 3);
		if (data_checksum == sent_checksum) {
			uc_response_pack.response_code = 0;
			memcpy(&uc_rx_pack, uc_input_buffer, UC_RECV_PACK_SIZE);
		} else {
			uc_response_pack.response_code = 1;
		}
		uc_response_pack.checksum = calculate_checksum(&uc_response_pack, UC_RESPONSE_PACK_SIZE);

		uc_send_packet(&uc_response_pack, UC_RESPONSE_PACK_SIZE);
		uc_receive_packet();
	}
}
#endif
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

