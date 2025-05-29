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
#include "can.h"
#include "cmsis_os.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "can_comm.h"
#include "can_isr.h"
#include "debug.h"
#include "dwt.h"
#include "event_center.h"
#include "imu.h"
#include "message_center.h"
#include "motors.h"
#include "stdio.h"
#include "stm32f407xx.h"
#include "uart_isr.h"

#include "Chassis_App.h"
#include "Comm_App.h"
#include "Control_App.h"
#include "Gimbal_App.h"
#include "IMU_App.h"
#include "Omni_Drive.h"
#include "PC_UART_App.h"
#include "Referee_App.h"
#include "Shoot_App.h"
#include "Swerve_Drive.h"
#include "Timer_App.h"
#include "WatchDog_App.h"
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
void StartDefaultTask(void const* argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t referee_parsed_flag = 0;
uint8_t referee_timeout_counter = 0;
uint8_t referee_timeout_check_flag = 0;
uint32_t prev_uart_timestamp = 0;

/* new defined variables*/
uint32_t debugger_signal_counter = 0;  //count the idle time
uint32_t debugger_signal_flag = 0;     //mark the debugger task
uint8_t ref_rx_frame[256] = {0};       //referee temp frame buffer
//uint8_t ref_rx_frame[MAX_REF_BUFFER_SZIE]={0}; //referee temp frame buffer
//uint8_t rc_rx_buffer[DBUS_BUFFER_LEN]; //rc temporary buffer
uint16_t chassis_gyro_counter = 0;  // used for backup robots without slipring
uint8_t chassis_gyro_flag = 0;      // used for backup robots without slipring

static MessageCenter& message_center = MessageCenter::get_instance();
static EventCenter event_center;
static Debug debug;
static CanComm can_comm;
static Motors motors;
static Imu imu;

#ifdef SWERVE_CHASSIS
static SwerveDrive swerve_drive(message_center, 0);
static ChassisApp<SwerveDrive> chassis_app(swerve_drive, message_center, debug);
#else

#ifdef OMNI_CHASSIS
static constexpr float omni_chassis_width = 0.40f;
static OmniDrive omni_drive(message_center, omni_chassis_width,
                            omni_chassis_width);
#else
static constexpr float mecanum_chassis_width = 0.41f;
static constexpr float mecanum_chassis_length = 0.35f;
static OmniDrive omni_drive(message_center, mecanum_chassis_width,
                            mecanum_chassis_length);
#endif

static ChassisApp<OmniDrive> chassis_app(omni_drive, message_center, debug);
#endif

static CommApp comm_app(message_center, debug, can_comm);
static TimerApp timer_app(motors, message_center, debug);
static PCUARTApp pc_uart_app(message_center);
static IMUApp imu_app(message_center, event_center, imu, debug);
static GimbalApp gimbal_app(message_center, event_center, debug);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

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
    message_center.init();
    can_comm.init();
    event_center.init();
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
    MX_SPI1_Init();
    MX_I2C3_Init();
    // MX_IWDG_Init();
    MX_TIM5_Init();
    MX_USART6_UART_Init();
    MX_TIM1_Init();
    MX_I2C2_Init();
    /* USER CODE BEGIN 2 */
#ifdef USE_IWDG
    /*To deactivate IWDG, go to main.h and comment #define USE_IWDG 1 */
    MX_IWDG_Init();  //enable IWDG, period 2s
#endif
    HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin,
                      GPIO_PIN_RESET);  // turn off the green led
    if (firmware_and_system_init() != HAL_OK) {
        Error_Handler();
    } else {
        HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin,
                          GPIO_PIN_SET);  // turn on the green led
    }

    BoardStatus_t board_status = debug.get_board_status();

    /* Call init function for freertos objects (in cmsis_os2.c) */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    osThreadCreate(osThread(defaultTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    osThreadDef(
        TimerTask, [](const void* arg) { timer_app.run(arg); }, osPriorityHigh,
        0, 256);
    osThreadCreate(osThread(TimerTask), NULL);

    osThreadDef(
        CommTask, [](const void* arg) { comm_app.run(arg); }, osPriorityHigh, 0,
        256);
    osThreadCreate(osThread(CommTask), NULL);

    //    osThreadDef(WDGTask, WatchDog_Task_Function, osPriorityHigh, 0, 256);
    //    WDGTaskHandle = osThreadCreate(osThread(WDGTask), (void*) board_status);

    if (board_status == CHASSIS_BOARD) {
        osThreadDef(
            ChassisTask, [](const void* arg) { chassis_app.run(arg); },
			osPriorityHigh, 0, 256);
        osThreadCreate(osThread(ChassisTask), NULL);

        osThreadDef(RCTask, RC_Task_Func, osPriorityHigh, 0, 384);
        osThreadCreate(osThread(RCTask), NULL);

        //    	  osThreadDef(RefTask, Referee_Task_Func, osPriorityHigh, 0, 384);
        //    	  RefTaskHandle = osThreadCreate(osThread(RefTask), NULL);

    } else if (board_status == GIMBAL_BOARD) {
        osThreadDef(
            GimbalTask, [](const void* arg) { gimbal_app.run(arg); },
			osPriorityHigh, 0, 512);
        osThreadCreate(osThread(GimbalTask), NULL);

        // osThreadDef(ShootTask, Shoot_Task_Func, osPriorityHigh, 0, 256);
        // osThreadCreate(osThread(ShootTask), NULL);

        osThreadDef(
            IMUTask, [](const void* arg) { imu_app.run(arg); }, osPriorityRealtime,
            0, 256);
        osThreadCreate(osThread(IMUTask), NULL);

        osThreadDef(
            PCUARTTask, [](const void* arg) { pc_uart_app.run(arg); },
            osPriorityHigh, 0, 256);
        osThreadCreate(osThread(PCUARTTask), NULL);
    }

    /* USER CODE END 2 */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
  */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef firmware_and_system_init(void) {
    /* CAN1 & CAN2 Init */
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK) {
        return HAL_ERROR;
    }
    /* CAN1 & CAN2 filter Init */
    can_filter_enable(&hcan1);
    can_filter_enable(&hcan2);

    /* Timer 13 IT Init */
    if (HAL_TIM_Base_Start_IT(&htim13) != HAL_OK) {
        return HAL_ERROR;
    }
    /* Heat PWM signal Init */
    if (HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR;
    }
    // referee_init(&referee);
    dwt_init();

    // TODO: Init event_center.

    UART_Config_t config;
    if (debug.get_board_status() == CHASSIS_BOARD) {
        config = CHASSIS;
    } else {
        config = GIMBAL;
    }
    init_uart_isr(config);
    init_can_isr();

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    /* USER CODE BEGIN Callback 0 */
    if (htim->Instance == TIM13) {
        if (debugger_signal_flag == 1)
            ++debugger_signal_counter;
        //		if(shoot_reserve_flag == 1)
        //			++shoot_reserve_counter;
        //		if(shoot_check_flag == 1)
        //			++shoot_check_counter;
        if (chassis_gyro_flag == 1)
            ++chassis_gyro_counter;
    }
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM5) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

void StartDefaultTask(void const* argument) {
    /* USER CODE BEGIN StartDefaultTask */
    (void) argument;
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
