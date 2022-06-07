/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define TRANSMITTER // Either set the board as TRANSMITTER, RECEIVER or DUMMY

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define DC_MOTOR1   0
/* USER CODE BEGIN PD */
// App Start Task
#define APP_TASK_START_STK_SIZE 128u
#define APP_TASK_START_PRIO 1u
static OS_TCB AppTaskStartTCB;
static CPU_STK AppTaskStartStk[APP_TASK_START_STK_SIZE];
static void AppTaskStart(void *p_arg);

#ifdef RECEIVER

#define RECEIVE_TASK_STK_SIZE 128u
#define RECEIVE_TASK_PRIO 3u
static OS_TCB ReceiveTaskTCB;
static CPU_STK ReceiveTaskStk[RECEIVE_TASK_STK_SIZE];
static void ReceiveTask(void *p_arg);

#endif

#ifdef TRANSMITTER

// Moisture Detection Task
#define MOISTURE_DETECTION_STK_SIZE 128u
#define MOISTURE_DETECTION_PRIO 2u
static OS_TCB MoistureDetectionTCB;
static CPU_STK MoistureDetectionStk[MOISTURE_DETECTION_STK_SIZE];
static void MoisturePercentageTask(void *p_arg);
int H2O_real;
int H2O_dec;
// Light Detection Task
#define LIGHT_DETECTION_TASK_STK_SIZE 128u
#define LIGHT_DETECTION_TASK_PRIO 2u
static OS_TCB LightDetectionTaskTCB;
static CPU_STK LightDetectionTaskStk[LIGHT_DETECTION_TASK_STK_SIZE];
static void LightMeasureTask(void *p_arg);
int luminousity;
// Air pressure + temperature detection
#define DUO_SENSOR_TASK_STK_SIZE 256u
#define DUO_SENSOR_TASK_PRIO 2u
static OS_TCB DuoSensorTaskTCB;
static CPU_STK DuoSensorTaskStk[DUO_SENSOR_TASK_STK_SIZE];
static void DuoSensorTask(void* p_arg);
int temperature;
int pressure;
// LCD Display for displaying results
#define LCD_TASK_STK_SIZE 128u
#define LCD_TASK_PRIO 1u
static OS_TCB LCDTaskTCB;
static CPU_STK LCDTaskStk[LCD_TASK_STK_SIZE];
static void LCDTask(void* p_arg);
// Pump for dispensing water
#define Pump_TASK_STK_SIZE 128u
#define Pump_TASK_PRIO 1u
static OS_TCB PumpTaskTCB;
static CPU_STK PumpTaskStk[Pump_TASK_STK_SIZE];
static void PumpTask(void *p_arg);

#endif

#ifdef DUMMY

// Sending useless data to master
#define DUMMY_TASK_STK_SIZE 128u
#define DUMMY_TASK_PRIO 1u
static OS_TCB DummyTaskTCB;
static CPU_STK DummyTaskStk[DUMMY_TASK_STK_SIZE];
static void DummyTask(void* p_arg);

#endif

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Buffer Size
#define BUFFER_SIZE 100

// Ring Buffer
struct ring_buffer rb;
// Semaphore
OS_SEM sem;
OS_SEM blinking;
OS_SEM busyADC;
OS_SEM busyUSART;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/////// NON-TASK FUNCTIONS ////////
void read_USART(OS_ERR* os_err, char* MSG, UART_HandleTypeDef* src);
void send_USART(OS_ERR* os_err, char* MSG, int size, UART_HandleTypeDef* src);
int read_ADC(OS_ERR* os_err, void(*f)(void));
void blink_LD2(OS_ERR* os_err);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int main(void)
{
  /* To store error code */
  OS_ERR os_err;

  /* Initialize uC/OS-III */
  OSInit(&os_err);

  if (os_err != OS_ERR_NONE)
  {
    while (DEF_TRUE)
      ;
  }

  OSSemCreate(
      (OS_SEM *)&blinking,
      (CPU_CHAR *)"Semaphore",
      (OS_SEM_CTR)0,
      (OS_ERR *)&os_err);
  
  OSSemCreate(
      (OS_SEM *)&busyADC,
      (CPU_CHAR *)"Semaphore",
      (OS_SEM_CTR)0,
      (OS_ERR *)&os_err);
  
  OSSemCreate(
      (OS_SEM *)&busyUSART,
      (CPU_CHAR *)"Semaphore",
      (OS_SEM_CTR)0,
      (OS_ERR *)&os_err);

  OSTaskCreate(
      /* pointer to task control block */
      (OS_TCB *)&AppTaskStartTCB,
      /* task name can be displayed by debuggers */
      (CPU_CHAR *)"App Task Start",
      /* pointer to the task */
      (OS_TASK_PTR)AppTaskStart,
      /* pointer to an OPTIONAL data area */
      (void *)0,
      /* task priority: the lower the number, the higher the priority */
      (OS_PRIO)APP_TASK_START_PRIO,
      /* pointer to task's stack base addr */
      (CPU_STK *)&AppTaskStartStk[0],
      /* task's stack limit to monitor and ensure that the stack 
       * doesn't overflow (10%) */
      (CPU_STK_SIZE)APP_TASK_START_STK_SIZE / 10,
      /* task's stack size */
      (CPU_STK_SIZE)APP_TASK_START_STK_SIZE,
      /* max number of message that the task can receive through 
       * internal message queue (5) */
      (OS_MSG_QTY)5u,
      /* amount of clock ticks for the time quanta 
       * when round robin is enabled */
      (OS_TICK)0u,
      /* pointer to an OPTIONAL user-supplied memory location 
       * use as a TCB extension */
      (void *)0,
      /* contain task-specific option 
       * OS_OPT_TASK_STK_CHK: allow stack checking 
       * OS_OPT_TASK_STK_CLR: stack needs to be cleared */
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      /* pointer to a variable that will receive an error code */
      (OS_ERR *)&os_err);

  if (os_err != OS_ERR_NONE)
  {
    while (DEF_TRUE)
      ;
  }

  /* Start Mulitasking */
  OSStart(&os_err);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void AppTaskStart(void *p_arg)
{
  OS_ERR os_err;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  LCD_Init();
  DC_MOTOR_Init(DC_MOTOR1);

#ifdef RECEIVER

  // Task meant for receiving messages through USART3
  OSTaskCreate(
    (OS_TCB *)&ReceiveTaskTCB,
    (CPU_CHAR *)"Receive Task",
    (OS_TASK_PTR)ReceiveTask,
    (void *)0,
    (OS_PRIO)RECEIVE_TASK_PRIO,
    (CPU_STK *)&ReceiveTaskStk[0],
    (CPU_STK_SIZE)RECEIVE_TASK_STK_SIZE / 10,
    (CPU_STK_SIZE)RECEIVE_TASK_STK_SIZE,
    (OS_MSG_QTY)5u,
    (OS_TICK)0u,
    (void *)0,
    (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
    (OS_ERR *)&os_err);
    
#endif
#ifdef TRANSMITTER

  // Task meant for reading the analog input from the moisture sensor, and turning this into a useful value
  OSTaskCreate(
    (OS_TCB *)&MoistureDetectionTCB,
    (CPU_CHAR *)"Moisture Detection",
    (OS_TASK_PTR)MoisturePercentageTask,
    (void *)0,
    (OS_PRIO)MOISTURE_DETECTION_PRIO,
    (CPU_STK *)&MoistureDetectionStk[0],
    (CPU_STK_SIZE)MOISTURE_DETECTION_STK_SIZE / 10,
    (CPU_STK_SIZE)MOISTURE_DETECTION_STK_SIZE,
    (OS_MSG_QTY)5u,
    (OS_TICK)0u,
    (void *)0,
    (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
    (OS_ERR *)&os_err);

  // Task meant for reading the analog input from the LDR, and turning this into a useful value
  OSTaskCreate(
    (OS_TCB *)&LightDetectionTaskTCB,
    (CPU_CHAR *)"Light Detection",
    (OS_TASK_PTR)LightMeasureTask,
    (void *)0,
    (OS_PRIO)LIGHT_DETECTION_TASK_PRIO,
    (CPU_STK *)&LightDetectionTaskStk[0],
    (CPU_STK_SIZE)LIGHT_DETECTION_TASK_STK_SIZE / 10,
    (CPU_STK_SIZE)LIGHT_DETECTION_TASK_STK_SIZE,
    (OS_MSG_QTY)5u,
    (OS_TICK)0u,
    (void *)0,
    (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
    (OS_ERR *)&os_err);
  
  // Task meant for reading the data from a BMP280 sensor
  OSTaskCreate(
    (OS_TCB *)&DuoSensorTaskTCB,
    (CPU_CHAR *)"Temp + Air Pressure Detection",
    (OS_TASK_PTR)DuoSensorTask,
    (void *)0,
    (OS_PRIO)DUO_SENSOR_TASK_PRIO,
    (CPU_STK *)&DuoSensorTaskStk[0],
    (CPU_STK_SIZE)DUO_SENSOR_TASK_STK_SIZE / 10,
    (CPU_STK_SIZE)DUO_SENSOR_TASK_STK_SIZE,
    (OS_MSG_QTY)5u,
    (OS_TICK)0u,
    (void *)0,
    (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
    (OS_ERR *)&os_err);

  // Task meant for reading the data from a BMP280 sensor
  OSTaskCreate(
    (OS_TCB *)&LCDTaskTCB,
    (CPU_CHAR *)"LCD Display",
    (OS_TASK_PTR)LCDTask,
    (void *)0,
    (OS_PRIO)LCD_TASK_PRIO,
    (CPU_STK *)&LCDTaskStk[0],
    (CPU_STK_SIZE)LCD_TASK_STK_SIZE / 10,
    (CPU_STK_SIZE)LCD_TASK_STK_SIZE,
    (OS_MSG_QTY)5u,
    (OS_TICK)0u,
    (void *)0,
    (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
    (OS_ERR *)&os_err);

  // Task meant for dispensing water
  OSTaskCreate(
    (OS_TCB *)&PumpTaskTCB,
    (CPU_CHAR *)"Pump Task",
    (OS_TASK_PTR)PumpTask,
    (void *)0,
    (OS_PRIO)Pump_TASK_PRIO,
    (CPU_STK *)&PumpTaskStk[0],
    (CPU_STK_SIZE)Pump_TASK_STK_SIZE / 10,
    (CPU_STK_SIZE)Pump_TASK_STK_SIZE,
    (OS_MSG_QTY)5u,
    (OS_TICK)0u,
    (void *)0,
    (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
    (OS_ERR *)&os_err);

#endif

#ifdef DUMMY

  // Task meant for dispensing water
  OSTaskCreate(
    (OS_TCB *)&DummyTaskTCB,
    (CPU_CHAR *)"Dummy Task",
    (OS_TASK_PTR)DummyTask,
    (void *)0,
    (OS_PRIO)DUMMY_TASK_PRIO,
    (CPU_STK *)&DummyTaskStk[0],
    (CPU_STK_SIZE)DUMMY_TASK_STK_SIZE / 10,
    (CPU_STK_SIZE)DUMMY_TASK_STK_SIZE,
    (OS_MSG_QTY)5u,
    (OS_TICK)0u,
    (void *)0,
    (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
    (OS_ERR *)&os_err);

#endif

  OSSemPost(
    (OS_SEM*)&blinking,
    (OS_OPT)OS_OPT_POST_1,
    (OS_ERR*)&os_err
  );
  OSSemPost(
    (OS_SEM*)&busyUSART,
    (OS_OPT)OS_OPT_POST_1,
    (OS_ERR*)&os_err
  );
  OSSemPost(
    (OS_SEM*)&busyADC,
    (OS_OPT)OS_OPT_POST_1,
    (OS_ERR*)&os_err
  );
}

#ifdef RECEIVER

static void ReceiveTask(void *p_arg){
  /*
  Task that receives data through USART3, cleans it, and then sends it through USART2
  Cleaning data means stripping all of the NULL-values from a char array

  USART3-RX is connected to PB11
  USART3-TX is connected to D6
  USART2 is connected to a PC using the USB cable
  */
  OS_ERR os_err;
  char MSG[BUFFER_SIZE];

  while(DEF_TRUE){
    // Read data from USART3, and store it in MSG
    read_USART(&os_err, MSG, &huart3);
    
    // Send MSG through USART2
    send_USART(&os_err, MSG, strlen(MSG), &huart2);
  }
}

#endif
#ifdef TRANSMITTER

static void LightMeasureTask(void *p_arg){
  /*
  Task that reads the analog input from the connected LDR, calculates the amount of lux that this voltage
   represents, and then sends it through USART3.
  
  ADC6 is PA6
  USART3-RX is PB11
  USART3-TX is D6
  When connecting 2 Nucleos using USART3, the RX of one Nucleo should connect to the TX of the other,
   as such, the TX of the first Nucleo should also connect to the RX of the other Nucleo.
  */
  OS_ERR os_err;
  int analog_in;
  char MSG[BUFFER_SIZE];
  double lux;

  while(DEF_TRUE){
    // Read the analog data from PA6
    analog_in = read_ADC(&os_err, MX_ADC6_Init);
    
    // Convert the read data into lux, and store an appropriate message in MSG
    lux = 3.0 * 1000000.0 * pow(((double)analog_in), -1.367);
    sprintf(MSG, "Lux: %d\n\r", (int)(round(lux)));

    luminousity = (int)(round(lux));

    // Send MSG through USART3
    send_USART(&os_err, MSG, BUFFER_SIZE, &huart3);
    
    // Wait for 2 seconds before trying to read ADC7 again
    OSTimeDlyHMSM(0, 0, 2, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);
  }
}

static void MoisturePercentageTask(void *p_arg){
  /*
  Task that reads the analog input from the connected moisture sensor, turns this into a percentage
   and then sends it through USART3.
  
  ADC7 is PA7
  USART3-RX is PB11
  USART3-TX is D6
  When connecting 2 Nucleos using USART3, the RX of one Nucleo should connect to the TX of the other,
   as such, the TX of the first Nucleo should also connect to the RX of the other Nucleo.
  */
  OS_ERR os_err;
  char MSG[BUFFER_SIZE];
  int minimum = 1200; // Minimal possible voltage, this means that the sensor is currently dipped in water
  int maximum = 2900; // Maximal possible voltage, this means that the sensor is currently measuring the air
  double max_value = maximum - minimum; // Maximal possible delta-Voltage

  while(DEF_TRUE){
    double delta_voltage;
    int voltage;

    // Read the analog data from the input, and turn it into the voltage that this value represents
    voltage = ((float)read_ADC(&os_err, MX_ADC7_Init)) / 4096 * 3300;

    // Calculate the moisture percentage
    // 100% is the moisture-level (conductivity) of water
    // 0% is the moisture-level (conductivity) of air
    delta_voltage = voltage - minimum;
    double percentage = ((double)(max_value - delta_voltage) / max_value * 100.00);
    H2O_real = (int)percentage;
    H2O_dec = (int)((percentage - (double)(int)percentage) * 10);

    // Make sure that percentages lower than 0, or higher than a 100, don't make it through
    // Then store the generated message in MSG
    if(voltage < minimum){
      sprintf(MSG, "MoisturePercent:100.0\r\n");
      H2O_real = 99;
      H2O_dec = 9;
    } else if(voltage > maximum){
      sprintf(MSG, "MoisturePercent:0.0\r\n");
      H2O_real = 0;
      H2O_dec = 0;
    } else {
      //float H2O_real_rasp = H2O_real/10; 
      sprintf(MSG, "MoisturePercent:%d.%d\r\n", H2O_real,H2O_dec);
    }

    // Send the MSG through USART3
    send_USART(&os_err, MSG, BUFFER_SIZE, &huart3);

    // Wait for 2 seconds before trying to read ADC7 again
    OSTimeDlyHMSM(0, 0, 2, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);
  }
}

static void DuoSensorTask(void* p_arg){
  /*
  Task that reads data from the BMP280 sensor, this data consists of both the temperature and pressure
   in the area that the sensor finds itself.
  
  The sensor is connected to multiple pins, from sensor to Nucleo these are as follows:
   VCC to 3,3V
   GND and SDO to ground
   SCL to PB6
   SDA to PB7
  */
  OS_ERR os_err;
  BMP280_Setup();
  char MSG[BUFFER_SIZE];

  while(DEF_TRUE){
    BMP280_Read(&temperature, &pressure);
    sprintf(MSG, "\rTemperature = %d Celsius | Pressure = %d Pa\n\r", temperature, pressure);
    send_USART(&os_err, MSG, BUFFER_SIZE, &huart3);

    OSTimeDlyHMSM(0, 0, 2, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);
  }
}

static void LCDTask(void* p_arg){
  /*
  Task that displays data using a LCD

  The LCD is connected using multiple pins, from LCD to Nucleo these are as follows:
   K to ground
   A to 5V
   D7 to D14
   D6 to D15
   D5 to D4
   D4 to D5
   E to PB2
   RW to ground
   RS to PB1
   V0 to the output of a variable resistor, which on its turn is also connected to 5V and the ground
   VDD to 5V
   VSS to ground
  */
  OS_ERR os_err;
  char MSG[8];

  while(DEF_TRUE){
    // Print the measurements of each sensor on the LCD
    LCD_Clear();
    LCD_Set_Cursor(1, 1);
    sprintf(MSG, "Lux:%d", luminousity);
    LCD_Write_String(MSG);

    LCD_Set_Cursor(1, 9);
    sprintf(MSG, "H2O:%d.%d", H2O_real, H2O_dec);
    LCD_Write_String(MSG);

    LCD_Set_Cursor(2, 1);
    sprintf(MSG, "Pa:%d", pressure);
    LCD_Write_String(MSG);

    LCD_Set_Cursor(2, 11);
    sprintf(MSG, "C:%d", temperature);
    LCD_Write_String(MSG);

    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);
  }
}

static void PumpTask(void *p_arg){
  /*
  Function that checks the moisture levels of the soil, once in every while. When the moisture levels are too low,
   the pump activates, and pump water for a short amount of time.
  
  The pump-constrols use multiple pins, from pump to Nucleo these are as follows:
   PWM to PA4
   cw direction to PB12
  */
  uint16_t AD_RES = 0;
  OS_ERR os_err;

  while (DEF_TRUE){
    if(H2O_real < 50){
      DC_MOTOR_Set_Speed(DC_MOTOR1, AD_RES>>2);
      OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);

      DC_MOTOR_Start(DC_MOTOR1, DIR_CW, 100);
      OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);
      DC_MOTOR_Stop(DC_MOTOR1);
    }
    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);
  }
}

#endif

#ifdef DUMMY

static void DummyTask(void* p_arg){
  /*
  Dummy task used to prove that master can communicate with multiple slaves at once
  
  SCL to PB6
  SDA to PB7
  */
  OS_ERR os_err;
  char MSG[BUFFER_SIZE];

  while(DEF_TRUE){
    sprintf(MSG, "testing\n\r");
    send_USART(&os_err, MSG, BUFFER_SIZE, &huart3);
    OSTimeDlyHMSM(0, 0, 10, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);
  }
}

#endif

////////////////////////////////////NON-TASK FUNCTIONS/////////////////////////////////////////////
void send_USART(OS_ERR* os_err, char* MSG, int size, UART_HandleTypeDef* src){
  /*
  Function that sends the given message through an USART connection.

  os_err: A variable in which any possible error message will be stored
  MSG: The message that needs to be sent
  size: The size of the message that needs to be sent; when using USART3, this is standard 100
  src: A reference to the USART channel that is to be used for sending data
  
  USART3-RX is PB11
  USART3-TX is D6
  When connecting 2 Nucleos using USART3, the RX of one Nucleo should connect to the TX of the other,
   as such, the TX of the first Nucleo should also connect to the RX of the other Nucleo.
  USART2 connects to a PC using the USB cable  
  */
  // Lock down USART, so that different messages won't interrupt each other
  OSSemPend(
    (OS_SEM*)&busyUSART,
    (OS_TICK)0,
    (OS_OPT)OS_OPT_PEND_BLOCKING,
    (CPU_TS*)NULL,
    (OS_ERR*)&os_err
  );

  // Send data to selected USART-channel
  HAL_UART_Transmit(src, (uint8_t*)MSG, BUFFER_SIZE, HAL_MAX_DELAY);
  memset(MSG, 0, BUFFER_SIZE);

  // Release the lockdown of USART
  OSSemPost(
    (OS_SEM*)&busyUSART,
    (OS_OPT)OS_OPT_POST_1,
    (OS_ERR*)&os_err
  );
}

void read_USART(OS_ERR* os_err, char* MSG, UART_HandleTypeDef* src){
  /*
  Function that receives data from the specified USART-channel, and stores this in a message

  os_err: A variable in which any possible error message will be stored
  MSG: The variable in which the read data is stored
  src: A pointer to the desired USART-channel
  */
 // Lock down USART, so that different messages won't interrupt each other
  OSSemPend(
    (OS_SEM*)&busyUSART,
    (OS_TICK)0,
    (OS_OPT)OS_OPT_PEND_BLOCKING,
    (CPU_TS*)NULL,
    (OS_ERR*)&os_err
  );

  // Receive data from the specified source/UART-channel
  HAL_UART_Receive(src, (uint8_t*)MSG, BUFFER_SIZE, HAL_MAX_DELAY);

  // Lift the lockdown
  OSSemPost(
    (OS_SEM*)&busyUSART,
    (OS_OPT)OS_OPT_POST_1,
    (OS_ERR*)&os_err
  );
}

int read_ADC(OS_ERR* os_err, void(*MX_ADC_INIT)(void)){
  /*
  Function that reads the analog input from the ADC, the channel that's being read from will vary depening
   on the MX_ADC_Init() that's used beforehand.

  os_err: A variable in which any possible error message will be stored
  MX_ADC_INIT: The function that is used to init the desired ADC-channel but without the ();
   for example: MX_ADC6_Init, and not MX_ADC6_Init()

  return: An integer that contains the read value
  */
  int temp;
  // Lock down the ADC, so that different function-calls won't read from the incorrect ADC-channel
  OSSemPend(
    (OS_SEM*)&busyADC,
    (OS_TICK)0,
    (OS_OPT)OS_OPT_PEND_BLOCKING,
    (CPU_TS*)NULL,
    (OS_ERR*)&os_err
  );

  // Init the ADC using the given MX_ADC_Init function
  (*MX_ADC_INIT)();

  // Start the ADC, read the data from the ADC, and stop the ADC
  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
  temp = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);

  // Lift the lockdown
  OSSemPost(
    (OS_SEM*)&busyADC,
    (OS_OPT)OS_OPT_POST_1,
    (OS_ERR*)&os_err
  );
  return temp;
}

void blink_LD2(OS_ERR* os_err){
  /*
  Function that blinks LD2, the built-in LED on the Nucleo, exactly once. Turning on for 1 second, and then 
   turning off for 1 second.
  
  os_err: A variable in which any possible error message will be stored
  */
  // Lock down the blinking, so multiple blinks won't execute during the same period
  OSSemPend(
    (OS_SEM*)&blinking,
    (OS_TICK)0,
    (OS_OPT)OS_OPT_PEND_BLOCKING,
    (CPU_TS*)NULL,
    (OS_ERR*)&os_err
  );

  // Toggle LED, wait 1 second, toggle LED and wait 1 second again
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, os_err);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, os_err);

  // Lift the lockdown
  OSSemPost(
    (OS_SEM*)&blinking,
    (OS_OPT)OS_OPT_POST_1,
    (OS_ERR*)&os_err
  );
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