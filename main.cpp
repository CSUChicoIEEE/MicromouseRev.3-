/*******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  *****************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <vector>
#include "stm32l4xx_hal.h"
using namespace std;

/* Private variables ---------------------------------------------------------*/
#define MAP_SIZE 16
#define MOVE_STACK_SIZE 32
#define WALL_THRESHOLD_S 500
#define WALL_THRESHOLD_L 3000
#define ONE_SQUARE 100
#define TURN_INSIDE 10
#define TURN_OUTSIDE 200
#define TURN_AROUND 150
#define NORTH 0x0
#define EAST 0x1
#define SOUTH 0x2
#define WEST 0x3

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;

static volatile uint32_t enCountRight = 0;
static volatile uint32_t enCountLeft = 0;
static bool rightMotorFinish = 0;
static bool leftMotorFinish = 0;
static uint8_t mapXpos;
static uint8_t mapYpos;
static uint8_t direction;
static uint8_t defaultDir;

struct movementVector {
  uint8_t pwmR1;
  uint8_t pwmR2;
  uint8_t pwmL1;
  uint8_t pwmL2;
  uint16_t rightMotorSteps;
  uint16_t leftMotorSteps;
  Movement moveType;
};

struct analogValues {
	static uint16_t rightBackIRVal;
	static uint16_t rightFrontIRVal;
	static uint16_t middleIRVal;
	static uint16_t leftFrontIRVal;
	static uint16_t leftBackIRVal;
};

analogValues analog1;

vector<movementVector> moveStack;

enum Movement {noMove,forward,turnRight,turnLeft,turnAround};

uint8_t MAP [MAP_SIZE][MAP_SIZE] = {};    // bits   X,X,X,DONE,NORTH,EAST,SOUTH,WEST   wall=1   bit 5 if 1 cell has been mapped

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void GPIO_Init(void);
static void ADC1_Init(void);
static void TIM1_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
static void genMoveVector(void);
static void genStartVector(void);
static void genRunVector(void);
	
static void exeMoveVector(void);

static void waitForButton(void);
static void mapCell(void);
static int checkMapComplete(void);
static void analogRead(void);
/***********************************************************************************
**                                   MAIN                                         **
***********************************************************************************/
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  GPIO_Init();
  ADC1_Init();
  TIM1_Init();

	if((GPIOB->IDR&0x80) == 0x80)
	{
		defaultDir = WEST;
	}
	else
	{
		defaultDir = NORTH;
	}
	mapXpos = 0;
	mapYpos = 0;
	direction = defaultDir;
	//TEST();
	
	waitForButton();
	while(1)
	{
		//MAPPING MODE 00
		while((GPIOB->IDR&0xC0) == 0x00) 
		{
			mapCell();
			genMoveVector();
			exeMoveVector();
			while((checkMapComplete()==1)&&((GPIOB->IDR&0xC0) == 0x00))
			{
				if((mapXpos != 0)&&(mapYpos != 0)&&(direction != defaultDir))
				{
					genStartVector();
					exeMoveVector();
				}
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_Delay(500);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
				HAL_Delay(500);
			}
		}
		//SOLVE MODE 01
		while((GPIOB->IDR&0xC0) == 0x40) 
		{
			waitForButton();
			genRunVector();
			exeMoveVector();
		}
	}
}
/***********************************************************************************
**                               MAIN END                                         **
***********************************************************************************/



/***********************************************************************************
Function   :  mapCell()
Description:  reads the ADCs and populates the current cell with wall data
Inputs     :  None
Outputs    :  None

Status     :  Complete, works as intended for the current implementation
***********************************************************************************/
void mapCell(void)
{
	if((MAP[mapXpos][mapYpos]&0x10)==0x10) //if current map position has not been mapped
	{ 
		MAP[mapXpos][mapYpos]|=0x10;
		analogRead();
		switch(direction) 
		{
			case NORTH:
				if(analog1.middleIRVal<=WALL_THRESHOLD_S)
				{
					MAP[mapXpos][mapYpos]|=0x08;
				}
				if(analog1.leftFrontIRVal<=WALL_THRESHOLD_S) 
				{
					MAP[mapXpos][mapYpos]|=0x01;
				}
				if(analog1.rightFrontIRVal<=WALL_THRESHOLD_S) 
				{
					MAP[mapXpos][mapYpos]|=0x04;
				}
				break;
			case WEST:
				if(analog1.middleIRVal<=WALL_THRESHOLD_S) 
				{
					MAP[mapXpos][mapYpos]|=0x01;
				}
				if(analog1.leftFrontIRVal<=WALL_THRESHOLD_S) 
				{
					MAP[mapXpos][mapYpos]|=0x02;
				}
				if(analog1.rightFrontIRVal<=WALL_THRESHOLD_S) 
				{
					MAP[mapXpos][mapYpos]|=0x08;
				}
				break;
			case SOUTH:
				if(analog1.middleIRVal<=WALL_THRESHOLD_S) 
				{
					MAP[mapXpos][mapYpos]|=0x02;
				}
				if(analog1.leftFrontIRVal<=WALL_THRESHOLD_S) 
				{
					MAP[mapXpos][mapYpos]|=0x04;
				}
				if(analog1.rightFrontIRVal<=WALL_THRESHOLD_S) 
				{
					MAP[mapXpos][mapYpos]|=0x01;
				}
				break;
			case EAST:
				if(analog1.middleIRVal<=WALL_THRESHOLD_S) 
				{
					MAP[mapXpos][mapYpos]|=0x04;
				}
				if(analog1.leftFrontIRVal<=WALL_THRESHOLD_S) 
				{
					MAP[mapXpos][mapYpos]|=0x08;
				}
				if(analog1.rightFrontIRVal<=WALL_THRESHOLD_S) 
				{
					MAP[mapXpos][mapYpos]|=0x02;
				}
				break;
		}
	}
}

/***********************************************************************************
Function   :  waitForButton()
Description:  runs loop waiting for the button to be pressed. After the
              button is pressed, it will delay for 3 seconds then return.
Inputs     :  None
Outputs    :  None

Status     :  Complete
***********************************************************************************/
void waitForButton(void)
{
	//loop while button is not pressed 
	while((GPIOA->IDR&0x1000)==0x0000)
	{
		//empty on purpose
	}
	//delay for final adjustments
	HAL_Delay(3000);
}

/***********************************************************************************
Function   :  exeMoveVector()
Description:  runs the motors to move to the next cell
Inputs     :  None
Outputs    :  None

Status     :  Mostly Complete, needs movement control system code
***********************************************************************************/
void exeMoveVector(void)
{
	movementVector currentMove;
	
	//Repeats while there is still movements on the stack to be executed
	while(moveStack.empty() == 0)
	{
		currentMove = moveStack.back();   //gets the next movement to execute
		moveStack.pop_back();             //deletes the movement we loaded off the stack
		rightMotorFinish = 0;             //clears movement complete flags
		leftMotorFinish = 0;
		resetEnCounts();                  //resets the encoder counters 
		setMotorMove(currentMove);        //sets the PWMs for the movement
		setNewPos(currentMove.moveType);  //sets the position of the uM to the destination
		
		//loops until the movement has completed
		while((rightMotorFinish == 0)&&(leftMotorFinish))
		{
			//movement control system goes here
			
			//checks to see if the movement is done
			if(currentMove.rightMotorSteps<=enCountRight)
			{
				rightMotorFinish = 1;
			}
			if(currentMove.leftMotorSteps<=enCountLeft)
			{
				leftMotorFinish = 1;
			}
		}
	}
}

/***********************************************************************************
Function   :  setMotorMove()
Description:  Sets the PWMs up for the next movement 
Inputs     :  move
Outputs    :  None

Status     :  Not Started
***********************************************************************************/
void setMotorMove(movementVector move)
{
	
}

/***********************************************************************************
Function   :  genMoveVector()
Description:  generates the movement steps to get to the next unmapped cell of the maze
Inputs     :  None
Outputs    :  None

Status     :  Not Started
***********************************************************************************/
void genMoveVector(void)
{
	
}

/***********************************************************************************
Function   :  genStartVector()
Description:  generates the movement steps to get to position (0,0)
Inputs     :  None
Outputs    :  None

Status     :  Not Started
***********************************************************************************/
void genStartVector(void)
{
	
}

/***********************************************************************************
Function   :  genRunVector()
Description:  generates the movement steps of the solution to the maze
Inputs     :  None
Outputs    :  None

Status     :  Not Started
***********************************************************************************/
void genRunVector(void)
{
	
}

/***********************************************************************************
Function   :  setNewPos()
Description:  Sets the uMouse's position and direction to the values they will be 
              at the destination of the move
Inputs     :  Movement
Outputs    :  None

Status     :  Complete for current implementation
***********************************************************************************/
void setNewPos(Movement move)
{
	if(move == Forward)
	{
		switch (direction)
		{
			case NORTH:
				mapYpos++;
				break;
			case WEST:
				mapXpos++;
				break;
			case SOUTH:
				mapYpos--;
				break;
			case EAST:
				mapXpos--;
				break;
		}
	}
	if(move == turnRight)
	{
		direction++;
		direction %= 0x04;
	}
	if(move == turnLeft)
	{
		direction--;
		direction %= 0x04;
	}
	if(move == turnAround)
	{
		direction++;
		direction++;
		direction %= 0x04;
	}
}

/***********************************************************************************
Function   :  resetEnCount()
Description:  resets the encoder count
Inputs     :  None
Outputs    :  None

Status     :  Complete
***********************************************************************************/
void resetEnCounts()
{
	enCountRight = 0;
	enCountLeft = 0;
}

/***********************************************************************************
Function   :  SystemClock_Config()
Description:  Configures the Base Clock that all other periferals use
Inputs     :  None
Outputs    :  None

Status     :  Probably works? Auto-generated code
***********************************************************************************/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    /**Configure the main internal regulator output voltage 
    */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/***********************************************************************************
Function   :  ADC1_Init()
Description:  Configures the analog pins and starts the ADC
Inputs     :  None
Outputs    :  None

Status     :  Probably works? Auto-generated code
***********************************************************************************/
static void ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    //Common config
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  HAL_ADC_Init(&hadc1);

    //Configure Regular Channel
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/***********************************************************************************
Function   :  TIM1_init()
Description:  Create a Base Timer that constantly counts the way we want it to.
							We do this by creating a base platform called TimHandle which
							has all the properties we want. Then, we make a empty copy called
							TIM1 which will get all the setup values by tranfering it over with a
							function called TIM_Base_SetConfig().
Inputs     :  None
Outputs    :  None

Status     :  PROTOTYPE
***********************************************************************************/
static void TIM1_Init(void)
{
	uint32_t PWM_Steps = 256;																					// Number of individual PWM steps we can use from 8 bits. 
																																		// This is because 2^8(bits) means 256 possible combinations.
	
	uint32_t PWM_Frequency = 10000;																		// PWM Frequency is set at 10KHz.
	
	uint32_t TIMER_Frequency = 1000;																	// TIM1 Frequency is set at 1Khz.
	
  uint32_t COUNTER_Frequency = PWM_Steps * PWM_Frequency;						// This is the counter frequency that counts backwards 
																																		// so that we can lock onto a signal.
	
  uint32_t PSC_Value = (TIMER_Frequency / COUNTER_Frequency) - 1;		// The PSC_Value is the Prescaler value to setup the timer scaling.
	
  uint16_t ARR_Value = PWM_Steps - 1;																// This represents the frequency for the base timer.

	
	TIM_HandleTypeDef TimHandle;																			// YOU MUST DO THIS LINE FIRST OR YOU CAN NEVER MAKE A TIMER OBJECT.
	TIM_Base_InitTypeDef TimBase;
																																		
	TimHandle.Instance = TIM1;																				// Next, we take the structure.Instnce so we can give it a nickname called TIM1.
																																		// This way we can use the timer just by calling TIM1 later on.
//	TimBase.Prescaler = PSC_Value;
//	TimBase.CounterMode = TIM_COUNTERMODE_DOWN;
//	TimBase.Period = ARR_Value;
//	TimBase.ClockDivision = 0;
	
	TimHandle.Init.Prescaler = PSC_Value;															// Now we setup the base timer using the TimHandle.
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TimHandle.Init.Period = ARR_Value;
	TimHandle.Init.ClockDivision = 0;																	// Do not divide the clock. We want the timer to count naturally up to 256.
//	TimHandle.Init.RepetitionCounter = 0;														// Set this if you are using TIM_COUNTERMODE_DOWN.
//	TimHandle.Init.AutoReloadPreload = 0;

	TIM_Base_SetConfig( TIM1, &TimBase);
	HAL_TIM_Base_Init(&TimHandle);
	
	while (1)
	{
	}
	
}


/***********************************************************************************
Function   :  GPIO_Init()
Description:  Configures GPIO pins for input, output, and external interrupt usage
Inputs     :  None
Outputs    :  None

Status     :  Complete, but may be updated with pin configs from other init functions
***********************************************************************************/
static void GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	

  /* Configure GPIO pin Output Level */
	/* Settings all LED's to LOW */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /* Configure GPIO pins : PA2 PA6 PA7 */
	/* GPIO pins for LED's */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure GPIO pins : PB0 PB1 PB4 PB5 */
	/* GPIO pins for ENCODER */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure GPIO pin : PA12 */
	/* GPIO pins for DIGITAL IN (BUTTON) */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure GPIO pins : PB6 PB7 */
	/* GPIO pins for DIGITAL IN (DIP SWITCHES) */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
	/* Configure GPIO pins : A8 A9 A10 A11 */
	/* GPIO pins for PWM/TIMER1 OUTPUT to (MOTORS) */
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/***********************************************************************************
Function   :  EXTI_Init()
Description:  Sets Priority and enables the external interupts used for the encoders
Inputs     :  None
Outputs    :  None

Status     :  I made this but don't know if it works
***********************************************************************************/
static void EXTI_Init(void)
{
	HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/***********************************************************************************
Function   :  analogRead()
Description:  Gets the analog values from each of the ADC channels and loads the
              analog value struct
Inputs     :  None
Outputs    :  None

Status     :  Not Started
***********************************************************************************/
static void analogRead(void)
{
	
}

/***********************************************************************************
Functions  :  EXTI Handlers
Description:  When an external interrupt occurs, run the code listed
Inputs     :  None
Outputs    :  None

Status     :  Complete with the current implementation
***********************************************************************************/
//Encoder Handler for Right A
void EXTI0_IRQHandler(void)
{
    enCountRight++;
}

//Encoder Handler for Right B
void EXTI1_IRQHandler(void)
{
    enCountRight++;
}

//Encoder Handler for Left A
void EXTI4_IRQHandler(void)
{
    enCountLeft++;
}

//Encoder Handler for Left B
void EXTI5_9_IRQHandler(void)
{
    enCountLeft++;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
