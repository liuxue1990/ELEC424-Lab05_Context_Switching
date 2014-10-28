
/**
  ******************************************************************************
  * @file    sys_clk_init.c
  * @author  Xue Liu
  * @version V0.1
  * @date    10-15-2014
  * @brief   Initialize the system clock by using external clock
  ******************************************************************************
  */


/**
  ******************************************************************************
  * Include Files
  ******************************************************************************
  */
  
#include "sys_clk_init.h"
/**
  ******************************************************************************
  * Private Definations
  ******************************************************************************
  */

#define MOTOR_SPEED 20
#define VECT_TAB_OFFSET  0x0
/**
 * Definations of LEDS
 */
#define LEDn 2

#define LED_RED_PIN GPIO_Pin_4
#define LED_RED_GPIO_PORT GPIOB
#define LED_RED_GPIO_CLK RCC_APB2Periph_GPIOB

#define LED_GREEN_PIN GPIO_Pin_5
#define LED_GREEN_GPIO_PORT GPIOB
#define LED_GREEN_GPIO_CLK RCC_APB2Periph_GPIOB
  
typedef enum { LED_RED = 0, LED_GREEN = 1 } Led_TypeDef;
/**
  ******************************************************************************
  * Private Functions Declaration
  ******************************************************************************
  */
static void SetSysClockTo72_User(void);
void Motor_RCC_Configuration(void);
void Motor_GPIO_Configuration(void);
void Motor_TIM_Configuration(TIM_TypeDef *TIM_GROUP, uint16_t CCR1_Pre,
                             uint16_t CCR2_Pre);
void LED_Init(Led_TypeDef LED_NUM);
/**
*
******************************************************************************
* Private Global Variables
******************************************************************************
*/

GPIO_TypeDef *LED_PORT[LEDn] = {
    LED_RED_GPIO_PORT, LED_GREEN_GPIO_PORT,
};
const uint16_t LED_PIN[LEDn] = {LED_RED_PIN, LED_GREEN_PIN};
const uint32_t LED_CLK[LEDn] = {LED_RED_GPIO_CLK, LED_GREEN_GPIO_CLK};

/**
  ******************************************************************************
  * Public Functions
  ******************************************************************************
  */

/**
 * @brief init_system_clk
 * @details Initalize the RCC clock system
 */
void init_system_clk(void)
{
    /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
  RCC->CFGR &= (uint32_t)0xF0FF0000;
  
  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  RCC->CFGR &= (uint32_t)0xFF80FFFF;

  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x009F0000;
    
  /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
  /* Configure the Flash Latency cycles and enable prefetch buffer */
  SetSysClockTo72_User();
  /* Vector Table Relocation in Internal FLASH. */
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; 
}

/**
 * @brief init blink 
 * @details Initializes pin PB5 which is connected to the green LED. 
 * Port mode is push-pull output.
 */
void init_blink(void){
  LED_Init(LED_GREEN);
}

/**
 * @brief init motors
 * @details Initializes pins connected to motors and configures respective 
 * timers to output PWM signals.
 */
void init_motors(void){
  Motor_RCC_Configuration(); 
  Motor_GPIO_Configuration();
  // Motor_TIM_Configuration(TIM3, MOTOR_SPEED, MOTOR_SPEED);
  // Motor_TIM_Configuration(TIM4, MOTOR_SPEED, MOTOR_SPEED);
}

/**
  ******************************************************************************
  * Private Functions
  ******************************************************************************
  */

/**
  * @brief  Sets System clock frequency to 72MHz and configure HCLK, PCLK2 
  *         and PCLK1 prescalers. 
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
static void SetSysClockTo72_User(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/    
  /* Enable HSE */    
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* Flash 2 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;    

 
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
      
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    
    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

    /*  PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
                                        RCC_CFGR_PLLMULL));
    // RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLXTPRE);

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }
    
    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock 
         configuration. User can add here some code to deal with this error */
  }
}

/**
 * @brief Motor_RCC_Configuration
 * @details Enable the Clocks of the Four Motors
 */
void Motor_RCC_Configuration(void) {
  /* Tim clocks of motors enable */
  RCC_APB1PeriphClockCmd(
      RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

  /* GPIOB clock for motors and LEDs enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
}

/**
 * @brief Motor GPIO Configuration
 * @details Enable the gpio channel of the four motors
 */
void Motor_GPIO_Configuration(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA Configuration:TIM3 Channel1, 2, 3 and 4 as alternate function
   * push-pull */
  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief Motor TIM Configuration
 * @details configure the TIM3 and TIM 4 for 4 motors
 *
 * @param TIM_GROUP the TIM Group for the motors
 * @param CCR1_Pre Motor 2(TIM3) or 4 (TIM4)
 * @param CCR2_Pre Motor 1(TIM3) or 3 (TIM4)
 */
void Motor_TIM_Configuration(TIM_TypeDef *TIM_GROUP, uint16_t CCR1_Pre,
                             uint16_t CCR2_Pre) {
  /* -----------------------------------------------------------------------
  TIM Configuration: generate 4 PWM signals with 4 different duty cycles:
  The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM counter
  clock at 24 MHz the Prescaler is computed as following:
   - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
  SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
  and Connectivity line devices and to 24 MHz for Low-Density Value line and
  Medium-Density Value line devices

  The TIMx is running at 36 KHz: TIMx Frequency = TIMx counter clock/(ARR + 1)
                                                = 24 MHz / 666 = 36 KHz
  TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
  TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
  TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
  TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
----------------------------------------------------------------------- */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  uint16_t PrescalerValue = 0;
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t)(SystemCoreClock / 24000000) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 665;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM_GROUP, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse =
      TIM_TimeBaseStructure.TIM_Period / 100 * CCR1_Pre;

  TIM_OC3Init(TIM_GROUP, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM_GROUP, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse =
      TIM_TimeBaseStructure.TIM_Period / 100 * CCR2_Pre;

  TIM_OC4Init(TIM_GROUP, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM_GROUP, TIM_OCPreload_Enable);

  /*TIM enable Group*/
  TIM_ARRPreloadConfig(TIM_GROUP, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM_GROUP, ENABLE);
}
/**
 * @brief Led init
 * @details take the led num and initialize it
 * 
 * @param LED_NUM the led num need to be initialized
 */
void LED_Init(Led_TypeDef LED_NUM) {
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(LED_CLK[LED_NUM], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LED_PIN[LED_NUM];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(LED_PORT[LED_NUM], &GPIO_InitStructure);
}

/**
  ******************************************************************************
  * End of the file
  ******************************************************************************
  */