
/**
  ******************************************************************************
  * @file    systick_context_switcher.c
  * @author  Xue Liu
  * @version V0.1
  * @date    10-25-2014
  * @brief   
  ******************************************************************************
  */

/**
  ******************************************************************************
  * Include files
  ******************************************************************************
  */
#include "tasks.h"
/**
  ******************************************************************************
  * Private function declaration
  ******************************************************************************
  */


/**
 *
  ******************************************************************************
  * Global Variables
  ******************************************************************************
  */

/**
 *
  ******************************************************************************
  * Public functions
  ******************************************************************************
  */

/**
 * @brief main
 * @details Main function
 */
void main(void) {
  init_system_clk();
  init_blink();
  init_motors();
  // task_spin_motors();
  task_blink_led();
  while (1) {
    
  }
}
/**
  ******************************************************************************
  * Private functions
  ******************************************************************************
  */

/**
  ******************************************************************************
  * Exception handlers
  ******************************************************************************
  */

/**
 * @brief SysTick_Handler
 * @details the systick handler
 */
void SysTick_Handler(void){

}

/**
 * @brief PendSV_handler
 * @details The PendSV handler
 */
void PendSV_Handler(void){

}
/**
  ******************************************************************************
  * End of the file
  ******************************************************************************
  */
