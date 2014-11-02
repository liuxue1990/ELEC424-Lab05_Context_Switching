
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
#include <stddef.h>

  /**
  ******************************************************************************
  * Defination
  ******************************************************************************
  */

#define TASK_STK_SIZE      100
#define OS_MAX_TASK_NUM    2
#define OS_TCB_NOT_IN_USE  ((uint8_t)0x00)
#define OS_TCB_IN_USE      ((uint8_t)0x01)
#define OS_DEFAULT_PSR_VAL ((uint32_t)0x21000000)

/**
 *
  ******************************************************************************
  * Global Variables
  ******************************************************************************
  */
typedef uint32_t OS_STK;

typedef struct os_tcb
{
  OS_STK *OS_TCB_Stk_Ptr;
  // OS_STK *OS_TCB_Stk_Bottom;
  // uint32_t OS_TCB_Stk_Size;
  // struct os_tcb *OS_TCB_Next;
  uint8_t OS_TCB_Stat;
  uint8_t OS_TCB_ID;
} OS_TCB;

typedef struct {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t psr;
} OS_HW_STK;

typedef struct {
  uint32_t r4;
  uint32_t r5;
  uint32_t r6;
  uint32_t r7;
  uint32_t r8;
  uint32_t r9;
  uint32_t r10;
  uint32_t r11;
} OS_SW_STK;

OS_TCB OS_TCB_Tbl[OS_MAX_TASK_NUM];
uint8_t OS_Cur_Task = 1;
uint8_t OS_TCB_Tbl_Next = 0;

OS_STK Task_Spin_Motors_Stk[TASK_STK_SIZE];
OS_STK Task_Blink_Led_Stk[TASK_STK_SIZE];
static OS_STK *Main_Stk_Ptr;
static uint32_t systick_counter;
uint8_t os_scheduler_num = 0;
/**
  ******************************************************************************
  * Private function declaration
  ******************************************************************************
  */
  void OS_Init(void);
  void OS_Start(void);
  void OS_Task_Create(void(*task)(void), OS_STK *ptos, uint16_t stk_size);
  void OS_Scheduler(void);
  static inline void Start_Task(OS_TCB* ptcb);
  static inline void Save_Context(void);
  static inline void Load_Context(void);

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
  // task_blink_led();
  SysTick_Config(SystemCoreClock / 10);
  OS_Init();
  OS_Task_Create(task_spin_motors, (OS_STK *)&Task_Spin_Motors_Stk, TASK_STK_SIZE);
  OS_Task_Create(task_blink_led, (OS_STK *)&Task_Blink_Led_Stk, TASK_STK_SIZE);
  OS_Start();
  while (1) {
    
  }
}
/**
  ******************************************************************************
  * Private functions
  ******************************************************************************
  */

  void OS_Init(void){
    // OS_TCB_Tbl[0].OS_TCB_Stk_Ptr = 
    // NVIC_SetPriority(SysTick_IRQn, DEV_MIDDLE_PRIORITY);
  }
  void OS_Start(void){
    Start_Task(&OS_TCB_Tbl[OS_Cur_Task]);
    // NVIC_SetPriority(SysTick_IRQn, DEV_MIDDLE_PRIORITY);
  }
  /**
   * @brief create a task in the os
   * @details initilize the stack and 
   * 
   * @param k [description]
   * @param ptos [description]
   */
  void OS_Task_Create(void(*task)(void), OS_STK *ptos, uint16_t stk_size){
    
    uint8_t idx;
    OS_HW_STK *stk_frame;
    __disable_irq();
    for (idx = 0; idx < OS_MAX_TASK_NUM; ++idx)
    {
      if (OS_TCB_Tbl[idx].OS_TCB_Stat == OS_TCB_NOT_IN_USE)
      {
        stk_frame = (OS_HW_STK *)(ptos + stk_size - sizeof(OS_SW_STK) / sizeof(uint32_t));
        stk_frame->r0 = 1;
        stk_frame->r1 = 0;
        stk_frame->r2 = 0;
        stk_frame->r12 = 0;
        stk_frame->lr = 0;
        stk_frame->pc = (uint32_t)task;
        stk_frame->psr = OS_DEFAULT_PSR_VAL;
        OS_TCB_Tbl[idx].OS_TCB_Stat = OS_TCB_IN_USE;
        OS_TCB_Tbl[idx].OS_TCB_ID = idx;
        // could have some problem here 
        OS_TCB_Tbl[idx].OS_TCB_Stk_Ptr = ptos + stk_size - sizeof(OS_SW_STK) / sizeof(uint32_t) - sizeof(OS_SW_STK) / sizeof(uint32_t);
        break;
      }
    }
    __enable_irq();
  }

  void OS_Scheduler(void){
    uint32_t scratch;
    scratch = __get_PSP();
    OS_TCB_Tbl[OS_Cur_Task].OS_TCB_Stk_Ptr = (OS_STK*)scratch;
    __disable_irq();
    OS_Cur_Task++;
    if (OS_Cur_Task == OS_MAX_TASK_NUM)
      OS_Cur_Task = 0;
    __enable_irq();
    __set_PSP(OS_TCB_Tbl[OS_Cur_Task].OS_TCB_Stk_Ptr);
  }
/**
  ******************************************************************************
  * ASM Inline functions
  ******************************************************************************
  */
static inline void Start_Task(OS_TCB* ptcb){
  OS_STK* ptos;
  OS_HW_STK *stk_frame;
  uint32_t scratch;

  ptos = ptcb->OS_TCB_Stk_Ptr + sizeof(OS_HW_STK) / sizeof(uint32_t) + sizeof(OS_SW_STK) / sizeof(uint32_t);
  stk_frame = (OS_HW_STK*)(ptcb->OS_TCB_Stk_Ptr + sizeof(OS_SW_STK) / sizeof(uint32_t));
  scratch = stk_frame->pc;

  __set_CONTROL(2);
  __set_PSP(ptos);
  __set_CONTROL(3);
  __ASM volatile("BX %0" : "=r"(scratch));
}

/**
 * @brief save current context
 * @details save r4-r11 to process stack, the Cortex-m3 pushes the other 
 * register automatically when do the context switching
 */
static inline void Save_Context(void){
  uint32_t register scratch asm("r12");
  __ASM volatile ("MRS %0, psp\n\t"
      "STMDB %0!, {r4-r11}\n\t"
      "MSR psp, %0\n\t"  : "=r" (scratch) );
}
/**
 * @brief load 
 * @details [long description]
 */
static inline void Load_Context(void){
  uint32_t register scratch asm("r12");
  __ASM volatile ("MRS %0, psp\n\t"
      "LDMFD %0!, {r4-r11}\n\t"
      "MSR psp, %0\n\t"  : "=r" (scratch) );
}

/**
  ******************************************************************************
  * Exception handlers
  ******************************************************************************
  */


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