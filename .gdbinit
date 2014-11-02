target remote:3333
monitor reset halt
display Task_Spin_Motors_Stk + 100 - OS_TCB_Tbl[0].OS_TCB_Stk_Ptr
display Task_Blink_Led_Stk + 100 - OS_TCB_Tbl[1].OS_TCB_Stk_Ptr
display Task_Spin_Motors_Stk + 100 - (OS_STK*)($psp)
display Task_Blink_Led_Stk + 100 - (OS_STK*)($psp)
display/x Task_Blink_Led_Stk
display/x Task_Spin_Motors_Stk
display os_scheduler_num
display OS_Cur_Task
b Start_Task
c
