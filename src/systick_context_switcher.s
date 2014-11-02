.cpu cortex-m3
.fpu softvfp   
.syntax unified 
.thumb
.text
.global  SysTick_Handler
.thumb_func
SysTick_Handler:
	PUSH {lr}
	MRS r12, psp
	STMDB r12!, {r4-r11}
	MSR psp, r12
    LDR     R0, =OS_Scheduler
    BLX     R0
	MRS r12, PSP
    LDMFD r12!, {r4-r11}
    MSR psp, r12
    POP {pc}
