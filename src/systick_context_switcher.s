.equ NVIC_INT_CTRL , 0xE000ED04
.equ NVIC_SYS_PRI , 0xE000ED22
.equ NVIC_PENDSV_PRI , 0xFF
.equ NVIC_PENDSV_SET , 0x10000000
 
.syntax		unified
.cpu		cortex-m3
.thumb
.file	"systick_context_switcher.s"
 
.text
.align 2
.global __asm_set_pendsv
.thumb
.thumb_func
__asm_set_pendsv:
	ldr		r0, =NVIC_SYS_PRI
	ldr		r1, =NVIC_PENDSV_PRI
	strb	r1, [r0]
	bx 		lr
 
.text
.align 2
.global __asm_trigger_pendsv
.thumb
.thumb_func
__asm_trigger_pendsv:
	ldr 	r0, =NVIC_INT_CTRL
	ldr 	r1, =NVIC_PENDSV_SET
	str 	r1, [r0]
	bx 		lr
	