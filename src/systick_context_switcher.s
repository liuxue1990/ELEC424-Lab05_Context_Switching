.syntax		unified
.cpu		cortex-m3
.fpu		softvfp
.thumb
.file	"systick_context_switcher.s"

.text
.align 2
.global __asm_reset_thread
.thumb
.thumb_func
__asm_start_thread:
    ldr     r1,=msp_top
    msr     msp,r1		    
    mov     r1,#2
    msr     control,r1         
    ldr     sp,[r0]
    bl      int_half_enable_asyn_signal
    svc     0   