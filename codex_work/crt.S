.global crtStart
.global main
.global irqCallback

    .section	.start_jump,"ax",@progbits
crtStart:
  //long jump to allow crtInit to be anywhere
  //do it always in 12 bytes
  lui x2,       %hi(crtInit)
  addi x2, x2,  %lo(crtInit)
  jalr x1,x2
  nop

/* 【修改点在这里】把 .text 改成了 .boot_code */
.section .boot_code, "ax", @progbits

.global  trap_entry
.align 5
trap_entry:
  sw x1,  - 1*4(sp)
  sw x5,  - 2*4(sp)
  sw x6,  - 3*4(sp)
  sw x7,  - 4*4(sp)
  sw x10, - 5*4(sp)
  sw x11, - 6*4(sp)
  sw x12, - 7*4(sp)
  sw x13, - 8*4(sp)
  sw x14, - 9*4(sp)
  sw x15, -10*4(sp)
  sw x16, -11*4(sp)
  sw x17, -12*4(sp)
  sw x28, -13*4(sp)
  sw x29, -14*4(sp)
  sw x30, -15*4(sp)
  sw x31, -16*4(sp)
  addi sp,sp,-16*4
  call irqCallback
  lw x1 , 15*4(sp)
  lw x5,  14*4(sp)
  lw x6,  13*4(sp)
  lw x7,  12*4(sp)
  lw x10, 11*4(sp)
  lw x11, 10*4(sp)
  lw x12,  9*4(sp)
  lw x13,  8*4(sp)
  lw x14,  7*4(sp)
  lw x15,  6*4(sp)
  lw x16,  5*4(sp)
  lw x17,  4*4(sp)
  lw x28,  3*4(sp)
  lw x29,  2*4(sp)
  lw x30,  1*4(sp)
  lw x31,  0*4(sp)
  addi sp,sp,16*4
  mret

crtInit:
  .option push
  .option norelax
  la gp, __global_pointer$
  .option pop
  la sp, _stack_start   /* 这里会正确加载 DDR 里的栈顶地址 */
  # Enable M-mode FPU state before any C code can execute. The full
  # application is built with ilp32f, so main may save F registers in its
  # prologue before it can call a C helper.
  csrr t0, mstatus
  li t1, 0x00006000
  or t0, t0, t1
  csrw mstatus, t0
  call boot_uart_init
  li a2, 'A'
  call boot_uart_putc

bss_init:
  la a0, _bss_start     /* 这里会正确指向 DDR 里的 BSS 段起始 */
  la a1, _bss_end       /* 这里会正确指向 DDR 里的 BSS 段结束 */
  li a2, 'B'
  call boot_uart_putc
bss_loop:
  beq a0,a1,bss_done
  sw zero,0(a0)
  add a0,a0,4
  j bss_loop
bss_done:
  li a2, 'C'
  call boot_uart_putc

ctors_init:
  la a0, _ctors_start
  addi sp,sp,-4
ctors_loop:
  la a1, _ctors_end
  beq a0,a1,ctors_done
  lw a3,0(a0)
  add a0,a0,4
  sw a0,0(sp)
  jalr  a3
  lw a0,0(sp)
  j ctors_loop
ctors_done:
  addi sp,sp,4

  # Disable interrupts but keep M-mode FPU state enabled before entering C.
  csrw mie, zero
  csrr t0, mstatus
  li t1, 0x00006000
  or t0, t0, t1
  csrw mstatus, t0

  li a2, 'D'
  call boot_uart_putc
  call main        /* 跳转到 main，此时 main 在 DDR 里 */
infinitLoop:
  j infinitLoop

boot_uart_putc:
  li t0, 0xF0011000
1:
  lw t1, 20(t0)
  andi t1, t1, 0x20
  beqz t1, 1b
  sw a2, 0(t0)
  ret

boot_uart_init:
  li t0, 0xF0011000
  sw zero, 4(t0)
  li t1, 0x80
  sw t1, 12(t0)
  li t1, 54
  sw t1, 0(t0)
  sw zero, 4(t0)
  li t1, 0x03
  sw t1, 12(t0)
  li t1, 0x07
  sw t1, 8(t0)
  li t1, 1000
2:
  addi t1, t1, -1
  bnez t1, 2b
  ret
