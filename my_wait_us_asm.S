	.cpu cortex-m0plus
	.arch armv6s-m
	.syntax unified
	.code	16
	.thumb_func
	.fpu softvfp

.globl my_wait_us_asm
.type  my_wait_us_asm, %function

my_wait_us_asm:
WAIT_1_US:
    ADDS    R1, R2, #0      // clock cycle 1
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0      // clock cycle 2
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0      // clock cycle 3
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0      // clock cycle 4
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0      // clock cycle 5
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0      // clock cycle 6
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0      // clock cycle 7
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0      // clock cycle 8
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0      // clock cycle 9
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0      // clock cycle 10
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0      // clock cycle 11
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0      // clock cycle 12
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0
    ADDS    R1, R2, #0

;    SUBS    R0, R0, #1      // clock cycle 167
;    CMP	    R0, #0
;    BEQ     EXIT
;    LDR     R2, =WAIT_1_US
;    BX	    R2

;EXIT:
    BX      LR      // return
