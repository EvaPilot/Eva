;------------------------------------------------------------------------------
;-         ATMEL Microcontroller Software Support  -  ROUSSET  -
;------------------------------------------------------------------------------
; The software is delivered "AS IS" without warranty or condition of any
; kind, either express, implied or statutory. This includes without
; limitation any warranty or condition with respect to merchantability or
; fitness for any particular purpose, or against the infringements of
; intellectual property rights of others.
;-----------------------------------------------------------------------------
;- File source          : it_handler.s
;- Object               : Example of IT handler calling a C function
;- Compilation flag     : None
;-
;- 1.0 16/03/01 	ODi, : Creation ARM ADS
;------------------------------------------------------------------------------
	AREA        itHandler, CODE, READONLY
;------------------------------------------------------------------------------
;- LISR vector handler for system peripherals
;--------------------------------------------
;- This macro save the context, call the LISR dispatch routine, and restore
;- the context
;------------------------------------------------------------------------------
    INCLUDE   AT91RM9200.inc

;--------------------------------
;- ARM Core Mode and Status Bits
;--------------------------------

ARM_MODE_USER           EQU     0x10
ARM_MODE_FIQ            EQU     0x11
ARM_MODE_IRQ            EQU     0x12
ARM_MODE_SVC            EQU     0x13
ARM_MODE_ABORT          EQU     0x17
ARM_MODE_UNDEF          EQU     0x1B
ARM_MODE_SYS            EQU     0x1F

I_BIT                   EQU     0x80
F_BIT                   EQU     0x40
T_BIT                   EQU     0x20


;------------------------------------------------------------------------------
;- IRQ Entry
;-----------
;------------------------------------------------------------------------------
	MACRO
	IRQ_ENTRY   $reg

;- Adjust and save LR_irq in IRQ stack
 	sub         r14, r14, #4
	stmfd       sp!, {r14}

;- Write in the IVR to support Protect Mode
;- No effect in Normal Mode
;- De-assert the NIRQ and clear the source in Protect Mode
	ldr         r14, =AT91C_BASE_AIC
	str         r14, [r14, #AIC_IVR]

;- Save SPSR and r0 in IRQ stack
	mrs         r14, SPSR
	stmfd       sp!, {r0, r14}

;- Enable Interrupt and Switch in SYS Mode
	mrs         r0, CPSR
	bic         r0, r0, #I_BIT
	orr         r0, r0, #ARM_MODE_SYS
	msr         CPSR_c, r0

;- Save scratch/used registers and LR in User Stack
	IF  "$reg" = ""
	stmfd       sp!, { r1-r3, r12, r14}
	ELSE
	stmfd       sp!, { r1-r3, $reg, r12, r14}
	ENDIF

	MEND

;------------------------------------------------------------------------------
;- IRQ Exit
; ---------
;------------------------------------------------------------------------------
 	MACRO
	IRQ_EXIT    $reg
   
;- Restore scratch/used registers and LR from User Stack
 	IF  "$reg" = ""
	ldmia       sp!, { r1-r3, r12, r14}
	ELSE
	ldmia       sp!, { r1-r3, $reg, r12, r14}
	ENDIF

;- Disable Interrupt and switch back in IRQ mode
	mrs         r0, CPSR
	bic         r0, r0, #ARM_MODE_SYS
	orr         r0, r0, #I_BIT:OR:ARM_MODE_IRQ
	msr         CPSR_c, r0

;- Mark the End of Interrupt on the AIC
	ldr         r0, =AT91C_BASE_AIC
	str         r0, [r0, #AIC_EOICR]

;- Restore SPSR_irq and r0 from IRQ stack
	ldmia       sp!, {r0, r14}
	msr         SPSR_cxsf, r14

;- Restore adjusted  LR_irq from IRQ stack directly in the PC
	ldmia       sp!, {pc}^

	MEND



;------------------------------------------------------------------------------
	EXPORT AT91F_ST_ASM_HANDLER
	IMPORT AT91F_ST_HANDLER
	
;	EXPORT AT91F_UARTDBGU_ASM_HANDLER
;	IMPORT AT91F_UARTDBGU_HANDLER

	EXPORT AT91F_UART0_ASM_HANDLER
	IMPORT AT91F_UART0_HANDLER

	EXPORT AT91F_UART1_ASM_HANDLER
	IMPORT AT91F_UART1_HANDLER

	EXPORT AT91F_UART2_ASM_HANDLER
	IMPORT AT91F_UART2_HANDLER
		
	EXPORT AT91F_IRQ0_ASM_HANDLER
	IMPORT AT91F_IRQ0_HANDLER

AT91F_ST_ASM_HANDLER
	IRQ_ENTRY

	ldr     r1, =AT91F_ST_HANDLER
	mov     r14, pc
	bx      r1

	IRQ_EXIT


;------------------------------------------------------------------------------


;AT91F_UARTDBGU_ASM_HANDLER
;	IRQ_ENTRY

;	ldr     r1, =AT91F_UARTDBGU_HANDLER
;	mov     r14, pc
;	bx      r1

;	IRQ_EXIT

;------------------------------------------------------------------------------


AT91F_UART0_ASM_HANDLER
	IRQ_ENTRY

	ldr     r1, =AT91F_UART0_HANDLER
	mov     r14, pc
	bx      r1

	IRQ_EXIT
;------------------------------------------------------------------------------


AT91F_UART1_ASM_HANDLER
	IRQ_ENTRY

	ldr     r1, =AT91F_UART1_HANDLER
	mov     r14, pc
	bx      r1

	IRQ_EXIT
;------------------------------------------------------------------------------


AT91F_UART2_ASM_HANDLER
	IRQ_ENTRY

	ldr     r1, =AT91F_UART2_HANDLER
	mov     r14, pc
	bx      r1

	IRQ_EXIT


;------------------------------------------------------------------------------


AT91F_IRQ0_ASM_HANDLER
	IRQ_ENTRY

	ldr     r1, =AT91F_IRQ0_HANDLER
	mov     r14, pc
	bx      r1

	IRQ_EXIT
	END	