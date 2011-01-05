
; 16.16 signed binary math lib for AVR's with RAM V1.0
; ALL code by Rolf R Bakke,  April 2010

;Uses no AVR registers exept X and Y for the store and load command, and r16 for all commands


.equ	Result0=B16_WorkAdd+0
.equ	Result1=B16_WorkAdd+1
.equ	Result2=B16_WorkAdd+2
.equ	Result3=B16_WorkAdd+3
.equ	Result4=B16_WorkAdd+4
.equ	Result5=B16_WorkAdd+5

.equ	Op1_0=B16_WorkAdd+6
.equ	Op1_1=B16_WorkAdd+7
.equ	Op1_2=B16_WorkAdd+8
.equ	Op1_3=B16_WorkAdd+9
.equ	Op1_4=B16_WorkAdd+10
.equ	Op1_5=B16_WorkAdd+11

.equ	Op2_0=B16_WorkAdd+12
.equ	Op2_1=B16_WorkAdd+13
.equ	Op2_2=B16_WorkAdd+14
.equ	Op2_3=B16_WorkAdd+15

.equ	Op1Index=B16_WorkAdd+16	;Operand #1 index
.equ	Op2Index=B16_WorkAdd+17	;Operand #2 index

.equ	ResultSign=B16_WorkAdd+18

.equ	RegSave=B16_WorkAdd+19

.def	a=r16			;work registers
.def    b=r17
.def	c=r18
.def	d=r19





b16_sr:	sts RegSave+0,r17
	sts RegSave+1,r18
	sts RegSave+2,r19
	sts RegSave+3,zl
	sts RegSave+4,zh
	ret

b16_lr:	
	lds r17,RegSave+0
	lds r18,RegSave+1
	lds r19,RegSave+2
	lds zl,RegSave+3
	lds zh,RegSave+4
	ret


b16_clr_c:
	rcall b16_getregadd
	clr a
	st z+,a
	st z+,a
	st z+,a
	st z,a
	ret


b16_cmp_c:			;Only use breq, brne, brge and brlt after the compare   (OK)
	lds a,Op1Index
	rcall b16_getregadd
	rcall b16_GetOp1
		
	lds a,Op2Index
	rcall b16_getregadd
	rcall b16_GetOp2

	lds a,Op1_3
	lds b,Op2_3
	cp a,b

	lds a,Op1_2
	lds b,Op2_2
	cpc a,b

	lds a,Op1_1
	lds b,Op2_1
	cpc a,b

	lds a,Op1_0
	lds b,Op2_0
	cpc a,b

	ret



b16_move_c:				;(OK)
	lds a,Op2Index
	rcall b16_load_c
	lds a,Op1Index
	rcall b16_store_c
	ret
		

b16_store_c:
	rcall b16_getregadd
	st z+,xh
	st z+,xl
	st z+,yh
	st z,yl
	ret


b16_load_c:
	rcall b16_getregadd
	ld xh,z+
	ld xl,z+
	ld yh,z+
	ld yl,z
	ret





;	-*- 16.16 signed binary addition -*-   (OK)

;	  sum     augend      addend  
;	Result = Operand#1 + Operand#2

b16_add_c:
	lds a,Op1Index
	rcall b16_getregadd
	push zl
	push zh
	rcall b16_GetOp1	

	lds a,Op2Index
	rcall b16_getregadd
	rcall b16_GetOp2	

	lds a,Op1_3
	lds b,Op2_3
	add a,b
	sts Result3,a

	lds a,Op1_2
	lds b,Op2_2
	adc a,b
	sts Result2,a

	lds a,Op1_1
	lds b,Op2_1
	adc a,b
	sts Result1,a

	lds a,Op1_0
	lds b,Op2_0
	adc a,b
	sts Result0,a
	
	pop zh
	pop zl
	rcall b16_StoreResult

	ret



;	-*- 16.16 signed binary subtraction -*-  (OK)


;	difference   minuend    subtrahend
;	   Result = Operand#1 - Operand#2


b16_sub_c:
	lds a,Op1Index
	rcall b16_getregadd
	push zl
	push zh
	rcall b16_GetOp1	
	
	lds a,Op2Index
	rcall b16_getregadd
	rcall b16_GetOp2	

	lds a,Op1_3
	lds b,Op2_3
	sub a,b
	sts Result3,a

	lds a,Op1_2
	lds b,Op2_2
	sbc a,b
	sts Result2,a

	lds a,Op1_1
	lds b,Op2_1
	sbc a,b
	sts Result1,a

	lds a,Op1_0
	lds b,Op2_0
	sbc a,b
	sts Result0,a
	
	pop zh
	pop zl
	rcall b16_StoreResult

	ret



;	-*- 16.16 signed binary multiply -*-   (OK)

;	product  multiplicand  multiplier
;	Result = Operand#1  *  Operand#2

b16_mul_c:
	lds a,Op1Index
	rcall b16_getregadd
	push zl
	push zh
	rcall b16_GetOp1	

	lds a,Op2Index
	rcall b16_getregadd
	rcall b16_GetOp2	

	rcall b16_pre	;calculate result sign and convert operands to their absolute values

	clr a
	sts Result0,a	;clear the product
	sts Result1,a
	sts Result2,a
	sts Result3,a
	sts Result4,a	;and the extended part of the product
	sts Result5,a

	lds b,Op1_3	;shift 16 to right
	sts Op1_5,b

	lds b,Op1_2
	sts Op1_4,b

	lds b,Op1_1
	sts Op1_3,b

	lds b,Op1_0
	sts Op1_2,b

	sts Op1_0,a	;clear the high part of the multiplicand
	sts Op1_1,a	

	clt		;clear the multiplicand overflow flag 

	ldi c,32	;there is 32 bits

b16_mu2:lds a,Op2_0
	lsr a		;get the LSB of multiplier
	sts Op2_0,a

	lds a,Op2_1
	ror a
	sts Op2_1,a

	lds a,Op2_2
	ror a
	sts Op2_2,a

	lds a,Op2_3
	ror a
	sts Op2_3,a

	brcc b16_mu3	;one?

	brtc b16_mu9	;yes: if we get to here, and the multiplicand overflow flag is set, we have product overflow
b16_mua:rjmp b16_mu4

b16_mu9:lds a,Result5	;product = product + multiplicand
	lds b,Op1_5
	add a,b
	sts Result5,a
		
	lds a,Result4
	lds b,Op1_4
	adc a,b	
	sts Result4,a
	
	lds a,Result3
	lds b,Op1_3
	adc a,b	
	sts Result3,a
	
	lds a,Result2
	lds b,Op1_2
	adc a,b	
	sts Result2,a
	
	lds a,Result1
	lds b,Op1_1
	adc a,b	
	sts Result1,a
	
	lds a,Result0
	lds b,Op1_0
	adc a,b	
	sts Result0,a
	
	brmi b16_mua	;product overflow?
	brcs b16_mua

b16_mu3:lds a,Op1_5	;shift multiplicand to the left
	lsl a	
	sts Op1_5,a

	lds a,Op1_4
	rol a
	sts Op1_4,a

	lds a,Op1_3
	rol a
	sts Op1_3,a

	lds a,Op1_2
	rol a
	sts Op1_2,a

	lds a,Op1_1
	rol a
	sts Op1_1,a

	lds a,Op1_0
	rol a
	sts Op1_0,a


	brcc b16_mu5	;multiplicand overflow?

	set		;yes, set the multiplicand overflow flag

b16_mu5:dec c		;more bits?
	breq b16_mu6	;no
	
	rjmp b16_mu2	;yes

b16_mu6:lds c,ResultSign	;is result sign negative? (see b16_pre)
	tst c
	brpl b16_mu1	;no
		
	rcall b16_neg	;yes, negate product

b16_mu1:clc		;return with no error
	rjmp b16_mu8

b16_mu4:sec		;yes: set error flag and return

b16_mu8:pop zh
	pop zl
	rcall b16_StoreResult

	ret




;	-*- 16.16 signed binary divide -*-   (OK)


;	Register usage:

;	quotient  dividend       divisor
;	 Result = Operand#1  /  Operand#2



/*     NOTE: B16div is removed due to lack of program space
					
b16_div_c:
	lds a,Op1Index
	rcall b16_getregadd
	push zl
	push zh
	rcall b16_GetOp1	

	lds a,Op2Index
	rcall b16_getregadd
	rcall b16_GetOp2	

	rcall b16_pre	;calculate result sign and convert operands to their absolute values

	lds a,Op1_0	;count # of used bits in dividend
	sts Result0,a
	lds a,Op1_1
	sts Result1,a
	lds a,Op1_2
	sts Result2,a
	lds a,Op1_3
	sts Result3,a
	rcall b16_bc

	mov b,c		;b = # of used bits in dividend

	lds a,Op2_0	;count # of used bits in divisor
	sts Result0,a
	lds a,Op2_1
	sts Result1,a
	lds a,Op2_2
	sts Result2,a
	lds a,Op2_3
	sts Result3,a
	rcall b16_bc

	clr a		;clear quotient
	sts Result0,a
	sts Result1,a
	sts Result2,a
	sts Result3,a

	tst c		;check if divisor is zero
	brne b16_di4

	sec		;yes: return with error and quotient = 0
	rjmp b16_di20	

b16_di4:tst b		;check if dividend is zero
	brne b16_di5

	clc		;yes: return with no error and quotient = 0
	rjmp b16_di20

b16_di5:sub b,c		;do we need to align the divisor to the dividend?
	ldi c,17	;there is 17 bits
	brmi b16_di9	;no
	breq b16_di9	;no

	mov c,b		;yes: shift divisor left b times

b16_di7:lds a,Op2_3
	lsl a
	sts Op2_3,a

	lds a,Op2_2
	rol a
	sts Op2_2,a

	lds a,Op2_1
	rol a
	sts Op2_1,a

	lds a,Op2_0
	rol a
	sts Op2_0,a

	dec c
	brne b16_di7

	ldi c,17	;there is 17 bits
	add c,b		;if we scaled the divisor, we need to scale the quotient as well

			;(From now on the dividend renames to the remainder)

b16_di9:lds a,Op1_3	;remainder = remainder - divisor (does divisor fit inside remainder?)
	lds b,Op2_3
	sub a,b		
	sts Op1_3,a

	lds a,Op1_2
	lds b,Op2_2
	sbc a,b
	sts Op1_2,a

	lds a,Op1_1
	lds b,Op2_1
	sbc a,b
	sts Op1_1,a

	lds a,Op1_0
	lds b,Op2_0
	sbc a,b
	sts Op1_0,a

	sec

	brpl b16_di10	;yes	
	
	lds a,Op1_3	;no: remainder = remainder + divisor (restore remainder)
	lds b,Op2_3
	add a,b		
	sts Op1_3,a

	lds a,Op1_2
	lds b,Op2_2
	adc a,b
	sts Op1_2,a

	lds a,Op1_1
	lds b,Op2_1
	adc a,b
	sts Op1_1,a

	lds a,Op1_0
	lds b,Op2_0
	adc a,b
	sts Op1_0,a

	clc

b16_di10:
	lds a,Result3	;shift result into quotient
	rol a
	sts Result3,a
	lds a,Result2
	rol a
	sts Result2,a
	lds a,Result1
	rol a
	sts Result1,a
	lds a,Result0
	rol a
	sts Result0,a

	brmi b16_di17	;qoutient overflow?

	lds a,Op1_3	;shift remainder left
	lsl a	
	sts Op1_3,a

	lds a,Op1_2
	rol a
	sts Op1_2,a

	lds a,Op1_1
	rol a
	sts Op1_1,a

	lds a,Op1_0
	rol a
	sts Op1_0,a

	dec c		;more bits?
	breq b16_di15	;no
	
	rjmp b16_di9	;yes	
	
b16_di15:lds c,ResultSign	;is result sign negative? (see b16_pre)
	tst c
	brpl b16_di13	;no
	
	rcall b16_neg	;yes, negate quotient

b16_di13:clc		;return with no error
	rjmp b16_di20

b16_di17:sec		;yes: set error flag and return

b16_di20:pop zh
	pop zl
	rcall b16_StoreResult

	ret

*/



;---- subroutines



	
b16_neg:ldi c,1		;negate product  (OK)
	ldi b,0xff
	lds a,Result3
	sub a,c
	eor a,b
	sts Result3,a

	clr c
	lds a,Result2
	sbc a,c
	eor a,b
	sts Result2,a

	lds a,Result1
	sbc a,c
	eor a,b
	sts Result1,a

	lds a,Result0
	sbc a,c
	eor a,b
	sts Result0,a

	ret



b16_bc:	ldi c,33	;bit counter
	sec
b16_bc1:dec c
	
	lds a,Result3
	rol a
	sts Result3,a

	lds a,Result2
	rol a
	sts Result2,a

	lds a,Result1
	rol a
	sts Result1,a

	lds a,Result0
	rol a
	sts Result0,a
	
	brcc b16_bc1

	ret



b16_pre:lds a,Op1_0	;ResultSign bit 7 = sign of OP1 EOR sign of OP2
	lds b,Op2_0
	mov c,a	
	eor c,b
	sts ResultSign,c

	tst a		;OP1 = ABS(OP1)
	brpl b16_pr1

	ldi c,0xff
	ldi d,1

	lds a,Op1_3
	eor a,c
	add a,d
	sts Op1_3,a

	clr d
	lds a,Op1_2
	eor a,c
	adc a,d
	sts Op1_2,a

	lds a,Op1_1
	eor a,c
	adc a,d
	sts Op1_1,a

	lds a,Op1_0
	eor a,c
	adc a,d
	sts Op1_0,a
	
b16_pr1:tst b		;OP2 = ABS(OP2)
	brpl b16_pr2

	ldi c,0xff
	ldi d,1

	lds a,Op2_3
	eor a,c
	add a,d
	sts Op2_3,a

	clr d
	lds a,Op2_2
	eor a,c
	adc a,d
	sts Op2_2,a

	lds a,Op2_1
	eor a,c
	adc a,d
	sts Op2_1,a

	lds a,Op2_0
	eor a,c
	adc a,d
	sts Op2_0,a	

b16_pr2:ret


b16_GetOp1:
	ld a,z+
	sts Op1_0,a
	ld a,z+
	sts Op1_1,a
	ld a,z+
	sts Op1_2,a
	ld a,z+
	sts Op1_3,a
	ret


b16_GetOp2:
	ld a,z+
	sts Op2_0,a
	ld a,z+
	sts Op2_1,a
	ld a,z+
	sts Op2_2,a
	ld a,z+
	sts Op2_3,a
	ret


b16_StoreResult:
	lds a,Result0
	st z+,a
	lds a,Result1
	st z+,a
	lds a,Result2
	st z+,a
	lds a,Result3
	st z+,a
	ret



b16_getregadd:
	ldi zl,low(b16_regadd)
	ldi zh,high(b16_regadd)
	lsl a
	lsl a
	add zl,a
	clr a
	adc zh,a
	ret
