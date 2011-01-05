
.macro b16ldi
;	rcall b16_sr

	ldi t,@0
	rcall b16_getregadd

	ldi t,high(int(@1+0x8000)-0x8000)
	st z+,t
	ldi t,low(int(@1+0x8000)-0x8000)
	st z+,t
	ldi t,high(int(frac(@1)*0x10000+0x10000)-0x10000)
	st z+,t
	ldi t,low(int(frac(@1)*0x10000+0x10000)-0x10000)
	st z+,t

;	rcall b16_lr
.endmacro


.macro b16clr

	ldi t,@0
	rcall b16_clr_c

.endmacro.


.macro b16store			;X:Y  -> 16.16 register
;	rcall b16_sr

	ldi t,@0
	rcall b16_store_c

;	rcall b16_lr
.endmacro


.macro b16load			;X:Y  <- 16.16 register
;	rcall b16_sr

	ldi t,@0
	rcall b16_load_c

;	rcall b16_lr
.endmacro


.macro b16add
;	rcall b16_sr

	ldi t,@0
	sts Op1Index,t
	ldi t,@1
	sts Op2Index,t
	rcall b16_add_c

;	rcall b16_lr
.endmacro


.macro b16mul
;	rcall b16_sr

	ldi t,@0
	sts Op1Index,t
	ldi t,@1
	sts Op2Index,t
	rcall b16_mul_c

;	rcall b16_lr
.endmacro


.macro b16sub
;	rcall b16_sr

	ldi t,@0
	sts Op1Index,t
	ldi t,@1
	sts Op2Index,t
	rcall b16_sub_c

;	rcall b16_lr
.endmacro


.macro b16div
;	rcall b16_sr

	ldi t,@0
	sts Op1Index,t
	ldi t,@1
	sts Op2Index,t
	rcall b16_div_c

;	rcall b16_lr
.endmacro


.macro b16mov
;	rcall b16_sr

	ldi t,@0
	sts Op1Index,t
	ldi t,@1
	sts Op2Index,t
	rcall b16_move_c

;	rcall b16_lr
.endmacro


.macro b16cmp
;	rcall b16_sr

	ldi t,@0
	sts Op1Index,t
	ldi t,@1
	sts Op2Index,t
	rcall b16_cmp_c

;	rcall b16_lr
.endmacro

