.ifdef __dsPIC30F
        .include "p30fxxxx.inc"
.endif
.ifdef __dsPIC33F
        .include "p33Fxxxx.inc"
.endif

.section .nbss, bss, near    	
	_etm_scale_saturation_etmscalefactor2_count:	.space 2
	.global _etm_scale_saturation_etmscalefactor2_count
	_etm_scale_saturation_etmscalefactor16_count:	.space 2
	.global _etm_scale_saturation_etmscalefactor16_count
.text	



	;; ----------------------------------------------------------

	
	.global  _ETMScaleFactor2
	;; uses and does not restore W0->W3
	.text
_ETMScaleFactor2:
	;; Value is stored in w0
	;; Scale is stored in w1
	;; Offset is stored in w2 

	CP0		W2
	BRA		NN,  _ETMScaleFactor2_offset_not_negative

	;; The offset is negative
	ADD             W0,W2,W0                ; Add the offset to the base value
	;; Look for overflow
	BRA             C, _ETMScaleFactor2_addition_done
	;; There was overflow with the negative offset
	;; Increment the overflow counter and set the results to 0x0000
	MOV		#0x0000, W0
	INC		_etm_scale_saturation_etmscalefactor2_count
	BRA             _ETMScaleFactor2_addition_done	

_ETMScaleFactor2_offset_not_negative:		
	ADD             W0,W2,W0                ; Add the offset to the base value
	;; Look for overflow
	;; 	CP              W3,W2 ;If W3 is less than W2 then there was an overflow
	BRA             NC, _ETMScaleFactor2_addition_done
	;; There was an overflow in the addition
	;; Increment the overflow counter and set the results to 0xFFFF
	MOV		#0xFFFF, W0
	INC		_etm_scale_saturation_etmscalefactor2_count
	BRA             _ETMScaleFactor2_addition_done	

_ETMScaleFactor2_addition_done:		
	MUL.UU		W0,W1,W2 		; Multiply W0 by W1 and store in W2:W3, MSW is stored in W3
	MUL.UU		W3,#2,W0		; Multiply W3 by 2 and store the results in W0:W1 - W0(LSW) is the result we care about
						
	CP0		W1			; If W1 is Zero, then there was NOT an overflow
	BRA		Z, _ETMScaleFactor2_multiply_ok
	;; There was an overflow in the multiply opertion
	;; Increment the overflow counter and set the result to 0xFFFF
	MOV		#0xFFFF, W0
	INC		_etm_scale_saturation_etmscalefactor2_count
_ETMScaleFactor2_multiply_ok:	
	;; OR together W0, W1 into W0 to give the final results
	LSR		W2, #15, W1		; Take the 1 MSbits of W2 and store then as the 1 LSB of W1
	IOR		W0, W1, W0		; Add W1 to W0 (using bitwise or in this case)
	RETURN




	
	;; ----------------------------------------------------------

	
	.global  _ETMScaleFactor16
	;; uses and does not restore W0->W3
	.text
_ETMScaleFactor16:
	;; Value is stored in w0
	;; Scale is stored in w1
	;; Offset is stored in w2 	

	CP0		W2
	BRA		NN,  _ETMScaleFactor16_offset_not_negative

	;; The offset is negative
	ADD             W0,W2,W0                ; Add the offset to the base value
	;; Look for overflow
	BRA             C, _ETMScaleFactor16_addition_done
	;; There was overflow with the negative offset
	;; Increment the overflow counter and set the results to 0x0000
	MOV		#0x0000, W0
	INC		_etm_scale_saturation_etmscalefactor16_count
	BRA             _ETMScaleFactor16_addition_done	

_ETMScaleFactor16_offset_not_negative:		
	ADD             W0,W2,W0                ; Add the offset to the base value
	;; Look for overflow
	;; 	CP              W3,W2 ;If W3 is less than W2 then there was an overflow
	BRA             NC, _ETMScaleFactor16_addition_done
	;; There was an overflow in the addition
	;; Increment the overflow counter and set the results to 0xFFFF
	MOV		#0xFFFF, W0
	INC		_etm_scale_saturation_etmscalefactor16_count
	BRA             _ETMScaleFactor16_addition_done	

_ETMScaleFactor16_addition_done:		
	
	MUL.UU		W0,W1,W2 		; Multiply W0 by W1 and store in W2:W3, MSW is stored in W3
	MUL.UU		W3,#16,W0		; Multiply W3 by 16 and store the results in W0:W1 - W0(LSW) is the result we care about
						
	CP0		W1			; If W1 is Zero, then there was NOT an overflow
	BRA		Z, _ETMScaleFactor16_no_overflow
	;; There was an overflow in the multiply opertion
	;; Increment the overflow counter and set the result to 0xFFFF
	MOV		#0xFFFF, W0
	INC		_etm_scale_saturation_etmscalefactor16_count
_ETMScaleFactor16_no_overflow:	
	;; OR together W0, W1 into W0 to give the final results
	LSR		W2, #12, W1		; Take the 4 MSbits of W2 and store then as the 4 LSB of W1
	IOR		W0, W1, W0		; Add W1 to W0 (using bitwise or in this case)
	RETURN

	
