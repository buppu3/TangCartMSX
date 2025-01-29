; -----------------------------------------------------------------------------
;  Hello, world! for cZ80 step1 test program
; =============================================================================
;  Programmed by t.hara
; -----------------------------------------------------------------------------

REG_KEY_STATE		:= 0x10

REG_PALETTE_ADDR	:= 0x20
REG_PALETTE_COLOR	:= 0x21
REG_VRAM_ADDR		:= 0x22
REG_VRAM_DATA		:= 0x23

			org		0x0000

			di
			ld		sp, 0

			xor		a, a
			out		[REG_PALETTE_ADDR], a
palette_loop:
			push	af
			out		[REG_PALETTE_COLOR], a		; R
			srl		a
			out		[REG_PALETTE_COLOR], a		; G
			srl		a
			out		[REG_PALETTE_COLOR], a		; B
			pop		af
			inc		a
			jr		nz, palette_loop

			out		[REG_VRAM_ADDR], a
			out		[REG_VRAM_ADDR], a
			out		[REG_VRAM_ADDR], a

;			ld		hl, 360
;y_loop:
;			push	hl
;			ld		hl, 640
;x_loop:
;			ld		a, l
;			out		[REG_VRAM_DATA], a
;			dec		hl
;			ld		a, l
;			or		a, h
;			jr		nz, x_loop
;			pop		hl
;			dec		hl
;			ld		a, l
;			or		a, h
;			jr		nz, y_loop

			ld		b, 255
main_loop:
			; B���W�X�^�̖��邳����ɁA�p���b�g�̒l���X�V (R,G,B)=(b,b/2,b/4)
			ld		a, 128
			out		[REG_PALETTE_ADDR], a
			ld		a, b
			out		[REG_PALETTE_COLOR], a		; R
			srl		a
			out		[REG_PALETTE_COLOR], a		; G
			srl		a
			out		[REG_PALETTE_COLOR], a		; B

			; ���ԑ҂�
			ld		hl, 1000
wait_loop:
			dec		hl
			ld		a, l
			or		a, h
			jr		nz, wait_loop

			dec		b
			jp		main_loop

;wait_key_release:
;			call	getkey
;			or		a, a
;			jr		nz, wait_key_release
;
;main_loop:
;			call	getkey
;			or		a, a
;			jr		z, main_loop
;
;			rrca
;			jr		c, press_button0
;			rrca
;			jr		c, press_button1
;
;press_button0:
;			jr		wait_key_release
;
;press_button1:
;			jr		wait_key_release

; -----------------------------------------------------------------------------
;	getkey
;	input:
;		none
;	output:
;		A ..... button information
;			bit0 ... button[0]
;			bit1 ... button[1]
;	break:
;		B, C, F
; -----------------------------------------------------------------------------
			scope	getkey
getkey::
			in		a, [REG_KEY_STATE]
			rlca
			rlca
			ret
			endscope
