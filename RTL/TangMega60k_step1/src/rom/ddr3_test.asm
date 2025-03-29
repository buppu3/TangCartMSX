; -----------------------------------------------------------------------------
;  Test of DDR3-SDRAM Controller for TangPrimer20K_step3
; =============================================================================
;  Programmed by t.hara
; -----------------------------------------------------------------------------

UART			:= 0x10
DATA			:= 0x20		; (R/W) DATA �` (DATA + 0x0F) ���f�[�^�̓ǂݏ����A�h���X
BUSY_STATUS		:= 0x30		; (RO)  bit7 �� SDRAM���������t���O, bit0 �� BUSY�t���O
BUSY_COUNT		:= 0x31		; (RO)  BUSY�t���O�������Ă����T�C�N����
DELAY_STATUS	:= 0x32		; (RO)  bit0 �� DELAY�����t���O
DELAY_COUNT		:= 0x33		; (RO)  DELAY�����t���O�����܂ł̃T�C�N����
MASK			:= 0x30		; (WO)  MASK �` (MASK + 0x01) ���f�[�^�}�X�N�̏������݃A�h���X
ADDRESS			:= 0x32		; (WO)  ADDRESS �` (ADDRESS + 0x03) ���A�h���X�̏������݃A�h���X
WRITE_REQ		:= 0x36		; (WO)  �������ނ� DRAM �ɏ������݃A�N�Z�X�ɍs���B�������ݒl�͉��ł�����(���������j
READ_REQ		:= 0x37		; (WO)	�������ނ� DRAM �ɓǂݏo���A�N�Z�X�ɍs���B�������ݒl�͉��ł�����(���������j

			org		0x0000

			di
			ld		sp, 0x4000

			call	wait_key

			; �^�C�g����\�����Ă���A�L�[���͑҂�
start:
			ld		de, s_title
			call	puts
			call	wait_key

			; BUSY�`�F�b�N
			ld		de, s_busy_check
			call	puts
busy_check_loop:
			in		a, [BUSY_STATUS]
			or		a, a
			jr		nz, busy_check_loop
			ld		de, s_ok
			call	puts
			jp		start
s_title:
			db		"DDR3-SDRAM Test program", 0x0D, 0x0A, 0
s_busy_check:
			db		"SDRAM Busy Check ... ", 0
s_ok:
			db		"OK", 0x0D, 0x0A, 0

; -----------------------------------------------------------------------------
;	wait_key
;	input:
;		none
;	output:
;		none
;	comment:
;		�L�[����������܂ő҂��Ă���A�L�[���������̂�҂�
; -----------------------------------------------------------------------------
			scope	wait_key
wait_key::
			call	getkey
			and		a, 1
			jr		nz, wait_key
_loop:
			call	getkey
			and		a, 1
			jr		z, _loop
			ret
			endscope

; -----------------------------------------------------------------------------
;	getkey
;	input:
;		none
;	output:
;		A ..... button information
;			bit0 ... button[0]
;			bit1 ... button[1]
;			bit2 ... button[2]
;			bit3 ... button[3]
;			bit4 ... button[4]
;	break:
;		B, C, F
; -----------------------------------------------------------------------------
			scope	getkey
getkey::
			in		a, [UART]
			ret
			endscope

; -----------------------------------------------------------------------------
;	puts
;	input:
;		DE ...... target string address ( '\0' terminated )
;	output:
;		none
;	break:
;		A, B, C, D, E, H, F
; -----------------------------------------------------------------------------
			scope	puts
puts::
			push	af
loop:
			ld		a, [de]
			inc		de
			or		a, a
			jr		z, exit
			out		[UART], a
			jr		loop
exit:
			pop		af
			ret
			endscope
