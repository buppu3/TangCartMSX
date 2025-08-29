; =============================================================================
;	V9958 software test program
; -----------------------------------------------------------------------------
;	Programmed by t.hara (HRA!)
; =============================================================================

			org		0x100

start:
			; ����
			call	vdp_io_select
			call	copy_rom_font
			; �e�X�g
			call	screen5
			call	test001
			call	test002
			call	test003
			call	test004
			call	test005
			call	test006
			call	test007
			call	test008
			call	test009
			call	test010

			; ��n��
			call	clear_key_buffer

			; ���ʂ�\��
			ld		hl, test001_result
			ld		b, 10
	result_loop:
			push	bc

			ld		e, [hl]
			inc		hl
			ld		d, [hl]
			inc		hl
			push	hl

			ex		de, hl
			call	put_hl

			pop		hl
			pop		bc
			djnz	result_loop

			ld		e, 13
			ld		c, 2
			call	bdos
			ld		e, 10
			ld		c, 2
			call	bdos

			ld		c, _TERM0
			jp		bdos

include		"lib.asm"

; =============================================================================
;	SCREEN5
;	input:
;		none
;	output:
;		none
;	break:
;		AF
;	comment:
;		none
; =============================================================================
			scope	screen5
screen5::
			; R#0 = 0x0E
			ld		a, 0x06
			ld		e, 0
			call	write_control_register
			; R#1 = 0x40
			ld		a, 0x40
			ld		e, 1
			call	write_control_register
			; R#7 = 0x07
			ld		a, 0x07					; ���ӐF 7
			ld		e, 7
			call	write_control_register
			; R#8 = 0x02
			ld		a, 0x0A					; �X�v���C�g��\��
			ld		e, 8
			call	write_control_register
			; R#9 = 0x80
			ld		a, 0x80					; 212line
			ld		e, 9
			call	write_control_register
			; Pattern Name Table R#2 = 0b0pp11111 : p = page
			ld		a, 0b00011111
			ld		e, 2
			call	write_control_register
			; Sprite Attribute Table
			ld		hl, 0x7A00
			call	set_sprite_attribute_table
			; Sprite Pattern Generator Table
			ld		hl, 0x7000
			call	set_sprite_pattern_generator_table
			; Pattern Name Table ���N���A
			xor		a, a
			ld		[vram_bit16], a
			ld		hl, 0x0000
			ld		bc, 128 * 256
			ld		e, 0x44
			call	fill_vram

			ld		hl, 0x8000
			ld		d, 0
			ld		bc, 128 * 256
			ld		e, 0x85
			call	fill_vram

			ld		a, 1
			ld		[vram_bit16], a
			ld		hl, 0x0000
			ld		bc, 128 * 256
			ld		e, 0x88
			call	fill_vram

			ld		hl, 0x8000
			ld		d, 1
			ld		bc, 128 * 256
			ld		e, 0x99
			call	fill_vram

			xor		a, a
			ld		[vram_bit16], a
			ld		hl, (20 >> 1) + (30 * 128)		;	( 20, 30 ) �ɃA�h���X�Z�b�g
			call	set_vram_write_address
			ld		hl, data1
			ld		b, 15
			call	write_data

			ld		hl, (20 >> 1) + (33 * 128)		;	( 20, 33 ) �ɃA�h���X�Z�b�g
			call	set_vram_write_address
			ld		hl, data2
			ld		b, 8
			call	write_data

			ld		hl, (20 >> 1) + (36 * 128)		;	( 20, 36 ) �ɃA�h���X�Z�b�g
			call	set_vram_write_address
			ld		hl, data3
			ld		b, 10
			call	write_data

			ld		hl, (20 >> 1) + (39 * 128)		;	( 20, 39 ) �ɃA�h���X�Z�b�g
			call	set_vram_write_address
			ld		hl, data4
			ld		b, 10
			call	write_data

			ld		hl, (20 >> 1) + (42 * 128)		;	( 20, 42 ) �ɃA�h���X�Z�b�g
			call	set_vram_write_address
			ld		hl, data5
			ld		b, 15
			call	write_data

			ld		hl, (20 >> 1) + (45 * 128)		;	( 20, 45 ) �ɃA�h���X�Z�b�g
			call	set_vram_write_address
			ld		hl, data6
			ld		b, 15
			call	write_data

			ld		hl, (20 >> 1) + (48 * 128)		;	( 20, 48 ) �ɃA�h���X�Z�b�g
			call	set_vram_write_address
			ld		hl, data7
			ld		b, 15
			call	write_data

			ld		hl, (20 >> 1) + (51 * 128)		;	( 20, 51 ) �ɃA�h���X�Z�b�g
			call	set_vram_write_address
			ld		hl, data8
			ld		b, 15
			call	write_data
			call	wait_push_space_key
			ret

	write_data:
			ld		a, [hl]
			call	write_vram
			inc		hl
			djnz	write_data
			ret

	data1:	;	( 20, 30 )
			db		0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF	; 15byte
	data2:	;	( 20, 33 )
			db		0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0												; 8byte
	data3:	;	( 20, 36 )
			db		0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0xAA									; 10byte
	data4:	;	( 20, 39 )
			db		0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A									; 10byte
	data5:	;	( 20, 42 )
			db		0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF	; 15byte
	data6:	;	( 20, 45 )
			db		0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0	; 15byte
	data7:	;	( 20, 48 )
			db		0x44, 0x44, 0x44, 0x44, 0x44, 0xAA, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33	; 15byte
	data8:	;	( 20, 51 )
			db		0x44, 0x44, 0x44, 0x44, 0x44, 0xA5, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55	; 15byte
			endscope

; =============================================================================
;	�X�e�[�^�X���W�X�^�̓ǂ݂����ƌ��ʊi�[
; =============================================================================
			scope	read_status_bx
read_status_bx::
			ld		e, 8
			call	read_status_register
			ld		[hl], e
			inc		hl
			ld		e, 9
			call	read_status_register
			ld		[hl], e
			ret
			endscope

; =============================================================================
;	HL �̒l��\��
; =============================================================================
			scope	put_hl
put_hl::
			ld		de, 10000
			call	put_one
			ld		de, 1000
			call	put_one
			ld		de, 100
			call	put_one
			ld		de, 10
			call	put_one
			ld		de, 1
			call	put_one
			ld		e, ' '
			ld		c, 2
			call	bdos
			ret

	put_one:
			ld		b, 0
	loop:
			or		a, a
			inc		b
			sbc		hl, de
			jr		nc, loop

			add		hl, de
			push	hl
			dec		b
			ld		a, b
			add		a, '0'
			ld		e, a
			ld		c, 2		;	�R���\�[���o�� _CONOUT
			call	bdos
			pop		hl
			ret
			endscope

; =============================================================================
;	�E�T�� ���E�F
; =============================================================================
			scope	test001
test001::
			ld		a, 2
			ld		e, 7
			call	write_control_register

			ld		hl, data
			ld		a, 32
			ld		b, 15
			call	run_command
			call	wait_command
			; �X�e�[�^�X���W�X�^��ǂ�ŁA���ʕۊǏꏊ�ɏ�������
			ld		hl, test001_result
			call	read_status_bx
			call	wait_push_space_key
			ret
	data:
			dw		20			; SX
			dw		30			; SY
			dw		0			; DX (dummy)
			dw		0			; DY (dummy)
			dw		0			; NX (dummy)
			dw		0			; NY (dummy)
			db		6			; CLR
			db		0b0000000	; ARG [-][-][-][-][-][DIX][EQ][-] EQ=0: �s��v�ŃC���N�������g, EQ=1: ��v�ŃC���N�������g
			db		0x60		; CMD (SRCH)
			endscope

; =============================================================================
			scope	test002
test002::
			ld		a, 4
			ld		e, 7
			call	write_control_register

			ld		hl, data
			ld		a, 32
			ld		b, 15
			call	run_command
			call	wait_command
			; �X�e�[�^�X���W�X�^��ǂ�ŁA���ʕۊǏꏊ�ɏ�������
			ld		hl, test002_result
			call	read_status_bx
			call	wait_push_space_key
			ret
	data:
			dw		20			; SX
			dw		33			; SY
			dw		0			; DX (dummy)
			dw		0			; DY (dummy)
			dw		0			; NX (dummy)
			dw		0			; NY (dummy)
			db		12			; CLR
			db		0b0000000	; ARG [-][-][-][-][-][DIX][EQ][-] EQ=0: �s��v�ŃC���N�������g, EQ=1: ��v�ŃC���N�������g
			db		0x60		; CMD (SRCH)
			endscope

; =============================================================================
;	���T�� ���E�F
; =============================================================================
			scope	test003
test003::
			ld		a, 8
			ld		e, 7
			call	write_control_register

			ld		hl, data
			ld		a, 32
			ld		b, 15
			call	run_command
			call	wait_command
			; �X�e�[�^�X���W�X�^��ǂ�ŁA���ʕۊǏꏊ�ɏ�������
			ld		hl, test003_result
			call	read_status_bx
			call	wait_push_space_key
			ret
	data:
			dw		49			; SX
			dw		42			; SY
			dw		0			; DX (dummy)
			dw		0			; DY (dummy)
			dw		0			; NX (dummy)
			dw		0			; NY (dummy)
			db		11			; CLR
			db		0b0000100	; ARG [-][-][-][-][-][DIX][EQ][-] EQ=0: �s��v�ŃC���N�������g, EQ=1: ��v�ŃC���N�������g
			db		0x60		; CMD (SRCH)
			endscope

; =============================================================================
			scope	test004
test004::
			ld		a, 10
			ld		e, 7
			call	write_control_register

			ld		hl, data
			ld		a, 32
			ld		b, 15
			call	run_command
			call	wait_command
			; �X�e�[�^�X���W�X�^��ǂ�ŁA���ʕۊǏꏊ�ɏ�������
			ld		hl, test004_result
			call	read_status_bx
			call	wait_push_space_key
			ret
	data:
			dw		49			; SX
			dw		45			; SY
			dw		0			; DX (dummy)
			dw		0			; DY (dummy)
			dw		0			; NX (dummy)
			dw		0			; NY (dummy)
			db		7			; CLR
			db		0b0000100	; ARG [-][-][-][-][-][DIX][EQ][-] EQ=0: �s��v�ŃC���N�������g, EQ=1: ��v�ŃC���N�������g
			db		0x60		; CMD (SRCH)
			endscope

; =============================================================================
;	�E�T�� ���E�F�ȊO
; =============================================================================
			scope	test005
test005::
			ld		a, 2
			ld		e, 7
			call	write_control_register

			ld		hl, data
			ld		a, 32
			ld		b, 15
			call	run_command
			call	wait_command
			; �X�e�[�^�X���W�X�^��ǂ�ŁA���ʕۊǏꏊ�ɏ�������
			ld		hl, test005_result
			call	read_status_bx
			call	wait_push_space_key
			ret
	data:
			dw		20			; SX
			dw		36			; SY
			dw		0			; DX (dummy)
			dw		0			; DY (dummy)
			dw		0			; NX (dummy)
			dw		0			; NY (dummy)
			db		3			; CLR
			db		0b0000010	; ARG [-][-][-][-][-][DIX][EQ][-] EQ=0: �s��v�ŃC���N�������g, EQ=1: ��v�ŃC���N�������g
			db		0x60		; CMD (SRCH)
			endscope

; =============================================================================
			scope	test006
test006::
			ld		a, 4
			ld		e, 7
			call	write_control_register

			ld		hl, data
			ld		a, 32
			ld		b, 15
			call	run_command
			call	wait_command
			; �X�e�[�^�X���W�X�^��ǂ�ŁA���ʕۊǏꏊ�ɏ�������
			ld		hl, test006_result
			call	read_status_bx
			call	wait_push_space_key
			ret
	data:
			dw		20			; SX
			dw		39			; SY
			dw		0			; DX (dummy)
			dw		0			; DY (dummy)
			dw		0			; NX (dummy)
			dw		0			; NY (dummy)
			db		5			; CLR
			db		0b0000010	; ARG [-][-][-][-][-][DIX][EQ][-] EQ=0: �s��v�ŃC���N�������g, EQ=1: ��v�ŃC���N�������g
			db		0x60		; CMD (SRCH)
			endscope

; =============================================================================
;	���T�� ���E�F�ȊO
; =============================================================================
			scope	test007
test007::
			ld		a, 8
			ld		e, 7
			call	write_control_register

			ld		hl, data
			ld		a, 32
			ld		b, 15
			call	run_command
			call	wait_command
			; �X�e�[�^�X���W�X�^��ǂ�ŁA���ʕۊǏꏊ�ɏ�������
			ld		hl, test007_result
			call	read_status_bx
			call	wait_push_space_key
			ret
	data:
			dw		49			; SX
			dw		48			; SY
			dw		0			; DX (dummy)
			dw		0			; DY (dummy)
			dw		0			; NX (dummy)
			dw		0			; NY (dummy)
			db		3			; CLR
			db		0b0000110	; ARG [-][-][-][-][-][DIX][EQ][-] EQ=0: �s��v�ŃC���N�������g, EQ=1: ��v�ŃC���N�������g
			db		0x60		; CMD (SRCH)
			endscope

; =============================================================================
			scope	test008
test008::
			ld		a, 10
			ld		e, 7
			call	write_control_register

			ld		hl, data
			ld		a, 32
			ld		b, 15
			call	run_command
			call	wait_command
			; �X�e�[�^�X���W�X�^��ǂ�ŁA���ʕۊǏꏊ�ɏ�������
			ld		hl, test008_result
			call	read_status_bx
			call	wait_push_space_key
			ret
	data:
			dw		49			; SX
			dw		51			; SY
			dw		0			; DX (dummy)
			dw		0			; DY (dummy)
			dw		0			; NX (dummy)
			dw		0			; NY (dummy)
			db		5			; CLR
			db		0b0000110	; ARG [-][-][-][-][-][DIX][EQ][-] EQ=0: �s��v�ŃC���N�������g, EQ=1: ��v�ŃC���N�������g
			db		0x60		; CMD (SRCH)
			endscope

; =============================================================================
;	�E�T�� �n�_����ʊO
; =============================================================================
			scope	test009
test009::
			ld		a, 2
			ld		e, 7
			call	write_control_register

			ld		hl, data
			ld		a, 32
			ld		b, 15
			call	run_command
			call	wait_command
			; �X�e�[�^�X���W�X�^��ǂ�ŁA���ʕۊǏꏊ�ɏ�������
			ld		hl, test009_result
			call	read_status_bx
			call	wait_push_space_key
			ret
	data:
			dw		345			; SX
			dw		123			; SY
			dw		0			; DX (dummy)
			dw		0			; DY (dummy)
			dw		0			; NX (dummy)
			dw		0			; NY (dummy)
			db		15			; CLR
			db		0b0000000	; ARG [-][-][-][-][-][DIX][EQ][-] EQ=0: �s��v�ŃC���N�������g, EQ=1: ��v�ŃC���N�������g
			db		0x60		; CMD (SRCH)
			endscope

; =============================================================================
;	���T�� �n�_����ʊO
; =============================================================================
			scope	test010
test010::
			ld		a, 4
			ld		e, 7
			call	write_control_register

			ld		hl, data
			ld		a, 32
			ld		b, 15
			call	run_command
			call	wait_command
			; �X�e�[�^�X���W�X�^��ǂ�ŁA���ʕۊǏꏊ�ɏ�������
			ld		hl, test010_result
			call	read_status_bx
			call	wait_push_space_key
			ret
	data:
			dw		456			; SX
			dw		346			; SY
			dw		0			; DX (dummy)
			dw		0			; DY (dummy)
			dw		0			; NX (dummy)
			dw		0			; NY (dummy)
			db		12			; CLR
			db		0b0000110	; ARG [-][-][-][-][-][DIX][EQ][-] EQ=0: �s��v�ŃC���N�������g, EQ=1: ��v�ŃC���N�������g
			db		0x60		; CMD (SRCH)
			endscope

; =============================================================================
			scope	results
test001_result::
			dw		0
test002_result::
			dw		0
test003_result::
			dw		0
test004_result::
			dw		0
test005_result::
			dw		0
test006_result::
			dw		0
test007_result::
			dw		0
test008_result::
			dw		0
test009_result::
			dw		0
test010_result::
			dw		0
			endscope
