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
			di
			call	screen1
			call	sp1_pattern_test1
			call	sp1_pattern_test2

			; ��n��
			; R#15 = 0x00
			ld		a, 0x00
			ld		e, 15
			call	write_control_register
			ei
			call	clear_key_buffer
			ld		c, _TERM0
			jp		bdos

include "lib.asm"

; =============================================================================
;	SCREEN1
;	input:
;		none
;	output:
;		none
;	break:
;		AF
;	comment:
;		none
; =============================================================================
			scope	screen1
screen1::
			; R#0 = 0
			xor		a, a
			ld		e, a
			call	write_control_register
			; R#1 = 0x40
			ld		a, 0x40
			ld		e, 1
			call	write_control_register
			; R#7 = 0x07
			ld		a, 0x07					; ���ӐF 7
			ld		e, 7
			call	write_control_register
			; R#8 = 0x00
			ld		a, 0x00					; �X�v���C�g�\��
			ld		e, 8
			call	write_control_register
			; R#9 = 0x00
			ld		a, 0x00
			ld		e, 9
			call	write_control_register
			; R#25 = 0x00
			ld		a, 0x00
			ld		e, 25
			call	write_control_register
			; Pattern Name Table
			ld		hl, 0x1800
			call	set_pattern_name_table
			; Color Table
			ld		hl, 0x2000
			call	set_color_table
			; Sprite Attribute Table
			ld		hl, 0x1B00
			call	set_sprite_attribute_table
			; Sprite Pattern Generator Table
			ld		hl, 0x3800
			call	set_sprite_pattern_generator_table
			; Pattern Generator Table
			ld		hl, 0x0000
			call	set_pattern_generator_table
			; Pattern Name Table ���N���A
			call	cls
			; Font ���Z�b�g
			ld		hl, 0x0000
			call	set_font
			; Font�̐F���Z�b�g
			ld		hl, 0x2000
			ld		bc, 256 / 8
			ld		e, 0xF4
			call	fill_vram
			; �X�v���C�g�A�g���r���[�g��������
			ld		hl, 0x1B00
			ld		bc, 32 * 4
			ld		e, 208
			call	fill_vram
			ret
			endscope

; =============================================================================
;	[SCREEN1] cls
;	input:
;		none
;	output:
;		none
;	break:
;		AF
;	comment:
;		none
; =============================================================================
			scope	cls
cls::
			ld		hl, 0x1800
			ld		bc, 32 * 26
			ld		e, ' '
			call	fill_vram
			ret
			endscope

; =============================================================================
;	[SCREEN1] wait
;	input:
;		none
;	output:
;		none
;	break:
;		AF
;	comment:
;		none
; =============================================================================
			scope	wait
wait::
			; R#15 = 0x02
			ld		a, 0x02
			ld		e, 15
			call	write_control_register
			ld		bc, 5000
	loop:
			dec		bc
			ld		a, c
			or		a, b
			jr		nz, loop
			ret
			endscope

; =============================================================================
;	[SCREEN1] �X�v���C�g�p�^�[�� 256��ނ̒�`�̊m�F
;	input:
;		none
;	output:
;		none
;	break:
;		AF
;	comment:
;		none
; =============================================================================
			scope	sp1_pattern_test1
sp1_pattern_test1::
			call	cls
			; Put test name
			ld		hl, 0x18A0
			ld		de, s_message
			call	puts
			; R#1 = 0x41
			ld		a, 0x41							; 8x8, �g�傷��
			ld		e, 1
			call	write_control_register
			; R#6 = 0x00
			ld		a, 0x00							; �X�v���C�g�p�^�[�����t�H���g�Ɠ����ɂ���
			ld		e, 6
			call	write_control_register
			xor		a, a
	loop:
			push	af
			ld		[s_message2], a
			; put sprite
			ld		hl, 0x1B00
			call	set_vram_write_address
			xor		a, a
			call	write_vram						; Y
			xor		a, a
			call	write_vram						; X
			pop		af
			push	af
			call	write_vram						; pattern
			ld		a, 11
			call	write_vram						; color
			; �Ή����镶��
			ld		hl, 0x1880
			ld		de, s_message2
			call	puts
			call	wait
			pop		af
			inc		a
			jp		nz, loop
			; �L�[�҂�
			call	wait_push_space_key
			ret
	s_message:
			db		"[T001] 8x8 PATTERN TEST", 0
	s_message2:
			db		0, 0
			endscope

; =============================================================================
;	[SCREEN1] �X�v���C�g�p�^�[�� 64��ނ̒�`�̊m�F
;	input:
;		none
;	output:
;		none
;	break:
;		AF
;	comment:
;		none
; =============================================================================
			scope	sp1_pattern_test2
sp1_pattern_test2::
			call	cls
			; Put test name
			ld		hl, 0x18A0
			ld		de, s_message
			call	puts
			; R#1 = 0x43
			ld		a, 0x43							; 16x16, �g�傷��
			ld		e, 1
			call	write_control_register
			; R#6 = 0x00
			ld		a, 0x00							; �X�v���C�g�p�^�[�����t�H���g�Ɠ����ɂ���
			ld		e, 6
			call	write_control_register
			xor		a, a
	loop:
			push	af
			ld		[s_message2], a
			; put sprite
			ld		hl, 0x1B00
			call	set_vram_write_address
			xor		a, a
			call	write_vram						; Y
			xor		a, a
			call	write_vram						; X
			pop		af
			push	af
			call	write_vram						; pattern
			ld		a, 11
			call	write_vram						; color
			; �Ή����镶��
			ld		hl, 0x1880
			ld		de, s_message2
			call	puts
			call	wait
			pop		af
			inc		a
			jp		nz, loop
			; �L�[�҂�
			call	wait_push_space_key
			ret
	s_message:
			db		"[T002] 16x16 PATTERN TEST", 0
	s_message2:
			db		0, 0
			endscope
