; =============================================================================
;	V9968 software test program
; -----------------------------------------------------------------------------
;	Programmed by t.hara (HRA!)
; =============================================================================

			org		0x100

h_keyi		:= 0xFD9A

start:
			; ����
			call	vdp_io_select
			call	copy_rom_font
			; �e�X�g
			di
			call	screen1
			call	sc1_interrupt
			ei
			; ��n��
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
			; R#8 = 0x02
			ld		a, 0x02					; �X�v���C�g��\��
			ld		e, 8
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
			ld		hl, 0x1800
			ld		bc, 32 * 26
			call	fill_increment
			; Font ���Z�b�g
			ld		hl, 0x0000
			call	set_font
			; Font�̐F���Z�b�g
			ld		hl, 0x2000
			ld		bc, 256 / 8
			ld		e, 0xF4
			call	fill_vram
			ret
			endscope

; =============================================================================
;	SCREEN1 interrupt test
;	input:
;		none
;	output:
;		none
;	break:
;		AF
;	comment:
;		none
; =============================================================================
			scope	sc1_interrupt
sc1_interrupt::
			di
			; �{�̑��� VDP �����荞�݂𔭐������Ȃ��悤�ɋ֎~�ɂ���
			ld		c, 0x99
			; ���������������֎~����(R#1)
			ld		a, 0x40
			out		[c], a
			ld		a, 0x81
			out		[c], a
			; ���������荞�݂��֎~����(R#0)
			ld		a, 0x00
			out		[c], a
			ld		a, 0x80
			out		[c], a
			; ���荞�ݏ������[�`���� 4000h �֓]������ (H_KEYI ���ĂԂƂ��� page0 �� MAIN-ROM �ɐ؂�ւ���Ă���)
			ld		hl, h_keyi_new_org
			ld		de, h_keyi_new
			ld		bc, h_keyi_new_end - h_keyi_new
			ldir
			; ���������荞�݂̃��C���ԍ����w�肷��
			ld		a, [io_vdp_port1]
			ld		[my_vdp_port1], a
			ld		c, a
			ld		a, 100
			out		[c], a
			ld		a, 0x80 + 19
			out		[c], a
			; ��������������������(R#1)
			ld		a, 0x60
			out		[c], a
			ld		a, 0x81
			out		[c], a
			; ���荞�݃t�b�N��ݒ�
			ld		a, 0xC3			; JP
			ld		hl, h_keyi_new
			ld		[h_keyi + 0], a
			ld		[h_keyi + 1], hl
			; �{�^�������҂�
			ei
			call	wait_push_space_key
			di
			; ���������荞�݂�������(R#0)
			ld		a, 0x10
			out		[c], a
			ld		a, 0x80
			out		[c], a
			; �{�^�������҂�
			ei
			call	wait_push_space_key
			di
			; ���荞�݃t�b�N���~
			ld		a, 0xC9			; RET
			ld		[h_keyi + 0], a
			; ���������荞�݂��֎~����(R#0)
			ld		a, 0x00
			out		[c], a
			ld		a, 0x80
			out		[c], a
			; ���������������֎~����(R#1)
			ld		a, 0x40
			out		[c], a
			ld		a, 0x81
			out		[c], a
			ret

	h_keyi_new_org:
			org		0x4000
	h_keyi_new:
			; ���������荞�݂��ǂ������`�F�b�N
			; S#2
			ld		a, [my_vdp_port1]
			ld		c, a
			ld		a, 1
			out		[c], a
			ld		a, 0x8F
			out		[c], a
			in		a, [c]
			and		a, 1
			jp		nz, line_interrupt
			; R#15 = 0 �ɖ߂�
			out		[c], a
			ld		a, 0x8F
			out		[c], a
			; S#0 ��ǂ�
			in		a, [c]
			and		a, 0x80
			ret		z
			; ������������
	frame_interrupt:
			; �X�N���[���ʒu�𓮂���
			ld		a, [scroll1]
			inc		a
			ld		[scroll1], a

			ld		b, a
			rrca
			rrca
			rrca
			cpl
			and		a, 0x1F
			out		[c], a
			ld		a, 0x80 + 26
			out		[c], a
			out		[c], b
			ld		a, 0x80 + 27
			out		[c], a
			ret
			; ����������
	line_interrupt:
			; �X�N���[���ʒu�𓮂���
			ld		a, [scroll2]
			dec		a
			ld		[scroll2], a

			ld		b, a
			rrca
			rrca
			rrca
			cpl
			and		a, 0x1F
			out		[c], a
			ld		a, 0x80 + 26
			out		[c], a
			out		[c], b
			ld		a, 0x80 + 27
			out		[c], a
			; R#15 = 0 �ɖ߂�
			xor		a, a
			out		[c], a
			ld		a, 0x8F
			out		[c], a
			ret
	my_vdp_port1:
			db		0
	scroll1:
			db		0
	scroll2:
			db		0
	h_keyi_new_end:
			org		h_keyi_new_org + (h_keyi_new_end - h_keyi_new)
			endscope
