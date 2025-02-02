//
//	R80 Registers
//	Copyright (c) 2024 Takayuki Hara
//
//	�{�\�t�g�E�F�A����і{�\�t�g�E�F�A�Ɋ�Â��č쐬���ꂽ�h�����́A�ȉ��̏�����
//	�������ꍇ�Ɍ���A�ĔЕz����юg�p��������܂��B
//
//	1.�\�[�X�R�[�h�`���ōĔЕz����ꍇ�A��L�̒��쌠�\���A�{�����ꗗ�A����щ��L
//	  �Ɛӏ��������̂܂܂̌`�ŕێ����邱�ƁB
//	2.�o�C�i���`���ōĔЕz����ꍇ�A�Еz���ɕt���̃h�L�������g���̎����ɁA��L��
//	  ���쌠�\���A�{�����ꗗ�A����щ��L�Ɛӏ������܂߂邱�ƁB
//	3.���ʂɂ�鎖�O�̋��Ȃ��ɁA�{�\�t�g�E�F�A��̔��A����я��ƓI�Ȑ��i�⊈��
//	  �Ɏg�p���Ȃ����ƁB
//
//	�{�\�t�g�E�F�A�́A���쌠�҂ɂ���āu����̂܂܁v�񋟂���Ă��܂��B���쌠�҂́A
//	����ړI�ւ̓K�����̕ۏ؁A���i���̕ۏ؁A�܂�����Ɍ��肳��Ȃ��A�����Ȃ閾��
//	�I�������͈ÖقȕۏؐӔC�������܂���B���쌠�҂́A���R�̂�������킸�A���Q
//	�����̌�����������킸�A���ӔC�̍������_��ł��邩���i�ӔC�ł��邩�i�ߎ�
//	���̑��́j�s�@�s�ׂł��邩���킸�A���ɂ��̂悤�ȑ��Q����������\����m��
//	����Ă����Ƃ��Ă��A�{�\�t�g�E�F�A�̎g�p�ɂ���Ĕ��������i��֕i�܂��͑�p�T
//	�[�r�X�̒��B�A�g�p�̑r���A�f�[�^�̑r���A���v�̑r���A�Ɩ��̒��f���܂߁A�܂���
//	��Ɍ��肳��Ȃ��j���ڑ��Q�A�Ԑڑ��Q�A�����I�ȑ��Q�A���ʑ��Q�A�����I���Q�A��
//	���͌��ʑ��Q�ɂ��āA��ؐӔC�𕉂�Ȃ����̂Ƃ��܂��B
//
//	Note that above Japanese version license is the formal document.
//	The following translation is only for reference.
//
//	Redistribution and use of this software or any derivative works,
//	are permitted provided that the following conditions are met:
//
//	1. Redistributions of source code must retain the above copyright
//	   notice, this list of conditions and the following disclaimer.
//	2. Redistributions in binary form must reproduce the above
//	   copyright notice, this list of conditions and the following
//	   disclaimer in the documentation and/or other materials
//	   provided with the distribution.
//	3. Redistributions may not be sold, nor may they be used in a
//	   commercial product or activity without specific prior written
//	   permission.
//
//	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//	POSSIBILITY OF SUCH DAMAGE.
//-----------------------------------------------------------------------------

module cz80_registers (
	input			reset_n,
	input			clk,
	input			cen,
	input			we_h,
	input			we_l,
	input	[2:0]	address_a,
	input	[2:0]	address_b,
	input	[2:0]	address_c,
	input	[7:0]	wdata_h,
	input	[7:0]	wdata_l,
	output	[7:0]	rdata_ah,
	output	[7:0]	rdata_al,
	output	[7:0]	rdata_bh,
	output	[7:0]	rdata_bl,
	output	[7:0]	rdata_ch,
	output	[7:0]	rdata_cl
);
	reg		[7:0]	reg_b0;
	reg		[7:0]	reg_d0;
	reg		[7:0]	reg_h0;
	reg		[7:0]	reg_ixh;
	reg		[7:0]	reg_b1;
	reg		[7:0]	reg_d1;
	reg		[7:0]	reg_h1;
	reg		[7:0]	reg_iyh;

	reg		[7:0]	reg_c0;
	reg		[7:0]	reg_e0;
	reg		[7:0]	reg_l0;
	reg		[7:0]	reg_ixl;
	reg		[7:0]	reg_c1;
	reg		[7:0]	reg_e1;
	reg		[7:0]	reg_l1;
	reg		[7:0]	reg_iyl;

	always @( posedge clk ) begin
		if( !reset_n ) begin
			reg_b0		<= 8'h00;
			reg_d0		<= 8'h00;
			reg_h0		<= 8'h00;
			reg_ixh		<= 8'h00;
			reg_b1		<= 8'h00;
			reg_d1		<= 8'h00;
			reg_h1		<= 8'h00;
			reg_iyh		<= 8'h00;
		end
		else if( cen ) begin
			if( we_h ) begin
				case( address_a )
				3'd0:		reg_b0		<= wdata_h;
				3'd1:		reg_d0		<= wdata_h;
				3'd2:		reg_h0		<= wdata_h;
				3'd3:		reg_ixh		<= wdata_h;
				3'd4:		reg_b1		<= wdata_h;
				3'd5:		reg_d1		<= wdata_h;
				3'd6:		reg_h1		<= wdata_h;
				default:	reg_iyh		<= wdata_h;
				endcase
			end
		end
	end

	always @( posedge clk ) begin
		if( !reset_n ) begin
			reg_c0		<= 8'h00;
			reg_e0		<= 8'h00;
			reg_l0		<= 8'h00;
			reg_ixl		<= 8'h00;
			reg_c1		<= 8'h00;
			reg_e1		<= 8'h00;
			reg_l1		<= 8'h00;
			reg_iyl		<= 8'h00;
		end
		if( cen ) begin
			if( we_l ) begin
				case( address_a )
				3'd0:		reg_c0		<= wdata_l;
				3'd1:		reg_e0		<= wdata_l;
				3'd2:		reg_l0		<= wdata_l;
				3'd3:		reg_ixl		<= wdata_l;
				3'd4:		reg_c1		<= wdata_l;
				3'd5:		reg_e1		<= wdata_l;
				3'd6:		reg_l1		<= wdata_l;
				default:	reg_iyl		<= wdata_l;
				endcase
			end
		end
	end

	function [7:0] register_sel(
		input	[2:0]	address,
		input	[7:0]	reg_c0,
		input	[7:0]	reg_d0,
		input	[7:0]	reg_l0,
		input	[7:0]	reg_x0,
		input	[7:0]	reg_c1,
		input	[7:0]	reg_d1,
		input	[7:0]	reg_l1,
		input	[7:0]	reg_x1
	);
		case( address )
		3'd0:		register_sel = reg_c0;
		3'd1:		register_sel = reg_d0;
		3'd2:		register_sel = reg_l0;
		3'd3:		register_sel = reg_x0;
		3'd4:		register_sel = reg_c1;
		3'd5:		register_sel = reg_d1;
		3'd6:		register_sel = reg_l1;
		default:	register_sel = reg_x1;
		endcase
	endfunction

	assign rdata_ah = register_sel( address_a, reg_b0, reg_d0, reg_h0, reg_ixh, reg_b1, reg_d1, reg_h1, reg_iyh );
	assign rdata_al = register_sel( address_a, reg_c0, reg_e0, reg_l0, reg_ixl, reg_c1, reg_e1, reg_l1, reg_iyl );
	assign rdata_bh = register_sel( address_b, reg_b0, reg_d0, reg_h0, reg_ixh, reg_b1, reg_d1, reg_h1, reg_iyh );
	assign rdata_bl = register_sel( address_b, reg_c0, reg_e0, reg_l0, reg_ixl, reg_c1, reg_e1, reg_l1, reg_iyl );
	assign rdata_ch = register_sel( address_c, reg_b0, reg_d0, reg_h0, reg_ixh, reg_b1, reg_d1, reg_h1, reg_iyh );
	assign rdata_cl = register_sel( address_c, reg_c0, reg_e0, reg_l0, reg_ixl, reg_c1, reg_e1, reg_l1, reg_iyl );
endmodule
