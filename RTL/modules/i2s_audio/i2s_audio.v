//
//	i2c_audio.v
//	i2c DAC for Audio
//
//	Copyright (C) 2025 Takayuki Hara
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
//
//-----------------------------------------------------------------------------

module i2s_audio(
	input			clk,				//	42.95454MHz
	input			reset_n,
	input	[15:0]	sound_in,
	output			i2s_audio_en,
	output			i2s_audio_din,
	output			i2s_audio_lrclk,
	output			i2s_audio_bclk
);
	localparam		c_512khz	= 8'd83;
	reg		[7:0]	ff_divider;
	wire			w_512khz_pulse;
	reg				ff_bclk;
	reg				ff_lrclk;
	reg		[3:0]	ff_bit_count;
	reg		[15:0]	ff_shift_reg;

	assign i2s_audio_en		= 1'b1;

	always @( posedge clk ) begin
		if( !reset_n ) begin
			ff_divider <= 8'd0;
		end
		else if( w_512khz_pulse ) begin
			ff_divider <= 8'd0;
		end
		else begin
			ff_divider <= ff_divider + 8'd1;
		end
	end

	assign w_512khz_pulse	= (ff_divider == c_512khz);

	always @( posedge clk ) begin
		if( !reset_n ) begin
			ff_bclk <= 1'b0;
		end
		else if( w_512khz_pulse ) begin
			ff_bclk <= ~ff_bclk;
		end
	end

	always @( posedge clk ) begin
		if( !reset_n ) begin
			ff_lrclk <= 1'b0;
		end
		else if( w_512khz_pulse && ff_bit_count == 4'd15 ) begin
			ff_lrclk <= ~ff_lrclk;
		end
	end

	always @( posedge clk ) begin
		if( !reset_n ) begin
			ff_bit_count <= 4'd0;
		end
		else if( w_512khz_pulse && !ff_bclk ) begin
			ff_bit_count <= ff_bit_count + 4'd1;
		end
	end

	always @( posedge clk ) begin
		if( !reset_n ) begin
			ff_shift_reg <= 16'd0;
		end
		else if( w_512khz_pulse && !ff_bclk ) begin
			if( ff_bit_count == 4'd15 ) begin
				ff_shift_reg <= sound_in;
			end
			else begin
				ff_shift_reg <= { ff_shift_reg[14:0], 1'b0 };
			end
		end
	end

	assign i2s_audio_din	= ff_shift_reg[15];
endmodule
