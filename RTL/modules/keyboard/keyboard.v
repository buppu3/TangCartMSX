// -----------------------------------------------------------------------------
//	keyboard.v
//	Copyright (C) 2024 Takayuki Hara
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

module ip_keyboard (
	//	internal signals
	input			n_reset,
	input			clk,				//	85.90908MHz
	//	Keymatrix interface (from Physical Keyboard device)
	input			keymatrix_req,
	output			keymatrix_ack,
	input	[3:0]	keymatrix_row,
	input	[7:0]	keymatrix_col,
	//	Keymatrix (to PPI)
	input	[3:0]	ppi_keyboard_row,
	output	[7:0]	ppi_keyboard_col,
	//	LED interface (from PPI, PSG)
	input			caps_led,
	input			kana_led,
	//	LED interface (to Physical LED device)
	output			keyboard_led_req,
	input			keyboard_led_ack,
	output			keyboard_led_caps,
	output			keyboard_led_kana
);
	reg				ff_matrix_req;
	reg				ff_matrix_ack;
	reg		[7:0]	ff_matrix [0:15];
	wire			w_matrix_req;
	wire			w_matrix_ack;
	reg				ff_caps_led;
	reg				ff_kana_led;
	wire			w_caps_led_changed;
	wire			w_kana_led_changed;
	reg				ff_keyboard_led_req;

	// --------------------------------------------------------------------
	//	matrix
	// --------------------------------------------------------------------
	assign w_matrix_req		= keymatrix_req & ~ff_matrix_req;
	assign w_matrix_ack		= ~keymatrix_req & ff_matrix_ack;

	always @( posedge clk ) begin
		if( !n_reset ) begin
			ff_matrix_req	<= 1'b0;
		end
		else begin
			ff_matrix_req	<= keymatrix_req;
		end
	end

	always @( posedge clk ) begin
		if( !n_reset ) begin
			ff_matrix_ack	<= 1'b0;
		end
		else if( w_matrix_ack ) begin
			ff_matrix_ack	<= 1'b0;
		end
		else if( w_matrix_req ) begin
			ff_matrix_ack	<= 1'b1;
		end
		else begin
			//	hold
		end
	end

	always @( posedge clk ) begin
		if( !n_reset ) begin
			ff_matrix[0]	<= 8'hFF;
			ff_matrix[1]	<= 8'hFF;
			ff_matrix[2]	<= 8'hFF;
			ff_matrix[3]	<= 8'hFF;
			ff_matrix[4]	<= 8'hFF;
			ff_matrix[5]	<= 8'hFF;
			ff_matrix[6]	<= 8'hFF;
			ff_matrix[7]	<= 8'hFF;
			ff_matrix[8]	<= 8'hFF;
			ff_matrix[9]	<= 8'hFF;
			ff_matrix[10]	<= 8'hFF;
			ff_matrix[11]	<= 8'hFF;
			ff_matrix[12]	<= 8'hFF;
			ff_matrix[13]	<= 8'hFF;
			ff_matrix[14]	<= 8'hFF;
			ff_matrix[15]	<= 8'hFF;
		end
		else if( w_matrix_req ) begin
			ff_matrix[ keymatrix_row ]	<= keymatrix_col;
		end
		else begin
			//	hold
		end
	end

	// --------------------------------------------------------------------
	//	LED
	// --------------------------------------------------------------------
	always @( posedge clk ) begin
		if( !n_reset ) begin
			ff_caps_led	<= 1'b1;
			ff_kana_led	<= 1'b1;
		end
		else begin
			ff_caps_led	<= caps_led;
			ff_kana_led	<= kana_led;
		end
	end

	assign w_caps_led_changed	= ff_caps_led ^ caps_led;
	assign w_kana_led_changed	= ff_kana_led ^ kana_led;

	always @( posedge clk ) begin
		if( !n_reset ) begin
			ff_keyboard_led_req	<= 1'b0;
		end
		else if( keyboard_led_ack ) begin
			ff_keyboard_led_req	<= 1'b0;
		end
		else if( !ff_keyboard_led_req ) begin
			ff_keyboard_led_req	<= w_caps_led_changed | w_kana_led_changed;
		end
		else begin
			//	hold
		end
	end

	assign keymatrix_ack		= ff_matrix_ack;
	assign ppi_keyboard_col		= ff_matrix[ ppi_keyboard_row ];
	assign keyboard_led_req		= ff_keyboard_led_req;
	assign keyboard_led_caps	= caps_led;
	assign keyboard_led_kana	= kana_led;
endmodule
