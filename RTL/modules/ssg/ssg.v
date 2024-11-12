//
//	ssg_wave.v
//	SSG (YM2149. AY-3-8910 Compatible Processor)
//
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

module ssg (
	input			clk,
	input			reset,
	input			enable,
	input			bus_req,
	output			bus_ack,
	input			bus_wrt,
	input	[15:0]	bus_address,
	input	[7:0]	bus_wdata,
	output	[7:0]	bus_rdata,
	output			bus_rdata_en,

	inout	[5:0]	joystick_port1,
	inout	[5:0]	joystick_port2,
	output			strobe_port1,
	output			strobe_port2,

	input			keyboard_type,		//	PortA bit6: Keyboard type  0: 50���z��, 1: JIS�z�� 
	input			cmt_read,			//	PortA bit7: CMT Read Signal
	output			kana_led,			//	PortB bit7: KANA LED  0: ON, 1: OFF

	output	[7:0]	sound_out
);
	reg				ff_ack;
	reg				ff_ack;
	reg		[7:0]	ff_rdata;
	reg				ff_rdata_en;

	reg		[7:0]	ff_port_a;
	reg		[7:0]	ff_port_b;

	reg		[4:0]	ff_ssg_state;
	reg		[3:0]	ff_ssg_register_ptr;

	reg		[11:0]	ff_ssg_ch_a_counter;
	reg		[11:0]	ff_ssg_ch_b_counter;
	reg		[11:0]	ff_ssg_ch_c_counter;
	reg				ff_ssg_ch_a_tone_wave;
	reg				ff_ssg_ch_b_tone_wave;
	reg				ff_ssg_ch_c_tone_wave;

	reg				ff_ssg_noise;
	reg		[4:0]	ff_ssg_noise_counter;
	reg		[17:0]	ff_ssg_noise_generator;

	wire			w_ssg_tone_disable;
	wire			w_ssg_noise_disable;
	wire			w_ssg_tone_wave;
	wire	[4:0]	w_ssg_volume;
	wire	[3:0]	w_ssg_ch_level;

	reg		[15:0]	ff_ssg_envelope_counter;
	reg		[4:0]	ff_ssg_envelope_ptr;
	reg		[3:0]	ff_ssg_envelope_volume;
	reg				ff_ssg_envelope_req;
	reg				ff_ssg_envelope_ack;

	reg		[11:0]	ff_ssg_ch_a_frequency;
	reg		[11:0]	ff_ssg_ch_b_frequency;
	reg		[11:0]	ff_ssg_ch_c_frequency;
	reg		[4:0]	ff_ssg_noise_frequency;
	reg		[5:0]	ff_ssg_ch_select;
	reg		[4:0]	ff_ssg_ch_a_volume;
	reg		[4:0]	ff_ssg_ch_b_volume;
	reg		[4:0]	ff_ssg_ch_c_volume;
	reg		[15:0]	ff_ssg_envelope_frequency;
	wire	[7:0]	w_out_level;
	reg				ff_hold;
	reg				ff_alternate;
	reg				ff_attack;
	reg				ff_continue;

	reg		[9:0]	ff_ssg_mixer;
	reg		[7:0]	ff_sound_out;

	//--------------------------------------------------------------
	// Miscellaneous control / clock enable (divider)
	//--------------------------------------------------------------
	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_state <= 5'd0;
		end
		else if( enable ) begin
			ff_ssg_state <= ff_ssg_state - 5'd1;
		end
		else begin
			//	hold
		end
	end

	// -------------------------------------------------------------
	// Interface port
	// -------------------------------------------------------------
	always @( posedge clk ) begin
		if( reset ) begin
			ff_port_a		<= 8'd0;
		end
		else begin
			ff_port_a[5:0]	<= ff_port_b[6] ? joystick_port2 : joystick_port1;
			ff_port_a[6]	<= keyboard_type;
			ff_port_a[7]	<= cmt_read;
		end
	end

	always @( posedge clk ) begin
		if( reset ) begin
			ff_port_b <= 8'd0;
		end
		else if( bus_req && bus_wrt && ff_ack && bus_address == 2'd1 && ff_register_ptr == 4'd15 ) begin
			ff_port_b <= bus_wdata;
		end
		else begin
			//	hold
		end
	end

	assign bus_ack				= ff_ack;
	assign bus_rdata			= ff_rdata;
	assign bus_rdata_en			= ff_rdata_en;

	assign strobe_port1			= ff_port_b[4];
	assign strobe_port2			= ff_port_b[5];
	assign kana_led				= ff_port_b[7];

	assign joystick_port1[5:4]	= ( ff_port_b[1:0] == 2'd0 ) ? 2'b00 :
								  ( ff_port_b[1:0] == 2'd1 ) ? 2'b0z :
								  ( ff_port_b[1:0] == 2'd2 ) ? 2'bz0 : 2'bzz;

	assign joystick_port2[5:4]	= ( ff_port_b[3:2] == 2'd0 ) ? 2'b00 :
								  ( ff_port_b[3:2] == 2'd1 ) ? 2'b0z :
								  ( ff_port_b[3:2] == 2'd2 ) ? 2'bz0 : 2'bzz;

	assign sound_out			= ff_sound_out;

	// -------------------------------------------------------------
	// Register access
	// -------------------------------------------------------------
	always @( posedge clk ) begin
		if( reset ) begin
			ff_ack <= 1'b0;
		end
		else if( ff_ack ) begin
			ff_ack <= 1'b0;
		end
		else if( bus_req ) begin
			ff_ack <= 1'b1;
		end
		else begin
			//	hold
		end
	end

	//--------------------------------------------------------------
	// Register read
	//--------------------------------------------------------------
	always @( posedge clk ) begin
		if( reset ) begin
			ff_rdata_en <= 1'b0;
		end
		else if( bus_req && !bus_wrt && ff_ack ) begin
			ff_rdata_en <= 1'b1;
		end
		else begin
			ff_rdata_en <= 1'b0;
		end
	end

	always @( posedge clk ) begin
		if( reset ) begin
			ff_rdata <= 8'd0;
		end
		else if( bus_req && !bus_wrt && ff_ack ) begin
			if( bus_address[1:0] == 2'd2 ) begin
				case( ff_ssg_register_ptr )
				4'd0:		ff_rdata <= ff_ssg_ch_a_frequency[7:0];
				4'd1:		ff_rdata <= { 4'd0, ff_ssg_ch_a_frequency[11:8] };
				4'd2:		ff_rdata <= ff_ssg_ch_b_frequency[7:0];
				4'd3:		ff_rdata <= { 4'd0, ff_ssg_ch_b_frequency[11:8] };
				4'd4:		ff_rdata <= ff_ssg_ch_c_frequency[7:0];
				4'd5:		ff_rdata <= { 4'd0, ff_ssg_ch_c_frequency[11:8] };
				4'd6:		ff_rdata <= { 3'd0, ff_ssg_noise_frequency };
				4'd7:		ff_rdata <= { 2'd2, ff_ssg_ch_select };
				4'd8:		ff_rdata <= { 3'd0, ff_ssg_ch_a_volume };
				4'd9:		ff_rdata <= { 3'd0, ff_ssg_ch_b_volume };
				4'd10:		ff_rdata <= { 3'd0, ff_ssg_ch_c_volume };
				4'd11:		ff_rdata <= ff_ssg_envelope_frequency[7:0];
				4'd12:		ff_rdata <= ff_ssg_envelope_frequency[15:8];
				4'd13:		ff_rdata <= { 4'd0, ff_continue, ff_attack, ff_alternate, ff_hold };
				4'd14:		ff_rdata <= ff_port_a;
				4'd15:		ff_rdata <= ff_port_b;
				default:	ff_rdata <= 8'hFF;
				endcase
			end
			else begin
				ff_rdata <= 8'hFF;
			end
		end
		else begin
			ff_rdata <= 1'b0;
		end
	end

	//--------------------------------------------------------------
	// Register write
	//--------------------------------------------------------------
	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_register_ptr			<= 4'd0;
			ff_ssg_ch_a_frequency		<= 12'hFFF;
			ff_ssg_ch_b_frequency		<= 12'hFFF;
			ff_ssg_ch_c_frequency		<= 12'hFFF;
			ff_ssg_noise_frequency		<= 5'd0;
			ff_ssg_ch_select			<= 6'd0;
			ff_ssg_ch_a_volume			<= 5'd0;
			ff_ssg_ch_b_volume			<= 5'd0;
			ff_ssg_ch_c_volume			<= 5'd0;
			ff_ssg_envelope_frequency	<= 16'hFFFF;
			ff_hold 					<= 1'b1;
			ff_alternate				<= 1'b1;
			ff_attack					<= 1'b1;
			ff_continue					<= 1'b1;
			ff_ssg_envelope_req			<= 1'b0;
		end
		else if( bus_req && bus_wrt && bus_ack && bus_address[1:0] == 2'd0 ) begin
			ff_ssg_register_ptr <= bus_wdata[3:0];
		end
		else if( bus_req && bus_wrt && bus_ack && bus_address[1:0] == 2'd1 ) begin
			case( ff_ssg_register_ptr )
			4'd0:		ff_ssg_ch_a_frequency[7:0]		<= bus_wdata;
			4'd1:		ff_ssg_ch_a_frequency[11:8]		<= bus_wdata[3:0];
			4'd2:		ff_ssg_ch_b_frequency[7:0]		<= bus_wdata;
			4'd3:		ff_ssg_ch_b_frequency[11:8]		<= bus_wdata[3:0];
			4'd4:		ff_ssg_ch_c_frequency[7:0]		<= bus_wdata;
			4'd5:		ff_ssg_ch_c_frequency[11:8]		<= bus_wdata[3:0];
			4'd6:		ff_ssg_noise_frequency			<= bus_wdata[4:0];
			4'd7:		ff_ssg_ch_select				<= bus_wdata[5:0];
			4'd8:		ff_ssg_ch_a_volume				<= bus_wdata[4:0];
			4'd9:		ff_ssg_ch_b_volume				<= bus_wdata[4:0];
			4'd10:		ff_ssg_ch_c_volume				<= bus_wdata[4:0];
			4'd11:		ff_ssg_envelope_frequency[7:0]	<= bus_wdata;
			4'd12:		ff_ssg_envelope_frequency[15:8]	<= bus_wdata;
			4'd13:
				begin
					ff_hold							<= bus_wdata[0];
					ff_alternate					<= bus_wdata[1];
					ff_attack						<= bus_wdata[2];
					ff_continue						<= bus_wdata[3];
					ff_ssg_envelope_req				<= ~ff_ssg_envelope_ack;
				end
			default:
				begin
					//	hold
				end
			endcase
		end
	end

	//--------------------------------------------------------------
	// Tone generator (Port A)
	//--------------------------------------------------------------
	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_ch_a_counter		<= 12'd0;
		end
		else if( enable && (ff_ssg_state[3:0] == 4'd0) ) begin
			if( ff_ssg_ch_a_counter != 12'd0) begin
				ff_ssg_ch_a_counter <= ff_ssg_ch_a_counter - 12'd1;
			end
			else if( ff_ssg_ch_a_frequency != 12'd0 ) begin
				ff_ssg_ch_a_counter <= ff_ssg_ch_a_frequency - 12'd1;
			end
		end
	end

	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_ch_a_tone_wave	<= 1'b0;
		end
		else if( enable && (ff_ssg_state[3:0] == 4'd0) ) begin
			if( ff_ssg_ch_a_counter == 12'd0 ) begin
				ff_ssg_ch_a_tone_wave <= ~ff_ssg_ch_a_tone_wave;
			end
			else begin
				//	hold
			end
		end
	end

	//--------------------------------------------------------------
	// Tone generator (Port B)
	//--------------------------------------------------------------
	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_ch_b_counter		<= 12'd0;
		end
		else if( enable && (ff_ssg_state[3:0] == 4'd0) ) begin
			if( ff_ssg_ch_b_counter != 12'd0) begin
				ff_ssg_ch_b_counter <= ff_ssg_ch_b_counter - 12'd1;
			end
			else if( ff_ssg_ch_b_frequency != 12'd0 ) begin
				ff_ssg_ch_b_counter <= ff_ssg_ch_b_frequency - 12'd1;
			end
		end
	end

	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_ch_b_tone_wave	<= 1'b0;
		end
		else if( enable && (ff_ssg_state[3:0] == 4'd0) ) begin
			if( ff_ssg_ch_b_counter == 12'd0 ) begin
				ff_ssg_ch_b_tone_wave <= ~ff_ssg_ch_b_tone_wave;
			end
			else begin
				//	hold
			end
		end
	end

	//--------------------------------------------------------------
	// Tone generator (Port C)
	//--------------------------------------------------------------
	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_ch_c_counter		<= 12'd0;
		end
		else if( enable && (ff_ssg_state[3:0] == 4'd0) ) begin
			if( ff_ssg_ch_c_counter != 12'd0 ) begin
				ff_ssg_ch_c_counter <= ff_ssg_ch_c_counter - 12'd1;
			end
			else if( ff_ssg_ch_c_frequency != 12'd0 ) begin
				ff_ssg_ch_c_counter <= ff_ssg_ch_c_frequency - 12'd1;
			end
		end
	end

	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_ch_c_tone_wave	<= 1'b0;
		end
		else if( enable && (ff_ssg_state[3:0] == 4'd0) ) begin
			if( ff_ssg_ch_c_counter == 12'd0 ) begin
				ff_ssg_ch_c_tone_wave <= ~ff_ssg_ch_c_tone_wave;
			end
			else begin
				//	hold
			end
		end
	end

	//--------------------------------------------------------------
	// Noise generator
	//--------------------------------------------------------------
	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_noise_counter <= 5'd0;
		end
		else begin
			if( enable && (ff_ssg_state == 5'd0) ) begin
				if( ff_ssg_noise_counter != 5'd0 ) begin
					ff_ssg_noise_counter <= ff_ssg_noise_counter - 1;
				end
				else if( ff_ssg_noise_frequency != 5'd0 ) begin
					ff_ssg_noise_counter <= ff_ssg_noise_frequency - 1;
				end
			end
		end
	end

	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_noise_generator	<= 18'd0;
		end
		else begin
			if( enable && (ff_ssg_state == 5'd0) ) begin
				if( ff_ssg_noise_counter == 5'd0 ) begin
					if( ff_ssg_noise_generator == 18'd0 ) begin
						ff_ssg_noise_generator[0]		<= 1'b1;
						ff_ssg_noise_generator[17:1]	<= ff_ssg_noise_generator[16:0];
					end
					else begin
						ff_ssg_noise_generator[0]		<= ff_ssg_noise_generator[16] ^ ff_ssg_noise_generator[13];
						ff_ssg_noise_generator[17:1]	<= ff_ssg_noise_generator[16:0];
					end
				end
			end
		end
	end

	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_noise <= 1'b0;
		end
		else if( enable ) begin
			ff_ssg_noise <= ff_ssg_noise_generator[17];
		end
		else begin
			//	hold
		end
	end

	//--------------------------------------------------------------
	// Envelope generator
	//--------------------------------------------------------------
	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_envelope_counter	<= 16'd0;
		end
		else if( enable && ff_ssg_state == 5'd0 ) begin
			// Envelope period counter
			if( ff_ssg_envelope_counter != 16'd0 && ff_ssg_envelope_req == ff_ssg_envelope_ack ) begin
				ff_ssg_envelope_counter <= ff_ssg_envelope_counter - 1;
			end
			else if( ff_ssg_envelope_frequency != 16'd0) begin
				ff_ssg_envelope_counter <= ff_ssg_envelope_frequency - 1;
			end
		end
		else begin
			//	hold
		end
	end

	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_envelope_ptr		<= 5'b11111;
		end
		else if( enable && ff_ssg_state == 5'd0 ) begin
			if( ff_ssg_envelope_req != ff_ssg_envelope_ack ) begin
				ff_ssg_envelope_ptr <= 5'b11111;
			end
			else if( ff_ssg_envelope_counter == 16'd0 && (ff_ssg_envelope_ptr[4] || (!ff_hold && ff_continue)) ) begin
				ff_ssg_envelope_ptr <= ff_ssg_envelope_ptr - 1;
			end
		end
		else begin
			//	hold
		end
	end

	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_envelope_volume	<= 4'd0;
		end
		else if( enable && ff_ssg_state == 5'd0 ) begin
			if( !ff_ssg_envelope_ptr[4] && !ff_continue ) begin
				ff_ssg_envelope_volume <= 4'd0;
			end
			else if( ff_ssg_envelope_ptr[4] || !(ff_alternate ^ ff_hold) ) begin
				ff_ssg_envelope_volume <= ff_attack ? ~ff_ssg_envelope_ptr : ff_ssg_envelope_ptr;
			end
			else begin
				ff_ssg_envelope_volume <= ff_attack ? ff_ssg_envelope_ptr : ~ff_ssg_envelope_ptr;
			end
		end
		else begin
			//	hold
		end
	end

	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_envelope_ack		<= 1'b0;
		end
		else if( enable && ff_ssg_state == 5'd0 ) begin
			ff_ssg_envelope_ack <= ff_ssg_envelope_req;
		end
		else begin
			//	hold
		end
	end

	//--------------------------------------------------------------
	// Mixer control
	//--------------------------------------------------------------
	function [7:0] func_out_level(
		input	[3:0]	w_ssg_ch_level
	);
		case( w_ssg_ch_level )
		4'd15:		func_out_level = 8'd255;	//	1V
		4'd14:		func_out_level = 8'd180;	//	1V / (sqrt(2) **  1)
		4'd13:		func_out_level = 8'd127;	//	1V / (sqrt(2) **  2)
		4'd12:		func_out_level = 8'd90;		//	1V / (sqrt(2) **  3)
		4'd11:		func_out_level = 8'd63;		//	1V / (sqrt(2) **  4)
		4'd10:		func_out_level = 8'd45;		//	1V / (sqrt(2) **  5)
		4'd9:		func_out_level = 8'd31;		//	1V / (sqrt(2) **  6)
		4'd8:		func_out_level = 8'd22;		//	1V / (sqrt(2) **  7)
		4'd7:		func_out_level = 8'd15;		//	1V / (sqrt(2) **  8)
		4'd6:		func_out_level = 8'd11;		//	1V / (sqrt(2) **  9)
		4'd5:		func_out_level = 8'd7;		//	1V / (sqrt(2) ** 10)
		4'd4:		func_out_level = 8'd5;		//	1V / (sqrt(2) ** 11)
		4'd3:		func_out_level = 8'd3;		//	1V / (sqrt(2) ** 12)
		4'd2:		func_out_level = 8'd2;		//	1V / (sqrt(2) ** 13)
		4'd1:		func_out_level = 8'd1;		//	1V / (sqrt(2) ** 14)
		default:	func_out_level = 8'd0;		//	1V / (sqrt(2) ** 15)
		endcase
	endfunction

	assign w_ssg_tone_disable	= ( ff_ssg_state[1:0] == 2'd3 ) ? ff_ssg_ch_select[0] : 
								  ( ff_ssg_state[1:0] == 2'd2 ) ? ff_ssg_ch_select[1] : 
								  ( ff_ssg_state[1:0] == 2'd1 ) ? ff_ssg_ch_select[2] : 1'b1;

	assign w_ssg_noise_disable	= ( ff_ssg_state[1:0] == 2'd3 ) ? ff_ssg_ch_select[3] : 
								  ( ff_ssg_state[1:0] == 2'd2 ) ? ff_ssg_ch_select[4] : 
								  ( ff_ssg_state[1:0] == 2'd1 ) ? ff_ssg_ch_select[5] : 1'b1;

	assign w_ssg_tone_wave		= ( ff_ssg_state[1:0] == 2'd3 ) ? ff_ssg_ch_a_tone_wave : 
								  ( ff_ssg_state[1:0] == 2'd2 ) ? ff_ssg_ch_b_tone_wave : 
								  ( ff_ssg_state[1:0] == 2'd1 ) ? ff_ssg_ch_c_tone_wave : 1'b1;

	assign w_ssg_volume			= ( ff_ssg_state[1:0] == 2'd3 ) ? ff_ssg_ch_a_volume : 
								  ( ff_ssg_state[1:0] == 2'd2 ) ? ff_ssg_ch_b_volume : 
								  ( ff_ssg_state[1:0] == 2'd1 ) ? ff_ssg_ch_c_volume : 5'd0;

	assign w_ssg_ch_level		= ( !( w_ssg_tone_disable  || w_ssg_tone_wave ) ) ? 4'd0 :
								  ( !( w_ssg_noise_disable || ff_ssg_noise    ) ) ? 4'd0 :
								  ( w_ssg_volume[4]                             ) ? ff_ssg_envelope_volume : w_ssg_volume[3:0];

	assign w_out_level			= func_out_level( w_ssg_ch_level );

	always @( posedge clk ) begin
		if( reset ) begin
			ff_ssg_mixer	<= 10'd0;
		end
		else if( enable ) begin
			if( ff_ssg_state[1:0] == 2'd0 ) begin
				ff_ssg_mixer[9:2]	<= 8'd0;
			end
			else begin
				ff_ssg_mixer		<= { 2'b00, w_out_level } + ff_ssg_mixer;
			end
		end
	end

	always @( posedge clk ) begin
		if( reset ) begin
			ff_sound_out	 <= 8'd0;
		end
		else if( enable ) begin
			if( ff_ssg_state[1:0] == 2'd0 ) begin
				ff_sound_out	<= ff_ssg_mixer[9:2];
			end
			else begin
				//	hold
			end
		end
	end
endmodule
