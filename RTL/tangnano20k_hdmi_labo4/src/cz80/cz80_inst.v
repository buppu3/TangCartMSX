//
//	Z80 compatible microprocessor core, asynchronous top level
//	Copyright (c) 2002 Daniel Wallner (jesus@opencores.org)
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
//	This module is based on T80(Version : 0250_T80) by Daniel Wallner and 
//	modified by Takayuki Hara.
//
//	The following modifications have been made.
//	-- Convert VHDL code to Verilog code.
//	-- Some minor bug fixes.
//-----------------------------------------------------------------------------

module cz80_inst (
	input			reset_n		,
	input			clk_n		,
	input			enable		,
	input			wait_n		,
	input			int_n		,
	input			nmi_n		,
	input			busrq_n		,
	output			m1_n		,
	output			mreq_n		,
	output			iorq_n		,
	output			rd_n		,
	output			wr_n		,
	output			rfsh_n		,
	output			halt_n		,
	output			busak_n		,
	output	[15:0]	a			,
	inout	[7:0]	d			
);
	reg					ff_reset_n;
	wire				w_intcycle_n;
	wire				w_iorq;
	wire				w_noread;
	wire				w_write;
	reg					ff_mreq;
	reg					ff_mreq_inhibit;
	reg					ff_ireq_inhibit;
	reg					ff_req_inhibit;
	reg					ff_rd;
	wire				w_mreq_n_i;
	reg					ff_iorq_n_i;
	wire				w_rd_n_i;
	reg					ff_wr_n_i;
	wire				w_wr_n_j;
	wire				w_rfsh_n_i;
	wire				w_busak_n_i;
	wire	[15:0]		w_a_i;
	wire	[7:0]		w_di;
	wire	[7:0]		w_do;
	reg		[7:0]		ff_di_reg;
	reg		[7:0]		ff_dinst;
	reg					ff_wait_n;
	wire	[2:0]		w_m_cycle;
	wire	[2:0]		w_t_state;
	wire				w_m1_n;

	assign busak_n		= w_busak_n_i;
	assign w_mreq_n_i	= ~ff_mreq | (ff_req_inhibit & ff_mreq_inhibit);
	assign w_rd_n_i		= ~ff_rd | ff_req_inhibit;
	assign w_wr_n_j		= ff_wr_n_i;
	assign w_di			= ff_dinst;

	assign mreq_n		= w_busak_n_i ? w_mreq_n_i							: 1'bz;
	assign iorq_n		= w_busak_n_i ? (ff_iorq_n_i | ff_ireq_inhibit)		: 1'bz;
	assign rd_n			= w_busak_n_i ? w_rd_n_i							: 1'bz;
	assign wr_n			= w_busak_n_i ? w_wr_n_j							: 1'bz;
	assign rfsh_n		= w_busak_n_i ? w_rfsh_n_i							: 1'bz;
	assign a			= w_busak_n_i ? w_a_i								: 16'dz;
	assign d			= ( w_write && w_busak_n_i ) ? w_do					: 8'dz;

	always @( posedge clk_n ) begin
		if( !reset_n ) begin
			ff_reset_n <= 1'b0;
		end
		else begin
			ff_reset_n <= 1'b1;
		end
	end

	cz80 u_cz80 (
		.reset_n		( reset_n			),
		.clk_n			( clk_n				),
		.cen			( enable			),
		.wait_n			( ff_wait_n			),
		.int_n			( int_n				),
		.nmi_n			( nmi_n				),
		.busrq_n		( busrq_n			),
		.m1_n			( w_m1_n			),
		.iorq			( w_iorq			),
		.noread			( w_noread			),
		.write			( w_write			),
		.rfsh_n			( w_rfsh_n_i		),
		.halt_n			( halt_n			),
		.busak_n		( w_busak_n_i		),
		.a				( w_a_i				),
		.dinst			( w_di				),
		.di				( ff_di_reg			),
		.do				( w_do				),
		.mc				( w_m_cycle			),
		.ts				( w_t_state			),
		.intcycle_n		( w_intcycle_n		),
		.inte			( 					),
		.stop			( 					)
	);

	assign m1_n		= w_m1_n;

	always @( negedge clk_n ) begin
		if( !ff_reset_n ) begin
			ff_dinst <= 8'd0;
		end
		else if( !w_rd_n_i ) begin
			ff_dinst <= d;
		end
	end

	always @( negedge clk_n ) begin
		ff_wait_n			<= wait_n;
	end

	always @( negedge clk_n ) begin
		if( !ff_reset_n ) begin
			ff_di_reg <= 8'd0;
		end
		else if( !w_rd_n_i && w_t_state == 3'd3 && w_busak_n_i ) begin
			ff_di_reg <= d;
		end
	end

	always @( posedge clk_n ) begin
		ff_ireq_inhibit		<= ~w_iorq;
	end

	always @( negedge clk_n ) begin
		if( !ff_reset_n ) begin
			ff_wr_n_i <= 1'b1;
		end
		else if( !w_iorq ) begin
			if( w_t_state == 3'd2 ) begin
				ff_wr_n_i <= ~w_write;
			end
			else if( w_t_state == 3'd3 ) begin
				ff_wr_n_i <= 1'b1;
			end
		end
		else begin
			if( w_t_state == 3'd1 && !ff_iorq_n_i ) begin
				ff_wr_n_i <= ~w_write;
			end
			else if( w_t_state == 3'd3 ) begin
				ff_wr_n_i <= 1'b1;
			end
		end
	end

	always @( posedge clk_n ) begin
		if( !ff_reset_n ) begin
			ff_req_inhibit <= 1'b0;
		end
		else if( w_m_cycle == 3'd1 && w_t_state == 3'd2 && ff_wait_n == 1'b1 ) begin
			ff_req_inhibit <= 1'b1;
		end
		else begin
			ff_req_inhibit <= 1'b0;
		end
	end

	always @( negedge clk_n ) begin
		if( !ff_reset_n ) begin
			ff_mreq_inhibit <= 1'b0;
		end
		else if( w_m_cycle == 3'd1 && w_t_state == 3'd2 ) begin
			ff_mreq_inhibit <= 1'b1;
		end
		else begin
			ff_mreq_inhibit <= 1'b0;
		end
	end

	always @( negedge clk_n ) begin
		if( !ff_reset_n ) begin
			ff_rd <= 1'b0;
			ff_iorq_n_i <= 1'b1;
			ff_mreq <= 1'b0;
		end
		else if( w_m_cycle == 3'd1 ) begin
			if( w_t_state == 3'd1 ) begin
				ff_rd <= w_intcycle_n;
				ff_mreq <= w_intcycle_n;
				ff_iorq_n_i <= w_intcycle_n;
			end
			else if( w_t_state == 3'd3 ) begin
				ff_rd <= 1'b0;
				ff_iorq_n_i <= 1'b1;
				ff_mreq <= 1'b1;
			end
			else if( w_t_state == 3'd4 ) begin
				ff_mreq <= 1'b0;
			end
		end
		else begin
			if( w_t_state == 3'd1 && !w_noread ) begin
				ff_iorq_n_i <= ~w_iorq;
				ff_mreq <= ~w_iorq;
				if( !w_iorq ) begin
					ff_rd <= ~w_write;
				end
				else if( !ff_iorq_n_i ) begin
					ff_rd <= ~w_write;
				end
			end
			if( w_t_state == 3'd3 ) begin
				ff_rd <= 1'b0;
				ff_iorq_n_i <= 1'b1;
				ff_mreq <= 1'b0;
			end
		end
	end
endmodule
