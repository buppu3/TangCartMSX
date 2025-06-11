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

module cz80_wrap (
	input			reset_n		,
	input			clk_n		,		//	85.90908MHz
	input			int_n		,
	output	[15:0]	bus_address	,
	output			bus_memreq	,
	output			bus_ioreq	,
	output			bus_valid	,
	input			bus_ready	,
	output			bus_write	,
	output	[7:0]	bus_wdata	,
	input	[7:0]	bus_rdata	,
	input			bus_rdata_en
);
	reg				ff_enable;
	reg		[15:0]	ff_bus_address;
	reg				ff_bus_memreq;
	reg				ff_bus_ioreq;
	reg				ff_bus_valid;
	reg				ff_bus_write;
	reg		[7:0]	ff_bus_wdata;
	reg		[7:0]	ff_bus_rdata;
	reg				ff_request;
	wire			w_mreq_n;
	wire			w_iorq_n;
	wire			w_rd_n;
	wire			w_wr_n;
	wire	[15:0]	w_a;
	wire	[7:0]	w_d;
	wire			w_enable;

	assign bus_address	= ff_bus_address;
	assign bus_memreq	= ff_bus_memreq;
	assign bus_ioreq	= ff_bus_ioreq;
	assign bus_valid	= ff_bus_valid;
	assign bus_write	= ff_bus_write;
	assign bus_wdata	= ff_bus_wdata;

	always @( posedge clk_n ) begin
		if( !reset_n ) begin
			ff_enable		<= 1'b0;
		end
		else begin
			ff_enable		<= ~ff_enable;
		end
	end

	always @( posedge clk_n ) begin
		if( bus_rdata_en ) begin
			ff_bus_rdata	<= bus_rdata;
		end
	end

	always @( posedge clk_n ) begin
		if( !reset_n ) begin
			ff_request		<= 1'b0;
			ff_bus_address	<= 16'd0;
			ff_bus_memreq	<= 1'b0;
			ff_bus_ioreq	<= 1'b0;
			ff_bus_valid	<= 1'b0;
			ff_bus_write	<= 1'b0;
			ff_bus_wdata	<= 8'd0;
		end
		else if( ff_request ) begin
			if( bus_ready ) begin
				ff_bus_valid	<= 1'b0;
			end
			if( w_mreq_n && w_iorq_n ) begin
				ff_bus_memreq	<= 1'b0;
				ff_bus_ioreq	<= 1'b0;
				ff_bus_valid	<= 1'b0;
				ff_request		<= 1'b0;
			end
		end
		else if( !w_mreq_n || !w_iorq_n ) begin
			if( !w_rd_n ) begin
				ff_request		<= 1'b1;
				ff_bus_address	<= w_a;
				ff_bus_memreq	<= ~w_mreq_n;
				ff_bus_ioreq	<= ~w_iorq_n;
				ff_bus_valid	<= 1'b1;
				ff_bus_write	<= 1'b0;
				ff_bus_wdata	<= 8'b0;
			end
			else if( !w_wr_n ) begin
				ff_request		<= 1'b1;
				ff_bus_address	<= w_a;
				ff_bus_memreq	<= ~w_mreq_n;
				ff_bus_ioreq	<= ~w_iorq_n;
				ff_bus_valid	<= 1'b1;
				ff_bus_write	<= 1'b1;
				ff_bus_wdata	<= w_d;
			end
			else begin
			end
		end
	end

	assign w_enable	= ff_bus_valid ? 1'b0: ff_enable;
	assign w_d		= (w_rd_n == 1'b0) ? ff_bus_rdata : 8'dz;

	cz80_inst u_cz80_inst (
		.reset_n	( reset_n		),
		.clk_n		( clk_n			),
		.enable		( w_enable		),
		.wait_n		( 1'b1			),
		.int_n		( int_n			),
		.nmi_n		( 1'b1			),
		.busrq_n	( 1'b1			),
		.m1_n		( 				),
		.mreq_n		( w_mreq_n		),
		.iorq_n		( w_iorq_n		),
		.rd_n		( w_rd_n		),
		.wr_n		( w_wr_n		),
		.rfsh_n		( 				),
		.halt_n		( 				),
		.busak_n	( 				),
		.a			( w_a			),
		.d			( w_d			)
	);
endmodule
