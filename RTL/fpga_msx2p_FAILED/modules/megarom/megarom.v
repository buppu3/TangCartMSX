//
//	megarom.v
//	Multi MegaROM Controller
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

module megarom (
	input			clk,
	input			reset_n,
	input			sltsl,
	input			mreq_n,
	input			wr_n,
	input			rd_n,
	input	[15:0]	address,
	input	[7:0]	wdata,
	output	[7:0]	rdata,
	output			rdata_en,
	output			mem_cs_n,
	//	ROM interface
	output			megarom_rd_n,
	output	[21:0]	megarom_address,
	//	Mode select
	input	[2:0]	mode,				//	0: ASC16, 1: ASC8, 2: KonamiSCC/SCC+, 3: ---, 4: Linear, 5: ---, 6: KonamiVRC, 7: ---
	//	SCC sound out
	output	[10:0]	sound_out
);
	reg		[7:0]	ff_rdata;
	reg				ff_rdata_en;
	reg				ff_rd_n;
	wire			w_wr;
	wire			w_rd;
	reg				ff_enable;

	reg		[7:0]	ff_bank0;
	reg		[7:0]	ff_bank1;
	reg		[7:0]	ff_bank2;
	reg		[7:0]	ff_bank3;
	wire	[7:0]	w_address16;
	wire	[7:0]	w_address8;
	wire			w_scc_mem_cs_n;
	wire	[7:0]	w_scc_rdata;
	wire			w_scc_rdata_en;
	wire	[7:0]	w_scc_address;

	always @( posedge clk ) begin
		if( !reset_n ) begin
			ff_rd_n		<= 1'b1;
		end
		else begin
			ff_rd_n		<= rd_n;
		end
	end

	assign w_wr		= (!mreq_n &&             !wr_n && sltsl);
	assign w_rd		= (!mreq_n &&  ff_rd_n && !rd_n && sltsl);

	// --------------------------------------------------------------------
	//	Bank register
	// --------------------------------------------------------------------
	always @( posedge clk ) begin
		if( !reset_n ) begin
			ff_bank0 <= 8'd0;
		end
		else if( w_wr ) begin
			if(      mode[2:1] == 2'b00 && address[15:11] == 5'b0110_0 ) begin	//	0,1: ASC8/ASC16: 6000-67FFh
				ff_bank0 <= wdata;
			end
			else begin		//	4,5,6,7: Linear, VRC
				//	hold
			end
		end
		else begin
			//	hold
		end
	end

	always @( posedge clk ) begin
		if( !reset_n ) begin
			if( mode[2:1] == 2'b00 ) begin
				ff_bank1 <= 8'd0;
			end
			else begin
				ff_bank1 <= 8'd1;
			end
		end
		else if( w_wr ) begin
			if(      mode[2:1] == 2'b00 && address[15:11] == 5'b0110_1 ) begin	//	0,1: ASC8    : 6800-6FFFh
				ff_bank1 <= wdata;
			end
			else if( mode[2:1] == 2'b11 && address[15:13] == 3'b011    ) begin	//	6,7: VRC     : 6000-7FFFh
				ff_bank1 <= wdata;
			end
			else begin		//	4,5: Linear
				//	hold
			end
		end
		else begin
			//	hold
		end
	end

	always @( posedge clk ) begin
		if( !reset_n ) begin
			if( mode[2:1] == 2'b00 ) begin
				ff_bank2 <= 8'd0;
			end
			else begin
				ff_bank2 <= 8'd2;
			end
		end
		else if( w_wr ) begin
			if(      mode[2:1] == 2'b00 && address[15:11] == 5'b0111_0 ) begin	//	0,1: ASC8/ASC16: 7000-77FFh
				ff_bank2 <= wdata;
			end
			else if( mode[2:1] == 2'b11 && address[15:13] == 3'b100    ) begin	//	6,7: VRC       : 8000-9FFFh
				ff_bank2 <= wdata;
			end
			else begin		//	4,5: Linear
				//	hold
			end
		end
		else begin
			//	hold
		end
	end

	always @( posedge clk ) begin
		if( !reset_n ) begin
			if( mode[2:1] == 2'b00 ) begin
				ff_bank3 <= 8'd0;
			end
			else begin
				ff_bank3 <= 8'd3;
			end
		end
		else if( w_wr ) begin
			if(      mode[2:1] == 2'b00 && address[15:11] == 5'b0111_1 ) begin	//	0,1: ASC8    : 7800-7FFFh
				ff_bank3 <= wdata;
			end
			else if( mode[2:1] == 2'b11 && address[15:13] == 3'b101    ) begin	//	6,7: VRC     : A000-BFFFh
				ff_bank3 <= wdata;
			end
			else begin		//	4,5: Linear
				//	hold
			end
		end
		else begin
			//	hold
		end
	end

	// --------------------------------------------------------------------
	//	SCC
	// --------------------------------------------------------------------
	always @( posedge clk ) begin
		if( !reset_n ) begin
			ff_enable <= 1'b0;
		end
		else begin
			if( mode == 3'd2 ) begin
				ff_enable <= ~ff_enable;
			end
			else begin
				ff_enable <= 1'b0;
			end
		end
	end

	scc_core #(
		.add_offset		( 1					)
	) u_scc_core (
		.nreset			( reset_n			),
		.clk			( clk				),
		.enable			( ff_enable			),
		.wrreq			( w_wr				),
		.rdreq			( w_rd				),
		.wr_active		( ~wr_n				),
		.rd_active		( ~rd_n				),
		.a				( address			),
		.d				( wdata				),
		.q				( w_scc_rdata		),
		.q_en			( w_scc_rdata_en	),
		.mem_ncs		( w_scc_mem_cs_n	),
		.mem_a			( w_scc_address		),
		.left_out		( sound_out			)
	);

	// --------------------------------------------------------------------
	//	Address select
	// --------------------------------------------------------------------
	assign w_address16				= (address[14]    == 1'b1 ) ? ff_bank0:				//	4000h-7FFFh MSB 2bit = 01-01
	                  				                              ff_bank2;				//	8000h-BFFFh MSB 2bit = 10-10
	assign w_address8				= (address[14:13] == 2'b10) ? ff_bank0:				//	4000h-5FFFh MSB 3bit = 010-010
	                 				  (address[14:13] == 2'b11) ? ff_bank1:				//	6000h-7FFFh MSB 3bit = 011-011
	                 				  (address[14:13] == 2'b00) ? ff_bank2: 			//	8000h-9FFFh MSB 3bit = 100-100
	                 				                              ff_bank3;				//	A000h-BFFFh MSB 3bit = 101-101

	assign megarom_rd_n				= ff_rd_n;
	assign megarom_address[21:13]	= (mode == 3'd4) ? { 6'd0, address[15:13] }:		//	Linear
	                             	  (mode == 3'd2) ? { 1'b0, w_scc_address }:			//	SCC bank
	                             	  (mode == 3'd0) ? { w_address16, address[13] }:	//	16K bank
	                             	                   { 1'b0, w_address8 };			//	8K bank
	assign megarom_address[12:0]	= address[12:0];

	assign rdata					= w_scc_rdata;
	assign rdata_en					= w_scc_rdata_en;
	assign mem_cs_n					= (mode == 3'd2) ? w_scc_mem_cs_n : ~sltsl;
endmodule
