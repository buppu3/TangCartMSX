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
	//	ROM interface
	output			megarom_rd_n,
	output	[21:0]	megarom_address,
	//	Mode select
	input	[2:0]	mode				//	0: ASC16, 1: ASC8, 2: KonamiSCC, 3: KonamiSCC+, 4: Linear, 5: KonamiVRC, 6: ---, 7: ---
);
	reg		[7:0]	ff_rdata;
	reg				ff_rdata_en;
	reg				ff_rd_n;
	wire			w_wr;
	wire			w_rd;

	reg		[7:0]	ff_bank0;
	reg		[7:0]	ff_bank1;
	reg		[7:0]	ff_bank2;
	reg		[7:0]	ff_bank3;
	wire	[7:0]	w_address16;
	wire	[7:0]	w_address8;

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
			if( mode[2:1] == 2'b00 && address[15:11] == 5'b0110_0 ) begin	//	ASC8, ASC16: 6000-67FFh
				ff_bank0 <= wdata;
			end
			else begin
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
			if( mode[2:1] == 2'b00 && address[15:11] == 5'b0110_1 ) begin	//	ASC8: 6800-6FFFh
				ff_bank1 <= wdata;
			end
			else begin
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
			if( mode[2:1] == 2'b00 && address[15:11] == 5'b0111_0 ) begin	//	ASC8, ASC16: 7000-77FFh
				ff_bank2 <= wdata;
			end
			else begin
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
			if( mode[2:1] == 2'b00 && address[15:11] == 5'b0111_1 ) begin	//	ASC8: 7800-7FFFh
				ff_bank3 <= wdata;
			end
			else begin
				//	hold
			end
		end
		else begin
			//	hold
		end
	end

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
	assign megarom_address[21:13]	= (mode == 3'd0) ? { w_address16, address[13] }: { 1'b0, w_address8 };
	assign megarom_address[12:0]	= address[12:0];
endmodule
