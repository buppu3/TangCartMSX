//
// ip_sdram_dummy.vhd
//	 16384 bytes of block memory
//	 Revision 1.00
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

module ip_sdram (
	input				n_reset			,
	input				clk				,
	input				clk_sdram		,
	input				rd_n			,
	input				wr_n			,
	input				exec			,
	output				busy			,
	input	[16:0]		address			,
	input	[7:0]		wdata			,
	output	[15:0]		rdata			,
	output				rdata_en		,
	output				O_sdram_clk		,
	output				O_sdram_cke		,
	output				O_sdram_cs_n	,
	output				O_sdram_ras_n	,
	output				O_sdram_cas_n	,
	output				O_sdram_wen_n	,
	inout	[31:0]		IO_sdram_dq		,
	output	[10:0]		O_sdram_addr	,
	output	[1:0]		O_sdram_ba		,
	output	[3:0]		O_sdram_dqm		
);
	reg		[7:0]	ff_ram [0:16383];
	reg		[7:0]	ff_rdata;
	reg		[7:0]	ff_rdata_d1;
	reg		[7:0]	ff_rdata_d2;
	reg		[7:0]	ff_rdata_d3;

	always @( posedge clk ) begin
		ff_rdata_d1 <= ff_rdata;
		ff_rdata_d2 <= ff_rdata_d1;
		ff_rdata_d3 <= ff_rdata_d2;
	end

	always @( posedge clk ) begin
		if( exec ) begin
			if(      rd_n == 1'b0 ) begin
				ff_rdata <= ff_ram[ address[13:0] ];
			end
			else if( wr_n == 1'b0 ) begin
				ff_ram[ address[13:0] ] <= wdata;
			end
		end
	end

	assign O_sdram_clk		= 1'b0;
	assign O_sdram_cke		= 1'b0;
	assign O_sdram_cs_n		= 1'b0;
	assign O_sdram_ras_n	= 1'b0;
	assign O_sdram_cas_n	= 1'b0;
	assign O_sdram_wen_n	= 1'b0;
	assign IO_sdram_dq		= 32'hZ;
	assign O_sdram_addr		= 11'd0;
	assign O_sdram_ba		= 2'b00;
	assign O_sdram_dqm		= 4'b0000;

	assign busy				= 1'b0;
	assign rdata			= { ff_rdata_d1, ff_rdata_d1 };
	assign rdata_en			= 1'b0;
endmodule
