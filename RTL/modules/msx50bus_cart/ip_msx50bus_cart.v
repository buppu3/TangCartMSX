// -----------------------------------------------------------------------------
//	ip_msx50bus_cart.v
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

module ip_msx50bus_cart (
	//	cartridge slot signals
	input			n_reset,
	input			clk,
	input	[15:0]	adr,
	input	[7:0]	i_data,
	output	[7:0]	o_data,
	output			is_output,
	input			n_sltsl,
	input			n_rd,
	input			n_wr,
	input			n_ioreq,
	input			n_mereq,
	//	internal signals
	output	[15:0]	bus_address,
	output			bus_io_req,
	output			bus_memory_req,
	input			bus_ack,
	output			bus_wrt,
	output	[7:0]	bus_wdata,
	input	[7:0]	bus_rdata,
	input			bus_rdata_en
);
	//	Flip-flops for asynchronous switching
	reg				ff_n_sltsl;
	reg				ff_n_ioreq;
	reg				ff_n_rd;
	reg				ff_n_wr;
	//	Make up pulse
	wire			w_io_req_pulse;
	wire			w_memory_req_pulse;
	wire			w_wrt_pulse;
	//	Latch
	reg		[15:0]	ff_address;
	reg		[7:0]	ff_rdata;
	reg		[7:0]	ff_wdata;
	reg				ff_io_req;
	reg				ff_memory_req;
	reg				ff_is_output;

	// --------------------------------------------------------------------
	//	Asynchronous switching and low-pass
	// --------------------------------------------------------------------
	always @( posedge clk ) begin
		if( !n_reset ) begin
			ff_n_sltsl	<= 1'b1;
			ff_n_rd		<= 1'b1;
			ff_n_wr		<= 1'b1;
			ff_n_ioreq	<= 1'b1;
		end
		else begin
			ff_n_sltsl	<= n_sltsl;
			ff_n_rd		<= n_rd;
			ff_n_wr		<= n_wr;
			ff_n_ioreq	<= n_ioreq;
		end
	end

	assign w_io_req			= ~ff_n_ioreq & (~ff_n_wr | ~ff_n_rd);
	assign w_memory_req		= ~ff_n_sltsl & (~ff_n_wr | ~ff_n_rd);
	assign w_wrt			= ~ff_n_wr;

	// --------------------------------------------------------------------
	//	Make up pulse
	// --------------------------------------------------------------------
	always @( posedge clk ) begin
		if( !n_reset ) begin
			ff_io_req		<= 1'b0;
			ff_memory_req	<= 1'b0;
			ff_wrt			<= 1'b0;
		end
		else begin
			ff_io_req		<= w_io_req;
			ff_memory_req	<= w_memory_req;
			ff_wrt			<= w_wrt;
		end
	end

	assign w_io_req_pulse		= ~ff_io_req     & w_io_req;
	assign w_memory_req_pulse	= ~ff_memory_req & w_memory_req;
	assign w_wrt_pulse			= ~ff_wrt        & w_wrt;

	// --------------------------------------------------------------------
	//	Latch
	// --------------------------------------------------------------------
	always @( posedge clk ) begin
		if( !n_reset ) begin
			ff_bus_address		<= 'd0;
		end
		else if( w_io_req_pulse || w_memory_req_pulse ) begin
			ff_bus_address		<= adr;
		end
		else begin
			//	hold
		end
	end

	always @( posedge clk ) begin
		if( w_wrt_pulse ) begin
			ff_wdata	<= i_data;
		end
		else begin
			//	hold
		end
	end

	always @( posedge clk ) begin
		if( bus_ack ) begin
			ff_io_req	<= 1'b0;
		end
		else if( w_io_req_pulse ) begin
			ff_io_req	<= 1'b1;
		end
		else begin
			//	hold
		end
	end

	always @( posedge clk ) begin
		if( bus_ack ) begin
			ff_memory_req	<= 1'b0;
		end
		else if( w_memory_req_pulse ) begin
			ff_memory_req	<= 1'b1;
		end
		else begin
			//	hold
		end
	end

	always @( posedge clk ) begin
		if( bus_ack ) begin
			ff_wrt	<= 1'b0;
		end
		else if( w_wrt_pulse ) begin
			ff_wrt	<= 1'b1;
		end
		else begin
			//	hold
		end
	end

	always @( posedge clk ) begin
		if( !n_reset ) begin
			ff_rdata		<= 8'd0;
			ff_is_output	<= 1'b0;
		end
		else if( ff_n_rd == 1'b1 ) begin
			ff_rdata		<= 8'd0;
			ff_is_output	<= 1'b0;
		end
		else if( bus_rdata_en ) begin
			ff_rdata		<= bus_rdata;
			ff_is_output	<= 1'b1;
		end
		else begin
			//	hold
		end
	end

	// --------------------------------------------------------------------
	//	Internal BUS signals
	// --------------------------------------------------------------------
	assign bus_address		= ff_address;
	assign bus_io_req		= ff_io_req;
	assign bus_memory_req	= ff_memory_req;
	assign bus_wrt			= ff_wrt;
	assign bus_wdata		= ff_wdata;

	// --------------------------------------------------------------------
	//	MSX BUS response
	// --------------------------------------------------------------------
	assign o_data			= ff_rdata;
	assign is_output		= ff_is_output;
endmodule
