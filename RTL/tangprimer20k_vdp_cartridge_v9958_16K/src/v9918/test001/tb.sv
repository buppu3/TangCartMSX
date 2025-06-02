// -----------------------------------------------------------------------------
//	Test of v9918
//	Copyright (C)2025 Takayuki Hara (HRA!)
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
// --------------------------------------------------------------------

module tb ();
	localparam		clk_base	= 1_000_000_000/42_954_540;	//	ps
	localparam		vdp_port0	= 8'h98;
	localparam		vdp_port1	= 8'h99;
	reg						clk;			//	42.95454MHz
	reg						reset_n;
	reg						initial_busy;
	reg			[15:0]		bus_address;
	reg						bus_ioreq;
	reg						bus_write;
	reg						bus_valid;
	reg			[7:0]		bus_wdata;
	wire		[7:0]		bus_rdata;
	wire					bus_rdata_en;
	wire					int_n;
	wire		[13:0]		p_dram_address;
	wire					p_dram_write;
	wire					p_dram_valid;
	wire					p_dram_ready;
	wire		[7:0]		p_dram_wdata;
	wire		[7:0]		p_dram_rdata;
	wire					p_dram_rdata_en;
	wire					p_vdp_enable;
	wire		[5:0]		p_vdp_r;
	wire		[5:0]		p_vdp_g;
	wire		[5:0]		p_vdp_b;
	wire		[10:0]		p_vdp_hcounter;
	wire		[10:0]		p_vdp_vcounter;
	int						i;

	// --------------------------------------------------------------------
	//	DUT
	// --------------------------------------------------------------------
	vdp_inst u_v9918 (
		.clk					( clk					),
		.reset_n				( reset_n				),
		.initial_busy			( initial_busy			),
		.bus_address			( bus_address			),
		.bus_ioreq				( bus_ioreq				),
		.bus_write				( bus_write				),
		.bus_valid				( bus_valid				),
		.bus_wdata				( bus_wdata				),
		.bus_rdata				( bus_rdata				),
		.bus_rdata_en			( bus_rdata_en			),
		.int_n					( int_n					),
		.p_dram_address			( p_dram_address		),
		.p_dram_write			( p_dram_write			),
		.p_dram_valid			( p_dram_valid			),
		.p_dram_ready			( p_dram_ready			),
		.p_dram_wdata			( p_dram_wdata			),
		.p_dram_rdata			( p_dram_rdata			),
		.p_dram_rdata_en		( p_dram_rdata_en		),
		.p_vdp_enable			( p_vdp_enable			),
		.p_vdp_r				( p_vdp_r				),
		.p_vdp_g				( p_vdp_g				),
		.p_vdp_b				( p_vdp_b				),
		.p_vdp_hcounter			( p_vdp_hcounter		),
		.p_vdp_vcounter			( p_vdp_vcounter		)
	);

	ip_ram u_ram (
		.reset_n				( reset_n				),
		.clk					( clk					),
		.bus_address			( p_dram_address		),
		.bus_valid				( p_dram_valid			),
		.bus_ready				( p_dram_ready			),
		.bus_write				( p_dram_write			),
		.bus_wdata				( p_dram_wdata			),
		.bus_rdata				( p_dram_rdata			),
		.bus_rdata_en			( p_dram_rdata_en		)
	);

	// --------------------------------------------------------------------
	//	clock
	// --------------------------------------------------------------------
	always #(clk_base/2) begin
		clk <= ~clk;
	end

	// --------------------------------------------------------------------
	//	tasks
	// --------------------------------------------------------------------
	task write_io(
		input	[7:0]	address,
		input	[7:0]	wdata
	);
		bus_ioreq		<= 1'b1;
		bus_write		<= 1'b1;
		bus_valid		<= 1'b1;
		bus_address		<= { 8'h00, address };
		bus_wdata		<= wdata;
		@( posedge clk );

		bus_ioreq		<= 1'b0;
		bus_write		<= 1'b0;
		bus_valid		<= 1'b0;
		bus_address		<= 16'd0;
		bus_wdata		<= 8'd0;
		repeat( 23 ) @( posedge clk );
	endtask: write_io

	// --------------------------------------------------------------------
	//	Test bench
	// --------------------------------------------------------------------
	initial begin
		clk = 0;			//	42.95454MHz
		reset_n = 0;
		initial_busy = 0;
		bus_address = 0;
		bus_ioreq = 0;
		bus_write = 0;
		bus_valid = 0;
		bus_wdata = 0;

		@( negedge clk );
		@( negedge clk );

		reset_n = 1;
		@( negedge clk );
		repeat( 4 ) @( posedge clk );

		// --------------------------------------------------------------------
		write_io( vdp_port1, 8'h00 );
		write_io( vdp_port1, 8'h80 );

		write_io( vdp_port1, 8'h40 );
		write_io( vdp_port1, 8'h81 );

		for( i = 0; i < 16; i++ ) begin
			write_io( vdp_port1, i );
			write_io( vdp_port1, 8'h87 );
		end
		@( posedge clk );

		for( i = 0; i < 16; i++ ) begin
			write_io( vdp_port1, i );
			write_io( vdp_port1, 8'h87 );
		end
		@( posedge clk );

		// --------------------------------------------------------------------
		write_io( vdp_port1, 8'h00 );
		write_io( vdp_port1, 8'h40 );

		for( i = 0; i < 16384; i++ ) begin
			write_io( vdp_port0, i & 255 );
		end

		repeat( 1368 * 524 ) @( posedge clk );
		$finish;
	end
endmodule
