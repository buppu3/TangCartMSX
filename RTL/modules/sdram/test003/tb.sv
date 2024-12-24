// -----------------------------------------------------------------------------
//	Test of ip_sdram_tangnano20k_c.v
//	Copyright (C)2024 Takayuki Hara (HRA!)
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
	localparam		clk_base	= 1_000_000_000/108_000;	//	ps
	reg				reset_n;
	reg				clk;				//	85.90908MHz
	reg				clk_sdram;			//	85.90908MHz
	wire			sdram_busy;
	reg				cpu_freeze;

	reg				mreq_n;
	reg		[22:0]	address;
	reg				wr_n;
	reg				rd_n;
	reg				rfsh_n;
	reg		[ 7:0]	wdata;
	wire	[ 7:0]	rdata;
	wire			rdata_en;

	wire			O_sdram_clk;
	wire			O_sdram_cke;
	wire			O_sdram_cs_n;		// chip select
	wire			O_sdram_cas_n;		// columns address select
	wire			O_sdram_ras_n;		// row address select
	wire			O_sdram_wen_n;		// write enable
	wire	[31:0]	IO_sdram_dq;		// 32 bit bidirectional data bus
	wire	[10:0]	O_sdram_addr;		// 11 bit multiplexed address bus
	wire	[ 1:0]	O_sdram_ba;			// two banks
	wire	[ 3:0]	O_sdram_dqm;		// data mask
	reg		[ 1:0]	ff_video_clk;

	int				i;

	// --------------------------------------------------------------------
	//	DUT
	// --------------------------------------------------------------------
	ip_sdram u_sdram_controller (
		.reset_n			( reset_n			),
		.clk				( clk				),
		.clk_sdram			( clk				),
		.sdram_busy			( sdram_busy		),
		.cpu_freeze			( cpu_freeze		),
		.mreq_n				( mreq_n			),
		.address			( address			),
		.wr_n				( wr_n				),
		.rd_n				( rd_n				),
		.rfsh_n				( rfsh_n			),
		.wdata				( wdata				),
		.rdata				( rdata				),
		.rdata_en			( rdata_en			),
		.O_sdram_clk		( O_sdram_clk		),
		.O_sdram_cke		( O_sdram_cke		),
		.O_sdram_cs_n		( O_sdram_cs_n		),
		.O_sdram_cas_n		( O_sdram_cas_n		),
		.O_sdram_ras_n		( O_sdram_ras_n		),
		.O_sdram_wen_n		( O_sdram_wen_n		),
		.IO_sdram_dq		( IO_sdram_dq		),
		.O_sdram_addr		( O_sdram_addr		),
		.O_sdram_ba			( O_sdram_ba		),
		.O_sdram_dqm		( O_sdram_dqm		)
	);

	// --------------------------------------------------------------------
	mt48lc2m32b2 u_sdram (
		.Dq					( IO_sdram_dq		), 
		.Addr				( O_sdram_addr		), 
		.Ba					( O_sdram_ba		), 
		.Clk				( O_sdram_clk		), 
		.Cke				( O_sdram_cke		), 
		.Cs_n				( O_sdram_cs_n		), 
		.Ras_n				( O_sdram_ras_n		), 
		.Cas_n				( O_sdram_cas_n		), 
		.We_n				( O_sdram_wen_n		), 
		.Dqm				( O_sdram_dqm		)
	);

	// --------------------------------------------------------------------
	//	clock
	// --------------------------------------------------------------------
	always #(clk_base/2) begin
		clk <= ~clk;
		clk_sdram <= ~clk_sdram;
	end

	always @( posedge clk ) begin
		ff_video_clk <= ff_video_clk + 2'd1;
	end

	// --------------------------------------------------------------------
	//	Tasks
	// --------------------------------------------------------------------
	task write_data(
		input	[22:0]	p_address,
		input	[7:0]	p_data
	);
		$display( "write_data( 0x%06X, 0x%02X )", p_address, p_data );
		address		<= p_address;
		wdata		<= p_data;
		mreq_n		<= 1'b0;
		wr_n		<= 1'b0;
		@( posedge clk );
		@( posedge clk );
		@( posedge clk );
		@( posedge clk );

		address		<= 0;
		wdata		<= 0;
		mreq_n		<= 1'b1;
		wr_n		<= 1'b1;
		repeat( 12 ) @( posedge clk );
	endtask: write_data

	// --------------------------------------------------------------------
	task read_data(
		input	[22:0]	p_address,
		input	[15:0]	p_data
	);
		int time_out;

		$display( "read_data( 0x%06X, 0x%02X )", p_address, p_data );
		address		<= p_address;
		mreq_n		<= 1'b0;
		rd_n		<= 1'b0;
		@( posedge clk );
		@( posedge clk );
		@( posedge clk );
		@( posedge clk );

		address		<= 0;
		mreq_n		<= 1'b1;
		rd_n		<= 1'b1;
		while( !rdata_en ) begin
			@( posedge clk );
		end
		assert( rdata == p_data );
		if( rdata != p_data ) begin
			$display( "-- p_data = %08X", p_data );
		end
		repeat( 16 ) @( posedge clk );
	endtask: read_data

	// --------------------------------------------------------------------
	//	Test bench
	// --------------------------------------------------------------------
	initial begin
		reset_n = 0;
		clk = 0;
		clk_sdram = 1;
		mreq_n = 1;
		wr_n = 1;
		rd_n = 1;
		rfsh_n = 1;
		address = 0;
		wdata = 0;
		ff_video_clk = 0;
		cpu_freeze = 1;

		@( negedge clk );
		@( negedge clk );
		@( posedge clk );

		reset_n			= 1;
		@( posedge clk );

		while( sdram_busy ) begin
			@( posedge clk );
		end

		repeat( 16 ) @( posedge clk );
		repeat( 7 ) @( posedge clk );

		$display( "write -------------------------" );
		write_data( 'h000000, 'h12 );
		write_data( 'h000001, 'h23 );
		write_data( 'h000002, 'h34 );
		write_data( 'h000003, 'h45 );
		write_data( 'h000004, 'h56 );
		write_data( 'h000005, 'h67 );
		write_data( 'h000006, 'h78 );
		write_data( 'h000007, 'h89 );

		$display( "read -------------------------" );
		read_data(  'h000000, 'h12 );
		read_data(  'h000001, 'h23 );
		read_data(  'h000002, 'h34 );
		read_data(  'h000003, 'h45 );
		read_data(  'h000004, 'h56 );
		read_data(  'h000005, 'h67 );
		read_data(  'h000006, 'h78 );
		read_data(  'h000007, 'h89 );

		read_data(  'h000007, 'h89 );
		read_data(  'h000006, 'h78 );
		read_data(  'h000005, 'h67 );
		read_data(  'h000004, 'h56 );
		read_data(  'h000003, 'h45 );
		read_data(  'h000002, 'h34 );
		read_data(  'h000001, 'h23 );
		read_data(  'h000000, 'h12 );

		$display( "write -------------------------" );
		write_data( 'h400000, 'h21 );
		write_data( 'h400001, 'h32 );
		write_data( 'h400002, 'h43 );
		write_data( 'h400003, 'h54 );
		write_data( 'h400004, 'h65 );
		write_data( 'h400005, 'h76 );
		write_data( 'h400006, 'h87 );
		write_data( 'h400007, 'h98 );

		$display( "read -------------------------" );
		read_data(  'h400000, 'h21 );
		read_data(  'h400001, 'h32 );
		read_data(  'h400002, 'h43 );
		read_data(  'h400003, 'h54 );
		read_data(  'h400004, 'h65 );
		read_data(  'h400005, 'h76 );
		read_data(  'h400006, 'h87 );
		read_data(  'h400007, 'h98 );

		read_data(  'h000000, 'h12 );
		read_data(  'h000001, 'h23 );
		read_data(  'h000002, 'h34 );
		read_data(  'h000003, 'h45 );
		read_data(  'h000004, 'h56 );
		read_data(  'h000005, 'h67 );
		read_data(  'h000006, 'h78 );
		read_data(  'h000007, 'h89 );

		read_data(  'h400007, 'h98 );
		read_data(  'h400006, 'h87 );
		read_data(  'h400005, 'h76 );
		read_data(  'h400004, 'h65 );
		read_data(  'h400003, 'h54 );
		read_data(  'h400002, 'h43 );
		read_data(  'h400001, 'h32 );
		read_data(  'h400000, 'h21 );

		read_data(  'h000007, 'h89 );
		read_data(  'h000006, 'h78 );
		read_data(  'h000005, 'h67 );
		read_data(  'h000004, 'h56 );
		read_data(  'h000003, 'h45 );
		read_data(  'h000002, 'h34 );
		read_data(  'h000001, 'h23 );
		read_data(  'h000000, 'h12 );

		$display( "Wait -------------------------------" );
		for( i = 0; i < 100; i++ ) begin
			$display( "** %d **", i );
			repeat( 100 ) @( posedge clk );
		end

		$display( "delay read -------------------------" );
		read_data(  'h400000, 'h21 );
		read_data(  'h400001, 'h32 );
		read_data(  'h400002, 'h43 );
		read_data(  'h400003, 'h54 );
		read_data(  'h400004, 'h65 );
		read_data(  'h400005, 'h76 );
		read_data(  'h400006, 'h87 );
		read_data(  'h400007, 'h98 );

		read_data(  'h000000, 'h12 );
		read_data(  'h000001, 'h23 );
		read_data(  'h000002, 'h34 );
		read_data(  'h000003, 'h45 );
		read_data(  'h000004, 'h56 );
		read_data(  'h000005, 'h67 );
		read_data(  'h000006, 'h78 );
		read_data(  'h000007, 'h89 );

		read_data(  'h400007, 'h98 );
		read_data(  'h400006, 'h87 );
		read_data(  'h400005, 'h76 );
		read_data(  'h400004, 'h65 );
		read_data(  'h400003, 'h54 );
		read_data(  'h400002, 'h43 );
		read_data(  'h400001, 'h32 );
		read_data(  'h400000, 'h21 );

		read_data(  'h000007, 'h89 );
		read_data(  'h000006, 'h78 );
		read_data(  'h000005, 'h67 );
		read_data(  'h000004, 'h56 );
		read_data(  'h000003, 'h45 );
		read_data(  'h000002, 'h34 );
		read_data(  'h000001, 'h23 );
		read_data(  'h000000, 'h12 );

		$finish;
	end
endmodule
