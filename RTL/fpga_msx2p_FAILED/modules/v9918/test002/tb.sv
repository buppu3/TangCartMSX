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
	localparam		clk_base	= 1_000_000_000/42_954_540;	//	ps
	reg						clk;			//	42.95454MHz
	reg						reset_n;
	reg						iorq_n;
	reg						wr_n;
	reg						rd_n;
	reg						address;
	wire		[7:0]		rdata;
	wire					rdata_en;
	reg			[7:0]		wdata;

	wire					int_n;

	wire					w_vram_read_n;
	wire					w_vram_write_n;
	wire		[13:0]		w_vram_address;
	reg			[7:0]		ff_vram_rdata;
	wire		[7:0]		w_vram_wdata;
	wire		[7:0]		w_vram_rdata;
	wire					w_vram_rdata_en;

	// video wire
	wire					pvideo_clk;
	wire					pvideo_data_en;

	wire		[5:0]		pvideor;
	wire		[5:0]		pvideog;
	wire		[5:0]		pvideob;

	wire					pvideohs_n;
	wire					pvideovs_n;

	wire					p_video_dh_clk;
	wire					p_video_dl_clk;

	wire					vram_mreq_n;
	wire		[22:0]		vram_address;
	wire					vram_wr_n;
	wire					vram_rd_n;
	wire					vram_rfsh_n;
	wire		[ 7:0]		vram_wdata;
	reg			[31:0]		vram_rdata;
	reg						vram_rdata_en;
	wire					video_de;
	wire					video_hs;
	wire					video_vs;
	wire		[7:0]		video_r;
	wire		[7:0]		video_g;
	wire		[7:0]		video_b;

	int						i, j, k;
	reg						timing;

	// --------------------------------------------------------------------
	//	DUT
	// --------------------------------------------------------------------
	vdp_inst u_vdp (
		.clk					( clk					),
		.reset_n				( reset_n				),
		.initial_busy			( 1'b0					),
		.iorq_n					( iorq_n				),
		.wr_n					( wr_n					),
		.rd_n					( rd_n					),
		.address				( address				),
		.rdata					( rdata					),
		.rdata_en				( rdata_en				),
		.wdata					( wdata					),
		.int_n					( int_n					),
		.p_dram_oe_n			( w_vram_read_n			),
		.p_dram_we_n			( w_vram_write_n		),
		.p_dram_address			( w_vram_address		),
		.p_dram_rdata			( ff_vram_rdata			),
		.p_dram_wdata			( w_vram_wdata			),
		.pvideo_clk				( pvideo_clk			),
		.pvideo_data_en			( pvideo_data_en		),
		.pvideor				( pvideor				),
		.pvideog				( pvideog				),
		.pvideob				( pvideob				),
		.pvideohs_n				( pvideohs_n			),
		.pvideovs_n				( pvideovs_n			),
		.p_video_dh_clk			( p_video_dh_clk		),
		.p_video_dl_clk			( p_video_dl_clk		)
	);

	// --------------------------------------------------------------------
	ip_ram u_vram (
		.clk					( clk					),
		.n_cs					( 1'b0					),
		.n_wr					( w_vram_write_n		),
		.n_rd					( w_vram_read_n			),
		.address				( w_vram_address		),
		.wdata					( w_vram_wdata			),
		.rdata					( w_vram_rdata			),
		.rdata_en				( w_vram_rdata_en		)
	);

	always @( posedge clk ) begin
		if( w_vram_rdata_en ) begin
			ff_vram_rdata <= w_vram_rdata;
		end
	end

	// --------------------------------------------------------------------
	//	target
	// --------------------------------------------------------------------
	ip_video u_video (
		.reset_n				( reset_n				),
		.clk					( clk					),
		.iorq_n					( iorq_n				),
		.address				( address				),
		.wr_n					( wr_n					),
		.wdata					( wdata					),
		.vram_mreq_n			( vram_mreq_n			),
		.vram_address			( vram_address			),
		.vram_wr_n				( vram_wr_n				),
		.vram_rd_n				( vram_rd_n				),
		.vram_rfsh_n			( vram_rfsh_n			),
		.vram_wdata				( vram_wdata			),
		.vram_rdata				( vram_rdata			),
		.vram_rdata_en			( vram_rdata_en			),
		.video_de				( video_de				),
		.video_hs				( video_hs				),
		.video_vs				( video_vs				),
		.video_r				( video_r				),
		.video_g				( video_g				),
		.video_b				( video_b				)
	);

	ip_video_vram u_video_vram (
		.clk					( clk					),
		.n_mreq					( vram_mreq_n			),
		.n_wr					( vram_wr_n				),
		.n_rd					( vram_rd_n				),
		.address				( vram_address			),
		.wdata					( vram_wdata			),
		.rdata					( vram_rdata			),
		.rdata_en				( vram_rdata_en			)
	);

	// --------------------------------------------------------------------
	//	clock
	// --------------------------------------------------------------------
	always #(clk_base/2) begin
		clk <= ~clk;
	end

	// --------------------------------------------------------------------
	//	Tasks
	// --------------------------------------------------------------------
	task write_data(
		input			p_address,
		input	[7:0]	p_data
	);
		$display( "write_data( 0x%06X, 0x%02X )", p_address, p_data );
		address		<= p_address;
		wdata		<= p_data;
		iorq_n		<= 1'b0;
		wr_n		<= 1'b0;
		@( posedge clk );
		@( posedge clk );

		iorq_n		<= 1'b1;
		wr_n		<= 1'b1;
		@( posedge clk );
		@( posedge clk );
		@( posedge clk );
		@( posedge clk );
		@( posedge clk );
		@( posedge clk );
	endtask: write_data

	// --------------------------------------------------------------------
	task read_data(
		input	[22:0]	p_address,
		input	[15:0]	p_data
	);
		int time_out;

		$display( "read_data( 0x%06X, 0x%02X )", p_address, p_data );
		address		<= p_address;
		iorq_n		<= 1'b0;
		@( posedge clk );
		rd_n		<= 1'b0;
		repeat( 16 ) @( negedge clk );

		iorq_n		<= 1'b1;
		rd_n		<= 1'b1;
		@( posedge clk );

		assert( rdata == p_data );
		if( rdata != p_data ) begin
			$display( "-- p_data = %08X (ref: %08X)", rdata, p_data );
		end

		@( posedge clk );
		@( posedge clk );
		@( posedge clk );
		@( posedge clk );
	endtask: read_data

	// --------------------------------------------------------------------
	task set_vram_address(
		input	[13:0]	address
	);
		write_data( 1, address[7:0] );
		write_data( 1, { 2'b01, address[13:8] } );
	endtask: set_vram_address

	// --------------------------------------------------------------------
	task write_reg(
		input	[2:0]	reg_num,
		input	[7:0]	data
	);
		write_data( 1, data );
		write_data( 1, { 5'b10000, reg_num } );
	endtask: write_reg

	// --------------------------------------------------------------------
	//	Test bench
	// --------------------------------------------------------------------
	initial begin
		timing = 0;
		reset_n = 0;
		clk = 0;
		reset_n = 0;
		iorq_n = 1;
		wr_n = 1;
		rd_n = 1;
		address = 0;
		wdata = 0;
		ff_vram_rdata = 0;

		@( negedge clk );
		@( negedge clk );
		@( posedge clk );

		reset_n			= 1;
		@( posedge clk );

		//	�A���������݂̃A�h���X�w�� 
		write_data( 1, 8'h00 );
		write_data( 1, 8'h40 );

		write_data( 0, 8'hAA );
		write_data( 0, 8'h55 );
		write_data( 0, 8'hA5 );
		write_data( 0, 8'h5A );
		write_data( 0, 8'h12 );
		write_data( 0, 8'h34 );
		write_data( 0, 8'h56 );
		write_data( 0, 8'h78 );

		//	�A���ǂݏo���̃A�h���X�w��
		write_data( 1, 8'h00 );
		write_data( 1, 8'h00 );

		read_data( 0, 8'hAA );
		read_data( 0, 8'h55 );
		read_data( 0, 8'hA5 );
		read_data( 0, 8'h5A );
		read_data( 0, 8'h12 );
		read_data( 0, 8'h34 );
		read_data( 0, 8'h56 );
		read_data( 0, 8'h78 );

		//	�X�e�[�^�X���W�X�^�̓ǂݏo��
		read_data( 1, 8'h1f );

		//	���W�X�^��������
		write_reg( 1, 8'hA0 );

		repeat( 100 ) @( posedge clk );

		// ====================================================================
		//	GRAPHIC1 (SCREEN1) �ɃZ�b�g���� 
		timing = 1;
		write_reg( 0, 8'h00 );			//	R#0 = 0x00 : Mode 0 : SCREEN1
		write_reg( 1, 8'h02 );			//	R#1 = 0x02 : Mode 1 : SCREEN1, 16x16 Sprite
		write_reg( 2, 8'h06 );			//	R#2 = 0x06 : Pattern Name Table      0x1800 = 01_1000_0000_0000 �� 01_10
		write_reg( 3, 8'h80 );			//	R#3 = 0x80 : Color Table             0x2000 = 10_0000_0000_0000 �� 10_0000_00
		write_reg( 4, 8'h00 );			//	R#4 = 0x00 : Pattern Generator Table 0x0000 = 00_0000_0000_0000 �� 00_0
		write_reg( 5, 8'h36 );			//	R#5 = 0x36 : Sprite Attribute Table  0x1B00 = 01_1011_0000_0000 �� 01_1011_0
		write_reg( 6, 8'h07 );			//	R#6 = 0x07 : Sprite Generator Table  0x3800 = 11_1000_0000_0000 �� 11_1
		write_reg( 7, 8'h07 );			//	R#6 = 0x07 : Back Color
		timing = 0;

		//	VRAM���[���N���A���� 
		set_vram_address( 14'h0000 );
		for( i = 0; i < 16384; i++ ) begin
			write_data( 0, 8'h00 );
		end

		//	Pattern Name Table �ɃC���N�������g�l���Z�b�g���� 
		set_vram_address( 14'h1800 );
		for( i = 0; i < 768; i++ ) begin
			write_data( 0, i & 255 );
		end

		//	Pattern Generator Table �ɃC���N�������g�l���Z�b�g���� 
		set_vram_address( 14'h0000 );
		for( i = 0; i < 256; i++ ) begin
			for( j = 0; j < 8; j++ ) begin
				write_data( 0, j + ((i << 4) & 255) );
			end
		end

		//	Color Table �ɒ萔�l���Z�b�g���� 
		set_vram_address( 14'h2000 );
		for( i = 0; i < 32; i++ ) begin
			write_data( 0, 8'hF4 );
		end

		//	Sprite Generator Table �� FFh ���Z�b�g���� 
		set_vram_address( 14'h3800 );
		for( i = 0; i < 256; i++ ) begin
			for( j = 0; j < 8; j++ ) begin
				write_data( 0, 8'hFF );
			end
		end

		//	Sprite Attribute Table �ɃC���N�������g�l���Z�b�g���� 
		set_vram_address( 14'h1B00 );
		for( i = 0; i < 32; i++ ) begin
			write_data( 0, 8'h00 );				//	Y���W 
			write_data( 0, i * 8 );				//	X���W 
			write_data( 0, 0 );					//	�p�^�[���ԍ� 
			write_data( 0, (i & 7) + 8 );		//	�F 
		end

		timing = 1;
		@( posedge clk );

		timing = 0;

		# 50ms
		@( posedge clk );
		$finish;
	end
endmodule
