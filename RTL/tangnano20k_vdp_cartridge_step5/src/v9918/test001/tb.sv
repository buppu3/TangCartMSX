// -----------------------------------------------------------------------------
//	Test of v9918.v
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
	reg						clk;
	reg						reset_n;
	reg						initial_busy;
	reg			[15:0]		bus_address;
	reg						bus_ioreq;
	reg						bus_write;
	reg						bus_valid;
	reg			[7:0]		bus_wdata;
	wire		[7:0]		bus_rdata;
	wire					bus_rdata_en;
	wire					bus_ready;

	wire					int_n;

	wire		[13:0]		p_dram_address;
	wire					p_dram_write;
	wire					p_dram_valid;
	wire					p_dram_ready;
	wire		[7:0]		p_dram_wdata;
	wire		[7:0]		p_dram_rdata;
	wire					p_dram_rdata_en;

	// video wire
	wire					p_vdp_enable;
	wire		[5:0]		p_vdp_r;
	wire		[5:0]		p_vdp_g;
	wire		[5:0]		p_vdp_b;
	wire		[10:0]		p_vdp_hcounter;
	wire		[10:0]		p_vdp_vcounter;

	int						i, j, k;
	reg						timing;

	// --------------------------------------------------------------------
	//	DUT
	// --------------------------------------------------------------------
	vdp_inst u_vdp (
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

	assign bus_ready		= 1'b1;

	// --------------------------------------------------------------------
	ip_ram u_vram (
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
	//	Tasks
	// --------------------------------------------------------------------
	task write_data(
		input			p_address,
		input	[7:0]	p_data
	);
		$display( "write_data( 0x%06X, 0x%02X )", p_address, p_data );
		bus_address	<= p_address;
		bus_wdata	<= p_data;
		bus_ioreq	<= 1'b1;
		bus_write	<= 1'b1;
		bus_valid	<= 1'b1;
		@( posedge clk );
		while( bus_ready == 1'b0 ) @( posedge clk );

		bus_ioreq	<= 1'b0;
		bus_write	<= 1'b0;
		bus_valid	<= 1'b0;
		@( posedge clk );
	endtask: write_data

	// --------------------------------------------------------------------
	task read_data(
		input	[22:0]	p_address,
		input	[15:0]	p_data
	);
		int time_out;

		$display( "read_data( 0x%06X, 0x%02X )", p_address, p_data );
		bus_address	<= p_address;
		bus_wdata	<= p_data;
		bus_ioreq	<= 1'b1;
		bus_write	<= 1'b1;
		bus_valid	<= 1'b1;
		@( posedge clk );
		while( bus_ready == 1'b0 ) @( posedge clk );

		bus_ioreq	<= 1'b0;
		bus_valid	<= 1'b0;
		while( bus_rdata_en == 1'b0 ) @( posedge clk );

		assert( bus_rdata == p_data );
		if( bus_rdata != p_data ) begin
			$display( "-- p_data = %08X (ref: %08X)", bus_rdata, p_data );
		end

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
		clk = 0;
		reset_n = 0;
		initial_busy = 0;
		bus_address = 0;
		bus_ioreq = 0;
		bus_write = 0;
		bus_valid = 0;
		bus_wdata = 0;

		@( negedge clk );
		@( negedge clk );
		@( posedge clk );

		reset_n			= 1;
		@( posedge clk );

//		//	�A���������݂̃A�h���X�w�� 
//		write_data( 1, 8'h00 );
//		write_data( 1, 8'h40 );
//
//		write_data( 0, 8'hAA );
//		write_data( 0, 8'h55 );
//		write_data( 0, 8'hA5 );
//		write_data( 0, 8'h5A );
//		write_data( 0, 8'h12 );
//		write_data( 0, 8'h34 );
//		write_data( 0, 8'h56 );
//		write_data( 0, 8'h78 );
//
//		//	�A���ǂݏo���̃A�h���X�w��
//		write_data( 1, 8'h00 );
//		write_data( 1, 8'h00 );
//
//		read_data( 0, 8'hAA );
//		read_data( 0, 8'h55 );
//		read_data( 0, 8'hA5 );
//		read_data( 0, 8'h5A );
//		read_data( 0, 8'h12 );
//		read_data( 0, 8'h34 );
//		read_data( 0, 8'h56 );
//		read_data( 0, 8'h78 );
//
//		//	�X�e�[�^�X���W�X�^�̓ǂݏo��
//		read_data( 1, 8'h1f );
//
//		//	���W�X�^��������
//		write_reg( 1, 8'hA0 );
//
//		repeat( 100 ) @( posedge clk );
//
//		// ====================================================================
//		//	GRAPHIC1 (SCREEN1) �ɃZ�b�g���� 
//		timing = 1;
//		write_reg( 0, 8'h00 );			//	R#0 = 0x00 : Mode 0 : SCREEN1
//		write_reg( 1, 8'h02 );			//	R#1 = 0x02 : Mode 1 : SCREEN1, 16x16 Sprite
//		write_reg( 2, 8'h06 );			//	R#2 = 0x06 : Pattern Name Table      0x1800 = 01_1000_0000_0000 �� 01_10
//		write_reg( 3, 8'h80 );			//	R#3 = 0x80 : Color Table             0x2000 = 10_0000_0000_0000 �� 10_0000_00
//		write_reg( 4, 8'h00 );			//	R#4 = 0x00 : Pattern Generator Table 0x0000 = 00_0000_0000_0000 �� 00_0
//		write_reg( 5, 8'h36 );			//	R#5 = 0x36 : Sprite Attribute Table  0x1B00 = 01_1011_0000_0000 �� 01_1011_0
//		write_reg( 6, 8'h07 );			//	R#6 = 0x07 : Sprite Generator Table  0x3800 = 11_1000_0000_0000 �� 11_1
//		timing = 0;
//
//		//	VRAM���[���N���A���� 
//		set_vram_address( 14'h0000 );
//		for( i = 0; i < 16384; i++ ) begin
//			write_data( 0, 8'h00 );
//		end
//
//		//	Pattern Name Table �ɃC���N�������g�l���Z�b�g���� 
//		set_vram_address( 14'h1800 );
//		for( i = 0; i < 768; i++ ) begin
//			write_data( 0, i & 255 );
//		end
//
//		//	Pattern Generator Table �ɃC���N�������g�l���Z�b�g���� 
//		set_vram_address( 14'h0000 );
//		for( i = 0; i < 256; i++ ) begin
//			for( j = 0; j < 8; j++ ) begin
//				write_data( 0, j + ((i << 4) & 255) );
//			end
//		end
//
//		//	Color Table �ɒ萔�l���Z�b�g���� 
//		set_vram_address( 14'h2000 );
//		for( i = 0; i < 32; i++ ) begin
//			write_data( 0, 8'hF4 );
//		end
//
//		//	Sprite Generator Table �� FFh ���Z�b�g���� 
//		set_vram_address( 14'h3800 );
//		for( i = 0; i < 256; i++ ) begin
//			for( j = 0; j < 8; j++ ) begin
//				write_data( 0, 8'hFF );
//			end
//		end
//
//		//	Sprite Attribute Table �ɃC���N�������g�l���Z�b�g���� 
//		set_vram_address( 14'h1B00 );
//		for( i = 0; i < 32; i++ ) begin
//			write_data( 0, 8'h00 );				//	Y���W 
//			write_data( 0, i * 8 );				//	X���W 
//			write_data( 0, 0 );					//	�p�^�[���ԍ� 
//			write_data( 0, (i & 7) + 8 );		//	�F 
//		end
//
//		timing = 1;
//		@( posedge clk );
//
//		timing = 0;
//
//		# 17ms
		repeat( 1368 * 600 ) @( posedge clk );
		$finish;
	end
endmodule
