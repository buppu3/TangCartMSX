// -----------------------------------------------------------------------------
//	Test of secondary_slot_inst.v
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
// -----------------------------------------------------------------------------

module tb ();
	localparam		clk_base	= 1_000_000_000/74_250;	//	ps
	reg				reset_n;
	reg				clk;
	reg				iorq_n;
	reg		[7:0]	address;
	reg				wr_n;
	reg		[7:0]	wdata;
	wire			vram_mreq_n;
	wire	[22:0]	vram_address;
	wire			vram_wr_n;
	wire			vram_rd_n;
	wire			vram_rfsh_n;
	wire	[ 7:0]	vram_wdata;
	reg		[31:0]	vram_rdata;
	reg				vram_rdata_en;

	reg		[31:0]	vram_rdata0;
	reg		[31:0]	vram_rdata1;
	reg		[31:0]	vram_rdata2;
	reg		[31:0]	vram_rdata3;
	reg		[31:0]	vram_rdata4;
	reg		[31:0]	vram_rdata5;
	reg				vram_rdata_en0;
	reg				vram_rdata_en1;
	reg				vram_rdata_en2;
	reg				vram_rdata_en3;
	reg				vram_rdata_en4;
	reg				vram_rdata_en5;

	wire			video_de;
	wire			video_hs;
	wire			video_vs;
	wire	[7:0]	video_r;
	wire	[7:0]	video_g;
	wire	[7:0]	video_b;
	int				i, j, k, p;
	reg		[7:0]	ff_ram [0: 8192 * 1024 - 1];

	// --------------------------------------------------------------------
	//	DUT
	// --------------------------------------------------------------------
	ip_video u_dut (
		.reset_n			( reset_n			),
		.clk				( clk				),
		.iorq_n				( iorq_n			),
		.wr_n				( wr_n				),
		.address			( address			),
		.wdata				( wdata				),
		.vram_mreq_n		( vram_mreq_n		),
		.vram_address		( vram_address		),
		.vram_wr_n			( vram_wr_n			),
		.vram_rd_n			( vram_rd_n			),
		.vram_rfsh_n		( vram_rfsh_n		),
		.vram_wdata			( vram_wdata		),
		.vram_rdata			( vram_rdata		),
		.vram_rdata_en		( vram_rdata_en		),
		.video_de			( video_de			),
		.video_hs			( video_hs			),
		.video_vs			( video_vs			),
		.video_r			( video_r			),
		.video_g			( video_g			),
		.video_b			( video_b			)
	);

	// --------------------------------------------------------------------
	//	clock
	// --------------------------------------------------------------------
	always #(clk_base/2) begin
		clk <= ~clk;
	end

	// --------------------------------------------------------------------
	//	RAM
	// --------------------------------------------------------------------
	always @( posedge clk ) begin
		if( !vram_mreq_n ) begin
			if( !vram_wr_n ) begin
				ff_ram[ vram_address ] <= vram_wdata;
				vram_rdata_en0	<= 1'b0;
			end
			else if( !vram_rd_n ) begin
				vram_rdata0		<= { 
					ff_ram[ { vram_address[22:2], 2'd3 } ], 
					ff_ram[ { vram_address[22:2], 2'd2 } ], 
					ff_ram[ { vram_address[22:2], 2'd1 } ], 
					ff_ram[ { vram_address[22:2], 2'd0 } ] };
				vram_rdata_en0	<= 1'b1;
			end
			else begin
				vram_rdata_en0	<= 1'b0;
			end
		end
		else begin
			vram_rdata_en0	<= 1'b0;
		end
	end

	always @( posedge clk ) begin
		vram_rdata_en1 <= vram_rdata_en0;
		vram_rdata_en2 <= vram_rdata_en1;
		vram_rdata_en3 <= vram_rdata_en2;
		vram_rdata_en4 <= vram_rdata_en3;
		vram_rdata_en5 <= vram_rdata_en4;
		vram_rdata_en  <= vram_rdata_en5;

		vram_rdata1 <= vram_rdata0;
		vram_rdata2 <= vram_rdata1;
		vram_rdata3 <= vram_rdata2;
		vram_rdata4 <= vram_rdata3;
		vram_rdata5 <= vram_rdata4;
		vram_rdata  <= vram_rdata5;
	end

	// --------------------------------------------------------------------
	//	task
	// --------------------------------------------------------------------
	task set_palette_address(
		input	[7:0]	palette
	);
		iorq_n	<= 1'b0;
		wr_n	<= 1'b0;
		address	<= 8'h20;
		wdata	<= palette;
		@( posedge clk );

		iorq_n	<= 1'b1;
		wr_n	<= 1'b1;
		@( posedge clk );
	endtask: set_palette_address

	// --------------------------------------------------------------------
	task set_palette_color(
		input	[7:0]	palette_r,
		input	[7:0]	palette_g,
		input	[7:0]	palette_b
	);
		iorq_n	<= 1'b0;
		wr_n	<= 1'b0;
		address	<= 8'h21;
		wdata	<= palette_r;
		@( posedge clk );

		iorq_n	<= 1'b1;
		wr_n	<= 1'b1;
		@( posedge clk );

		iorq_n	<= 1'b0;
		wr_n	<= 1'b0;
		address	<= 8'h21;
		wdata	<= palette_g;
		@( posedge clk );

		iorq_n	<= 1'b1;
		wr_n	<= 1'b1;
		@( posedge clk );

		iorq_n	<= 1'b0;
		wr_n	<= 1'b0;
		address	<= 8'h21;
		wdata	<= palette_b;
		@( posedge clk );

		iorq_n	<= 1'b1;
		wr_n	<= 1'b1;
		@( posedge clk );
	endtask: set_palette_color

	// --------------------------------------------------------------------
	task set_address(
		input	[22:0]	p_address
	);
		iorq_n	<= 1'b0;
		wr_n	<= 1'b0;
		address	<= 8'h22;
		wdata	<= p_address[7:0];
		@( posedge clk );

		iorq_n	<= 1'b1;
		wr_n	<= 1'b1;
		@( posedge clk );

		iorq_n	<= 1'b0;
		wr_n	<= 1'b0;
		address	<= 8'h22;
		wdata	<= p_address[15:8];
		@( posedge clk );

		iorq_n	<= 1'b1;
		wr_n	<= 1'b1;
		@( posedge clk );

		iorq_n	<= 1'b0;
		wr_n	<= 1'b0;
		address	<= 8'h22;
		wdata	<= { 1'b0, p_address[22:16] };
		@( posedge clk );

		iorq_n	<= 1'b1;
		wr_n	<= 1'b1;
		@( posedge clk );
	endtask: set_address

	// --------------------------------------------------------------------
	task set_data(
		input	[7:0]	data
	);
		iorq_n	<= 1'b0;
		wr_n	<= 1'b0;
		address	<= 8'h23;
		wdata	<= data;
		@( posedge clk );

		iorq_n	<= 1'b1;
		wr_n	<= 1'b1;
		@( posedge clk );
	endtask: set_data

	// --------------------------------------------------------------------
	//	Test bench
	// --------------------------------------------------------------------
	initial begin
		reset_n			= 0;
		clk				= 0;
		iorq_n			= 1;
		address			= 0;
		wr_n			= 1;
		wdata			= 0;
		vram_rdata		= 0;
		vram_rdata_en	= 0;

		@( negedge clk );
		@( negedge clk );
		@( posedge clk );

		reset_n				= 1'b1;
		@( posedge clk );
		repeat( 10 ) @( posedge clk );

		// --------------------------------------------------------------------
		//	set palette color
		// --------------------------------------------------------------------
		set_palette_address( 0 );
		for( i = 0; i < 8; i++ ) begin
			for( j = 0; j < 8; j++ ) begin
				for( k = 0; k < 4; k++ ) begin
					set_palette_color( i * 255 / 7, j * 255 / 7, k * 255 / 3 );
				end
			end
		end

		repeat( 10 ) @( posedge clk );

		set_address( 0 );
		for( i = 0; i < 360; i++ ) begin
			for( j = 0; j < 640; j++ ) begin
				set_data( i & 255 );
				repeat( 20 ) @( posedge clk );
			end
		end

		repeat( 100000 ) @( posedge clk );
		$finish;
	end
endmodule
