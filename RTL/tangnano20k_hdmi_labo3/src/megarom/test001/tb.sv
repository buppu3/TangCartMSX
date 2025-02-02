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
	localparam		clk_base	= 1_000_000_000/42_955;	//	ps
	reg				clk;
	reg				reset_n;
	reg				mreq_n;
	reg				wr_n;
	reg				rd_n;
	reg		[15:0]	address;
	reg		[7:0]	wdata;
	wire			megarom_rd_n;
	wire	[21:0]	megarom_address;
	reg		[2:0]	mode;

	// --------------------------------------------------------------------
	//	DUT
	// --------------------------------------------------------------------
	megarom u_megarom (
		.clk				( clk				),
		.reset_n			( reset_n			),
		.mreq_n				( mreq_n			),
		.wr_n				( wr_n				),
		.rd_n				( rd_n				),
		.address			( address			),
		.wdata				( wdata				),
		.megarom_rd_n		( megarom_rd_n		),
		.megarom_address	( megarom_address	),
		.mode				( mode				)
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
	task reg_write(
		input	[15:0]	p_address,
		input	[7:0]	p_data
	);
		int count;

		count			<= 0;
		sltsl			<= 1'b1;
		mreq_n			<= 1'b0;
		wr_n			<= 1'b0;
		address			<= p_address;
		wdata			<= p_data;
		@( posedge clk );
		sltsl			<= 1'b0;
		mreq_n			<= 1'b1;
		wr_n			<= 1'b1;
		@( posedge clk );
	endtask : reg_write

	// --------------------------------------------------------------------
	task reg_read(
		input	[15:0]	p_address,
		input	[7:0]	p_reference_data
	);
		int count;

		count			<= 0;
		sltsl			<= 1'b1;
		mreq_n			<= 1'b0;
		rd_n			<= 1'b0;
		address			<= p_address;
		wdata			<= 8'd0;
		@( posedge clk );

		sltsl			<= 1'b0;
		mreq_n			<= 1'b1;
		rd_n			<= 1'b1;
		while( !rdata_en && count < 5 ) begin
			count	<= count + 1;
			@( posedge clk );
		end

		if( rdata == p_reference_data ) begin
			$display( "[OK] read( %04X ) == %02X", p_address, p_reference_data );
		end
		else begin
			$display( "[NG] read( %04X ) == %02X != %02X", p_address, p_reference_data, rdata );
		end
		@( posedge clk );
	endtask : reg_read

	// --------------------------------------------------------------------
	task check_sltsl(
		input	[15:0]	p_address,
		input	[3:0]	p_sltsl
	);
		sltsl		<= 1'b1;
		mreq_n		<= 1'b0;
		wr_n		<= 1'b1;
		rd_n		<= 1'b1;
		address		<= p_address;
		@( posedge clk );

		if( p_sltsl == { sltsl_ext3, sltsl_ext2, sltsl_ext1, sltsl_ext0 } ) begin
			$display( "[OK] SLTSL == %02X", p_sltsl );
		end
		else begin
			$display( "[NG] SLTSL == %02X != %02X", p_sltsl, { sltsl_ext3, sltsl_ext2, sltsl_ext1, sltsl_ext0 } );
		end
		@( posedge clk );
	endtask : check_sltsl

	// --------------------------------------------------------------------
	//	Test bench
	// --------------------------------------------------------------------
	initial begin
		reset_n				= 0;
		clk					= 0;
		sltsl				= 0;
		mreq_n				= 1;
		wr_n				= 0;
		rd_n				= 1;
		address				= 0;
		wdata				= 0;

		@( negedge clk );
		@( negedge clk );
		@( posedge clk );

		reset_n				= 1'b1;
		@( posedge clk );
		repeat( 10 ) @( posedge clk );

		$display( "<<TEST001>> FFFFh register Test" );
		reg_write( 16'hFFFF, 8'h12 );
		reg_read( 16'hFFFF, ~8'h12 );
		reg_write( 16'hFFFF, 8'h23 );
		reg_read( 16'hFFFF, ~8'h23 );
		reg_write( 16'hFFFF, 8'h34 );
		reg_read( 16'hFFFF, ~8'h34 );
		reg_write( 16'hFFFF, 8'h56 );
		reg_read( 16'hFFFF, ~8'h56 );
		reg_write( 16'hFFFF, 8'haf );
		reg_read( 16'hFFFF, ~8'haf );
		reg_write( 16'hFFFF, 8'h9a );
		reg_read( 16'hFFFF, ~8'h9a );

		$display( "<<TEST002>> SLTSL signals Test" );
		reg_write( 16'hFFFF, { 2'd3, 2'd2, 2'd1, 2'd0 } );
		check_sltsl( 16'h0000, 4'b0001 );
		check_sltsl( 16'h4000, 4'b0010 );
		check_sltsl( 16'h8000, 4'b0100 );
		check_sltsl( 16'hC000, 4'b1000 );

		reg_write( 16'hFFFF, { 2'd0, 2'd1, 2'd2, 2'd3 } );
		check_sltsl( 16'h0000, 4'b1000 );
		check_sltsl( 16'h4000, 4'b0100 );
		check_sltsl( 16'h8000, 4'b0010 );
		check_sltsl( 16'hC000, 4'b0001 );
		$finish;
	end
endmodule
