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
	localparam		clk_base	= 1_000_000_000/85_909;	//	ps
	reg						reset;
	reg						clk;
	reg						bus_sltsl;
	reg						bus_memory_req;
	wire					bus_ack;
	reg						bus_wrt;
	reg			[15:0]		bus_address;
	reg			[7:0]		bus_wdata;
	wire		[7:0]		bus_rdata;
	wire					bus_rdata_en;
	wire					sltsl_ext0;
	wire					sltsl_ext1;
	wire					sltsl_ext2;
	wire					sltsl_ext3;

	// --------------------------------------------------------------------
	//	DUT
	// --------------------------------------------------------------------
	secondary_slot_inst u_secondary_slot_inst (
		.reset				( reset				),
		.clk				( clk				),
		.bus_sltsl			( bus_sltsl			),
		.bus_memory_req		( bus_memory_req	),
		.bus_ack			( bus_ack			),
		.bus_wrt			( bus_wrt			),
		.bus_address		( bus_address		),
		.bus_wdata			( bus_wdata			),
		.bus_rdata			( bus_rdata			),
		.bus_rdata_en		( bus_rdata_en		),
		.sltsl_ext0			( sltsl_ext0		),
		.sltsl_ext1			( sltsl_ext1		),
		.sltsl_ext2			( sltsl_ext2		),
		.sltsl_ext3			( sltsl_ext3		)
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
		bus_sltsl		<= 1'b1;
		bus_memory_req	<= 1'b1;
		bus_wrt			<= 1'b1;
		bus_address		<= p_address;
		bus_wdata		<= p_data;
		@( posedge clk );

		while( !bus_ack && count < 5 ) begin
			count	<= count + 1;
			@( posedge clk );
		end

		bus_sltsl		<= 1'b0;
		bus_memory_req	<= 1'b0;
		bus_wrt			<= 1'b0;
		@( posedge clk );
	endtask : reg_write

	// --------------------------------------------------------------------
	task reg_read(
		input	[15:0]	p_address,
		input	[7:0]	p_reference_data
	);
		int count;

		count			<= 0;
		bus_sltsl		<= 1'b1;
		bus_memory_req	<= 1'b1;
		bus_wrt			<= 1'b0;
		bus_address		<= p_address;
		bus_wdata		<= 8'd0;
		@( posedge clk );

		while( !bus_ack && count < 5 ) begin
			count	<= count + 1;
			@( posedge clk );
		end

		bus_sltsl		<= 1'b0;
		bus_memory_req	<= 1'b0;

		while( !bus_rdata_en ) begin
			@( posedge clk );
		end

		if( bus_rdata == p_reference_data ) begin
			$display( "[OK] read( %04X ) == %02X", p_address, p_reference_data );
		end
		else begin
			$display( "[NG] read( %04X ) == %02X != %02X", p_address, p_reference_data, bus_rdata );
		end
		@( posedge clk );
	endtask : reg_read

	// --------------------------------------------------------------------
	task check_sltsl(
		input	[15:0]	p_address,
		input	[3:0]	p_sltsl
	);
		bus_sltsl		<= 1'b1;
		bus_memory_req	<= 1'b1;
		bus_wrt			<= 1'b0;
		bus_address		<= p_address;
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
		reset				= 1;
		clk					= 0;
		bus_sltsl			= 0;
		bus_memory_req		= 0;
		bus_wrt				= 0;
		bus_address			= 0;
		bus_wdata			= 0;

		@( negedge clk );
		@( negedge clk );
		@( posedge clk );

		reset			= 1'b0;
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
