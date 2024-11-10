// -----------------------------------------------------------------------------
//	Test of ip_msxbus.v
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
	localparam		clk_base	= 1000000000/33000;
	//	cartridge slot signals
	reg				n_reset;
	reg				clk;
	reg		[15:0]	adr;
	reg		[7:0]	i_data;
	wire	[7:0]	o_data;
	wire			is_output;
	reg				n_sltsl;
	reg				n_rd;
	reg				n_wr;
	reg				n_ioreq;
	reg				n_mereq;
	//	internal signals
	wire	[15:0]	bus_address;
	reg				bus_read_ready;
	reg		[7:0]	bus_read_data;
	wire	[7:0]	bus_write_data;
	wire			bus_io_read;
	wire			bus_io_write;
	wire			bus_memory_read;
	wire			bus_memory_write;
	integer			count;
	logic	[15:0]	last_address;
	logic	[7:0]	last_write_data;
	logic	[7:0]	last_read_data;
	integer			test_no;
	logic			last_io_read;
	logic			last_io_write;
	logic			last_memory_read;
	logic			last_memory_write;
	logic	[7:0]	save_read_data;

	// --------------------------------------------------------------------
	//	DUT
	// --------------------------------------------------------------------
	ip_msxbus u_msxbus (
		.n_reset			( n_reset			),
		.clk				( clk				),
		.adr				( adr				),
		.i_data				( i_data			),
		.o_data				( o_data			),
		.is_output			( is_output			),
		.n_sltsl			( n_sltsl			),
		.n_rd				( n_rd				),
		.n_wr				( n_wr				),
		.n_ioreq			( n_ioreq			),
		.n_mereq			( n_mereq			),
		.bus_address		( bus_address		),
		.bus_read_ready		( bus_read_ready	),
		.bus_read_data		( bus_read_data		),
		.bus_write_data		( bus_write_data	),
		.bus_io_read		( bus_io_read		),
		.bus_io_write		( bus_io_write		),
		.bus_memory_read	( bus_memory_read	),
		.bus_memory_write	( bus_memory_write	)
	);

	// --------------------------------------------------------------------
	//	clock
	// --------------------------------------------------------------------
	always #(clk_base/2) begin
		clk <= ~clk;
	end

	// --------------------------------------------------------------------
	//	Memory write access
	// --------------------------------------------------------------------
	task memory_write(
		input	[15:0]	address,
		input	[7:0]	data
	);
		count = 0;
		adr = 16'dX;
		i_data = 8'dX;
		#170ns

		adr = address;
		#145ns

		n_mereq = 1'b0;
		#10ns

		n_sltsl = 1'b0;
		#55ns

		i_data = data;
		#209ns

		n_wr = 1'b0;
		#259ns

		n_wr = 1'b1;
		#25ns

		n_mereq = 1'b1;
		#10ns

		n_sltsl = 1'b1;
		#55ns

		@( posedge clk );
	endtask: memory_write

	// --------------------------------------------------------------------
	//	Memory read access
	// --------------------------------------------------------------------
	task memory_read(
		input	[15:0]	address,
		input	[7:0]	data
	);
		count = 0;
		adr = 16'dX;
		i_data = 8'dX;
		save_read_data = data;
		#170ns

		adr = address;
		#145ns

		n_mereq = 1'b0;
		#10ns

		n_sltsl = 1'b0;
		n_rd = 1'b0;
		#702ns

		last_read_data <= o_data;
		n_mereq = 1'b1;
		n_rd = 1'b1;
		#10ns

		n_sltsl = 1'b1;
		#279ns

		@( posedge clk );
	endtask: memory_read

	// --------------------------------------------------------------------
	//	Memory write access
	// --------------------------------------------------------------------
	task io_write(
		input	[15:0]	address,
		input	[7:0]	data
	);
		count = 0;
		adr = 16'dX;
		i_data = 8'dX;
		n_sltsl = 1'b1;
		#170ns

		adr = address;
		#145ns

		n_ioreq = 1'b0;
		#10ns

		n_sltsl = 1'b1;
		#55ns

		i_data = data;
		#209ns

		n_wr = 1'b0;
		#259ns

		n_wr = 1'b1;
		#25ns

		n_ioreq = 1'b1;
		#10ns

		n_sltsl = 1'b1;
		#55ns

		@( posedge clk );
	endtask: io_write

	// --------------------------------------------------------------------
	//	I/O read access
	// --------------------------------------------------------------------
	task io_read(
		input	[15:0]	address,
		input	[7:0]	data
	);
		count = 0;
		adr = 16'dX;
		i_data = 8'dX;
		save_read_data = data;
		n_sltsl = 1'b1;
		#170ns

		adr = address;
		#145ns

		n_ioreq = 1'b0;
		#10ns

		n_rd = 1'b0;
		#702ns

		last_read_data <= o_data;
		n_ioreq = 1'b1;
		n_rd = 1'b1;
		#10ns

		n_sltsl = 1'b1;
		#279ns

		@( posedge clk );
	endtask: io_read

	// --------------------------------------------------------------------
	//	Access check task
	// --------------------------------------------------------------------
	task access_check();
		last_write_data <= 8'd0;
		last_address <= 16'hAAAA;
		forever begin
			@( posedge clk );
			if( bus_io_write || bus_memory_write ) begin
				count <= count + 1;
				last_write_data <= bus_write_data;
				last_address <= bus_address;
				last_io_read <= bus_io_read;
				last_memory_read <= bus_memory_read;
				last_io_write <= bus_io_write;
				last_memory_write <= bus_memory_write;
				while( bus_io_write || bus_memory_write ) begin
					@( posedge clk );
				end
				@( posedge clk );
			end
			else if( bus_io_read || bus_memory_read ) begin
				count <= count + 1;
				last_address <= bus_address;
				last_io_read <= bus_io_read;
				last_memory_read <= bus_memory_read;
				last_io_write <= bus_io_write;
				last_memory_write <= bus_memory_write;
				@( posedge clk );
				@( posedge clk );
				@( posedge clk );
				@( posedge clk );
				bus_read_data <= save_read_data;
				bus_read_ready <= 1'b1;
				@( posedge clk );
				bus_read_data <= 8'd0;
				bus_read_ready <= 1'b0;
				while( bus_io_read || bus_memory_read ) begin
					@( posedge clk );
				end
				@( posedge clk );
			end
		end
	endtask: access_check

	// --------------------------------------------------------------------
	//	Test bench
	// --------------------------------------------------------------------
	initial begin
		test_no			= 0;
		n_reset			= 0;
		clk				= 0;
		adr				= 0;
		i_data			= 0;
		n_sltsl			= 1;
		n_rd			= 1;
		n_wr			= 1;
		n_ioreq			= 1;
		n_mereq			= 1;
		bus_read_ready	= 0;
		bus_read_data	= 0;

		@( negedge clk );
		@( negedge clk );

		fork
			access_check();
		join_none

		n_reset			= 1;
		repeat( 10 ) @( posedge clk );

		// --------------------------------------------------------------------
		//	memory write test
		// --------------------------------------------------------------------
		test_no			= 1;
		$display( "Memory Write Test" );
		@( posedge clk );
		#19ns
		memory_write( 16'h1234, 8'h12 );
		assert( count == 1 );
		assert( last_write_data == 8'h12 );
		assert( last_address == 16'h1234 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b1 );

		@( posedge clk );
		#38ns
		memory_write( 16'h2345, 8'h23 );
		assert( count == 1 );
		assert( last_write_data == 8'h23 );
		assert( last_address == 16'h2345 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b1 );

		@( posedge clk );
		#57ns
		memory_write( 16'h3456, 8'h34 );
		assert( count == 1 );
		assert( last_write_data == 8'h34 );
		assert( last_address == 16'h3456 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b1 );

		@( posedge clk );
		#76ns
		memory_write( 16'h4567, 8'h45 );
		assert( count == 1 );
		assert( last_write_data == 8'h45 );
		assert( last_address == 16'h4567 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b1 );

		@( posedge clk );
		#95ns
		memory_write( 16'h5678, 8'h65 );
		assert( count == 1 );
		assert( last_write_data == 8'h65 );
		assert( last_address == 16'h5678 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b1 );

		@( posedge clk );
		#114ns
		memory_write( 16'h6789, 8'hAB );
		assert( count == 1 );
		assert( last_write_data == 8'hAB );
		assert( last_address == 16'h6789 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b1 );

		@( posedge clk );
		#133ns
		memory_write( 16'h789A, 8'hCD );
		assert( count == 1 );
		assert( last_write_data == 8'hCD );
		assert( last_address == 16'h789A );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b1 );

		@( posedge clk );
		#152ns
		memory_write( 16'h89AB, 8'hEF );
		assert( count == 1 );
		assert( last_write_data == 8'hEF );
		assert( last_address == 16'h89AB );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b1 );

		// --------------------------------------------------------------------
		//	I/O write test
		// --------------------------------------------------------------------
		test_no			= 2;
		$display( "I/O Write Test" );
		@( posedge clk );
		#19ns
		io_write( 16'h1234, 8'h12 );
		assert( count == 1 );
		assert( last_write_data == 8'h12 );
		assert( last_address == 16'h1234 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b1 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#38ns
		io_write( 16'h2345, 8'h23 );
		assert( count == 1 );
		assert( last_write_data == 8'h23 );
		assert( last_address == 16'h2345 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b1 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#57ns
		io_write( 16'h3456, 8'h34 );
		assert( count == 1 );
		assert( last_write_data == 8'h34 );
		assert( last_address == 16'h3456 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b1 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#76ns
		io_write( 16'h4567, 8'h45 );
		assert( count == 1 );
		assert( last_write_data == 8'h45 );
		assert( last_address == 16'h4567 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b1 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#95ns
		io_write( 16'h5678, 8'h65 );
		assert( count == 1 );
		assert( last_write_data == 8'h65 );
		assert( last_address == 16'h5678 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b1 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#114ns
		io_write( 16'h6789, 8'hAB );
		assert( count == 1 );
		assert( last_write_data == 8'hAB );
		assert( last_address == 16'h6789 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b1 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#133ns
		io_write( 16'h789A, 8'hCD );
		assert( count == 1 );
		assert( last_write_data == 8'hCD );
		assert( last_address == 16'h789A );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b1 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#152ns
		io_write( 16'h89AB, 8'hEF );
		assert( count == 1 );
		assert( last_write_data == 8'hEF );
		assert( last_address == 16'h89AB );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b1 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		// --------------------------------------------------------------------
		//	memory read test
		// --------------------------------------------------------------------
		test_no			= 3;
		$display( "Memory Read Test" );
		@( posedge clk );
		#19ns
		memory_read( 16'h1234, 8'h12 );
		assert( count == 1 );
		assert( last_read_data == 8'h12 );
		assert( last_address == 16'h1234 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b1 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#38ns
		memory_read( 16'h2345, 8'h23 );
		assert( count == 1 );
		assert( last_read_data == 8'h23 );
		assert( last_address == 16'h2345 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b1 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#57ns
		memory_read( 16'h3456, 8'h34 );
		assert( count == 1 );
		assert( last_read_data == 8'h34 );
		assert( last_address == 16'h3456 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b1 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#76ns
		memory_read( 16'h4567, 8'h45 );
		assert( count == 1 );
		assert( last_read_data == 8'h45 );
		assert( last_address == 16'h4567 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b1 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#95ns
		memory_read( 16'h5678, 8'h65 );
		assert( count == 1 );
		assert( last_read_data == 8'h65 );
		assert( last_address == 16'h5678 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b1 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#114ns
		memory_read( 16'h6789, 8'hAB );
		assert( count == 1 );
		assert( last_read_data == 8'hAB );
		assert( last_address == 16'h6789 );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b1 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#133ns
		memory_read( 16'h789A, 8'hCD );
		assert( count == 1 );
		assert( last_read_data == 8'hCD );
		assert( last_address == 16'h789A );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b1 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#152ns
		memory_read( 16'h89AB, 8'hEF );
		assert( count == 1 );
		assert( last_read_data == 8'hEF );
		assert( last_address == 16'h89AB );
		assert( last_io_read == 1'b0 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b1 );
		assert( last_memory_write == 1'b0 );

		// --------------------------------------------------------------------
		//	I/O read test
		// --------------------------------------------------------------------
		test_no			= 4;
		$display( "I/O Read Test" );
		@( posedge clk );
		#19ns
		io_read( 16'h1234, 8'h12 );
		assert( count == 1 );
		assert( last_read_data == 8'h12 );
		assert( last_address == 16'h1234 );
		assert( last_io_read == 1'b1 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#38ns
		io_read( 16'h2345, 8'h23 );
		assert( count == 1 );
		assert( last_read_data == 8'h23 );
		assert( last_address == 16'h2345 );
		assert( last_io_read == 1'b1 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#57ns
		io_read( 16'h3456, 8'h34 );
		assert( count == 1 );
		assert( last_read_data == 8'h34 );
		assert( last_address == 16'h3456 );
		assert( last_io_read == 1'b1 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#76ns
		io_read( 16'h4567, 8'h45 );
		assert( count == 1 );
		assert( last_read_data == 8'h45 );
		assert( last_address == 16'h4567 );
		assert( last_io_read == 1'b1 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#95ns
		io_read( 16'h5678, 8'h65 );
		assert( count == 1 );
		assert( last_read_data == 8'h65 );
		assert( last_address == 16'h5678 );
		assert( last_io_read == 1'b1 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#114ns
		io_read( 16'h6789, 8'hAB );
		assert( count == 1 );
		assert( last_read_data == 8'hAB );
		assert( last_address == 16'h6789 );
		assert( last_io_read == 1'b1 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#133ns
		io_read( 16'h789A, 8'hCD );
		assert( count == 1 );
		assert( last_read_data == 8'hCD );
		assert( last_address == 16'h789A );
		assert( last_io_read == 1'b1 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		@( posedge clk );
		#152ns
		io_read( 16'h89AB, 8'hEF );
		assert( count == 1 );
		assert( last_read_data == 8'hEF );
		assert( last_address == 16'h89AB );
		assert( last_io_read == 1'b1 );
		assert( last_io_write == 1'b0 );
		assert( last_memory_read == 1'b0 );
		assert( last_memory_write == 1'b0 );

		$finish;
	end
endmodule
