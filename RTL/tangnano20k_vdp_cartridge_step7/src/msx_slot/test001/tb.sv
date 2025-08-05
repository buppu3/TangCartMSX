// -----------------------------------------------------------------------------
//	Test of msx_slot.v
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
	localparam		clk_base		= 1_000_000_000/85_909_080;	//	ps
	localparam		cpu_clk_base	= 1_000_000_000/ 3_579_545;	//	ps
	reg				clk;
	reg				reset_n;
	reg				initial_busy;
	reg				p_slot_clk;
	reg				p_slot_reset_n;
	reg				p_slot_sltsl_n;
	reg				p_slot_mreq_n;
	reg				p_slot_ioreq_n;
	reg				p_slot_wr_n;
	reg				p_slot_rd_n;
	reg		[15:0]	p_slot_address;
	wire	[7:0]	p_slot_data;
	reg		[7:0]	ff_slot_data;
	wire			p_slot_data_dir;
	wire			p_slot_int;
	reg				int_n;
	wire			bus_memreq;
	wire			bus_ioreq;
	wire	[15:0]	bus_address;
	wire			bus_write;
	wire			bus_valid;
	reg				bus_ready;
	wire	[7:0]	bus_wdata;
	reg		[7:0]	bus_rdata;
	reg				bus_rdata_en;
	string			s_state;

	// --------------------------------------------------------------------
	//	DUT
	// --------------------------------------------------------------------
	msx_slot u_msx_slot (
		.clk					( clk					),
		.reset_n				( reset_n				),
		.initial_busy			( initial_busy			),
		.p_slot_reset_n			( p_slot_reset_n		),
		.p_slot_sltsl_n			( p_slot_sltsl_n		),
		.p_slot_mreq_n			( p_slot_mreq_n			),
		.p_slot_ioreq_n			( p_slot_ioreq_n		),
		.p_slot_wr_n			( p_slot_wr_n			),
		.p_slot_rd_n			( p_slot_rd_n			),
		.p_slot_address			( p_slot_address		),
		.p_slot_data			( p_slot_data			),
		.p_slot_data_dir		( p_slot_data_dir		),
		.p_slot_int				( p_slot_int			),
		.int_n					( int_n					),
		.bus_memreq				( bus_memreq			),
		.bus_ioreq				( bus_ioreq				),
		.bus_address			( bus_address			),
		.bus_write				( bus_write				),
		.bus_valid				( bus_valid				),
		.bus_ready				( bus_ready				),
		.bus_wdata				( bus_wdata				),
		.bus_rdata				( bus_rdata				),
		.bus_rdata_en			( bus_rdata_en			)
	);

	assign p_slot_data	= (p_slot_data_dir == 1'b1) ? 8'hZZ: ff_slot_data;

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
		input	[15:0]	address,
		input	[7:0]	wdata
	);
		fork
			//	CPU clock
			begin
				//	T1
				s_state		= "T1";
				p_slot_clk	= 1'b1;
				#(cpu_clk_base/2) p_slot_clk	= 1'b0;
				//	T2
				s_state		= "T2";
				#(cpu_clk_base/2) p_slot_clk	= 1'b1;
				#(cpu_clk_base/2) p_slot_clk	= 1'b0;
				//	TW
				s_state		= "TW";
				#(cpu_clk_base/2) p_slot_clk	= 1'b1;
				#(cpu_clk_base/2) p_slot_clk	= 1'b0;
				//	T3
				s_state		= "T3";
				#(cpu_clk_base/2) p_slot_clk	= 1'b1;
				#(cpu_clk_base/2) p_slot_clk	= 1'b0;
				//	T4
				s_state		= "T4";
				#(cpu_clk_base/2) p_slot_clk	= 1'b1;
				#(cpu_clk_base/2) p_slot_clk	= 1'b0;
				//	T5
				s_state		= "T5";
				#(cpu_clk_base/2) p_slot_clk	= 1'b1;
				#(cpu_clk_base/2) p_slot_clk	= 1'b0;
			end
			//	Address
			begin
				#170ns p_slot_address = address;
			end
			//	/IORQ
			begin
				p_slot_ioreq_n = 1'b1;
				//	T1
				@( negedge p_slot_clk );
				@( posedge p_slot_clk );
				#135ns p_slot_ioreq_n = 1'b0;
				@( negedge p_slot_clk );
				@( negedge p_slot_clk );
				@( negedge p_slot_clk );
				#145ns p_slot_ioreq_n = 1'b1;
			end
			//	/WR
			begin
				p_slot_wr_n = 1'b1;
				//	T1
				@( negedge p_slot_clk );
				@( posedge p_slot_clk );
				#125ns p_slot_wr_n = 1'b0;
				@( negedge p_slot_clk );
				@( negedge p_slot_clk );
				@( negedge p_slot_clk );
				#120ns p_slot_wr_n = 1'b1;
			end
			//	bus_ready
			begin
				assert( bus_valid == 1'b0 );
				@( posedge bus_valid );
				@( posedge clk );
				@( posedge clk );
				@( posedge clk );
				bus_ready = 1'b1;
				@( negedge clk );
				assert( bus_address == address );
				assert( bus_valid == 1'b1 );
				assert( bus_write == 1'b1 );
				assert( bus_ioreq == 1'b1 );
				@( posedge clk );
				bus_ready = 1'b0;
				@( negedge clk );
				assert( bus_valid == 1'b0 );
			end
			//	others
			begin
				ff_slot_data	= wdata;
				p_slot_sltsl_n	= 1'b1;
				p_slot_mreq_n	= 1'b1;
			end
		join
	endtask: write_io

	// --------------------------------------------------------------------
	task write_io_ex(
		input	[15:0]	address,
		input	[7:0]	wdata
	);
		fork
			//	CPU clock
			begin
				//	T1
				s_state		= "T1";
				p_slot_clk	= 1'b1;
				#(cpu_clk_base/2) p_slot_clk	= 1'b0;
				//	T2
				s_state		= "T2";
				#(cpu_clk_base/2) p_slot_clk	= 1'b1;
				#(cpu_clk_base/2) p_slot_clk	= 1'b0;
				//	TW
				s_state		= "TW";
				#(cpu_clk_base/2) p_slot_clk	= 1'b1;
				#(cpu_clk_base/2) p_slot_clk	= 1'b0;
				//	T3
				s_state		= "T3";
				#(cpu_clk_base/2) p_slot_clk	= 1'b1;
				#(cpu_clk_base/2) p_slot_clk	= 1'b0;
				//	T4
				s_state		= "T4";
				#(cpu_clk_base/2) p_slot_clk	= 1'b1;
				#(cpu_clk_base/2) p_slot_clk	= 1'b0;
				//	T5
				s_state		= "T5";
				#(cpu_clk_base/2) p_slot_clk	= 1'b1;
				#(cpu_clk_base/2) p_slot_clk	= 1'b0;
			end
			//	Address
			begin
				#170ns p_slot_address = address;
			end
			//	/IORQ
			begin
				p_slot_ioreq_n = 1'b1;
				//	T1
				@( negedge p_slot_clk );
				@( posedge p_slot_clk );
				#175ns p_slot_ioreq_n = 1'b0;
				@( negedge p_slot_clk );
				@( negedge p_slot_clk );
				@( negedge p_slot_clk );
				#185ns p_slot_ioreq_n = 1'b1;
			end
			//	/WR
			begin
				p_slot_wr_n = 1'b1;
				//	T1
				@( negedge p_slot_clk );
				@( posedge p_slot_clk );
				#165ns p_slot_wr_n = 1'b0;
				@( negedge p_slot_clk );
				@( negedge p_slot_clk );
				@( negedge p_slot_clk );
				#150ns p_slot_wr_n = 1'b1;
			end
			//	bus_ready
			begin
				assert( bus_valid == 1'b0 );
				@( posedge bus_valid );
				@( posedge clk );
				@( posedge clk );
				@( posedge clk );
				bus_ready = 1'b1;
				@( negedge clk );
				assert( bus_address == address );
				assert( bus_valid == 1'b1 );
				assert( bus_write == 1'b1 );
				assert( bus_ioreq == 1'b1 );
				@( posedge clk );
				bus_ready = 1'b0;
				@( negedge clk );
				assert( bus_valid == 1'b0 );
			end
			//	others
			begin
				ff_slot_data	= wdata;
				p_slot_sltsl_n	= 1'b1;
				p_slot_mreq_n	= 1'b1;
			end
		join
	endtask: write_io_ex

	// --------------------------------------------------------------------
	//	Test bench
	// --------------------------------------------------------------------
	initial begin
		clk				= 0;			//	42.95454MHz
		initial_busy		= 0;
		p_slot_clk			= 0;
		p_slot_reset_n		= 0;
		p_slot_sltsl_n		= 1;
		p_slot_mreq_n		= 1;
		p_slot_ioreq_n		= 1;
		p_slot_wr_n			= 1;
		p_slot_rd_n			= 1;
		p_slot_address		= 0;
		bus_ready			= 0;
		bus_rdata			= 0;
		bus_rdata_en		= 0;
		int_n				= 1;

		@( negedge clk );
		@( negedge clk );

		p_slot_reset_n		= 1;
		@( posedge clk );
		@( posedge clk );

		// --------------------------------------------------------------------
		write_io( 16'h98, 8'h12 );
		write_io( 16'h89, 8'h23 );
		write_io( 16'h78, 8'h34 );
		write_io( 16'h67, 8'h45 );
		write_io( 16'h56, 8'h56 );
		write_io( 16'h45, 8'h67 );

		write_io_ex( 16'h98, 8'h12 );
		write_io_ex( 16'h89, 8'h23 );
		write_io_ex( 16'h78, 8'h34 );
		write_io_ex( 16'h67, 8'h45 );
		write_io_ex( 16'h56, 8'h56 );
		write_io_ex( 16'h45, 8'h67 );

		repeat( 10 ) @( posedge clk );
		$finish;
	end
endmodule
