// -----------------------------------------------------------------------------
//	Test of vdp_ssg entity
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
	localparam		clk_base	= 1_000_000_000/87_750;	//	ps
	reg					reset;
	reg					clk;
	reg					enable;

	wire		[10:0]	h_cnt;
	wire		[10:0]	v_cnt;
	wire		[1:0]	dot_state;
	wire		[2:0]	eight_dot_state;
	wire		[8:0]	pre_dot_counter_x;
	wire		[8:0]	pre_dot_counter_y;
	wire		[8:0]	pre_dot_counter_yp;
	wire				pre_window_y;
	wire				pre_window_y_sp;
	wire				field;
	wire				window_x;
	wire				p_video_dh_clk;
	wire				p_video_dl_clk;
	wire				p_video_vs_n;

	wire				hd;
	wire				vd;
	wire				hsync;
	wire				hsync_en;
	wire				v_blanking_start;

	reg					vdp_r9_pal_mode;
	reg					reg_r9_interlace_mode;
	reg					reg_r9_y_dots;
	reg			[7:0]	reg_r18_adj;
	reg			[7:0]	reg_r23_vstart_line;
	reg					reg_r25_msk;
	reg			[2:0]	reg_r27_h_scroll;
	reg					reg_r25_yjk;
	reg					centeryjk_r25_n;
	int					i, j;

	// --------------------------------------------------------------------
	//	DUT
	// --------------------------------------------------------------------
	vdp_ssg u_vdp_ssg (
		.reset					( reset					),
		.clk					( clk					),
		.enable					( enable				),
		.h_cnt					( h_cnt					),
		.v_cnt					( v_cnt					),
		.dot_state				( dot_state				),
		.eight_dot_state		( eight_dot_state		),
		.pre_dot_counter_x		( pre_dot_counter_x		),
		.pre_dot_counter_y		( pre_dot_counter_y		),
		.pre_dot_counter_yp		( pre_dot_counter_yp	),
		.pre_window_y			( pre_window_y			),
		.pre_window_y_sp		( pre_window_y_sp		),
		.field					( field					),
		.window_x				( window_x				),
		.p_video_dh_clk			( p_video_dh_clk		),
		.p_video_dl_clk			( p_video_dl_clk		),
		.p_video_vs_n			( p_video_vs_n			),
		.hd						( hd					),
		.vd						( vd					),
		.hsync					( hsync					),
		.hsync_en				( hsync_en				),
		.v_blanking_start		( v_blanking_start		),
		.vdp_r9_pal_mode		( vdp_r9_pal_mode		),
		.reg_r9_interlace_mode	( reg_r9_interlace_mode	),
		.reg_r9_y_dots			( reg_r9_y_dots			),
		.reg_r18_adj			( reg_r18_adj			),
		.reg_r23_vstart_line	( reg_r23_vstart_line	),
		.reg_r25_msk			( reg_r25_msk			),
		.reg_r27_h_scroll		( reg_r27_h_scroll		),
		.reg_r25_yjk			( reg_r25_yjk			),
		.centeryjk_r25_n		( centeryjk_r25_n		)
	);

	// --------------------------------------------------------------------
	//	clock
	// --------------------------------------------------------------------
	always #(clk_base/2) begin
		clk <= ~clk;
	end

	// --------------------------------------------------------------------
	//	Test bench
	// --------------------------------------------------------------------
	initial begin
		reset					= 1;
		clk						= 0;
		enable					= 1;
		vdp_r9_pal_mode			= 0;
		reg_r9_interlace_mode	= 0;
		reg_r9_y_dots			= 0;
		reg_r18_adj				= 0;
		reg_r23_vstart_line		= 0;
		reg_r25_msk				= 0;
		reg_r27_h_scroll		= 0;
		reg_r25_yjk				= 0;
		centeryjk_r25_n			= 0;

		@( negedge clk );
		@( negedge clk );
		@( posedge clk );

		reset					= 0;
		@( posedge clk );

		for( i = 0; i < 10; i++ ) begin
			$display( "[%d]", i );
			repeat( 1368 * 1000 ) begin
				@( posedge clk );
			end
		end

		$finish;
	end
endmodule
