// -----------------------------------------------------------------------------
//	Test of video_out_hmag.v
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
	reg				clk;			//	42.95454MHz
	reg				reset_n;
	reg				enable;
	reg		[10:0]	vdp_hcounter;
	reg		[10:0]	vdp_vcounter;
	reg		[10:0]	h_cnt;
	reg		[5:0]	vdp_r;
	reg		[5:0]	vdp_g;
	reg		[5:0]	vdp_b;
	wire	[7:0]	video_r;
	wire	[7:0]	video_g;
	wire	[7:0]	video_b;
	reg		[7:0]	reg_left_offset;			//	0 ..... 112
	reg		[7:0]	reg_denominator;			//	144 ... 200
	reg		[7:0]	reg_normalize;				//	32768 / reg_denominator
	int				i, j;

	// --------------------------------------------------------------------
	//	DUT
	// --------------------------------------------------------------------
	video_out_hmag u_video_out_hmag (
		.clk					( clk					),
		.reset_n				( reset_n				),
		.enable					( enable				),
		.vdp_hcounter			( vdp_hcounter			),
		.vdp_vcounter			( vdp_vcounter[1:0]		),
		.h_cnt					( h_cnt					),
		.vdp_r					( vdp_r					),
		.vdp_g					( vdp_g					),
		.vdp_b					( vdp_b					),
		.video_r				( video_r				),
		.video_g				( video_g				),
		.video_b				( video_b				),
		.reg_left_offset		( reg_left_offset		),
		.reg_denominator		( reg_denominator		),
		.reg_normalize			( reg_normalize			)
	);

	// --------------------------------------------------------------------
	//	clock
	// --------------------------------------------------------------------
	always #(clk_base/2) begin
		clk <= ~clk;
	end

	always @( posedge clk ) begin
		if( !reset_n ) begin
			h_cnt <= 11'd0;
		end
		else if( h_cnt == 11'd1367 ) begin
			h_cnt <= 11'd0;
		end
		else begin
			h_cnt <= h_cnt + 11'd1;
		end
	end

	always @( posedge clk ) begin
		if( !reset_n ) begin
			vdp_hcounter <= 11'd0;
		end
		else if( !enable ) begin
			//	hold
		end
		else if( vdp_hcounter == 11'd1367 ) begin
			vdp_hcounter <= 11'd0;
		end
		else begin
			vdp_hcounter <= vdp_hcounter + 11'd1;
		end
	end

	always @( posedge clk ) begin
		if( !reset_n ) begin
			vdp_vcounter <= 11'd0;
		end
		else if( !enable ) begin
			//	hold
		end
		else if( vdp_hcounter == 11'd1367 ) begin
			if( vdp_vcounter == 11'd523 ) begin
				vdp_vcounter <= 11'd0;
			end
			else begin
				vdp_vcounter <= vdp_vcounter + 11'd1;
			end
		end
	end

	always @( posedge clk ) begin
		if( !reset_n ) begin
			enable <= 0;
		end
		else begin
			enable <= ~enable;
		end
	end

	// --------------------------------------------------------------------
	//	Test bench
	// --------------------------------------------------------------------
	initial begin
		clk				= 0;			//	42.95454MHz
		reset_n			= 0;
		vdp_r			= 0;
		vdp_g			= 0;
		vdp_b			= 0;
		reg_left_offset	= 16;			//	0 ..... 112
		reg_denominator	= 640 / 4;		//	144 ... 200
		reg_normalize	= 32768 / reg_denominator;
		
		@( negedge clk );
		@( negedge clk );
		@( posedge clk );

		reset_n			= 1;
		@( posedge clk );

		i = 0;
		j = 0;
		repeat( 1368 * 10 ) begin
			if( i == 3 ) begin
				i <= 0;
				if( j == 6 ) begin
					j <= 0;
				end
				else begin
					j <= j + 1;
				end
				
				case( j )
				0:
					begin
						vdp_r	<= 6'h3F;
						vdp_g	<= 6'h3F;
						vdp_b	<= 6'h3F;
					end
				1:
					begin
						vdp_r	<= 6'h3F;
						vdp_g	<= 6'h3F;
						vdp_b	<= 6'h3F;
					end
				2:
					begin
						vdp_r	<= 6'h3F;
						vdp_g	<= 6'h3F;
						vdp_b	<= 6'h3F;
					end
				default:
					begin
						vdp_r	<= 6'h01;
						vdp_g	<= 6'h01;
						vdp_b	<= 6'h01;
					end
				endcase
			end
			else begin
				i <= i + 1;
			end
			@( posedge clk );
		end
		$finish;
	end
endmodule
