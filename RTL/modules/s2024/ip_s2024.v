// -----------------------------------------------------------------------------
//	ip_s2024.v
//	Copyright (C) 2024 Takayuki Hara
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
//-----------------------------------------------------------------------------

module ip_s2024 (
	//	internal signals
	input			n_reset,
	input			clk,				//	85.90908MHz
	//	cartridge slot signals
	output			slot_pin01_n_cs1,
	output			slot_pin02_n_cs2,
	output			slot_pin03_n_cs12,
	output			slot_pin04_n_sltsl1,
	output			slot_pin04_n_sltsl2,
	output			slot_pin06_n_rfsh,
	input			slot_pin07_n_wait,
	input			slot_pin08_n_int,
	output			slot_pin09_n_m1,
	output			slot_pin11_n_iorq,
	output			slot_pin12_n_merq,
	output			slot_pin13_n_wr,
	output			slot_pin14_n_rd,
	inout			slot_pin15_n_reset,
	output	[15:0]	slot_pin17_pin32_a,
	inout	[7:0]	slot_pin33_pin40_d,
	output			slot_pin42_clock,
	output			slot_d_output,
	//	internal signals
	output	[15:0]	bus_address,
	output			bus_io_req,
	output			bus_memory_req,
	input			bus_ack,
	output			bus_wrt,
	output	[7:0]	bus_wdata,
	input	[7:0]	bus_rdata,
	input			bus_rdata_en,
	//	slot
	input			sltsl1,
	input			sltsl2,
	//	CPU0
	input			cpu0_reset_n,
	output			cpu0_enable,
	output			cpu0_wait_n,
	output			cpu0_int_n,
	input			cpu0_m1_n,
	input			cpu0_mreq_n,
	input			cpu0_iorq_n,
	input			cpu0_rd_n,
	input			cpu0_wr_n,
	input			cpu0_rfsh_n,
	input			cpu0_halt_n,
	input			cpu0_busak_n,
	input	[15:0]	cpu0_a,
	inout	[7:0]	cou0_d,
	//	CPU1
	input			cpu1_reset_n,
	output			cpu1_enable,
	output			cpu1_wait_n,
	output			cpu1_int_n,
	input			cpu1_m1_n,
	input			cpu1_mreq_n,
	input			cpu1_iorq_n,
	input			cpu1_rd_n,
	input			cpu1_wr_n,
	input			cpu1_rfsh_n,
	input			cpu1_halt_n,
	input			cpu1_busak_n,
	input	[15:0]	cpu1_a,
	inout	[7:0]	cou1_d,
);
endmodule
