//	
//	Z80 compatible microprocessor core
//	Copyright (c) 2002 Daniel Wallner (jesus@opencores.org)
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
//	This module is based on T80(Version : 0250_T80) by Daniel Wallner and 
//	modified by Takayuki Hara.
//
//	The following modifications have been made.
//	-- Convert VHDL code to Verilog code.
//	-- Some minor bug fixes.
//-----------------------------------------------------------------------------

module T80 is
	generic(
		Mode		: integer := 0;	 -- 0 => Z80, 1 => Fast Z80
		IOWait		: integer := 0;	 -- 0 => Single I/O cycle, 1 => Std I/O cycle
		Flag_C		: integer := 0;
		Flag_N		: integer := 1;
		Flag_P		: integer := 2;
		Flag_X		: integer := 3;
		Flag_H		: integer := 4;
		Flag_Y		: integer := 5;
		Flag_Z		: integer := 6;
		Flag_S		: integer := 7
	);
	port(
		RESET_n		: in std_logic;
		CLK_n		: in std_logic;
		CEN			: in std_logic;
		WAIT_n		: in std_logic;
		INT_n		: in std_logic;
		NMI_n		: in std_logic;
		BUSRQ_n		: in std_logic;
		M1_n		: out std_logic;
		IORQ		: out std_logic;
		NoRead		: out std_logic;
		Write		: out std_logic;
		RFSH_n		: out std_logic;
		HALT_n		: out std_logic;
		BUSAK_n		: out std_logic;
		A			: out std_logic_vector(15 downto 0);
		DInst		: in std_logic_vector(7 downto 0);
		DI			: in std_logic_vector(7 downto 0);
		DO			: out std_logic_vector(7 downto 0);
		MC			: out std_logic_vector(2 downto 0);
		TS			: out std_logic_vector(2 downto 0);
		IntCycle_n	: out std_logic;
		IntE		: out std_logic;
		Stop		: out std_logic;
		p_PC		: out std_logic_vector( 15 downto 0 )		-- added by t.hara 2020.07.28
	);
end T80;

architecture rtl of T80 is

	constant aNone	: std_logic_vector(2 downto 0) := "111";
	constant aBC	: std_logic_vector(2 downto 0) := "000";
	constant aDE	: std_logic_vector(2 downto 0) := "001";
	constant aXY	: std_logic_vector(2 downto 0) := "010";
	constant aIOA	: std_logic_vector(2 downto 0) := "100";
	constant aSP	: std_logic_vector(2 downto 0) := "101";
	constant aZI	: std_logic_vector(2 downto 0) := "110";

	-- Registers
	signal ACC, F			: std_logic_vector(7 downto 0);
	signal Ap, Fp			: std_logic_vector(7 downto 0);
	signal I				: std_logic_vector(7 downto 0);
	signal R				: unsigned(7 downto 0);
	signal SP, PC			: unsigned(15 downto 0);
	signal RegDIH			: std_logic_vector(7 downto 0);
	signal RegDIL			: std_logic_vector(7 downto 0);
	signal RegBusA			: std_logic_vector(15 downto 0);
	signal RegBusB			: std_logic_vector(15 downto 0);
	signal RegBusC			: std_logic_vector(15 downto 0);
	signal RegAddrA_r		: std_logic_vector(2 downto 0);
	signal RegAddrA			: std_logic_vector(2 downto 0);
	signal RegAddrB_r		: std_logic_vector(2 downto 0);
	signal RegAddrB			: std_logic_vector(2 downto 0);
	signal RegAddrC			: std_logic_vector(2 downto 0);
	signal RegWEH			: std_logic;
	signal RegWEL			: std_logic;
	signal Alternate		: std_logic;

	-- Help Registers
	signal TmpAddr			: std_logic_vector(15 downto 0);	-- Temporary address register
	signal IR				: std_logic_vector(7 downto 0);		-- Instruction register
	signal ISet				: std_logic_vector(1 downto 0);		-- Instruction set selector
	signal RegBusA_r		: std_logic_vector(15 downto 0);

	signal ID16				: signed(15 downto 0);
	signal Save_Mux			: std_logic_vector(7 downto 0);

	signal TState			: unsigned(2 downto 0);
	signal MCycle			: std_logic_vector(2 downto 0);
	signal IntE_FF1			: std_logic;
	signal IntE_FF2			: std_logic;
	signal Halt_FF			: std_logic;
	signal BusReq_s			: std_logic;
	signal BusAck			: std_logic;
	signal ClkEn			: std_logic;
	signal NMI_s			: std_logic;
	signal INT_s			: std_logic;
	signal IStatus			: std_logic_vector(1 downto 0);

	signal DI_Reg			: std_logic_vector(7 downto 0);
	signal T_Res			: std_logic;
	signal XY_State			: std_logic_vector(1 downto 0);
	signal Pre_XY_F_M		: std_logic_vector(2 downto 0);
	signal NextIs_XY_Fetch	: std_logic;
	signal XY_Ind			: std_logic;
	signal No_BTR			: std_logic;
	signal BTR_r			: std_logic;
	signal Auto_Wait		: std_logic;
	signal Auto_Wait_t1		: std_logic;
	signal Auto_Wait_t2		: std_logic;
	signal IncDecZ			: std_logic;

	-- ALU signals
	signal BusB				: std_logic_vector(7 downto 0);
	signal BusA				: std_logic_vector(7 downto 0);
	signal ALU_Q			: std_logic_vector(7 downto 0);
	signal F_Out			: std_logic_vector(7 downto 0);

	-- Registered micro code outputs
	signal Read_To_Reg_r	: std_logic_vector(4 downto 0);
	signal Arith16_r		: std_logic;
	signal Z16_r			: std_logic;
	signal ALU_Op_r			: std_logic_vector(3 downto 0);
	signal ALU_cpi_r		: std_logic;
	signal Save_ALU_r		: std_logic;
	signal PreserveC_r		: std_logic;
	signal MCycles			: std_logic_vector(2 downto 0);

	-- Micro code outputs
	signal MCycles_d		: std_logic_vector(2 downto 0);
	signal TStates			: std_logic_vector(2 downto 0);
	signal IntCycle			: std_logic;
	signal NMICycle			: std_logic;
	signal Inc_PC			: std_logic;
	signal Inc_WZ			: std_logic;
	signal IncDec_16		: std_logic_vector(3 downto 0);
	signal Prefix			: std_logic_vector(1 downto 0);
	signal Read_To_Acc		: std_logic;
	signal Read_To_Reg		: std_logic;
	signal Set_BusB_To		: std_logic_vector(3 downto 0);
	signal Set_BusA_To		: std_logic_vector(3 downto 0);
	signal ALU_Op			: std_logic_vector(3 downto 0);
	signal ALU_cpi			: std_logic;
	signal Save_ALU			: std_logic;
	signal PreserveC		: std_logic;
	signal Arith16			: std_logic;
	signal Set_Addr_To		: std_logic_vector(2 downto 0);
	signal Jump				: std_logic;
	signal JumpE			: std_logic;
	signal JumpXY			: std_logic;
	signal Call				: std_logic;
	signal RstP				: std_logic;
	signal LDZ				: std_logic;
	signal LDW				: std_logic;
	signal LDSPHL			: std_logic;
	signal IORQ_i			: std_logic;
	signal Special_LD		: std_logic_vector(2 downto 0);
	signal ExchangeDH		: std_logic;
	signal ExchangeRp		: std_logic;
	signal ExchangeAF		: std_logic;
	signal ExchangeRS		: std_logic;
	signal I_DJNZ			: std_logic;
	signal I_CPL			: std_logic;
	signal I_CCF			: std_logic;
	signal I_SCF			: std_logic;
	signal I_RETN			: std_logic;
	signal I_BT				: std_logic;
	signal I_BC				: std_logic;
	signal I_BTR			: std_logic;
	signal I_RLD			: std_logic;
	signal I_RRD			: std_logic;
	signal I_INRC			: std_logic;
	signal SetDI			: std_logic;
	signal SetEI			: std_logic;
	signal IMode			: std_logic_vector(1 downto 0);
	signal Halt				: std_logic;
	signal XYbit_undoc		: std_logic;

begin

	mcode : T80_MCode
		generic map(
			Mode => Mode,
			Flag_C => Flag_C,
			Flag_N => Flag_N,
			Flag_P => Flag_P,
			Flag_X => Flag_X,
			Flag_H => Flag_H,
			Flag_Y => Flag_Y,
			Flag_Z => Flag_Z,
			Flag_S => Flag_S)
		port map(
			IR => IR,
			ISet => ISet,
			MCycle => MCycle,
			F => F,
			NMICycle => NMICycle,
			IntCycle => IntCycle,
			XY_State => XY_State,
			MCycles => MCycles_d,
			TStates => TStates,
			Prefix => Prefix,
			Inc_PC => Inc_PC,
			Inc_WZ => Inc_WZ,
			IncDec_16 => IncDec_16,
			Read_To_Acc => Read_To_Acc,
			Read_To_Reg => Read_To_Reg,
			Set_BusB_To => Set_BusB_To,
			Set_BusA_To => Set_BusA_To,
			ALU_Op => ALU_Op,
			ALU_cpi => ALU_cpi,
			Save_ALU => Save_ALU,
			PreserveC => PreserveC,
			Arith16 => Arith16,
			Set_Addr_To => Set_Addr_To,
			IORQ => IORQ_i,
			Jump => Jump,
			JumpE => JumpE,
			JumpXY => JumpXY,
			Call => Call,
			RstP => RstP,
			LDZ => LDZ,
			LDW => LDW,
			LDSPHL => LDSPHL,
			Special_LD => Special_LD,
			ExchangeDH => ExchangeDH,
			ExchangeRp => ExchangeRp,
			ExchangeAF => ExchangeAF,
			ExchangeRS => ExchangeRS,
			I_DJNZ => I_DJNZ,
			I_CPL => I_CPL,
			I_CCF => I_CCF,
			I_SCF => I_SCF,
			I_RETN => I_RETN,
			I_BT => I_BT,
			I_BC => I_BC,
			I_BTR => I_BTR,
			I_RLD => I_RLD,
			I_RRD => I_RRD,
			I_INRC => I_INRC,
			SetDI => SetDI,
			SetEI => SetEI,
			IMode => IMode,
			Halt => Halt,
			NoRead => NoRead,
			Write => Write,
			XYbit_undoc => XYbit_undoc);

	alu : cz80_alu
		generic map(
			Mode => Mode,
			Flag_C => Flag_C,
			Flag_N => Flag_N,
			Flag_P => Flag_P,
			Flag_X => Flag_X,
			Flag_H => Flag_H,
			Flag_Y => Flag_Y,
			Flag_Z => Flag_Z,
			Flag_S => Flag_S)
		port map(
			Arith16 => Arith16_r,
			Z16 => Z16_r,
			ALU_cpi => ALU_cpi_r,
			ALU_Op => ALU_Op_r,
			IR => IR(5 downto 0),
			ISet => ISet,
			BusA => BusA,
			BusB => BusB,
			F_In => F,
			Q => ALU_Q,
			F_Out => F_Out);

	ClkEn <= CEN and not BusAck;

	T_Res <= 1'b1 when TState = unsigned(TStates) else 1'b0;

	NextIs_XY_Fetch <= 1'b1 when XY_State != "00" and XY_Ind = 1'b0 and
							((Set_Addr_To = aXY) or
							(MCycle = "001" and IR = "11001011") or
							(MCycle = "001" and IR = "00110110")) else 1'b0;

	Save_Mux <= BusB when ExchangeRp = 1'b1 else
		DI_Reg when Save_ALU_r = 1'b0 else
		ALU_Q;

	process (RESET_n, CLK_n)
	begin
		if RESET_n = 1'b0 begin
			PC <= (others => 1'b0);	-- Program Counter
			A <= (others => 1'b0);
			TmpAddr <= (others => 1'b0);
			IR <= "00000000";
			ISet <= "00";
			XY_State <= "00";
			IStatus <= "00";
			MCycles <= "000";
			DO <= "00000000";

			ACC <= (others => 1'b1);
			F <= (others => 1'b1);
			Ap <= (others => 1'b1);
			Fp <= (others => 1'b1);
			I <= (others => 1'b0);
			R <= (others => 1'b0);
			SP <= (others => 1'b1);
			Alternate <= 1'b0;

			Read_To_Reg_r <= "00000";
			F <= (others => 1'b1);
			Arith16_r <= 1'b0;
			BTR_r <= 1'b0;
			Z16_r <= 1'b0;
			ALU_Op_r <= "0000";
			ALU_cpi_r <= 1'b0;
			Save_ALU_r <= 1'b0;
			PreserveC_r <= 1'b0;
			XY_Ind <= 1'b0;

		else if CLK_n'event and CLK_n = 1'b1 begin

			if ClkEn = 1'b1 begin

			ALU_Op_r <= "0000";
			ALU_cpi_r <= 1'b0;
			Save_ALU_r <= 1'b0;
			Read_To_Reg_r <= "00000";

			MCycles <= MCycles_d;

			if IMode != "11" begin
				IStatus <= IMode;
			end

			Arith16_r <= Arith16;
			PreserveC_r <= PreserveC;
			if ISet = "10" and ALU_OP(2) = 1'b0 and ALU_OP(0) = 1'b1 and MCycle = "011" begin
				Z16_r <= 1'b1;
			else
				Z16_r <= 1'b0;
			end

			if MCycle  = "001" and TState(2) = 1'b0 begin
			-- MCycle = 1 and TState = 1, 2, or 3

				if TState = 2 and Wait_n = 1'b1 begin
					A(7 downto 0) <= std_logic_vector(R);
					A(15 downto 8) <= I;
					R(6 downto 0) <= R(6 downto 0) + 1;

					if Jump = 1'b0 and Call = 1'b0 and NMICycle = 1'b0 and IntCycle = 1'b0 and not (Halt_FF = 1'b1 or Halt = 1'b1) begin
						PC <= PC + 1;
					end

					if IntCycle = 1'b1 and IStatus = "01" begin
						IR <= "11111111";
					else if Halt_FF = 1'b1 or (IntCycle = 1'b1 and IStatus = "10") or NMICycle = 1'b1 begin
						IR <= "00000000";
					else
						IR <= DInst;
					end

					ISet <= "00";
					if Prefix != "00" begin
						if Prefix = "11" begin
							if IR(5) = 1'b1 begin
								XY_State <= "10";
							else
								XY_State <= "01";
							end
						else
							if Prefix = "10" begin
								XY_State <= "00";
								XY_Ind <= 1'b0;
							end
							ISet <= Prefix;
						end
					else
						XY_State <= "00";
						XY_Ind <= 1'b0;
					end
				end

			else
			-- either (MCycle > 1) OR (MCycle = 1 AND TState > 3)

				if MCycle = "110" begin
					XY_Ind <= 1'b1;
					if Prefix = "01" begin
						ISet <= "01";
					end
				end

				if T_Res = 1'b1 begin
					BTR_r <= (I_BT or I_BC or I_BTR) and not No_BTR;
					if Jump = 1'b1 begin
						A(15 downto 8) <= DI_Reg;
						A(7 downto 0) <= TmpAddr(7 downto 0);
						PC(15 downto 8) <= unsigned(DI_Reg);
						PC(7 downto 0) <= unsigned(TmpAddr(7 downto 0));
					else if JumpXY = 1'b1 begin
						A <= RegBusC;
						PC <= unsigned(RegBusC);
					else if Call = 1'b1 or RstP = 1'b1 begin
						A <= TmpAddr;
						PC <= unsigned(TmpAddr);
					else if MCycle = MCycles and NMICycle = 1'b1 begin
						A <= "0000000001100110";
						PC <= "0000000001100110";
					else if MCycle = "011" and IntCycle = 1'b1 and IStatus = "10" begin
						A(15 downto 8) <= I;
						A(7 downto 0) <= TmpAddr(7 downto 0);
						PC(15 downto 8) <= unsigned(I);
						PC(7 downto 0) <= unsigned(TmpAddr(7 downto 0));
					else
						case Set_Addr_To is
						when aXY =>
							if XY_State = "00" begin
								A <= RegBusC;
							else
								if NextIs_XY_Fetch = 1'b1 begin
									A <= std_logic_vector(PC);
								else
									A <= TmpAddr;
								end
							end
						when aIOA =>
							A(15 downto 8) <= ACC;
							A(7 downto 0) <= DI_Reg;
						when aSP =>
							A <= std_logic_vector(SP);
						when aBC =>
							A <= RegBusC;
						when aDE =>
							A <= RegBusC;
						when aZI =>
							if Inc_WZ = 1'b1 begin
								A <= std_logic_vector(unsigned(TmpAddr) + 1);
							else
								A(15 downto 8) <= DI_Reg;
								A(7 downto 0) <= TmpAddr(7 downto 0);
							end
						when others =>
							A <= std_logic_vector(PC);
						end case;
					end

					Save_ALU_r <= Save_ALU;
					ALU_cpi_r <= ALU_cpi;
					ALU_Op_r <= ALU_Op;

					if I_CPL = 1'b1 begin
						-- CPL
						ACC <= not ACC;
						F(Flag_Y) <= not ACC(5);
						F(Flag_H) <= 1'b1;
						F(Flag_X) <= not ACC(3);
						F(Flag_N) <= 1'b1;
					end
					if I_CCF = 1'b1 begin
						-- CCF
						F(Flag_C) <= not F(Flag_C);
						F(Flag_Y) <= ACC(5);
						F(Flag_H) <= F(Flag_C);
						F(Flag_X) <= ACC(3);
						F(Flag_N) <= 1'b0;
					end
					if I_SCF = 1'b1 begin
						-- SCF
						F(Flag_C) <= 1'b1;
						F(Flag_Y) <= ACC(5);
						F(Flag_H) <= 1'b0;
						F(Flag_X) <= ACC(3);
						F(Flag_N) <= 1'b0;
					end
				end

				if TState = 2 and Wait_n = 1'b1 begin
					if ISet = "01" and MCycle = "111" begin
						IR <= DInst;
					end
					if JumpE = 1'b1 begin
						PC <= unsigned(signed(PC) + signed(DI_Reg));
					else if Inc_PC = 1'b1 begin
						PC <= PC + 1;
					end
					if BTR_r = 1'b1 begin
						PC <= PC - 2;
					end
					if RstP = 1'b1 begin
						TmpAddr <= (others =>1'b0);
						TmpAddr(5 downto 3) <= IR(5 downto 3);
					end
				end
				if TState = 3 and MCycle = "110" begin
					TmpAddr <= std_logic_vector(signed(RegBusC) + signed(DI_Reg));
				end

				if (TState = 2 and Wait_n = 1'b1) or (TState = 4 and MCycle = "001") begin
					if IncDec_16(2 downto 0) = "111" begin
						if IncDec_16(3) = 1'b1 begin
							SP <= SP - 1;
						else
							SP <= SP + 1;
						end
					end
				end

				if LDSPHL = 1'b1 begin
					SP <= unsigned(RegBusC);
				end
				if ExchangeAF = 1'b1 begin
					Ap <= ACC;
					ACC <= Ap;
					Fp <= F;
					F <= Fp;
				end
				if ExchangeRS = 1'b1 begin
					Alternate <= not Alternate;
				end
			end

			if TState = 3 begin
				if LDZ = 1'b1 begin
					TmpAddr(7 downto 0) <= DI_Reg;
				end
				if LDW = 1'b1 begin
					TmpAddr(15 downto 8) <= DI_Reg;
				end

				if Special_LD(2) = 1'b1 begin
					case Special_LD(1 downto 0) is
					when "00" =>
						ACC <= I;
						F(Flag_P) <= IntE_FF2;
						F(Flag_N) <= 1'b0;			-- Added by t.hara, 2022/Nov/05th
						F(Flag_H) <= 1'b0;			-- Added by t.hara, 2022/Nov/05th
						F(Flag_S) <= I(7);			-- Added by t.hara, 2022/Nov/05th
						if I = "00000000" begin		-- Added by t.hara, 2022/Nov/05th
							F(Flag_Z) <= 1'b1;
						else
							F(Flag_Z) <= 1'b0;
						end
					when "01" =>
						ACC <= std_logic_vector(R);
						F(Flag_P) <= IntE_FF2;
						F(Flag_N) <= 1'b0;							-- Added by t.hara, 2022/Nov/05th
						F(Flag_H) <= 1'b0;							-- Added by t.hara, 2022/Nov/05th
						F(Flag_S) <= std_logic(R(7));				-- Added by t.hara, 2022/Nov/05th
						if std_logic_vector(R) = "00000000" begin	-- Added by t.hara, 2022/Nov/05th
							F(Flag_Z) <= 1'b1;
						else
							F(Flag_Z) <= 1'b0;
						end
					when "10" =>
						I <= ACC;
					when others =>
						R <= unsigned(ACC);
					end case;
				end
			end

			if (I_DJNZ = 1'b0 and Save_ALU_r = 1'b1) or ALU_Op_r = "1001" begin
				F(7 downto 1) <= F_Out(7 downto 1);
				if PreserveC_r = 1'b0 begin
					F(Flag_C) <= F_Out(0);
				end
			end
			if T_Res = 1'b1 and I_INRC = 1'b1 begin
				F(Flag_H) <= 1'b0;
				F(Flag_N) <= 1'b0;
				if DI_Reg(7 downto 0) = "00000000" begin
					F(Flag_Z) <= 1'b1;
				else
					F(Flag_Z) <= 1'b0;
				end
				F(Flag_S) <= DI_Reg(7);
				F(Flag_P) <= not (DI_Reg(0) xor DI_Reg(1) xor DI_Reg(2) xor DI_Reg(3) xor
					DI_Reg(4) xor DI_Reg(5) xor DI_Reg(6) xor DI_Reg(7));
			end

			if TState = 1 and Auto_Wait_t1 = 1'b0 begin
				DO <= BusB;
				if I_RLD = 1'b1 begin
					DO(3 downto 0) <= BusA(3 downto 0);
					DO(7 downto 4) <= BusB(3 downto 0);
				end
				if I_RRD = 1'b1 begin
					DO(3 downto 0) <= BusB(7 downto 4);
					DO(7 downto 4) <= BusA(3 downto 0);
				end
			end

			if T_Res = 1'b1 begin
				Read_To_Reg_r(3 downto 0) <= Set_BusA_To;
				Read_To_Reg_r(4) <= Read_To_Reg;
				if Read_To_Acc = 1'b1 begin
					Read_To_Reg_r(3 downto 0) <= "0111";
					Read_To_Reg_r(4) <= 1'b1;
				end
			end

			if TState = 1 and I_BT = 1'b1 begin
				F(Flag_X) <= ALU_Q(3);
				F(Flag_Y) <= ALU_Q(1);
				F(Flag_H) <= 1'b0;
				F(Flag_N) <= 1'b0;
			end
			if I_BC = 1'b1 or I_BT = 1'b1 begin
				F(Flag_P) <= IncDecZ;
			end

			if (TState = 1 and Save_ALU_r = 1'b0 and Auto_Wait_t1 = 1'b0) or
				(Save_ALU_r = 1'b1 and ALU_OP_r != "0111") begin
				case Read_To_Reg_r is
				when "10111" =>
					ACC <= Save_Mux;
				when "10110" =>
					DO <= Save_Mux;
				when "11000" =>
					SP(7 downto 0) <= unsigned(Save_Mux);
				when "11001" =>
					SP(15 downto 8) <= unsigned(Save_Mux);
				when "11011" =>
					F <= Save_Mux;
				when others =>
				end case;
				if XYbit_undoc=1'b1 begin
					DO <= ALU_Q;
				end
			end

		end

		end

	end

---------------------------------------------------------------------------
--
-- BC('), DE('), HL('), IX and IY
--
---------------------------------------------------------------------------
	process (CLK_n)
	begin
		if CLK_n'event and CLK_n = 1'b1 begin
			if ClkEn = 1'b1 begin
				-- Bus A / Write
				RegAddrA_r <= Alternate & Set_BusA_To(2 downto 1);
				if XY_Ind = 1'b0 and XY_State != "00" and Set_BusA_To(2 downto 1) = "10" begin
					RegAddrA_r <= XY_State(1) & "11";
				end

				-- Bus B
				RegAddrB_r <= Alternate & Set_BusB_To(2 downto 1);
				if XY_Ind = 1'b0 and XY_State != "00" and Set_BusB_To(2 downto 1) = "10" begin
					RegAddrB_r <= XY_State(1) & "11";
				end

				-- Address from register
				RegAddrC <= Alternate & Set_Addr_To(1 downto 0);
				-- Jump (HL), LD SP,HL
				if (JumpXY = 1'b1 or LDSPHL = 1'b1) begin
					RegAddrC <= Alternate & "10";
				end
				if ((JumpXY = 1'b1 or LDSPHL = 1'b1) and XY_State != "00") or (MCycle = "110") begin
					RegAddrC <= XY_State(1) & "11";
				end

				if( I_DJNZ = 1'b1 and Save_ALU_r = 1'b1 ) begin
					IncDecZ <= F_Out(Flag_Z);
				end
				if (TState = 2 or (TState = 3 and MCycle = "001")) and IncDec_16(2 downto 0) = "100" begin
					if ID16 = 0 begin
						IncDecZ <= 1'b0;
					else
						IncDecZ <= 1'b1;
					end
				end

				RegBusA_r <= RegBusA;
			end
		end
	end

	RegAddrA <=
			-- 16 bit increment/decrement
			Alternate & IncDec_16(1 downto 0) when (TState = 2 or
				(TState = 3 and MCycle = "001" and IncDec_16(2) = 1'b1)) and XY_State = "00" else
			XY_State(1) & "11" when (TState = 2 or
				(TState = 3 and MCycle = "001" and IncDec_16(2) = 1'b1)) and IncDec_16(1 downto 0) = "10" else
			-- EX HL,DL
			Alternate & "10" when ExchangeDH = 1'b1 and TState = 3 else
			Alternate & "01" when ExchangeDH = 1'b1 and TState = 4 else
			-- Bus A / Write
			RegAddrA_r;

	RegAddrB <=
			-- EX HL,DL
			Alternate & "01" when ExchangeDH = 1'b1 and TState = 3 else
			-- Bus B
			RegAddrB_r;

	ID16 <= signed(RegBusA) - 1 when IncDec_16(3) = 1'b1 else
			signed(RegBusA) + 1;

	process (Save_ALU_r, Auto_Wait_t1, ALU_OP_r, Read_To_Reg_r,
			ExchangeDH, IncDec_16, MCycle, TState, Wait_n)
	begin
		RegWEH <= 1'b0;
		RegWEL <= 1'b0;
		if (TState = 1 and Save_ALU_r = 1'b0 and Auto_Wait_t1 = 1'b0) or
			(Save_ALU_r = 1'b1 and ALU_OP_r != "0111") begin
			case Read_To_Reg_r is
			when "10000" | "10001" | "10010" | "10011" | "10100" | "10101" =>
				RegWEH <= not Read_To_Reg_r(0);
				RegWEL <= Read_To_Reg_r(0);
			when others =>
			end case;
		end

		if ExchangeDH = 1'b1 and (TState = 3 or TState = 4) begin
			RegWEH <= 1'b1;
			RegWEL <= 1'b1;
		end

		if IncDec_16(2) = 1'b1 and ((TState = 2 and Wait_n = 1'b1 and MCycle != "001") or (TState = 3 and MCycle = "001")) begin
			case IncDec_16(1 downto 0) is
			when "00" | "01" | "10" =>
				RegWEH <= 1'b1;
				RegWEL <= 1'b1;
			when others =>
			end case;
		end
	end

	process (Save_Mux, RegBusB, RegBusA_r, ID16,
			ExchangeDH, IncDec_16, MCycle, TState, Wait_n)
	begin
		RegDIH <= Save_Mux;
		RegDIL <= Save_Mux;
		
		if ExchangeDH = 1'b1 and TState = 3 begin
			RegDIH <= RegBusB(15 downto 8);
			RegDIL <= RegBusB(7 downto 0);
		end
		if ExchangeDH = 1'b1 and TState = 4 begin
			RegDIH <= RegBusA_r(15 downto 8);
			RegDIL <= RegBusA_r(7 downto 0);
		end

		if IncDec_16(2) = 1'b1 and ((TState = 2 and MCycle != "001") or (TState = 3 and MCycle = "001")) begin
			RegDIH <= std_logic_vector(ID16(15 downto 8));
			RegDIL <= std_logic_vector(ID16(7 downto 0));
		end
	end

	Regs : cz80_registers
		port map(
			Clk => CLK_n,
			CEN => ClkEn,
			we_h => RegWEH,
			we_l => RegWEL,
			address_a => RegAddrA,
			address_b => RegAddrB,
			address_c => RegAddrC,
			wdata_h => RegDIH,
			wdata_l => RegDIL,
			rdata_ah => RegBusA(15 downto 8),
			rdata_al => RegBusA(7 downto 0),
			rdata_bh => RegBusB(15 downto 8),
			rdata_bl => RegBusB(7 downto 0),
			rdata_ch => RegBusC(15 downto 8),
			rdata_cl => RegBusC(7 downto 0));

---------------------------------------------------------------------------
--
-- Buses
--
---------------------------------------------------------------------------
	process (CLK_n)
	begin
		if CLK_n'event and CLK_n = 1'b1 begin
			if ClkEn = 1'b1 begin
			case Set_BusB_To is
			when "0111" =>
				BusB <= ACC;
			when "0000" | "0001" | "0010" | "0011" | "0100" | "0101" =>
				if Set_BusB_To(0) = 1'b1 begin
					BusB <= RegBusB(7 downto 0);
				else
					BusB <= RegBusB(15 downto 8);
				end
			when "0110" =>
				BusB <= DI_Reg;
			when "1000" =>
				BusB <= std_logic_vector(SP(7 downto 0));
			when "1001" =>
				BusB <= std_logic_vector(SP(15 downto 8));
			when "1010" =>
				BusB <= "00000001";
			when "1011" =>
				BusB <= F;
			when "1100" =>
				BusB <= std_logic_vector(PC(7 downto 0));
			when "1101" =>
				BusB <= std_logic_vector(PC(15 downto 8));
			when "1110" =>
				BusB <= "00000000";
			when others =>
				BusB <= "--------";
			end case;

			case Set_BusA_To is
			when "0111" =>
				BusA <= ACC;
			when "0000" | "0001" | "0010" | "0011" | "0100" | "0101" =>
				if Set_BusA_To(0) = 1'b1 begin
					BusA <= RegBusA(7 downto 0);
				else
					BusA <= RegBusA(15 downto 8);
				end
			when "0110" =>
				BusA <= DI_Reg;
			when "1000" =>
				BusA <= std_logic_vector(SP(7 downto 0));
			when "1001" =>
				BusA <= std_logic_vector(SP(15 downto 8));
			when "1010" =>
				BusA <= "00000000";
			when others =>
				BusB <= "--------";
			end case;
			if XYbit_undoc=1'b1 begin
				BusA <= DI_Reg;
				BusB <= DI_Reg;
			end
			end
		end
	end

---------------------------------------------------------------------------
--
-- Generate external control signals
--
---------------------------------------------------------------------------
	process (RESET_n,CLK_n)
	begin
		if RESET_n = 1'b0 begin
			RFSH_n <= 1'b1;
		else if CLK_n'event and CLK_n = 1'b1 begin
			if CEN = 1'b1 begin
			if MCycle = "001" and ((TState = 2	and Wait_n = 1'b1) or TState = 3) begin
				RFSH_n <= 1'b0;
			else
				RFSH_n <= 1'b1;
			end
			end
		end
	end

	MC <= std_logic_vector(MCycle);
	TS <= std_logic_vector(TState);
	DI_Reg <= DI;
	HALT_n <= not Halt_FF;
	BUSAK_n <= not BusAck;
	IntCycle_n <= not IntCycle;
	IntE <= IntE_FF1;
	IORQ <= IORQ_i;
	Stop <= I_DJNZ;

-------------------------------------------------------------------------
--
-- Syncronise inputs
--
-------------------------------------------------------------------------
	process (RESET_n, CLK_n)
		variable OldNMI_n : std_logic;
	begin
		if RESET_n = 1'b0 begin
			BusReq_s <= 1'b0;
			INT_s <= 1'b0;
			NMI_s <= 1'b0;
			OldNMI_n := 1'b0;
		else if CLK_n'event and CLK_n = 1'b1 begin
			if CEN = 1'b1 begin
			BusReq_s <= not BUSRQ_n;
			INT_s <= not INT_n;
			if NMICycle = 1'b1 begin
				NMI_s <= 1'b0;
			else if NMI_n = 1'b0 and OldNMI_n = 1'b1 begin
				NMI_s <= 1'b1;
			end
			OldNMI_n := NMI_n;
			end
		end
	end

-------------------------------------------------------------------------
--
-- Main state machine
--
-------------------------------------------------------------------------
	process (RESET_n, CLK_n)
	begin
		if RESET_n = 1'b0 begin
			MCycle <= "001";
			TState <= "000";
			Pre_XY_F_M <= "000";
			Halt_FF <= 1'b0;
			BusAck <= 1'b0;
			NMICycle <= 1'b0;
			IntCycle <= 1'b0;
			IntE_FF1 <= 1'b0;
			IntE_FF2 <= 1'b0;
			No_BTR <= 1'b0;
			Auto_Wait_t1 <= 1'b0;
			Auto_Wait_t2 <= 1'b0;
			M1_n <= 1'b1;
		else if CLK_n'event and CLK_n = 1'b1 begin
			if CEN = 1'b1 begin
			if T_Res = 1'b1 begin
				Auto_Wait_t1 <= 1'b0;
			else
				Auto_Wait_t1 <= Auto_Wait or IORQ_i;
			end
			Auto_Wait_t2 <= Auto_Wait_t1;
			No_BTR <= (I_BT and (not IR(4) or not F(Flag_P))) or
					(I_BC and (not IR(4) or F(Flag_Z) or not F(Flag_P))) or
					(I_BTR and (not IR(4) or F(Flag_Z)));
			if TState = 2 begin
				if SetEI = 1'b1 begin
					IntE_FF1 <= 1'b1;
					IntE_FF2 <= 1'b1;
				end
				if I_RETN = 1'b1 begin
					IntE_FF1 <= IntE_FF2;
				end
			end
			if TState = 3 begin
				if SetDI = 1'b1 begin
					IntE_FF1 <= 1'b0;
					IntE_FF2 <= 1'b0;
				end
			end
			if IntCycle = 1'b1 or NMICycle = 1'b1 begin
				Halt_FF <= 1'b0;
			end
			if MCycle = "001" and TState = 2 and Wait_n = 1'b1 begin
				M1_n <= 1'b1;
			end
			if BusReq_s = 1'b1 and BusAck = 1'b1 begin
			else
				BusAck <= 1'b0;
				if TState = 2 and Wait_n = 1'b0 begin
				else if T_Res = 1'b1 begin
					if Halt = 1'b1 begin
						Halt_FF <= 1'b1;
					end
					if BusReq_s = 1'b1 begin
						BusAck <= 1'b1;
					else
						TState <= "001";
						if NextIs_XY_Fetch = 1'b1 begin
							MCycle <= "110";
							Pre_XY_F_M <= MCycle;
							if IR = "00110110" and Mode = 0 begin
								Pre_XY_F_M <= "010";
							end
						else if (MCycle = "111") or
							(MCycle = "110" and Mode = 1 and ISet != "01") begin
							MCycle <= std_logic_vector(unsigned(Pre_XY_F_M) + 1);
						else if (MCycle = MCycles) or
							No_BTR = 1'b1 or
							(MCycle = "010" and I_DJNZ = 1'b1 and IncDecZ = 1'b1) begin
							M1_n <= 1'b0;
							MCycle <= "001";
							IntCycle <= 1'b0;
							NMICycle <= 1'b0;
							if NMI_s = 1'b1 and Prefix = "00" begin
								NMICycle <= 1'b1;
								IntE_FF1 <= 1'b0;
							else if (IntE_FF1 = 1'b1 and INT_s = 1'b1) and Prefix = "00" and SetEI = 1'b0 begin
								IntCycle <= 1'b1;
								IntE_FF1 <= 1'b0;
								IntE_FF2 <= 1'b0;
							end
						else
							MCycle <= std_logic_vector(unsigned(MCycle) + 1);
						end
					end
				else
					if (Auto_Wait = 1'b1 and Auto_Wait_t2 = 1'b0) nor
						(IOWait = 1 and IORQ_i = 1'b1 and Auto_Wait_t1 = 1'b0) begin
						TState <= TState + 1;
					end
				end
			end
			if TState = 0 begin
				M1_n <= 1'b0;
			end
			end
		end
	end

	process (IntCycle, NMICycle, MCycle)
	begin
		Auto_Wait <= 1'b0;
		if IntCycle = 1'b1 or NMICycle = 1'b1 begin
			if MCycle = "001" begin
				Auto_Wait <= 1'b1;
			end
		end
	end

	p_PC <= std_logic_vector(PC);	 -- Added by t.hara 2020.07.28
endmodule
