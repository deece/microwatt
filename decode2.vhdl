library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.decode_types.all;
use work.common.all;
use work.helpers.all;
use work.insn_helpers.all;

entity decode2 is
	port (
		clk   : in std_ulogic;
		rst   : in std_ulogic;

		complete_in : in std_ulogic;
		stall_out : out std_ulogic;

		flush_in: in std_ulogic;

		d_in  : in Decode1ToDecode2Type;

		e_out : out Decode2ToExecute1Type;
		m_out : out Decode2ToMultiplyType;
		l_out : out Decode2ToLoadstore1Type;

		r_in  : in RegisterFileToDecode2Type;
		r_out : out Decode2ToRegisterFileType;

		c_in  : in CrFileToDecode2Type;
		c_out : out Decode2ToCrFileType
	);
end entity decode2;

architecture behaviour of decode2 is
	type reg_type is record
		e : Decode2ToExecute1Type;
		m : Decode2ToMultiplyType;
		l : Decode2ToLoadstore1Type;
	end record;

	signal r, rin : reg_type;

	type decode_input_reg_t is record
		reg_valid : std_ulogic;
		reg       : std_ulogic_vector(4 downto 0);
		data      : std_ulogic_vector(63 downto 0);
	end record;

	function decode_input_reg_a (t : input_reg_a_t; insn_in : std_ulogic_vector(31 downto 0);
				     reg_data : std_ulogic_vector(63 downto 0)) return decode_input_reg_t is
		variable is_reg : std_ulogic;
	begin
		is_reg := '0' when insn_ra(insn_in) = "00000" else '1';

		case t is
		when RA =>
			return ('1', insn_ra(insn_in), reg_data);
		when RA_OR_ZERO =>
			--return (is_reg, insn_ra(insn_in), ra_or_zero(reg_data, insn_ra(insn_in)));
			return ('1', insn_ra(insn_in), ra_or_zero(reg_data, insn_ra(insn_in)));
		when RS =>
			return ('1', insn_rs(insn_in), reg_data);
		when NONE =>
			return ('0', (others => '0'), (others => '0'));
		end case;
	end;

	function decode_input_reg_b (t : input_reg_b_t; insn_in : std_ulogic_vector(31 downto 0);
				     reg_data : std_ulogic_vector(63 downto 0)) return decode_input_reg_t is
	begin
		case t is
		when RB =>
			return ('1', insn_rb(insn_in), reg_data);
		when RS =>
			return ('1', insn_rs(insn_in), reg_data);
		when CONST_UI =>
			return ('0', (others => '0'), std_ulogic_vector(resize(unsigned(insn_ui(insn_in)), 64)));
		when CONST_SI =>
			return ('0', (others => '0'), std_ulogic_vector(resize(signed(insn_si(insn_in)), 64)));
		when CONST_SI_HI =>
			return ('0', (others => '0'), std_ulogic_vector(resize(signed(insn_si(insn_in)) & x"0000", 64)));
		when CONST_UI_HI =>
			return ('0', (others => '0'), std_ulogic_vector(resize(unsigned(insn_si(insn_in)) & x"0000", 64)));
		when CONST_LI =>
			return ('0', (others => '0'), std_ulogic_vector(resize(signed(insn_li(insn_in)) & "00", 64)));
		when CONST_BD =>
			return ('0', (others => '0'), std_ulogic_vector(resize(signed(insn_bd(insn_in)) & "00", 64)));
		when CONST_DS =>
			return ('0', (others => '0'), std_ulogic_vector(resize(signed(insn_ds(insn_in)) & "00", 64)));
		when NONE =>
			return ('0', (others => '0'), (others => '0'));
		end case;
	end;

	function decode_input_reg_c (t : input_reg_c_t; insn_in : std_ulogic_vector(31 downto 0);
				     reg_data : std_ulogic_vector(63 downto 0)) return decode_input_reg_t is
	begin
		case t is
		when RS =>
			return ('1', insn_rs(insn_in), reg_data);
		when NONE =>
			return ('0', (others => '0'), (others => '0'));
		end case;
	end;

	function decode_output_reg (t : output_reg_a_t; insn_in : std_ulogic_vector(31 downto 0)) return std_ulogic_vector is
	begin
		case t is
		when RT =>
			return insn_rt(insn_in);
		when RA =>
			return insn_ra(insn_in);
		when NONE =>
			return "00000";
		end case;
	end;

	function decode_const_a (t : constant_a_t; insn_in : std_ulogic_vector(31 downto 0)) return std_ulogic_vector is
	begin
		case t is
		when SH =>
			return "00" & insn_sh(insn_in);
		when SH32 =>
			return "000" & insn_sh32(insn_in);
		when FXM =>
			return insn_fxm(insn_in);
		when BO =>
			return "000" & insn_bo(insn_in);
		when BF =>
			return "00000" & insn_bf(insn_in);
		when TOO =>
			return "000" & insn_to(insn_in);
		when BC =>
			return "000" & insn_bc(insn_in);
		when NONE =>
			return "00000000";
		end case;
	end;

	function decode_const_b (t : constant_b_t; insn_in : std_ulogic_vector(31 downto 0)) return std_ulogic_vector is
	begin
		case t is
		when MB =>
			return insn_mb(insn_in);
		when ME =>
			return insn_me(insn_in);
		when MB32 =>
			return "0" & insn_mb32(insn_in);
		when BI =>
			return "0" & insn_bi(insn_in);
		when L =>
			return "00000" & insn_l(insn_in);
		when NONE =>
			return "000000";
		end case;
	end;

	function decode_const_c (t : constant_c_t; insn_in : std_ulogic_vector(31 downto 0)) return std_ulogic_vector is
	begin
		case t is
		when ME32 =>
			return insn_me32(insn_in);
		when BH =>
			return "000" & insn_bh(insn_in);
		when NONE =>
			return "00000";
		end case;
	end;

	function decode_rc (t : rc_t; insn_in : std_ulogic_vector(31 downto 0)) return std_ulogic is
	begin
		case t is
		when RC =>
			return insn_rc(insn_in);
		when ONE =>
			return '1';
		when NONE =>
			return '0';
		end case;
	end;

	-- issue control signals
	signal control_valid_in : std_ulogic;
	signal control_valid_out : std_ulogic;
	signal control_sgl_pipe : std_logic;

	signal gpr_write_valid : std_ulogic;
	signal gpr_write : std_ulogic_vector(4 downto 0);

	signal gpr_a_read_valid : std_ulogic;
	signal gpr_a_read : std_ulogic_vector(4 downto 0);

	signal gpr_b_read_valid : std_ulogic;
	signal gpr_b_read : std_ulogic_vector(4 downto 0);

	signal gpr_c_read_valid : std_ulogic;
	signal gpr_c_read : std_ulogic_vector(4 downto 0);
begin
	control0: entity work.control
	generic map (
		PIPELINE_DEPTH => 2
	)
	port map (
		clk         => clk,
		rst         => rst,

		complete_in => complete_in,
		valid_in    => control_valid_in,
		flush_in    => flush_in,
		sgl_pipe_in => control_sgl_pipe,

		gpr_write_valid_in => gpr_write_valid,
		gpr_write_in       => gpr_write,

		gpr_a_read_valid_in  => gpr_a_read_valid,
		gpr_a_read_in        => gpr_a_read,

		gpr_b_read_valid_in  => gpr_b_read_valid,
		gpr_b_read_in        => gpr_b_read,

		gpr_c_read_valid_in  => gpr_c_read_valid,
		gpr_c_read_in        => gpr_c_read,

		valid_out   => control_valid_out,
		stall_out   => stall_out
	);

	decode2_0: process(clk)
	begin
		if rising_edge(clk) then
			if rin.e.valid = '1' or rin.l.valid = '1' or rin.m.valid = '1' then
				report "execute " & to_hstring(rin.e.nia);
			end if;
			r <= rin;
		end if;
	end process;

	r_out.read1_reg <= insn_ra(d_in.insn) when (d_in.decode.input_reg_a = RA) else
			   insn_ra(d_in.insn) when d_in.decode.input_reg_a = RA_OR_ZERO else
			   insn_rs(d_in.insn) when d_in.decode.input_reg_a = RS else
			   (others => '0');

	r_out.read2_reg <= insn_rb(d_in.insn) when d_in.decode.input_reg_b = RB else
			   insn_rs(d_in.insn) when d_in.decode.input_reg_b = RS else
			   (others => '0');

	r_out.read3_reg <= insn_rs(d_in.insn) when d_in.decode.input_reg_c = RS else
			   (others => '0');

	c_out.read <= d_in.decode.input_cr;

	decode2_1: process(all)
		variable v : reg_type;
		variable mul_a : std_ulogic_vector(63 downto 0);
		variable mul_b : std_ulogic_vector(63 downto 0);
		variable decoded_reg_a : decode_input_reg_t;
		variable decoded_reg_b : decode_input_reg_t;
		variable decoded_reg_c : decode_input_reg_t;
	begin
		v := r;

		v.e := Decode2ToExecute1Init;
		v.l := Decode2ToLoadStore1Init;
		v.m := Decode2ToMultiplyInit;

		mul_a := (others => '0');
		mul_b := (others => '0');

		--v.e.input_cr := d_in.decode.input_cr;
		--v.m.input_cr := d_in.decode.input_cr;
		--v.e.output_cr := d_in.decode.output_cr;

		decoded_reg_a := decode_input_reg_a (d_in.decode.input_reg_a, d_in.insn, r_in.read1_data);
		decoded_reg_b := decode_input_reg_b (d_in.decode.input_reg_b, d_in.insn, r_in.read2_data);
		decoded_reg_c := decode_input_reg_c (d_in.decode.input_reg_c, d_in.insn, r_in.read3_data);

		r_out.read1_enable <= decoded_reg_a.reg_valid;
		r_out.read2_enable <= decoded_reg_b.reg_valid;
		r_out.read3_enable <= decoded_reg_c.reg_valid;

		-- execute unit
		v.e.nia := d_in.nia;
		v.e.insn_type := d_in.decode.insn_type;
		v.e.read_reg1 := decoded_reg_a.reg;
		v.e.read_data1 := decoded_reg_a.data;
		v.e.read_reg2 := decoded_reg_b.reg;
		v.e.read_data2 := decoded_reg_b.data;
		v.e.write_reg := decode_output_reg(d_in.decode.output_reg_a, d_in.insn);
		v.e.rc := decode_rc(d_in.decode.rc, d_in.insn);
		v.e.cr := c_in.read_cr_data;
		v.e.input_carry := d_in.decode.input_carry;
		v.e.output_carry := d_in.decode.output_carry;
		if d_in.decode.lr = '1' then
			v.e.lr := insn_lk(d_in.insn);
		end if;
		v.e.const1 := decode_const_a(d_in.decode.const_a, d_in.insn);
		v.e.const2 := decode_const_b(d_in.decode.const_b, d_in.insn);
		v.e.const3 := decode_const_c(d_in.decode.const_c, d_in.insn);

		-- multiply unit
		v.m.insn_type := d_in.decode.insn_type;
		mul_a := decoded_reg_a.data;
		mul_b := decoded_reg_b.data;
		v.m.write_reg := decode_output_reg(d_in.decode.output_reg_a, d_in.insn);
		v.m.rc := decode_rc(d_in.decode.rc, d_in.insn);

		if d_in.decode.mul_32bit = '1' then
			if d_in.decode.mul_signed = '1' then
				v.m.data1 := (others => mul_a(31));
				v.m.data1(31 downto 0) := mul_a(31 downto 0);
				v.m.data2 := (others => mul_b(31));
				v.m.data2(31 downto 0) := mul_b(31 downto 0);
			else
				v.m.data1 := '0' & x"00000000" & mul_a(31 downto 0);
				v.m.data2 := '0' & x"00000000" & mul_b(31 downto 0);
			end if;
		else
			if d_in.decode.mul_signed = '1' then
				v.m.data1 := mul_a(63) & mul_a;
				v.m.data2 := mul_b(63) & mul_b;
			else
				v.m.data1 := '0' & mul_a;
				v.m.data2 := '0' & mul_b;
			end if;
		end if;

		-- load/store unit
		v.l.update_reg := decoded_reg_a.reg;
		v.l.addr1 := decoded_reg_a.data;
		v.l.addr2 := decoded_reg_b.data;
		v.l.data := decoded_reg_c.data;
		v.l.write_reg := decode_output_reg(d_in.decode.output_reg_a, d_in.insn);

		if d_in.decode.insn_type = OP_LOAD then
			v.l.load := '1';
		else
			v.l.load := '0';
		end if;

		case d_in.decode.length is
		when is1B =>
			v.l.length := "0001";
		when is2B =>
			v.l.length := "0010";
		when is4B =>
			v.l.length := "0100";
		when is8B =>
			v.l.length := "1000";
		when NONE =>
			v.l.length := "0000";
		end case;

		v.l.byte_reverse := d_in.decode.byte_reverse;
		v.l.sign_extend := d_in.decode.sign_extend;
		v.l.update := d_in.decode.update;

		-- issue control
		control_valid_in <= d_in.valid;
		control_sgl_pipe <= d_in.decode.sgl_pipe;

		gpr_write_valid <= '1' when d_in.decode.output_reg_a /= NONE else '0';
		gpr_write <= decode_output_reg(d_in.decode.output_reg_a, d_in.insn);

		gpr_a_read_valid <= decoded_reg_a.reg_valid;
		gpr_a_read <= decoded_reg_a.reg;

		gpr_b_read_valid <= decoded_reg_b.reg_valid;
		gpr_b_read <= decoded_reg_b.reg;

		gpr_c_read_valid <= decoded_reg_c.reg_valid;
		gpr_c_read <= decoded_reg_c.reg;

		v.e.valid := '0';
		v.m.valid := '0';
		v.l.valid := '0';
		case d_in.decode.unit is
		when ALU =>
			v.e.valid := control_valid_out;
		when LDST =>
			v.l.valid := control_valid_out;
		when MUL =>
			v.m.valid := control_valid_out;
		when NONE =>
			v.e.valid := control_valid_out;
			v.e.insn_type := OP_ILLEGAL;
		end case;

		if rst = '1' then
			v.e := Decode2ToExecute1Init;
			v.l := Decode2ToLoadStore1Init;
			v.m := Decode2ToMultiplyInit;
		end if;

		-- Update registers
		rin <= v;

		-- Update outputs
		e_out <= r.e;
		l_out <= r.l;
		m_out <= r.m;
	end process;
end architecture behaviour;
