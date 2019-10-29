-- Sim GPIO, based on potato GPIO by
-- Kristian Klomsten Skordal.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.sim_console.all;


--! @brief Generic Wishbone GPIO Module.
--!
--! The following registers are defined:
--! |---------|------------------------------------------------------------------|
--! | Address | Description                                                      |
--! |---------|------------------------------------------------------------------|
--! | 0x00    | Describes the port: (read-only)                                  |
--! |         |   bits 0-5: The number of pins                                   |
--! | 0x08    | Input values, one bit per pin (read/write),                      |
--! |         | If the pin is set to output, writing 1 will toggle the pin       |
--! | 0x10    | Output values, one bit per pin (read/write)                      |
--! | 0x18    | Set register, Output = Output | set register (write-only)        |
--! | 0x20    | Clear register, Output = Output & ~(clear register) (write-only) |
--! | 0x28    | Type, 4 bits per pin (read/write), pins 0-15                     |
--! | 0x30    | Type, 4 bits per pin (read/write), pins 16-31                    |
--! | 0x38    | Type, 4 bits per pin (read/write), pins 32-47                    |
--! | 0x40    | Type, 4 bits per pin (read/write), pins 48-63                    |
--! |         | Types:   MSB       LSB                                           |
--! |         |            0    0    0    0   Input, Interrupt disabled          |
--! |         |            0    0    1    0   Input, Interrupt when low          |
--! |         |            0    0    1    1   Input, Interrupt when high         |
--! |         |            0    1    0    0   Input, Interrupt when falling      |
--! |         |            0    1    0    1   Input, Interrupt when rising       |
--! |         |            0    1    1    1   Output                             |
--! |         |            others        Undefined                               |
--! | 0x48    | Interrupt Triggered, one bit per pin (read/write)                |
--! |---------|------------------------------------------------------------------|
--!
--! Writes to the output register for input pins are ignored.
entity pp_soc_gpio is
	generic(
		NUM_GPIOS : natural := 64
	);
	port(
		clk : in std_logic;
		reset : in std_logic;
		irq : out std_logic;

		-- GPIO interface:
		gpio : inout std_logic_vector(NUM_GPIOS - 1 downto 0);

		-- Wishbone interface:
		wb_adr_in  : in  std_logic_vector(7 downto 0);
		wb_dat_in  : in  std_logic_vector(63 downto 0);
		wb_dat_out : out std_logic_vector(63 downto 0);
		wb_cyc_in  : in  std_logic;
		wb_stb_in  : in  std_logic;
		wb_we_in   : in  std_logic;
		wb_ack_out : out std_logic
	);
end entity pp_soc_gpio;

architecture behaviour of pp_soc_gpio is

	type type_array is array (natural range 0 to NUM_GPIOS - 1) of
		std_logic_vector(3 downto 0);

	signal input_buffer : std_logic_vector(NUM_GPIOS - 1 downto 0);
	signal input_register_prev : std_logic_vector(NUM_GPIOS - 1 downto 0);
	signal input_register : std_logic_vector(NUM_GPIOS - 1 downto 0);
	signal output_register : std_logic_vector(NUM_GPIOS - 1 downto 0);
	signal type_register : type_array;
	signal irq_triggered_register : std_logic_vector(NUM_GPIOS - 1 downto 0);

	signal ack : std_logic := '0';

begin

	assert NUM_GPIOS > 0 and NUM_GPIOS <= 64
		report "Only a number between 1 and 64 (inclusive) GPIOs are supported!"
		severity FAILURE;

	wb_ack_out <= ack and wb_cyc_in and wb_stb_in;

	wishbone: process(clk)
	begin
		if rising_edge(clk) then
			for i in 0 to NUM_GPIOS - 1 loop
				gpio(i) <= output_register(i) when type_register(i) = b"0111" else 'Z';
				input_register_prev(i) <= input_register(i);
				input_register(i) <= input_buffer(i);
				input_buffer(i) <= gpio(i) when type_register(i) /= b"0111";

				case type_register(i) is
					when b"0010" =>
						if gpio(i) = '0' then
							irq_triggered_register(i) <= '1';
							irq <= '1';
						end if;
					when b"0011" =>
						if gpio(i) = '1' then
							irq_triggered_register(i) <= '1';
							irq <= '1';
						end if;
					when b"0100" =>
						if input_register(i) = '0' and input_register_prev(i) = '1' then
							irq_triggered_register(i) <= '1';
							irq <= '1';
						end if;
					when b"0101" =>
						if input_register(i) = '1' and input_register_prev(i) = '0' then
							irq_triggered_register(i) <= '1';
							irq <= '1';
						end if;
					when others =>
				end case;
			end loop;

			if reset = '1' then
				input_register <= (others => '0');
				output_register <= (others => '0');
				wb_dat_out <= (others => '0');
				for i in 0 to NUM_GPIOS - 1 loop
					type_register(i) <= (others => '0');
				end loop;
				irq_triggered_register <= (others => '0');
				ack <= '0';
			else
				if wb_cyc_in = '1' and wb_stb_in = '1' and ack = '0' then
					if wb_we_in = '1' then
						case wb_adr_in is
							when x"08" => --! Input Value
								output_register <= output_register xor wb_dat_in(NUM_GPIOS - 1 downto 0);
							when x"10" => --! Output Value
								output_register <= wb_dat_in(NUM_GPIOS - 1 downto 0);
							when x"18" => --! Set
								output_register <= output_register OR wb_dat_in(NUM_GPIOS - 1 downto 0);
							when x"20" => --! Clear
								output_register <= output_register AND NOT(wb_dat_in(NUM_GPIOS - 1 downto 0));
							when x"28" => --! Type Pins 0-15
								for i in 0 to MINIMUM(NUM_GPIOS - 1, 15) loop
									type_register(i) <= (wb_dat_in((i + 1) * 4 - 1 downto i * 4));
								end loop;
							when x"30" => --! Type Pins 16-31
								for i in 16 to MINIMUM(NUM_GPIOS - 1, 31) loop
									type_register(i) <= (wb_dat_in((i - 16 + 1) * 4 - 1 downto (i - 16) * 4));
								end loop;
							when x"38" => --! Type Pins 32-47
								for i in 32 to MINIMUM(NUM_GPIOS - 1, 47) loop
									type_register(i) <= (wb_dat_in((i - 32 + 1) * 4 - 1 downto (i - 32) * 4));
								end loop;
							when x"40" => --! Type Pins 48-63
								for i in 48 to MINIMUM(NUM_GPIOS - 1, 63) loop
									type_register(i) <= (wb_dat_in((i - 48 + 1) * 4 - 1 downto (i - 48) * 4));
								end loop;
							when others =>
						end case;
						ack <= '1';
					else
						case wb_adr_in is
							when x"00" => --! Description
								wb_dat_out <= std_logic_vector(to_unsigned(NUM_GPIOS, wb_dat_out'length));
							when x"08" => --! Input Value
								wb_dat_out <= std_logic_vector(resize(unsigned(input_register), wb_dat_out'length));
							when x"10" => --! Output value
								wb_dat_out <= std_logic_vector(resize(unsigned(output_register), wb_dat_out'length));
							when x"28" => --! Type Pins 0-15
								for i in 0 to MINIMUM(NUM_GPIOS - 1, 15) loop
									wb_dat_out((i + 1) * 4 - 1 downto i * 4) <= type_register(i);
								end loop;
							when x"30" => --! Type Pins 16-31
								for i in 21 to MINIMUM(NUM_GPIOS - 1, 31) loop
									wb_dat_out((i - 16 + 1) * 4 - 1 downto (i - 16) * 4) <= type_register(i);
								end loop;
							when x"38" => --! Type Pins 32-47
								for i in 32 to MINIMUM(NUM_GPIOS - 1, 47) loop
									wb_dat_out((i - 32 + 1) * 4 - 1 downto (i - 32) * 4) <= type_register(i);
								end loop;
							when x"40" => --! Type Pins 32-47
								for i in 32 to MINIMUM(NUM_GPIOS - 1, 47) loop
									wb_dat_out((i - 32 + 1) * 4 - 1 downto (i - 32) * 4) <= type_register(i);
								end loop;
							when x"48" => --! Interrupts triggered
								wb_dat_out <= std_logic_vector(resize(unsigned(irq_triggered_register), wb_dat_out'length));
								irq_triggered_register <= (others => '0');
								irq <= '0';
							when others =>
						end case;
						report "ack";
						ack <= '1';
					end if;
				elsif wb_stb_in = '0' then
					ack <= '0';
				end if;
			end if;
		end if;
	end process wishbone;

end architecture behaviour;