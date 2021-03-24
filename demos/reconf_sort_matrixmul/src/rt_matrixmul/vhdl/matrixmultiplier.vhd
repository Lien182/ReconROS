------------------------------------------------------------------------------
-- matrixmultiplier - entity/architecture pair
------------------------------------------------------------------------------
-- Filename:		matrixmultiplier
-- Version:			2.00.a
-- Description:	matrix multiplier(VHDL).
-- Date:				Wed June 7 16:32:00 2013
-- VHDL Standard:	VHDL'93
-- Author:			Achim Loesch	
------------------------------------------------------------------------------
-- Feel free to modify this file.
------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

------------------------------------------------------------------------------
-- Entity Section
------------------------------------------------------------------------------

entity matrixmultiplier is
	generic (
		G_LINE_LEN_MATRIX					: integer	:= 128;
		G_RAM_DATA_WIDTH					: integer	:= 32;
		
		G_RAM_SIZE_MATRIX_A_C			: integer	:= 128;
		G_RAM_ADDR_WIDTH_MATRIX_A_C	: integer	:= 7;
		
		G_RAM_SIZE_MATRIX_B				: integer	:= 16384;
		G_RAM_ADDR_WIDTH_MATRIX_B		: integer	:= 14
	);
	port(
		clk	: in std_logic;
		reset	: in std_logic;
		start	: in std_logic;
		done	: out std_logic;
		
		o_RAM_A_Addr	: out std_logic_vector(0 to G_RAM_ADDR_WIDTH_MATRIX_A_C - 1);
		i_RAM_A_Data	: in std_logic_vector(0 to 31);
		
		o_RAM_B_Addr	: out std_logic_vector(0 to G_RAM_ADDR_WIDTH_MATRIX_B   - 1);
		i_RAM_B_Data	: in std_logic_vector(0 to 31);
		
		o_RAM_C_Addr	: out std_logic_vector(0 to G_RAM_ADDR_WIDTH_MATRIX_A_C - 1);
		o_RAM_C_Data	: out std_logic_vector(0 to 31);
		o_RAM_C_WE		: out std_logic
	);
end matrixmultiplier;

------------------------------------------------------------------------------
-- Architecture Section
------------------------------------------------------------------------------

architecture Behavioral of matrixmultiplier is
	type STATE_TYPE is (
		STATE_IDLE,
		STATE_LOAD,
		STATE_LOAD_WAIT,
		STATE_SUM,
		STATE_DELAY_1,
		STATE_DELAY_2,
		STATE_STORE,
		STATE_STORE_WAIT,
		STATE_FINISH_CYCLE
	);
	
	signal state	: STATE_TYPE;
	
	signal temp		: std_logic_vector(0 to G_RAM_DATA_WIDTH-1);
	signal prod,delay            : std_logic_vector(0 to G_RAM_DATA_WIDTH-1);

	
begin
	
	multiply : process(clk, reset, start) is
		variable j			: integer := 0;
		variable k			: integer := 0;
	begin
		if (reset = '1') then
			done <= '0';
			
			o_RAM_A_Addr <= (others=>'0');
			o_RAM_B_Addr <= (others=>'0');
			o_RAM_C_Addr <= (others=>'0');
			o_RAM_C_Data <= (others=>'0');
			o_RAM_C_WE <= '0';
			
			state <= STATE_IDLE;
		elsif (clk'event and clk = '1') then
			o_RAM_C_WE <= '0';
			o_RAM_C_Data <= (others=>'0');

			case state is
				when STATE_IDLE =>
					done <= '0';
					if (start = '1') then
						j := 0;
						k := 0;
						temp <= (others=>'0');
						state <= STATE_LOAD;
					end if;
				when STATE_LOAD =>
					o_RAM_A_Addr <= conv_std_logic_vector(integer(k), G_RAM_ADDR_WIDTH_MATRIX_A_C);
					o_RAM_B_Addr <= conv_std_logic_vector(integer(k*G_LINE_LEN_MATRIX+j), G_RAM_ADDR_WIDTH_MATRIX_B);
					k := k + 1;
					state <= STATE_LOAD_WAIT;
				when STATE_LOAD_WAIT =>
					state <= STATE_DELAY_1;
				when STATE_DELAY_1 =>
					state <= STATE_DELAY_2;
				when STATE_DELAY_2 =>
					state <= STATE_SUM;
				when STATE_SUM =>
					
					temp <= temp + prod;
					if (k = G_LINE_LEN_MATRIX) then
						k := 0;
						state <= STATE_STORE;
					else
						state <= STATE_LOAD;
					end if;

				when STATE_STORE =>
					o_RAM_C_Addr <= conv_std_logic_vector(integer(j), G_RAM_ADDR_WIDTH_MATRIX_A_C);
					o_RAM_C_WE <= '1';
					o_RAM_C_Data <= temp;
					state <= STATE_STORE_WAIT;
				when STATE_STORE_WAIT =>
					o_RAM_C_WE <= '0';
					state <= STATE_FINISH_CYCLE;
				when STATE_FINISH_CYCLE =>
					j := j + 1;
					
					if (j = G_LINE_LEN_MATRIX) then
						j		:= 0;
						done	<= '1';
						state	<= STATE_IDLE;
					else
						temp	<= (others => '0');
						state	<= STATE_LOAD;
					end if;
			end case;
		end if;
	end process;

	process (clk)
	begin
		if clk'event and clk = '1' then
			delay <= conv_std_logic_vector(signed(i_RAM_A_Data)*signed(i_RAM_B_Data),G_RAM_DATA_WIDTH);
			prod <= delay;
		end if;
	end process;

end Behavioral;

