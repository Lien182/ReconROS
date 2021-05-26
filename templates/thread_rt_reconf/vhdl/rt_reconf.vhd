library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.math_real.all;

library reconos_v3_01_a;
use reconos_v3_01_a.reconos_pkg.all;

use work.reconos_thread_pkg.all;

entity rt_reconf is
	port (
		-- OSIF FIFO ports
		OSIF_Sw2Hw_Data    : in  std_logic_vector(31 downto 0);
		OSIF_Sw2Hw_Empty   : in  std_logic;
		OSIF_Sw2Hw_RE      : out std_logic;

		OSIF_Hw2Sw_Data    : out std_logic_vector(31 downto 0);
		OSIF_Hw2Sw_Full    : in  std_logic;
		OSIF_Hw2Sw_WE      : out std_logic;

		-- MEMIF FIFO ports
		MEMIF_Hwt2Mem_Data    : out std_logic_vector(31 downto 0);
		MEMIF_Hwt2Mem_Full    : in  std_logic;
		MEMIF_Hwt2Mem_WE      : out std_logic;

		MEMIF_Mem2Hwt_Data    : in  std_logic_vector(31 downto 0);
		MEMIF_Mem2Hwt_Empty   : in  std_logic;
		MEMIF_Mem2Hwt_RE      : out std_logic;

		HWT_Clk    : in  std_logic;
		HWT_Rst    : in  std_logic;
		HWT_Signal : in  std_logic;

		DEBUG : out std_logic_vector(135 downto 0)
	);
end entity rt_reconf;

architecture implementation of rt_reconf is

	type STATE_TYPE is (STATE_WAIT,STATE_THREAD_EXIT);

	signal state    : STATE_TYPE;
	signal i_osif   : i_osif_t;
	signal o_osif   : o_osif_t;
	signal ignore   : std_logic_vector(32 downto 0);
begin

	-- Simple dummy thread for static design synthesis
	-- Tie unused output ports low

	--OSIF_Sw2Hw_RE        <= '0';
	--OSIF_Hw2Sw_Data      <= (others => '0');
	--OSIF_Hw2Sw_WE        <= '0';

	MEMIF_Hwt2Mem_Data <= (others => '0');
	MEMIF_Hwt2Mem_WE   <= '0';
	MEMIF_Mem2Hwt_RE   <= '0';

	DEBUG                <= (others => '0');

	osif_setup (
		i_osif,
		o_osif,
		OSIF_Sw2Hw_Data,
		OSIF_Sw2Hw_Empty,
		OSIF_Sw2Hw_RE,
		OSIF_Hw2Sw_Data,
		OSIF_Hw2Sw_Full,
		OSIF_Hw2Sw_WE
	);

	reconos_fsm: process (HWT_Clk,HWT_Rst,HWT_Signal,o_osif) is
		variable done : boolean;
	begin
		if rising_edge(HWT_Clk) then
			if HWT_Rst = '1' then
				osif_reset(o_osif);
				state <= STATE_WAIT;
				done := False;
			else
				case state is
					when STATE_WAIT =>
						if HWT_Signal = '1' then
							state <= STATE_THREAD_EXIT;
						end if;

					when STATE_THREAD_EXIT =>
						osif_thread_exit(i_osif,o_osif);

				end case;
			end if;
		end if;
	end process;

end architecture;