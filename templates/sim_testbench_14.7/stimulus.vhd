library ieee;
use ieee.std_logic_1164.all;

library reconos_v3_01_a;
use reconos_v3_01_a.reconos_pkg.all;
use reconos_v3_01_a.reconos_pkg_tb.all;

entity stimulus is
	port (
		-- OSIF FIFO ports
		OSIF_Hw2Sw_Data    : in  std_logic_vector(31 downto 0);
		OSIF_Hw2Sw_Empty   : in  std_logic;
		OSIF_Hw2Sw_RE      : out std_logic;

		OSIF_Sw2Hw_Data    : out std_logic_vector(31 downto 0);
		OSIF_Sw2Hw_Full    : in  std_logic;
		OSIF_Sw2Hw_WE      : out std_logic;

		SYS_Rst    : out std_logic;
		HWT_Signal : out std_logic
	);
end entity stimulus;

architecture stimulus of stimulus is
	signal tb_i_osif : tb_i_osif_t;
	signal tb_o_osif : tb_o_osif_t;
begin
	HWT_Signal <='0';

	tb_osif_setup (
		tb_i_osif,
		tb_o_osif,
		OSIF_Hw2Sw_Data,
		OSIF_Hw2Sw_Empty,
		OSIF_Hw2Sw_RE,
		OSIF_Sw2Hw_Data,
		OSIF_Sw2Hw_Full,
		OSIF_Sw2Hw_WE
	);


	stimulus : process
	begin
		SYS_Rst <= '1';
		tb_osif_reset(tb_o_osif);
		wait for C_CLK_PRD * 10;
		SYS_Rst <= '0';
		wait for C_CLK_PRD;
		tb_o_osif.sw2hw_data <= OSIF_SIGNAL_THREAD_START;
		tb_o_osif.sw2hw_we <= '1';
		wait for C_CLK_PRD;
		tb_o_osif.sw2hw_we <= '0';
		wait for C_CLK_PRD;

		--		
		-- insert stimulus here
		--

		tb_osif_mbox_get(tb_i_osif, tb_o_osif, x"00000000");
		tb_osif_mbox_put(tb_i_osif, tb_o_osif);
		
		wait;
	end process stimulus;
end architecture stimulus;

