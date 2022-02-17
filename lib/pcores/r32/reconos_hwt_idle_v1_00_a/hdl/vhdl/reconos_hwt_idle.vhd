library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity reconos_hwt_idle is
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
		HWT_Signal : in  std_logic
	);
end entity reconos_hwt_idle;

architecture imp of reconos_hwt_idle is
begin

	OSIF_FIFO_Sw2Hw_RE      <= '0';
	OSIF_FIFO_Hw2Sw_WE      <= '0';
	OSIF_FIFO_Hw2Sw_Data    <= (others => '0');

	MEMIF_FIFO_Mem2Hwt_RE   <= '0';
	MEMIF_FIFO_Hwt2Mem_WE   <= '0';
	MEMIF_FIFO_Hwt2Mem_Data <= (others => '0');

end architecture;
