--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/64
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - Hardware Thread Idle
--
--   project:      ReconOS64
--   author:       see AUTHORS
--
--   description:  Hardware Thread in permanent idle state
--
-- ======================================================================

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

	-- Declare port attributes for the Vivado IP Packager
	ATTRIBUTE X_INTERFACE_INFO : STRING;
	ATTRIBUTE X_INTERFACE_PARAMETER : STRING;

	ATTRIBUTE X_INTERFACE_INFO of HWT_Clk: SIGNAL is "xilinx.com:signal:clock:1.0 HWT_Clk CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of HWT_Clk: SIGNAL is "ASSOCIATED_RESET HWT_Rst, ASSOCIATED_BUSIF OSIF_Sw2Hw:OSIF_Hw2Sw:MEMIF_Hwt2Mem:MEMIF_Mem2Hwt";

	ATTRIBUTE X_INTERFACE_INFO of HWT_Rst: SIGNAL is "xilinx.com:signal:reset:1.0 HWT_Rst RST";
	ATTRIBUTE X_INTERFACE_PARAMETER of HWT_Rst: SIGNAL is "POLARITY ACTIVE_HIGH";

	ATTRIBUTE X_INTERFACE_INFO of OSIF_Sw2Hw_Data:     SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 OSIF_Sw2Hw FIFO_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Sw2Hw_Empty:    SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 OSIF_Sw2Hw FIFO_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Sw2Hw_RE:       SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 OSIF_Sw2Hw FIFO_S_RE";

	ATTRIBUTE X_INTERFACE_INFO of OSIF_Hw2Sw_Data:     SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 OSIF_Hw2Sw FIFO_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Hw2Sw_Full:     SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 OSIF_Hw2Sw FIFO_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Hw2Sw_WE:       SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 OSIF_Hw2Sw FIFO_M_WE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Hwt2Mem_Data:  SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_Hwt2Mem FIFO_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Hwt2Mem_Full:  SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_Hwt2Mem FIFO_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Hwt2Mem_WE:    SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_Hwt2Mem FIFO_M_WE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Mem2Hwt_Data:  SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 MEMIF_Mem2Hwt FIFO_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Mem2Hwt_Empty: SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 MEMIF_Mem2Hwt FIFO_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Mem2Hwt_RE:    SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 MEMIF_Mem2Hwt FIFO_S_RE";

begin

	OSIF_FIFO_Sw2Hw_RE      <= '0';
	OSIF_FIFO_Hw2Sw_WE      <= '0';
	OSIF_FIFO_Hw2Sw_Data    <= (others => '0');

	MEMIF_FIFO_Mem2Hwt_RE   <= '0';
	MEMIF_FIFO_Hwt2Mem_WE   <= '0';
	MEMIF_FIFO_Hwt2Mem_Data <= (others => '0');

end architecture;
