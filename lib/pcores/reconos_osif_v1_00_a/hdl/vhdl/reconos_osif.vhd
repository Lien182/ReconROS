--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - OSIF - Top level entity
--
--   project:      ReconOS
--   author:       Christoph R??thing, University of Paderborn
--   description:  A AXI slave which maps the FIFOs of the HWTs to
--                 registers accessible from the AXI-Bus.
--                   Reg0: Read data
--                   Reg1: Write data
--                   Reg2: Fill - number of elements in receive-FIFO
--                   Reg3: Rem - free space in send-FIFO
--
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

<<if TOOL=="ise">>
library proc_common_v3_00_a;
use proc_common_v3_00_a.ipif_pkg.all;

library axi_lite_ipif_v1_01_a;
use axi_lite_ipif_v1_01_a.axi_lite_ipif;
<<end if>>

<<if TOOL=="vivado">>
library axi_lite_ipif_v3_0_4;
use 	 axi_lite_ipif_v3_0_4.ipif_pkg.all;
use 	 axi_lite_ipif_v3_0_4.axi_lite_ipif;
<<end if>>


library reconos_osif_v1_00_a;
use reconos_osif_v1_00_a.user_logic;

entity reconos_osif is
	--
	-- Generic definitions
	--
	--   C_S_AXI_ - @see axi bus
	--
	--   C_BASE_ADDR - lower address of axi slave
	--   C_HIGH_ADDR - higher address of axi slave
	--
	--   C_NUM_HWTS - number of hardware threads
	--
	--   C_OSIF_DATA_WIDTH   - width of the osif
	--   C_OSIF_LENGTH_WIDTH - width of the length in command word
	--   C_OSIF_OP_WIDTH     - width of the operation in command word
	--
	generic (
		C_S_AXI_ADDR_WIDTH : integer := 32;
		C_S_AXI_DATA_WIDTH : integer := 32;

		C_BASEADDR : std_logic_vector := x"FFFFFFFF";
		C_HIGHADDR : std_logic_vector := x"00000000";

		C_NUM_HWTS : integer := 1;

		C_OSIF_DATA_WIDTH   : integer := 32;
		C_OSIF_LENGTH_WIDTH : integer := 24;
		C_OSIF_OP_WIDTH     : integer := 8
	);

	--
	-- Port defintions
	--
	--   OSIF_Hw2Sw_#i#_In_/OSIF_Sw2Hw_#i#_In_ - fifo signal inputs
	--
	--   S_AXI_ - @see axi bus
	--
	port (
		<<generate for SLOTS>>
		OSIF_Hw2Sw_<<Id>>_In_Data  : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		OSIF_Hw2Sw_<<Id>>_In_Empty : in  std_logic;
		OSIF_Hw2Sw_<<Id>>_In_RE    : out std_logic;
		<<end generate>>

		<<generate for SLOTS>>
		OSIF_Sw2Hw_<<Id>>_In_Data  : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		OSIF_Sw2Hw_<<Id>>_In_Full  : in  std_logic;
		OSIF_Sw2Hw_<<Id>>_In_WE    : out std_logic;
		<<end generate>>

		S_AXI_ACLK    : in  std_logic;
		S_AXI_ARESETN : in  std_logic;
		S_AXI_AWADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		S_AXI_AWVALID : in  std_logic;
		S_AXI_WDATA   : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		S_AXI_WSTRB   : in  std_logic_vector(C_S_AXI_DATA_WIDTH / 8 - 1 downto 0);
		S_AXI_WVALID  : in  std_logic;
		S_AXI_BREADY  : in  std_logic;
		S_AXI_ARADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		S_AXI_ARVALID : in  std_logic;
		S_AXI_RREADY  : in  std_logic;
		S_AXI_ARREADY : out std_logic;
		S_AXI_RDATA   : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		S_AXI_RRESP   : out std_logic_vector(1 downto 0);
		S_AXI_RVALID  : out std_logic;
		S_AXI_WREADY  : out std_logic;
		S_AXI_BRESP   : out std_logic_vector(1 downto 0);
		S_AXI_BVALID  : out std_logic;
		S_AXI_AWREADY : out std_logic
	);
end entity reconos_osif;

architecture imp of reconos_osif is
	--
	-- Internal ipif signals
	--
	--   @see axi_lite_ipif_v1_01_a
	--
	signal bus2ip_clk    : std_logic;
	signal bus2ip_resetn : std_logic;
	signal bus2ip_data   : std_logic_vector(31 downto 0);
	signal bus2ip_cs     : std_logic_vector(C_NUM_HWTS - 1 downto 0);
	signal bus2ip_rdce   : std_logic_vector(C_NUM_HWTS * 4 - 1 downto 0);
	signal bus2ip_wrce   : std_logic_vector(C_NUM_HWTS * 4 - 1 downto 0);
	signal ip2bus_data   : std_logic_vector(31 downto 0);
	signal ip2bus_rdack  : std_logic;
	signal ip2bus_wrack  : std_logic;
	signal ip2bus_error  : std_logic;

	--
	-- Constants to configure ipif
	--
	--   @see axi_lite_ipif_v1_01_a
	--
	constant C_ADDR_PAD : std_logic_vector(31 downto 0) := (others => '0');

	constant C_ARD_ADDR_RANGE_ARRAY : SLV64_ARRAY_TYPE := (
		<<generate for SLOTS>>
		2 * <<_i>> + 0 => C_ADDR_PAD & std_logic_vector(unsigned(C_BASEADDR) + <<_i>> * 16),
		2 * <<_i>> + 1 => C_ADDR_PAD & std_logic_vector(unsigned(C_BASEADDR) + <<_i>> * 16 + 15)<<c,>>
		<<end generate>>
	);

	constant C_ARD_NUM_CE_ARRAY : INTEGER_ARRAY_TYPE := (
		<<generate for SLOTS>>
		<<_i>> => 4<<c,>>
		<<end generate>>
	);
begin

	-- == Instantiation of components =====================================

	--
	-- Instantiation of axi_lite_ipif_v1_01_a
	--
	--   @see axi_lite_ipif_ds765.pdf
	--
<<if TOOL=="ise">>
	ipif : entity axi_lite_ipif_v1_01_a.axi_lite_ipif
<<end if>>
<<if TOOL=="vivado">>
        ipif : entity axi_lite_ipif_v3_0_4.axi_lite_ipif
<<end if>>
		generic map (
			C_S_AXI_ADDR_WIDTH => C_S_AXI_ADDR_WIDTH,
			C_S_AXI_DATA_WIDTH => C_S_AXI_DATA_WIDTH,

			C_ARD_ADDR_RANGE_ARRAY => C_ARD_ADDR_RANGE_ARRAY,
			C_ARD_NUM_CE_ARRAY     => C_ARD_NUM_CE_ARRAY
		)

		port map (
			s_axi_aclk    => S_AXI_ACLK,
			s_axi_aresetn => S_AXI_ARESETN,
			s_axi_awaddr  => S_AXI_AWADDR,
			s_axi_awvalid => S_AXI_AWVALID,
			s_axi_wdata   => S_AXI_WDATA,
			s_axi_wstrb   => S_AXI_WSTRB,
			s_axi_wvalid  => S_AXI_WVALID,
			s_axi_bready  => S_AXI_BREADY,
			s_axi_araddr  => S_AXI_ARADDR,
			s_axi_arvalid => S_AXI_ARVALID,
			s_axi_rready  => S_AXI_RREADY,
			s_axi_arready => S_AXI_ARREADY,
			s_axi_rdata   => S_AXI_RDATA,
			s_axi_rresp   => S_AXI_RRESP,
			s_axi_rvalid  => S_AXI_RVALID,
			s_axi_wready  => S_AXI_WREADY,
			s_axi_bresp   => S_AXI_BRESP,
			s_axi_bvalid  => S_AXI_BVALID,
			s_axi_awready => S_AXI_AWREADY,

			bus2ip_clk    => bus2ip_clk,
			bus2ip_resetn => bus2ip_resetn,
			bus2ip_data   => bus2ip_data,
			bus2ip_cs     => bus2ip_cs,
			bus2ip_rdce   => bus2ip_rdce,
			bus2ip_wrce   => bus2ip_wrce,
			ip2bus_data   => ip2bus_data,
			ip2bus_rdack  => ip2bus_rdack,
			ip2bus_wrack  => ip2bus_wrack,
			ip2bus_error  => ip2bus_error
		);

	--
	-- Instantiation of user logic
	--
	--   The user logic includes the actual implementation of the bus
	--   attachment.
	--
	ul : entity reconos_osif_v1_00_a.user_logic
		generic map (
			C_NUM_HWTS => C_NUM_HWTS,

			C_OSIF_DATA_WIDTH   => C_OSIF_DATA_WIDTH,
			C_OSIF_LENGTH_WIDTH => C_OSIF_LENGTH_WIDTH,
			C_OSIF_OP_WIDTH     => C_OSIF_OP_WIDTH
		)

		port map (
			<<generate for SLOTS>>
			OSIF_Hw2Sw_<<Id>>_In_Data  => OSIF_Hw2Sw_<<Id>>_In_Data,
			OSIF_Hw2Sw_<<Id>>_In_Empty => OSIF_Hw2Sw_<<Id>>_In_Empty,
			OSIF_Hw2Sw_<<Id>>_In_RE    => OSIF_Hw2Sw_<<Id>>_In_RE,
			<<end generate>>

			<<generate for SLOTS>>
			OSIF_Sw2Hw_<<Id>>_In_Data  => OSIF_Sw2Hw_<<Id>>_In_Data,
			OSIF_Sw2Hw_<<Id>>_In_Full  => OSIF_Sw2Hw_<<Id>>_In_Full,
			OSIF_Sw2Hw_<<Id>>_In_WE    => OSIF_Sw2Hw_<<Id>>_In_WE,
			<<end generate>>

			BUS2IP_Clk    => bus2ip_clk,
			BUS2IP_Resetn => bus2ip_resetn,
			BUS2IP_Data   => bus2ip_data,
			BUS2IP_CS     => bus2ip_cs,
			BUS2IP_RdCE   => bus2ip_rdce,
			BUS2IP_WrCE   => bus2ip_wrce,
			IP2BUS_Data   => ip2bus_data,
			IP2BUS_RdAck  => ip2bus_rdack,
			IP2BUS_WrAck  => ip2bus_wrack,
			IP2BUS_Error  => ip2bus_error
		);
end architecture imp;