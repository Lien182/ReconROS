--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - Clock - Top level entity
--
--   project:      ReconOS64
--   author:       see AUTHORS
--   description:  A clock manager which can be configures via the AXI
--                 bus. Therefore it provides the following write only
--                 registers:
--                   Reg#i#: Clock 1 and 2 register of pll#i#
--
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library axi_lite_ipif_v3_0_4;
use 	axi_lite_ipif_v3_0_4.ipif_pkg.all;
use 	axi_lite_ipif_v3_0_4.axi_lite_ipif;



entity reconos_clock is
	--
	-- Generic definitions
	--
	--   C_S_AXI_ - @see axi bus
	--
	--   C_BASE_ADDR - lower address of axi slave
	--   C_HIGH_ADDR - higher address of axi slave
	--
	--   C_NUM_CLOCKS - number of clocks
	--
	--   C_CLKIN_PERIOD - input clock period
	--
	--   C_CLK#i# - pll generics
	--
	generic (
		C_S_AXI_ADDR_WIDTH : integer := 32;
		C_S_AXI_DATA_WIDTH : integer := 32;

		C_BASEADDR : std_logic_vector := x"FFFFFFFF";
		C_HIGHADDR : std_logic_vector := x"00000000";

		C_NUM_CLOCKS: integer := 1;


		-- Silly Vivado does not yet (Version 2017.1) support real typed generics.
		-- See AR# 58038 : https://www.xilinx.com/support/answers/58038.html
		C_CLKIN_PERIOD : integer := 10;

		<<generate for CLOCKS>>
		C_CLK<<Id>>_CLKFBOUT_MULT : integer := 16;
		C_CLK<<Id>>_DIVCLK_DIVIDE : integer := 1;
		C_CLK<<Id>>_CLKOUT_DIVIDE : integer := 16<<c;>>
		<<end generate>>
	);

	--
	-- Port defintions
	--
	--   CLK_Ref  - reference clock
	--
	--   CLK#i#_ - clock outputs
	--
	--   S_AXI_ - @see axi bus
	--
	port (
		CLK_Ref       : in std_logic;

		<<generate for CLOCKS>>
		CLK<<Id>>_Out    : out std_logic;
		CLK<<Id>>_Locked : out std_logic;
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
end entity reconos_clock;

architecture imp of reconos_clock is

	-- Declare port attributes for the Vivado IP Packager
	ATTRIBUTE X_INTERFACE_INFO : STRING;
	ATTRIBUTE X_INTERFACE_PARAMETER : STRING;

	ATTRIBUTE X_INTERFACE_INFO of CLK_Ref: SIGNAL is "xilinx.com:signal:clock:1.0 CLK_Ref CLK";

	<<generate for CLOCKS>>
	ATTRIBUTE X_INTERFACE_INFO of CLK<<Id>>_Out: SIGNAL is "xilinx.com:signal:clock:1.0 CLK<<Id>>_Out CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of CLK<<Id>>_Out: SIGNAL is "FREQ_HZ 100000000";
	<<end generate>>

	--
	-- Internal ipif signals
	--
	--   @see axi_lite_ipif_v1_01_a
	--
	signal bus2ip_clk    : std_logic;
	signal bus2ip_resetn : std_logic;
	signal bus2ip_data   : std_logic_vector(31 downto 0);
	signal bus2ip_cs     : std_logic_vector(C_NUM_CLOCKS - 1 downto 0);
	signal bus2ip_rdce   : std_logic_vector(C_NUM_CLOCKS - 1 downto 0);
	signal bus2ip_wrce   : std_logic_vector(C_NUM_CLOCKS - 1 downto 0);
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
		<<generate for CLOCKS>>
		2 * <<_i>> + 0 => C_ADDR_PAD & std_logic_vector(unsigned(C_BASEADDR) + <<_i>> * 4),
		2 * <<_i>> + 1 => C_ADDR_PAD & std_logic_vector(unsigned(C_BASEADDR) + <<_i>> * 4 + 3)<<c,>>
		<<end generate>>
	);

	constant C_ARD_NUM_CE_ARRAY : INTEGER_ARRAY_TYPE := (
		<<generate for CLOCKS>>
		<<_i>> => 1<<c,>>
		<<end generate>>
	);
begin

	-- == Instantiation of components =====================================

	--
	-- Instantiation of axi_lite_ipif_v1_01_a
	--
	--   @see axi_lite_ipif_ds765.pdf
	--

        ipif : entity axi_lite_ipif_v3_0_4.axi_lite_ipif

		generic map (
			C_S_AXI_ADDR_WIDTH => C_S_AXI_ADDR_WIDTH,
			C_S_AXI_DATA_WIDTH => C_S_AXI_DATA_WIDTH,

			C_ARD_ADDR_RANGE_ARRAY => C_ARD_ADDR_RANGE_ARRAY,
			C_ARD_NUM_CE_ARRAY     => C_ARD_NUM_CE_ARRAY,
			C_DPHASE_TIMEOUT       => 64
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
	ul : entity work.reconos_clock_user_logic
		generic map (
			C_NUM_CLOCKS => C_NUM_CLOCKS,

			C_CLKIN_PERIOD => REAL(C_CLKIN_PERIOD),

			<<generate for CLOCKS>>
			C_CLK<<Id>>_CLKFBOUT_MULT => C_CLK<<Id>>_CLKFBOUT_MULT,
			C_CLK<<Id>>_DIVCLK_DIVIDE => C_CLK<<Id>>_DIVCLK_DIVIDE,
			C_CLK<<Id>>_CLKOUT_DIVIDE => C_CLK<<Id>>_CLKOUT_DIVIDE<<c,>>
			<<end generate>>
		)

		port map (
			CLK_Ref => CLK_Ref,

			<<generate for CLOCKS>>
			CLK<<Id>>_Out    => CLK<<Id>>_Out,
			CLK<<Id>>_Locked => CLK<<Id>>_Locked,
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
