--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - Memory Controller
--
--   project:      ReconOS
--   author:       Christoph R??thing, University of Paderborn
--   description:  A memory controller connecting the memory fifos with
--                 the axi bus of the system.
--
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library axi_master_burst_v2_0_7;
use 	 axi_master_burst_v2_0_7.axi_master_burst;


entity reconos_memif_memory_controller is
	--
	-- Generic definitions
	--
	--   C_M_AXI_ - @see axi bus
	--
	--   C_MAX_BURST_LEN - maximal allowed burst length
	--
	--   C_MEMIF_DATA_WIDTH - width of the memif
	--
	generic (
		C_M_AXI_ADDR_WIDTH : integer := 32;
		C_M_AXI_DATA_WIDTH : integer := 32;

		C_MAX_BURST_LEN : integer := 64;

		C_MEMIF_DATA_WIDTH : integer := 32
	);

	--
	-- Port definitions
	--
	--   MEMIF_Hwt2Mem_/MEMIF_Mem2Hwt_ - fifo signal inputs
	--
	--   M_AXI_ - @see axi bus
	--
	port (
		MEMIF_Hwt2Mem_In_Data  : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		MEMIF_Hwt2Mem_In_Empty : in  std_logic;
		MEMIF_Hwt2Mem_In_RE    : out std_logic;

		MEMIF_Mem2Hwt_In_Data  : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		MEMIF_Mem2Hwt_In_Full  : in  std_logic;
		MEMIF_Mem2Hwt_In_WE    : out std_logic;

		M_AXI_ACLK    : in  std_logic;
		M_AXI_ARESETN : in  std_logic;
		M_AXI_ARREADY : in  std_logic;
		M_AXI_ARVALID : out std_logic;
		M_AXI_ARADDR  : out std_logic_vector(C_M_AXI_ADDR_WIDTH - 1 downto 0);
		M_AXI_ARLEN   : out std_logic_vector(7 downto 0);
		M_AXI_ARSIZE  : out std_logic_vector(2 downto 0);
		M_AXI_ARBURST : out std_logic_vector(1 downto 0);
		M_AXI_ARPROT  : out std_logic_vector(2 downto 0);
		M_AXI_RREADY  : out std_logic;
		M_AXI_RVALID  : in  std_logic;
		M_AXI_RDATA   : in  std_logic_vector(C_M_AXI_DATA_WIDTH - 1 downto 0);
		M_AXI_RRESP   : in  std_logic_vector(1 downto 0);
		M_AXI_RLAST   : in  std_logic;
		M_AXI_AWREADY : in  std_logic;
		M_AXI_AWVALID : out std_logic;
		M_AXI_AWADDR  : out std_logic_vector(C_M_AXI_ADDR_WIDTH - 1 downto 0);
		M_AXI_AWLEN   : out std_logic_vector(7 downto 0);
		M_AXI_AWSIZE  : out std_logic_vector(2 downto 0);
		M_AXI_AWBURST : out std_logic_vector(1 downto 0);
		M_AXI_AWPROT  : out std_logic_vector(2 downto 0);
		M_AXI_WREADY  : in  std_logic;
		M_AXI_WVALID  : out std_logic;
		M_AXI_WDATA   : out std_logic_vector(C_M_AXI_DATA_WIDTH - 1 downto 0);
		M_AXI_WSTRB   : out std_logic_vector(C_M_AXI_DATA_WIDTH / 8 - 1 downto 0);
		M_AXI_WLAST   : out std_logic;
		M_AXI_BREADY  : out std_logic;
		M_AXI_BVALID  : in  std_logic;
		M_AXI_BRESP   : in  std_logic_vector(1 downto 0);

		M_AXI_AWCACHE : out std_logic_vector(3 downto 0);
		M_AXI_ARCACHE : out std_logic_vector(3 downto 0);
		M_AXI_AWUSER  : out std_logic_vector(4 downto 0);
		M_AXI_ARUSER  : out std_logic_vector(4 downto 0);

		DEBUG : out std_logic_vector(67 downto 0)
	);
end entity reconos_memif_memory_controller;


architecture imp of reconos_memif_memory_controller is
	-- Declare port attributes for the Vivado IP Packager
	ATTRIBUTE X_INTERFACE_INFO : STRING;
	ATTRIBUTE X_INTERFACE_PARAMETER : STRING;
	
	ATTRIBUTE X_INTERFACE_INFO of M_AXI_ACLK: SIGNAL is "xilinx.com:signal:clock:1.0 M_AXI_ACLK CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of M_AXI_ACLK: SIGNAL is "ASSOCIATED_BUSIF MEMIF_Hwt2Mem_In:MEMIF_Mem2Hwt_In:M_AXI";
	
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Hwt2Mem_In_Data:     SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 MEMIF_Hwt2Mem_In FIFO_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Hwt2Mem_In_Empty:    SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 MEMIF_Hwt2Mem_In FIFO_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Hwt2Mem_In_RE:       SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 MEMIF_Hwt2Mem_In FIFO_S_RE";
	
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Mem2Hwt_In_Data:     SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_Mem2Hwt_In FIFO_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Mem2Hwt_In_Full:     SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_Mem2Hwt_In FIFO_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Mem2Hwt_In_WE:       SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_Mem2Hwt_In FIFO_M_WE";

	--
	-- Internal ipif signals
	--
	--   @see axi_master_burst_v1_00_a
	--
	signal bus2ip_clk             : std_logic;
	signal bus2ip_resetn          : std_logic;
	signal ip2bus_mst_addr        : std_logic_vector(31 downto 0);
	signal ip2bus_mst_be          : std_logic_vector(3 downto 0);
	signal ip2bus_mst_length      : std_logic_vector(11 downto 0);
	signal ip2bus_mst_type        : std_logic;
	signal ip2bus_mst_lock        : std_logic;
	signal ip2bus_mst_reset       : std_logic;
	signal bus2ip_mst_cmdack      : std_logic;
	signal bus2ip_mst_cmplt       : std_logic;
	signal bus2ip_mst_error       : std_logic;
	signal ip2bus_mstrd_req       : std_logic;
	signal bus2ip_mstrd_d         : std_logic_vector(31 downto 0);
	signal bus2ip_mstrd_rem       : std_logic_vector(3 downto 0);
	signal bus2ip_mstrd_sof_n     : std_logic;
	signal bus2ip_mstrd_eof_n     : std_logic;
	signal bus2ip_mstrd_src_rdy_n : std_logic;
	signal bus2ip_mstrd_src_dsc_n : std_logic;
	signal ip2bus_mstrd_dst_rdy_n : std_logic;
	signal ip2bus_mstrd_dst_dsc_n : std_logic;
	signal ip2bus_mstwr_req       : std_logic;
	signal ip2bus_mstwr_d         : std_logic_vector(31 downto 0);
	signal ip2bus_mstwr_rem       : std_logic_vector(3 downto 0);
	signal ip2bus_mstwr_sof_n     : std_logic;
	signal ip2bus_mstwr_eof_n     : std_logic;
	signal ip2bus_mstwr_src_rdy_n : std_logic;
	signal ip2bus_mstwr_src_dsc_n : std_logic;
	signal bus2ip_mstwr_dst_rdy_n : std_logic;
	signal bus2ip_mstwr_dst_dsc_n : std_logic;

	signal MEMIF_Hwt2Mem_In_Data_d  : std_logic_vector(31 downto 0);
	signal MEMIF_Hwt2Mem_In_Empty_d : std_logic;
	signal MEMIF_Hwt2Mem_In_RE_d    : std_logic;

	signal MEMIF_Mem2Hwt_In_Data_d  : std_logic_vector(31 downto 0);
	signal MEMIF_Mem2Hwt_In_Full_d  : std_logic;
	signal MEMIF_Mem2Hwt_In_WE_d    : std_logic;
begin

	-- == Static assignemnts of signals ===================================

	M_AXI_AWCACHE <= (others => '1');
	M_AXI_ARCACHE <= (others => '1');
	M_AXI_AWUSER  <= (others => '1');
	M_AXI_ARUSER  <= (others => '1');

	bus2ip_clk    <= M_AXI_ACLK;
	bus2ip_resetn <= M_AXI_ARESETN;

	MEMIF_Hwt2Mem_In_Data_d  <= MEMIF_Hwt2Mem_In_Data;
	MEMIF_Hwt2Mem_In_Empty_d <= MEMIF_Hwt2Mem_In_Empty;
	MEMIF_Hwt2Mem_In_RE      <= MEMIF_Hwt2Mem_In_RE_d;
	MEMIF_Mem2Hwt_In_Data    <= MEMIF_Mem2Hwt_In_Data_d;
	MEMIF_Mem2Hwt_In_Full_d  <= MEMIF_Mem2Hwt_In_Full;
	MEMIF_Mem2Hwt_In_WE      <= MEMIF_Mem2Hwt_In_WE_d;

	DEBUG(67 downto 36) <= MEMIF_Hwt2Mem_In_Data_d;
	DEBUG(35) <= MEMIF_Hwt2Mem_In_Empty_d;
	DEBUG(34) <= MEMIF_Hwt2Mem_In_RE_d;
	DEBUG(33 downto 2) <= MEMIF_Mem2Hwt_In_Data_d;
	DEBUG(1) <= MEMIF_Mem2Hwt_In_Full_d;
	DEBUG(0) <= MEMIF_Mem2Hwt_In_WE_d;


	-- == Instantiation of components =====================================

	--
	-- Instantiation of axi_master_burst_v1_00_a
	--
	--   @see ds844_axi_master_burst.pdf
	--
        ipif : entity axi_master_burst_v2_0_7.axi_master_burst
		generic map (
			C_M_AXI_ADDR_WIDTH => C_M_AXI_ADDR_WIDTH,
			C_M_AXI_DATA_WIDTH => C_M_AXI_DATA_WIDTH,

			C_MAX_BURST_LEN => C_MAX_BURST_LEN
		)

		port map (
			m_axi_aclk    => M_AXI_ACLK,
			m_axi_aresetn => M_AXI_ARESETN,
			m_axi_arready => M_AXI_ARREADY,
			m_axi_arvalid => M_AXI_ARVALID,
			m_axi_araddr  => M_AXI_ARADDR,
			m_axi_arlen   => M_AXI_ARLEN,
			m_axi_arsize  => M_AXI_ARSIZE,
			m_axi_arburst => M_AXI_ARBURST,
			m_axi_arprot  => M_AXI_ARPROT,
			m_axi_rready  => M_AXI_RREADY,
			m_axi_rvalid  => M_AXI_RVALID,
			m_axi_rdata   => M_AXI_RDATA,
			m_axi_rresp   => M_AXI_RRESP,
			m_axi_rlast   => M_AXI_RLAST,
			m_axi_awready => M_AXI_AWREADY,
			m_axi_awvalid => M_AXI_AWVALID,
			m_axi_awaddr  => M_AXI_AWADDR,
			m_axi_awlen   => M_AXI_AWLEN,
			m_axi_awsize  => M_AXI_AWSIZE,
			m_axi_awburst => M_AXI_AWBURST,
			m_axi_awprot  => M_AXI_AWPROT,
			m_axi_wready  => M_AXI_WREADY,
			m_axi_wvalid  => M_AXI_WVALID,
			m_axi_wdata   => M_AXI_WDATA,
			m_axi_wstrb   => M_AXI_WSTRB,
			m_axi_wlast   => M_AXI_WLAST,
			m_axi_bready  => M_AXI_BREADY,
			m_axi_bvalid  => M_AXI_BVALID,
			m_axi_bresp   => M_AXI_BRESP,

			ip2bus_mst_addr        => ip2bus_mst_addr,
			ip2bus_mst_be          => ip2bus_mst_be,
			ip2bus_mst_length      => ip2bus_mst_length,
			ip2bus_mst_type        => ip2bus_mst_type,
			ip2bus_mst_lock        => ip2bus_mst_lock,
			ip2bus_mst_reset       => ip2bus_mst_reset,
			bus2ip_mst_cmdack      => bus2ip_mst_cmdack,
			bus2ip_mst_cmplt       => bus2ip_mst_cmplt,
			bus2ip_mst_error       => bus2ip_mst_error,
			ip2bus_mstrd_req       => ip2bus_mstrd_req,
			bus2ip_mstrd_d         => bus2ip_mstrd_d,
			bus2ip_mstrd_rem       => bus2ip_mstrd_rem,
			bus2ip_mstrd_sof_n     => bus2ip_mstrd_sof_n,
			bus2ip_mstrd_eof_n     => bus2ip_mstrd_eof_n,
			bus2ip_mstrd_src_rdy_n => bus2ip_mstrd_src_rdy_n,
			bus2ip_mstrd_src_dsc_n => bus2ip_mstrd_src_dsc_n,
			ip2bus_mstrd_dst_rdy_n => ip2bus_mstrd_dst_rdy_n,
			ip2bus_mstrd_dst_dsc_n => ip2bus_mstrd_dst_dsc_n,
			ip2bus_mstwr_req       => ip2bus_mstwr_req,
			ip2bus_mstwr_d         => ip2bus_mstwr_d,
			ip2bus_mstwr_rem       => ip2bus_mstwr_rem,
			ip2bus_mstwr_sof_n     => ip2bus_mstwr_sof_n,
			ip2bus_mstwr_eof_n     => ip2bus_mstwr_eof_n,
			ip2bus_mstwr_src_rdy_n => ip2bus_mstwr_src_rdy_n,
			ip2bus_mstwr_src_dsc_n => ip2bus_mstwr_src_dsc_n,
			bus2ip_mstwr_dst_rdy_n => bus2ip_mstwr_dst_rdy_n,
			bus2ip_mstwr_dst_dsc_n => bus2ip_mstwr_dst_dsc_n
		);

	--
	-- Instantiation of user logic
	--
	--   The user logic includes the actual implementation of the memory
	--   controller.
	--
	ul : entity work.reconos_memif_axicontroller_v0_91_M00_AXI
		port map (
			MEMIF_Hwt2Mem_In_Data  => MEMIF_Hwt2Mem_In_Data_d,
			MEMIF_Hwt2Mem_In_Empty => MEMIF_Hwt2Mem_In_Empty_d,
			MEMIF_Hwt2Mem_In_RE    => MEMIF_Hwt2Mem_In_RE_d,

			MEMIF_Mem2Hwt_In_Data  => MEMIF_Mem2Hwt_In_Data_d,
			MEMIF_Mem2Hwt_In_Full  => MEMIF_Mem2Hwt_In_Full_d,
			MEMIF_Mem2Hwt_In_WE    => MEMIF_Mem2Hwt_In_WE_d,

			BUS2IP_Clk             => bus2ip_clk,
			BUS2IP_Resetn          => bus2ip_resetn,
			IP2BUS_Mst_Addr        => ip2bus_mst_addr,
			IP2BUS_Mst_BE          => ip2bus_mst_be,
			IP2BUS_Mst_Length      => ip2bus_mst_length,
			IP2BUS_Mst_Type        => ip2bus_mst_type,
			IP2BUS_Mst_Lock        => ip2bus_mst_lock,
			IP2BUS_Mst_Reset       => ip2bus_mst_reset,
			BUS2IP_Mst_CmdAck      => bus2ip_mst_cmdack,
			BUS2IP_Mst_Cmplt       => bus2ip_mst_cmplt,
			BUS2IP_Mst_Error       => bus2ip_mst_error,
			IP2BUS_MstRd_Req       => ip2bus_mstrd_req,
			BUS2IP_MstRd_D         => bus2ip_mstrd_d,
			BUS2IP_MstRd_Rem       => bus2ip_mstrd_rem,
			BUS2IP_MstRd_Sof_N     => bus2ip_mstrd_sof_n,
			BUS2IP_MstRd_Eof_N     => bus2ip_mstrd_eof_n,
			BUS2IP_MstRd_Src_Rdy_N => bus2ip_mstrd_src_rdy_n,
			BUS2IP_MstRd_Src_Dsc_N => bus2ip_mstrd_src_dsc_n,
			IP2BUS_MstRd_Dst_Rdy_N => ip2bus_mstrd_dst_rdy_n,
			IP2BUS_MstRd_Dst_Dsc_N => ip2bus_mstrd_dst_dsc_n,
			IP2BUS_MstWr_Req       => ip2bus_mstwr_req,
			IP2BUS_MstWr_D         => ip2bus_mstwr_d,
			IP2BUS_MstWr_Rem       => ip2bus_mstwr_rem,
			IP2BUS_MstWr_Sof_N     => ip2bus_mstwr_sof_n,
			IP2BUS_MstWr_Eof_N     => ip2bus_mstwr_eof_n,
			IP2BUS_MstWr_Src_Rdy_N => ip2bus_mstwr_src_rdy_n,
			IP2BUS_MstWr_Src_Dsc_N => ip2bus_mstwr_src_dsc_n,
			BUS2IP_MstWr_Dst_Rdy_N => bus2ip_mstwr_dst_rdy_n,
			BUS2IP_MstWr_Dst_Dsc_N => bus2ip_mstwr_dst_dsc_n
		);
end architecture imp;