--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/64
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
-- Company:  CEG UPB
-- Engineer: Christoph Rüthing
--           Lennart Clausing
--           Felix Jentzsch
-- 
-- Module Name:    reconos_memif_memory_controller
-- Project Name:   ReconOS64
-- Target Devices: Zynq UltraScale+
-- Tool Versions:  2018.2
-- Description:    A memory controller connecting the memory fifos with
--                 the axi bus of the system.
-- 
-- Dependencies:        "..._axi.vhd" submodule contains all AXI/user logic
-- 
-- Revision:            -1.0 First working 64-bit version
--                      -1.1 Burst transfer support
--                      -1.2 Added WVALID throttling if (HLS-based) HWT sends data too slowly
--
-- Additional Comments: -Based on Vivado AXI master template (create new peripheral wizard)
--                      -Supports variable burst length up to specified maximum
--                      -MEMIF read/write functions handle alignment to chunk borders to avoid crossing 4K boundaries with a burst
--                      -Designed to connect to 128bit HPC0 slave port of PS via Interconnect (or Smartconnect) with width conversion
-- 
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity reconos_memif_memory_controller is
	generic (
		-- Users to add parameters here
        C_MAX_BURST_LEN : integer := 32;
        C_MEMIF_DATA_WIDTH : integer := 64;
		-- User parameters ends

		-- Parameters of Axi Master Bus Interface M00_AXI
		C_M00_AXI_BURST_LEN	: integer	:= 32;
		C_M00_AXI_ID_WIDTH	: integer	:= 1;
		C_M00_AXI_ADDR_WIDTH	: integer	:= 40;
		C_M00_AXI_DATA_WIDTH	: integer	:= 64;
		C_M00_AXI_AWUSER_WIDTH	: integer	:= 2;
		C_M00_AXI_ARUSER_WIDTH	: integer	:= 2;
		C_M00_AXI_WUSER_WIDTH	: integer	:= 2;
		C_M00_AXI_RUSER_WIDTH	: integer	:= 2;
		C_M00_AXI_BUSER_WIDTH	: integer	:= 2
	);
	port (
		-- Users to add ports here
		MEMIF64_Hwt2Mem_In_Data  : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
        MEMIF64_Hwt2Mem_In_Empty : in  std_logic;
        MEMIF64_Hwt2Mem_In_RE    : out std_logic;

        MEMIF64_Mem2Hwt_In_Data  : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
        MEMIF64_Mem2Hwt_In_Full  : in  std_logic;
        MEMIF64_Mem2Hwt_In_WE    : out std_logic;

        debug                    : out std_logic_vector(63 downto 0);
		-- User ports ends

		-- Ports of Axi Master Bus Interface M00_AXI
		m00_axi_aclk	: in std_logic;
		m00_axi_aresetn	: in std_logic;
		
		-- WRITE ADDRESS channel
		m00_axi_awid	: out std_logic_vector(C_M00_AXI_ID_WIDTH-1 downto 0);
		m00_axi_awaddr	: out std_logic_vector(C_M00_AXI_ADDR_WIDTH-1 downto 0);
		m00_axi_awlen	: out std_logic_vector(7 downto 0);
		m00_axi_awsize	: out std_logic_vector(2 downto 0);
		m00_axi_awburst	: out std_logic_vector(1 downto 0);
		m00_axi_awlock	: out std_logic;
		m00_axi_awcache	: out std_logic_vector(3 downto 0);
		m00_axi_awprot	: out std_logic_vector(2 downto 0);
		m00_axi_awqos	: out std_logic_vector(3 downto 0);
		m00_axi_awuser	: out std_logic_vector(C_M00_AXI_AWUSER_WIDTH-1 downto 0);
		m00_axi_awvalid	: out std_logic;
		m00_axi_awready	: in std_logic;
		
		-- WRITE DATA channel
		m00_axi_wdata	: out std_logic_vector(C_M00_AXI_DATA_WIDTH-1 downto 0);
		m00_axi_wstrb	: out std_logic_vector(C_M00_AXI_DATA_WIDTH/8-1 downto 0);
		m00_axi_wlast	: out std_logic;
		m00_axi_wuser	: out std_logic_vector(C_M00_AXI_WUSER_WIDTH-1 downto 0);
		m00_axi_wvalid	: out std_logic;
		m00_axi_wready	: in std_logic;
		
		-- WRITE RESPONSE channel
		m00_axi_bid	: in std_logic_vector(C_M00_AXI_ID_WIDTH-1 downto 0);
		m00_axi_bresp	: in std_logic_vector(1 downto 0);
		m00_axi_buser	: in std_logic_vector(C_M00_AXI_BUSER_WIDTH-1 downto 0);
		m00_axi_bvalid	: in std_logic;
		m00_axi_bready	: out std_logic;
		
		-- READ ADDRESS channel
		m00_axi_arid	: out std_logic_vector(C_M00_AXI_ID_WIDTH-1 downto 0);
		m00_axi_araddr	: out std_logic_vector(C_M00_AXI_ADDR_WIDTH-1 downto 0);
		m00_axi_arlen	: out std_logic_vector(7 downto 0);
		m00_axi_arsize	: out std_logic_vector(2 downto 0);
		m00_axi_arburst	: out std_logic_vector(1 downto 0);
		m00_axi_arlock	: out std_logic;
		m00_axi_arcache	: out std_logic_vector(3 downto 0);
		m00_axi_arprot	: out std_logic_vector(2 downto 0);
		m00_axi_arqos	: out std_logic_vector(3 downto 0);
		m00_axi_aruser	: out std_logic_vector(C_M00_AXI_ARUSER_WIDTH-1 downto 0);
		m00_axi_arvalid	: out std_logic;
		m00_axi_arready	: in std_logic;
		
		-- READ ADDRESS channel
		m00_axi_rid	: in std_logic_vector(C_M00_AXI_ID_WIDTH-1 downto 0);
		m00_axi_rdata	: in std_logic_vector(C_M00_AXI_DATA_WIDTH-1 downto 0);
		m00_axi_rresp	: in std_logic_vector(1 downto 0);
		m00_axi_rlast	: in std_logic;
		m00_axi_ruser	: in std_logic_vector(C_M00_AXI_RUSER_WIDTH-1 downto 0);
		m00_axi_rvalid	: in std_logic;
		m00_axi_rready	: out std_logic
	);
end reconos_memif_memory_controller;

architecture arch_imp of reconos_memif_memory_controller is

	-- Declare port attributes for the Vivado IP Packager
	ATTRIBUTE X_INTERFACE_INFO : STRING;
	ATTRIBUTE X_INTERFACE_PARAMETER : STRING;

	ATTRIBUTE X_INTERFACE_INFO of m00_axi_aclk: SIGNAL is "xilinx.com:signal:clock:1.0 m00_axi_aclk CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of m00_axi_aclk: SIGNAL is "ASSOCIATED_BUSIF MEMIF64_Hwt2Mem_In:MEMIF64_Mem2Hwt_In:m00_axi";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_In_Data:     SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Hwt2Mem_In FIFO64_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_In_Empty:    SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Hwt2Mem_In FIFO64_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_In_RE:       SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Hwt2Mem_In FIFO64_S_RE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_In_Data:     SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Mem2Hwt_In FIFO64_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_In_Full:     SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Mem2Hwt_In FIFO64_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_In_WE:       SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Mem2Hwt_In FIFO64_M_WE";

	-- Component declaration
	component reconos64_memif_axicontroller_v0_91_M00_AXI is
		generic (
		C_MEMIF_DATA_WIDTH : integer := 64;
		
		C_M_AXI_BURST_LEN	: integer	:= 16;
		C_M_AXI_ID_WIDTH	: integer	:= 1;
		C_M_AXI_ADDR_WIDTH	: integer	:= 40;
		C_M_AXI_DATA_WIDTH	: integer	:= 64;
		C_M_AXI_AWUSER_WIDTH	: integer	:= 2;
		C_M_AXI_ARUSER_WIDTH	: integer	:= 2;
		C_M_AXI_WUSER_WIDTH	: integer	:= 2;
		C_M_AXI_RUSER_WIDTH	: integer	:= 2;
		C_M_AXI_BUSER_WIDTH	: integer	:= 2
		);
		port (
		MEMIF64_Hwt2Mem_In_Data  : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
        MEMIF64_Hwt2Mem_In_Empty : in  std_logic;
        MEMIF64_Hwt2Mem_In_RE    : out std_logic;

        MEMIF64_Mem2Hwt_In_Data  : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
        MEMIF64_Mem2Hwt_In_Full  : in  std_logic;
        MEMIF64_Mem2Hwt_In_WE    : out std_logic;
        
        DEBUG                    : out std_logic_vector(63 downto 0);
		
		M_AXI_ACLK	: in std_logic;
		M_AXI_ARESETN	: in std_logic;
		
		-- WRITE ADDRESS channel
		M_AXI_AWID	: out std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
		M_AXI_AWADDR	: out std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
		M_AXI_AWLEN	: out std_logic_vector(7 downto 0);
		M_AXI_AWSIZE	: out std_logic_vector(2 downto 0);
		M_AXI_AWBURST	: out std_logic_vector(1 downto 0);
		M_AXI_AWLOCK	: out std_logic;
		M_AXI_AWCACHE	: out std_logic_vector(3 downto 0);
		M_AXI_AWPROT	: out std_logic_vector(2 downto 0);
		M_AXI_AWQOS	: out std_logic_vector(3 downto 0);
		M_AXI_AWUSER	: out std_logic_vector(C_M_AXI_AWUSER_WIDTH-1 downto 0);
		M_AXI_AWVALID	: out std_logic;
		M_AXI_AWREADY	: in std_logic;
		
		-- WRITE DATA channel
		M_AXI_WDATA	: out std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
		M_AXI_WSTRB	: out std_logic_vector(C_M_AXI_DATA_WIDTH/8-1 downto 0);
		M_AXI_WLAST	: out std_logic;
		M_AXI_WUSER	: out std_logic_vector(C_M_AXI_WUSER_WIDTH-1 downto 0);
		M_AXI_WVALID	: out std_logic;
		M_AXI_WREADY	: in std_logic;
		
		-- WRITE RESPONSE channel
		M_AXI_BID	: in std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
		M_AXI_BRESP	: in std_logic_vector(1 downto 0);
		M_AXI_BUSER	: in std_logic_vector(C_M_AXI_BUSER_WIDTH-1 downto 0);
		M_AXI_BVALID	: in std_logic;
		M_AXI_BREADY	: out std_logic;
		
		-- READ ADDRESS channel
		M_AXI_ARID	: out std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
		M_AXI_ARADDR	: out std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
		M_AXI_ARLEN	: out std_logic_vector(7 downto 0);
		M_AXI_ARSIZE	: out std_logic_vector(2 downto 0);
		M_AXI_ARBURST	: out std_logic_vector(1 downto 0);
		M_AXI_ARLOCK	: out std_logic;
		M_AXI_ARCACHE	: out std_logic_vector(3 downto 0);
		M_AXI_ARPROT	: out std_logic_vector(2 downto 0);
		M_AXI_ARQOS	: out std_logic_vector(3 downto 0);
		M_AXI_ARUSER	: out std_logic_vector(C_M_AXI_ARUSER_WIDTH-1 downto 0);
		M_AXI_ARVALID	: out std_logic;
		M_AXI_ARREADY	: in std_logic;
		
		-- READ DATA channel
		M_AXI_RID	: in std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
		M_AXI_RDATA	: in std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
		M_AXI_RRESP	: in std_logic_vector(1 downto 0);
		M_AXI_RLAST	: in std_logic;
		M_AXI_RUSER	: in std_logic_vector(C_M_AXI_RUSER_WIDTH-1 downto 0);
		M_AXI_RVALID	: in std_logic;
		M_AXI_RREADY	: out std_logic
		);
	end component reconos64_memif_axicontroller_v0_91_M00_AXI;

begin

-- Instantiation of AXI/user logic submodule
reconos64_memif_axicontroller_v0_91_M00_AXI_inst : entity work.reconos64_memif_axicontroller_v0_91_M00_AXI
	generic map (
        C_MEMIF_DATA_WIDTH          => C_MEMIF_DATA_WIDTH,
        
		C_M_AXI_BURST_LEN	        => C_M00_AXI_BURST_LEN,
		C_M_AXI_ID_WIDTH	        => C_M00_AXI_ID_WIDTH,
		C_M_AXI_ADDR_WIDTH	        => C_M00_AXI_ADDR_WIDTH,
		C_M_AXI_DATA_WIDTH	        => C_M00_AXI_DATA_WIDTH,
		C_M_AXI_AWUSER_WIDTH	    => C_M00_AXI_AWUSER_WIDTH,
		C_M_AXI_ARUSER_WIDTH	    => C_M00_AXI_ARUSER_WIDTH,
		C_M_AXI_WUSER_WIDTH	        => C_M00_AXI_WUSER_WIDTH,
		C_M_AXI_RUSER_WIDTH	        => C_M00_AXI_RUSER_WIDTH,
		C_M_AXI_BUSER_WIDTH	        => C_M00_AXI_BUSER_WIDTH
	)
	port map (
        MEMIF64_Hwt2Mem_In_Data  => MEMIF64_Hwt2Mem_In_Data,
        MEMIF64_Hwt2Mem_In_Empty => MEMIF64_Hwt2Mem_In_Empty,
        MEMIF64_Hwt2Mem_In_RE    => MEMIF64_Hwt2Mem_In_RE,
    
        MEMIF64_Mem2Hwt_In_Data  => MEMIF64_Mem2Hwt_In_Data,
        MEMIF64_Mem2Hwt_In_Full  => MEMIF64_Mem2Hwt_In_Full,
        MEMIF64_Mem2Hwt_In_WE    => MEMIF64_Mem2Hwt_In_WE,
        
        DEBUG           => debug,
        
		M_AXI_ACLK	    => m00_axi_aclk,
		M_AXI_ARESETN	=> m00_axi_aresetn,
		M_AXI_AWID	    => m00_axi_awid,
		M_AXI_AWADDR	=> m00_axi_awaddr,
		M_AXI_AWLEN	    => m00_axi_awlen,
		M_AXI_AWSIZE	=> m00_axi_awsize,
		M_AXI_AWBURST	=> m00_axi_awburst,
		M_AXI_AWLOCK	=> m00_axi_awlock,
		M_AXI_AWCACHE	=> m00_axi_awcache,
		M_AXI_AWPROT	=> m00_axi_awprot,
		M_AXI_AWQOS	    => m00_axi_awqos,
		M_AXI_AWUSER	=> m00_axi_awuser,
		M_AXI_AWVALID	=> m00_axi_awvalid,
		M_AXI_AWREADY	=> m00_axi_awready,
		M_AXI_WDATA	    => m00_axi_wdata,
		M_AXI_WSTRB	    => m00_axi_wstrb,
		M_AXI_WLAST	    => m00_axi_wlast,
		M_AXI_WUSER	    => m00_axi_wuser,
		M_AXI_WVALID	=> m00_axi_wvalid,
		M_AXI_WREADY	=> m00_axi_wready,
		M_AXI_BID	    => m00_axi_bid,
		M_AXI_BRESP	    => m00_axi_bresp,
		M_AXI_BUSER	    => m00_axi_buser,
		M_AXI_BVALID	=> m00_axi_bvalid,
		M_AXI_BREADY	=> m00_axi_bready,
		M_AXI_ARID	    => m00_axi_arid,
		M_AXI_ARADDR	=> m00_axi_araddr,
		M_AXI_ARLEN	    => m00_axi_arlen,
		M_AXI_ARSIZE	=> m00_axi_arsize,
		M_AXI_ARBURST	=> m00_axi_arburst,
		M_AXI_ARLOCK	=> m00_axi_arlock,
		M_AXI_ARCACHE	=> m00_axi_arcache,
		M_AXI_ARPROT	=> m00_axi_arprot,
		M_AXI_ARQOS	    => m00_axi_arqos,
		M_AXI_ARUSER	=> m00_axi_aruser,
		M_AXI_ARVALID	=> m00_axi_arvalid,
		M_AXI_ARREADY	=> m00_axi_arready,
		M_AXI_RID	    => m00_axi_rid,
		M_AXI_RDATA	    => m00_axi_rdata,
		M_AXI_RRESP	    => m00_axi_rresp,
		M_AXI_RLAST	    => m00_axi_rlast,
		M_AXI_RUSER	    => m00_axi_ruser,
		M_AXI_RVALID	=> m00_axi_rvalid,
		M_AXI_RREADY	=> m00_axi_rready
	);

end arch_imp;
