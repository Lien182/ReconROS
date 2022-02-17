--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/64
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
-- Company:  CEG UPB
-- Engineer: see AUTHORS
-- 
-- Module Name:    reconos_osif
-- Project Name:   ReconOS64
-- Target Devices: Zynq UltraScale+
-- Tool Versions:  2018.2, 2020.1
-- Description:    An AXI slave which maps the FIFOs of the HWTs to
--                 registers accessible from the AXI-Bus.
-- 
-- Dependencies:        "..._axi.vhd" submodule contains all AXI/user logic
-- 
-- Revision:            -1.0 First working 64-bit version
--
-- Additional Comments: -Based on Vivado AXI slave template (create new peripheral wizard)
--                      -Slave registers (per HWT):
--							Reg0: Read data
--                   		Reg1: Write data
--                   		Reg2: Fill - number of elements in receive-FIFO
--                   		Reg3: Rem - free space in send-FIFO
-- 
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity reconos_osif is
	generic (
		-- Users to add parameters here
		C_NUM_HWTS : integer := 1;

		C_OSIF_DATA_WIDTH   : integer := 64;
		C_OSIF_LENGTH_WIDTH : integer := 24;
		C_OSIF_OP_WIDTH     : integer := 8;
		-- User parameters ends

		-- Parameters of Axi Slave Bus Interface S00_AXI
		C_S00_AXI_DATA_WIDTH	: integer	:= 64;
		C_S00_AXI_ADDR_WIDTH	: integer	:= 10 -- limits max number of HWTs to 16
	);
	port (
		-- Users to add ports here
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
		-- User ports ends

		-- Ports of Axi Slave Bus Interface S00_AXI
		s00_axi_aclk	: in std_logic;
		s00_axi_aresetn	: in std_logic;
		s00_axi_awaddr	: in std_logic_vector(C_S00_AXI_ADDR_WIDTH-1 downto 0);
		s00_axi_awprot	: in std_logic_vector(2 downto 0);
		s00_axi_awvalid	: in std_logic;
		s00_axi_awready	: out std_logic;
		s00_axi_wdata	: in std_logic_vector(C_S00_AXI_DATA_WIDTH-1 downto 0);
		s00_axi_wstrb	: in std_logic_vector((C_S00_AXI_DATA_WIDTH/8)-1 downto 0);
		s00_axi_wvalid	: in std_logic;
		s00_axi_wready	: out std_logic;
		s00_axi_bresp	: out std_logic_vector(1 downto 0);
		s00_axi_bvalid	: out std_logic;
		s00_axi_bready	: in std_logic;
		s00_axi_araddr	: in std_logic_vector(C_S00_AXI_ADDR_WIDTH-1 downto 0);
		s00_axi_arprot	: in std_logic_vector(2 downto 0);
		s00_axi_arvalid	: in std_logic;
		s00_axi_arready	: out std_logic;
		s00_axi_rdata	: out std_logic_vector(C_S00_AXI_DATA_WIDTH-1 downto 0);
		s00_axi_rresp	: out std_logic_vector(1 downto 0);
		s00_axi_rvalid	: out std_logic;
		s00_axi_rready	: in std_logic
	);
end reconos_osif;

architecture arch_imp of reconos_osif is

	-- Declare port attributes for the Vivado IP Packager
	ATTRIBUTE X_INTERFACE_INFO : STRING;
	ATTRIBUTE X_INTERFACE_PARAMETER : STRING;

	ATTRIBUTE X_INTERFACE_INFO of s00_axi_aclk: SIGNAL is "xilinx.com:signal:clock:1.0 s00_axi_aclk CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of s00_axi_aclk: SIGNAL is "ASSOCIATED_BUSIF <<generate for SLOTS>>OSIF_Hw2Sw_<<Id>>:OSIF_Sw2Hw_<<Id>>:<<end generate>>s00_axi";

	<<generate for SLOTS>>
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Hw2Sw_<<Id>>_In_Data:     SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 OSIF_Hw2Sw_<<Id>> FIFO64_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Hw2Sw_<<Id>>_In_Empty:    SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 OSIF_Hw2Sw_<<Id>> FIFO64_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Hw2Sw_<<Id>>_In_RE:       SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 OSIF_Hw2Sw_<<Id>> FIFO64_S_RE";
	<<end generate>>

	<<generate for SLOTS>>
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Sw2Hw_<<Id>>_In_Data:     SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 OSIF_Sw2Hw_<<Id>> FIFO64_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Sw2Hw_<<Id>>_In_Full:     SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 OSIF_Sw2Hw_<<Id>> FIFO64_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Sw2Hw_<<Id>>_In_WE:       SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 OSIF_Sw2Hw_<<Id>> FIFO64_M_WE";
	<<end generate>>

	-- component declaration
	component reconos_osif_axi is
		generic (
		C_S_AXI_DATA_WIDTH	: integer	:= 64;
		C_S_AXI_ADDR_WIDTH	: integer	:= 8
		);
		port (
		S_AXI_ACLK	: in std_logic;
		S_AXI_ARESETN	: in std_logic;
		S_AXI_AWADDR	: in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		S_AXI_AWPROT	: in std_logic_vector(2 downto 0);
		S_AXI_AWVALID	: in std_logic;
		S_AXI_AWREADY	: out std_logic;
		S_AXI_WDATA	: in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		S_AXI_WSTRB	: in std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
		S_AXI_WVALID	: in std_logic;
		S_AXI_WREADY	: out std_logic;
		S_AXI_BRESP	: out std_logic_vector(1 downto 0);
		S_AXI_BVALID	: out std_logic;
		S_AXI_BREADY	: in std_logic;
		S_AXI_ARADDR	: in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		S_AXI_ARPROT	: in std_logic_vector(2 downto 0);
		S_AXI_ARVALID	: in std_logic;
		S_AXI_ARREADY	: out std_logic;
		S_AXI_RDATA	: out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		S_AXI_RRESP	: out std_logic_vector(1 downto 0);
		S_AXI_RVALID	: out std_logic;
		S_AXI_RREADY	: in std_logic
		);
	end component reconos_osif_axi;

begin

-- Instantiation of Axi Bus Interface S00_AXI
reconos_osif_axi_inst : entity work.reconos_osif_axi
	generic map (
		C_S_AXI_DATA_WIDTH	=> C_S00_AXI_DATA_WIDTH,
		C_S_AXI_ADDR_WIDTH	=> C_S00_AXI_ADDR_WIDTH,

		-- OSIF user parameters
		C_NUM_HWTS => C_NUM_HWTS,

		C_OSIF_DATA_WIDTH   => C_OSIF_DATA_WIDTH,
		C_OSIF_LENGTH_WIDTH => C_OSIF_LENGTH_WIDTH,
		C_OSIF_OP_WIDTH     => C_OSIF_OP_WIDTH
	)
	port map (
		-- OSIF user ports
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

		S_AXI_ACLK	=> s00_axi_aclk,
		S_AXI_ARESETN	=> s00_axi_aresetn,
		S_AXI_AWADDR	=> s00_axi_awaddr,
		S_AXI_AWPROT	=> s00_axi_awprot,
		S_AXI_AWVALID	=> s00_axi_awvalid,
		S_AXI_AWREADY	=> s00_axi_awready,
		S_AXI_WDATA	=> s00_axi_wdata,
		S_AXI_WSTRB	=> s00_axi_wstrb,
		S_AXI_WVALID	=> s00_axi_wvalid,
		S_AXI_WREADY	=> s00_axi_wready,
		S_AXI_BRESP	=> s00_axi_bresp,
		S_AXI_BVALID	=> s00_axi_bvalid,
		S_AXI_BREADY	=> s00_axi_bready,
		S_AXI_ARADDR	=> s00_axi_araddr,
		S_AXI_ARPROT	=> s00_axi_arprot,
		S_AXI_ARVALID	=> s00_axi_arvalid,
		S_AXI_ARREADY	=> s00_axi_arready,
		S_AXI_RDATA	=> s00_axi_rdata,
		S_AXI_RRESP	=> s00_axi_rresp,
		S_AXI_RVALID	=> s00_axi_rvalid,
		S_AXI_RREADY	=> s00_axi_rready
	);

end arch_imp;
