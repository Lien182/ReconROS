--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/64
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - PROC_CONTROL
--
--   project:      ReconOS64
--   author:       see AUTHORS
--
--   description:  proc_control AXI module 
--                 top level - implementation in ..._axi.vhd
-- 
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity reconos_proc_control is
	generic (
		-- Users to add parameters here
        C_NUM_HWTS     : integer := 1;
		-- User parameters ends
		-- Do not modify the parameters beyond this line

		-- Parameters of Axi Slave Bus Interface S00_AXI
		C_S00_AXI_DATA_WIDTH	: integer	:= 64;
		C_S00_AXI_ADDR_WIDTH	: integer	:= 6
	);
	port (
		-- Users to add ports here
		<<generate for SLOTS>>
		PROC_Hwt_Rst_<<Id>>     : out std_logic;
		PROC_Hwt_Signal_<<Id>>  : out std_logic;
		<<end generate>>

        PROC_Sys_Rst         : out std_logic;
        PROC_Pgf_Int         : out std_logic;

        -- MMU related ports
        MMU_Pgf              : in  std_logic;
        MMU_Fault_Addr       : in  std_logic_vector(63 downto 0);
        MMU_Retry            : out std_logic;
        MMU_Pgd              : out std_logic_vector(63 downto 0);
        MMU_Tlb_Hits         : in  std_logic_vector(63 downto 0);
        MMU_Tlb_Misses       : in  std_logic_vector(63 downto 0);
		-- User ports ends
		-- Do not modify the ports beyond this line


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
end reconos_proc_control;

architecture procctrl of reconos_proc_control is

	-- Declare port attributes for the Vivado IP Packager
	ATTRIBUTE X_INTERFACE_INFO : STRING;
	ATTRIBUTE X_INTERFACE_PARAMETER : STRING;

	ATTRIBUTE X_INTERFACE_INFO of s00_axi_aclk: SIGNAL is "xilinx.com:signal:clock:1.0 s00_axi_aclk CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of s00_axi_aclk: SIGNAL is "ASSOCIATED_RESET <<generate for SLOTS>>PROC_Hwt_Rst_<<Id>>:<<end generate>>PROC_Sys_Rst:s00_axi_aresetn";

	ATTRIBUTE X_INTERFACE_INFO of PROC_Pgf_Int: SIGNAL is "xilinx.com:signal:interrupt:1.0 PROC_Pgf_Int INTERRUPT";
	ATTRIBUTE X_INTERFACE_PARAMETER of PROC_Pgf_Int: SIGNAL is "SENSITIVITY LEVEL_HIGH";

	ATTRIBUTE X_INTERFACE_INFO of PROC_Sys_Rst: SIGNAL is "xilinx.com:signal:reset:1.0 PROC_Sys_Rst RST";
	ATTRIBUTE X_INTERFACE_PARAMETER of PROC_Sys_Rst: SIGNAL is "POLARITY ACTIVE_HIGH";

	<<generate for SLOTS>>
	ATTRIBUTE X_INTERFACE_INFO of PROC_Hwt_Rst_<<Id>>: SIGNAL is "xilinx.com:signal:reset:1.0 PROC_Hwt_Rst_<<Id>> RST";
	ATTRIBUTE X_INTERFACE_PARAMETER of PROC_Hwt_Rst_<<Id>>: SIGNAL is "POLARITY ACTIVE_HIGH";
	<<end generate>>

    component ProcControlAXI_S00_AXI is
	generic (
            -- Users to add parameters here
            C_NUM_HWTS     : integer := 1;
            -- User parameters ends
            -- Do not modify the parameters beyond this line

            -- Width of S_AXI data bus
            C_S_AXI_DATA_WIDTH    : integer    := 32;
            -- Width of S_AXI address bus
            C_S_AXI_ADDR_WIDTH    : integer    := 6
        );
        port (
            PROC_Hwt_Rst     : out std_logic_vector(C_NUM_HWTS - 1 downto 0);
            PROC_Hwt_Signal  : out std_logic_vector(C_NUM_HWTS - 1 downto 0);
            PROC_Sys_Rst     : out std_logic;
            PROC_Pgf_Int     : out std_logic;

            MMU_Pgf          : in  std_logic;
            MMU_Fault_Addr   : in  std_logic_vector(63 downto 0);
            MMU_Retry        : out std_logic;
            MMU_Pgd          : out std_logic_vector(63 downto 0);
            MMU_Tlb_Hits     : in  std_logic_vector(63 downto 0);
            MMU_Tlb_Misses   : in  std_logic_vector(63 downto 0);

            S_AXI_ACLK       : in std_logic;
            S_AXI_ARESETN    : in std_logic;
            S_AXI_AWADDR     : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
            S_AXI_AWPROT     : in std_logic_vector(2 downto 0);
            S_AXI_AWVALID    : in std_logic;
            S_AXI_AWREADY    : out std_logic;
            S_AXI_WDATA      : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
            S_AXI_WSTRB      : in std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
            S_AXI_WVALID     : in std_logic;
            S_AXI_WREADY     : out std_logic;
            S_AXI_BRESP      : out std_logic_vector(1 downto 0);
            S_AXI_BVALID     : out std_logic;
            S_AXI_BREADY     : in std_logic;
            S_AXI_ARADDR     : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
            S_AXI_ARPROT     : in std_logic_vector(2 downto 0);
            S_AXI_ARVALID    : in std_logic;
            S_AXI_ARREADY    : out std_logic;
            S_AXI_RDATA      : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
            S_AXI_RRESP      : out std_logic_vector(1 downto 0);
            S_AXI_RVALID    : out std_logic;
            S_AXI_RREADY    : in std_logic
        );
    end component ProcControlAXI_S00_AXI;

    signal hwt_rst     : std_logic_vector(C_NUM_HWTS - 1 downto 0);
	signal hwt_sig     : std_logic_vector(C_NUM_HWTS - 1 downto 0);
	
begin

-- Instantiation of Axi Bus Interface S00_AXI
ProcControlAXI_v0_91_S00_AXI_inst : entity work.ProcControlAXI_S00_AXI
	generic map (
        C_NUM_HWTS          =>  C_NUM_HWTS,
		C_S_AXI_DATA_WIDTH	=> C_S00_AXI_DATA_WIDTH,
		C_S_AXI_ADDR_WIDTH	=> C_S00_AXI_ADDR_WIDTH
	)
	port map (

	    MMU_Pgf        => MMU_Pgf,
	    MMU_Fault_Addr => MMU_Fault_Addr,
	    MMU_Retry      => MMU_Retry,
	    MMU_Pgd        => MMU_Pgd,
	    MMU_Tlb_Hits   => MMU_Tlb_Hits,
	    MMU_Tlb_Misses => MMU_Tlb_Misses,

	    PROC_Sys_Rst    => PROC_Sys_Rst,

	    PROC_Hwt_Rst    => hwt_rst,
		PROC_Hwt_Signal => hwt_sig,
		PROC_Pgf_Int    => PROC_Pgf_Int,

		S_AXI_ACLK	    => s00_axi_aclk,
		S_AXI_ARESETN	=> s00_axi_aresetn,
		S_AXI_AWADDR	=> s00_axi_awaddr,
		S_AXI_AWPROT	=> s00_axi_awprot,
		S_AXI_AWVALID	=> s00_axi_awvalid,
		S_AXI_AWREADY	=> s00_axi_awready,
		S_AXI_WDATA	    => s00_axi_wdata,
		S_AXI_WSTRB	    => s00_axi_wstrb,
		S_AXI_WVALID	=> s00_axi_wvalid,
		S_AXI_WREADY	=> s00_axi_wready,
		S_AXI_BRESP	    => s00_axi_bresp,
		S_AXI_BVALID	=> s00_axi_bvalid,
		S_AXI_BREADY	=> s00_axi_bready,
		S_AXI_ARADDR	=> s00_axi_araddr,
		S_AXI_ARPROT	=> s00_axi_arprot,
		S_AXI_ARVALID	=> s00_axi_arvalid,
		S_AXI_ARREADY	=> s00_axi_arready,
		S_AXI_RDATA    	=> s00_axi_rdata,
		S_AXI_RRESP	    => s00_axi_rresp,
		S_AXI_RVALID	=> s00_axi_rvalid,
		S_AXI_RREADY	=> s00_axi_rready
	);

	-- Add user logic here
	<<generate for SLOTS>>
	PROC_Hwt_Rst_<<Id>> <= hwt_rst(<<_i>>);
	PROC_Hwt_Signal_<<Id>> <= hwt_sig(<<_i>>);
	<<end generate>>
	-- User logic ends

end procctrl;
