<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

<<if RECONFIGURABLE==True>>
entity rt_reconf is
<<end if>>
<<if RECONFIGURABLE==False>>
entity rt_<<NAME>> is
<<end if>>
	port (
		-- OSIF FIFO ports
		OSIF_Sw2Hw_Data    : in  std_logic_vector(63 downto 0);
		OSIF_Sw2Hw_Empty   : in  std_logic;
		OSIF_Sw2Hw_RE      : out std_logic;

		OSIF_Hw2Sw_Data    : out std_logic_vector(63 downto 0);
		OSIF_Hw2Sw_Full    : in  std_logic;
		OSIF_Hw2Sw_WE      : out std_logic;

		-- MEMIF FIFO ports
		MEMIF64_Hwt2Mem_Data    : out std_logic_vector(63 downto 0);
		MEMIF64_Hwt2Mem_Full    : in  std_logic;
		MEMIF64_Hwt2Mem_WE      : out std_logic;

		MEMIF64_Mem2Hwt_Data    : in  std_logic_vector(63 downto 0);
		MEMIF64_Mem2Hwt_Empty   : in  std_logic;
		MEMIF64_Mem2Hwt_RE      : out std_logic;

		HWT_Clk    : in  std_logic;
		HWT_Rst    : in  std_logic;
		HWT_Signal : in  std_logic;

<<generate for HWTOPICSSUB>>
		<<Name>>_TDATA : IN STD_LOGIC_VECTOR (31 downto 0);
		<<Name>>_TVALID : IN STD_LOGIC;
		<<Name>>_TREADY : OUT STD_LOGIC;
		<<Name>>_TKEEP : IN STD_LOGIC_VECTOR (3 downto 0);
		<<Name>>_TSTRB : IN STD_LOGIC_VECTOR (3 downto 0);
		<<Name>>_TUSER : IN STD_LOGIC_VECTOR (0 downto 0);
		<<Name>>_TLAST : IN STD_LOGIC_VECTOR (0 downto 0);
		<<Name>>_TID : IN STD_LOGIC_VECTOR (0 downto 0);
		<<Name>>_TDEST : IN STD_LOGIC_VECTOR (0 downto 0);
<<end generate>>

<<generate for HWTOPICSPUB>>
		<<Name>>_TDATA : OUT STD_LOGIC_VECTOR (31 downto 0);
		<<Name>>_TVALID : OUT STD_LOGIC;
		<<Name>>_TREADY : IN STD_LOGIC;
		<<Name>>_TKEEP : OUT STD_LOGIC_VECTOR (3 downto 0);
		<<Name>>_TSTRB : OUT STD_LOGIC_VECTOR (3 downto 0);
		<<Name>>_TUSER : OUT STD_LOGIC_VECTOR (0 downto 0);
		<<Name>>_TLAST : OUT STD_LOGIC_VECTOR (0 downto 0);
		<<Name>>_TID : 	OUT STD_LOGIC_VECTOR (0 downto 0);
		<<Name>>_TDEST : OUT STD_LOGIC_VECTOR (0 downto 0);
<<end generate>>

		DEBUG : out std_logic_vector(70 downto 0)
	);
<<if RECONFIGURABLE==True>>
end entity rt_reconf;

architecture implementation of rt_reconf is
<<end if>>
<<if RECONFIGURABLE==False>>
end entity rt_<<NAME>>;

architecture implementation of rt_<<NAME>> is
<<end if>>
	-- Declare port attributes for the Vivado IP Packager
	ATTRIBUTE X_CORE_INFO : STRING;
	ATTRIBUTE X_CORE_INFO OF implementation: ARCHITECTURE IS "rt_<<NAME>>,Vivado 2020.1";

	ATTRIBUTE X_INTERFACE_INFO : STRING;
	ATTRIBUTE X_INTERFACE_PARAMETER : STRING;

	ATTRIBUTE X_INTERFACE_INFO of HWT_Clk: SIGNAL is "xilinx.com:signal:clock:1.0 HWT_Clk CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of HWT_Clk: SIGNAL is "ASSOCIATED_RESET HWT_Rst, ASSOCIATED_BUSIF OSIF_Sw2Hw:OSIF_Hw2Sw:MEMIF64_Hwt2Mem:MEMIF64_Mem2Hwt<<generate for HWTOPICSSUB>>:<<Name>><<end generate>><<generate for HWTOPICSPUB>>:<<Name>><<end generate>>";

	ATTRIBUTE X_INTERFACE_INFO of HWT_Rst: SIGNAL is "xilinx.com:signal:reset:1.0 HWT_Rst RST";
	ATTRIBUTE X_INTERFACE_PARAMETER of HWT_Rst: SIGNAL is "POLARITY ACTIVE_HIGH";

	ATTRIBUTE X_INTERFACE_INFO of OSIF_Sw2Hw_Data:     SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 OSIF_Sw2Hw FIFO64_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Sw2Hw_Empty:    SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 OSIF_Sw2Hw FIFO64_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Sw2Hw_RE:       SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 OSIF_Sw2Hw FIFO64_S_RE";

	ATTRIBUTE X_INTERFACE_INFO of OSIF_Hw2Sw_Data:     SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 OSIF_Hw2Sw FIFO64_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Hw2Sw_Full:     SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 OSIF_Hw2Sw FIFO64_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Hw2Sw_WE:       SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 OSIF_Hw2Sw FIFO64_M_WE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_Data:  SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Hwt2Mem FIFO64_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_Full:  SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Hwt2Mem FIFO64_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_WE:    SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Hwt2Mem FIFO64_M_WE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_Data:  SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Mem2Hwt FIFO64_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_Empty: SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Mem2Hwt FIFO64_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_RE:    SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Mem2Hwt FIFO64_S_RE";

	<<generate for HWTOPICSSUB>>
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TDATA: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TDATA";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TVALID: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TVALID";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TREADY: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TREADY";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TKEEP: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TKEEP";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TSTRB: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TSTRB";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TUSER: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TUSER";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TLAST: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TLAST";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TID: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TID";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TDEST: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TDEST";
	<<end generate>>
	<<generate for HWTOPICSPUB>>
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TDATA: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TDATA";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TVALID: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TVALID";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TREADY: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TREADY";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TKEEP: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TKEEP";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TSTRB: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TSTRB";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TUSER: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TUSER";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TLAST: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TLAST";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TID: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TID";
	ATTRIBUTE X_INTERFACE_INFO OF <<Name>>_TDEST: SIGNAL IS "xilinx.com:interface:axis:1.0 <<Name>> TDEST";
	<<end generate>>

<<if VIVADO2021==False>>
	component rt_imp is
		port (

<<generate for HWTOPICSSUB>>
			<<Name>>_TDATA : IN STD_LOGIC_VECTOR (31 downto 0);
			<<Name>>_TVALID : IN STD_LOGIC;
			<<Name>>_TREADY : OUT STD_LOGIC;
			<<Name>>_TKEEP : IN STD_LOGIC_VECTOR (3 downto 0);
			<<Name>>_TSTRB : IN STD_LOGIC_VECTOR (3 downto 0);
			<<Name>>_TUSER : IN STD_LOGIC_VECTOR (0 downto 0);
			<<Name>>_TLAST : IN STD_LOGIC_VECTOR (0 downto 0);
			<<Name>>_TID : IN STD_LOGIC_VECTOR (0 downto 0);
			<<Name>>_TDEST : IN STD_LOGIC_VECTOR (0 downto 0);
<<end generate>>

<<generate for HWTOPICSPUB>>
			<<Name>>_TDATA : OUT STD_LOGIC_VECTOR (31 downto 0);
			<<Name>>_TVALID : OUT STD_LOGIC;
			<<Name>>_TREADY : IN STD_LOGIC;
			<<Name>>_TKEEP : OUT STD_LOGIC_VECTOR (3 downto 0);
			<<Name>>_TSTRB : OUT STD_LOGIC_VECTOR (3 downto 0);
			<<Name>>_TUSER : OUT STD_LOGIC_VECTOR (0 downto 0);
			<<Name>>_TLAST : OUT STD_LOGIC_VECTOR (0 downto 0);
			<<Name>>_TID : OUT STD_LOGIC_VECTOR (0 downto 0);
			<<Name>>_TDEST : OUT STD_LOGIC_VECTOR (0 downto 0);
<<end generate>>

			ap_clk : in std_logic;
			hwt_signal : IN STD_LOGIC;
			osif_sw2hw_v_dout    : in std_logic_vector (63 downto 0);
			osif_sw2hw_v_empty_n : in std_logic;
			osif_sw2hw_v_read    : out std_logic;

			osif_hw2sw_v_din     : out std_logic_vector (63 downto 0);
			osif_hw2sw_v_full_n  : in std_logic;
			osif_hw2sw_v_write   : out std_logic;

			memif_hwt2mem_v_din     : out std_logic_vector (63 downto 0);
			memif_hwt2mem_v_full_n  : in std_logic;
			memif_hwt2mem_v_write   : out std_logic;

			memif_mem2hwt_v_dout    : in std_logic_vector (63 downto 0);
			memif_mem2hwt_v_empty_n : in std_logic;
			memif_mem2hwt_v_read    : out std_logic;
		
			<<RTIMPRESETDECLARATION>>
			
		);
  	end component;
<<end if>>


<<if VIVADO2021==True>>
	component rt_imp is
		port (

<<generate for HWTOPICSSUB>>
			<<Name>>_TDATA : IN STD_LOGIC_VECTOR (31 downto 0);
			<<Name>>_TVALID : IN STD_LOGIC;
			<<Name>>_TREADY : OUT STD_LOGIC;
			<<Name>>_TKEEP : IN STD_LOGIC_VECTOR (3 downto 0);
			<<Name>>_TSTRB : IN STD_LOGIC_VECTOR (3 downto 0);
			<<Name>>_TUSER : IN STD_LOGIC_VECTOR (0 downto 0);
			<<Name>>_TLAST : IN STD_LOGIC_VECTOR (0 downto 0);
			<<Name>>_TID : IN STD_LOGIC_VECTOR (0 downto 0);
			<<Name>>_TDEST : IN STD_LOGIC_VECTOR (0 downto 0);
<<end generate>>

<<generate for HWTOPICSPUB>>
			<<Name>>_TDATA : OUT STD_LOGIC_VECTOR (31 downto 0);
			<<Name>>_TVALID : OUT STD_LOGIC;
			<<Name>>_TREADY : IN STD_LOGIC;
			<<Name>>_TKEEP : OUT STD_LOGIC_VECTOR (3 downto 0);
			<<Name>>_TSTRB : OUT STD_LOGIC_VECTOR (3 downto 0);
			<<Name>>_TUSER : OUT STD_LOGIC_VECTOR (0 downto 0);
			<<Name>>_TLAST : OUT STD_LOGIC_VECTOR (0 downto 0);
			<<Name>>_TID : OUT STD_LOGIC_VECTOR (0 downto 0);
			<<Name>>_TDEST : OUT STD_LOGIC_VECTOR (0 downto 0);
<<end generate>>

		    ap_local_block : OUT STD_LOGIC;
		    ap_local_deadlock : OUT STD_LOGIC;
		    ap_clk : IN STD_LOGIC;
		    ap_rst : IN STD_LOGIC;
		    hwt_signal : IN STD_LOGIC;
		    osif_sw2hw_dout : IN STD_LOGIC_VECTOR (63 downto 0);
		    osif_sw2hw_empty_n : IN STD_LOGIC;
		    osif_sw2hw_read : OUT STD_LOGIC;
		    osif_hw2sw_din : OUT STD_LOGIC_VECTOR (63 downto 0);
		    osif_hw2sw_full_n : IN STD_LOGIC;
		    osif_hw2sw_write : OUT STD_LOGIC;
		    memif_hwt2mem_din : OUT STD_LOGIC_VECTOR (63 downto 0);
		    memif_hwt2mem_full_n : IN STD_LOGIC;
		    memif_hwt2mem_write : OUT STD_LOGIC;
		    memif_mem2hwt_dout : IN STD_LOGIC_VECTOR (63 downto 0);
		    memif_mem2hwt_empty_n : IN STD_LOGIC;
		    memif_mem2hwt_read : OUT STD_LOGIC

		);
  	end component;
<<end if>>

	signal osif_sw2hw_v_dout    : std_logic_vector(63 downto 0);
	signal osif_sw2hw_v_empty_n : std_logic;
	signal osif_sw2hw_v_read    : std_logic;

	signal osif_hw2sw_v_din     : std_logic_vector(63 downto 0);
	signal osif_hw2sw_v_full_n  : std_logic;
	signal osif_hw2sw_v_write   : std_logic;

	signal memif_hwt2mem_v_din     : std_logic_vector(63 downto 0);
	signal memif_hwt2mem_v_full_n  : std_logic;
	signal memif_hwt2mem_v_write   : std_logic;

	signal memif_mem2hwt_v_dout    : std_logic_vector(63 downto 0);
	signal memif_mem2hwt_v_empty_n : std_logic;
	signal memif_mem2hwt_v_read    : std_logic;
	<<RTIMPRESETREMAPPINGSIGNAL>>
begin
	osif_sw2hw_v_dout    <= OSIF_Sw2Hw_Data;
	osif_sw2hw_v_empty_n <= not OSIF_Sw2Hw_Empty;
	OSIF_Sw2Hw_RE        <= osif_sw2hw_v_read;

	OSIF_Hw2Sw_Data      <= osif_hw2sw_v_din;
	osif_hw2sw_v_full_n  <= not OSIF_Hw2Sw_Full;
	OSIF_Hw2Sw_WE        <= osif_hw2sw_v_write;

	MEMIF64_Hwt2Mem_Data      <= memif_hwt2mem_v_din;
	memif_hwt2mem_v_full_n  <= not MEMIF64_Hwt2Mem_Full;
	MEMIF64_Hwt2Mem_WE        <= memif_hwt2mem_v_write;

	memif_mem2hwt_v_dout    <= MEMIF64_Mem2Hwt_Data;
	memif_mem2hwt_v_empty_n <= not MEMIF64_Mem2Hwt_Empty;
	MEMIF64_Mem2Hwt_RE        <= memif_mem2hwt_v_read;

	<<RTIMPRESETREMAPPING>>

	-- DEBUG(135 downto 104) <= osif_sw2hw_v_dout;
	-- DEBUG(103) <= not osif_sw2hw_v_empty_n;
	-- DEBUG(102) <= osif_sw2hw_v_read;
	-- DEBUG(101 downto 70) <= osif_hw2sw_v_din;
	-- DEBUG(69) <= not osif_hw2sw_v_full_n;
	-- DEBUG(68) <= osif_hw2sw_v_write;
	-- DEBUG(67 downto 36) <= memif_hwt2mem_v_din;
	-- DEBUG(35) <= not memif_hwt2mem_v_full_n;
	-- DEBUG(34) <= memif_hwt2mem_v_write;
	-- DEBUG(33 downto 2) <= memif_mem2hwt_v_dout;
	-- DEBUG(1) <= not memif_mem2hwt_v_empty_n;
	-- DEBUG(0) <= memif_mem2hwt_v_read;

<<if VIVADO2021==False>>
	rt_imp_inst : rt_imp
		port map (
			ap_clk => HWT_Clk,
			hwt_signal => HWT_Signal,

			osif_sw2hw_v_dout    => osif_sw2hw_v_dout,
			osif_sw2hw_v_empty_n => osif_sw2hw_v_empty_n,
			osif_sw2hw_v_read    => osif_sw2hw_v_read,

			osif_hw2sw_v_din     => osif_hw2sw_v_din,
			osif_hw2sw_v_full_n  => osif_hw2sw_v_full_n,
			osif_hw2sw_v_write   => osif_hw2sw_v_write,

<<generate for HWTOPICSSUB>>
			<<Name>>_TDATA => <<Name>>_TDATA, 
			<<Name>>_TVALID => <<Name>>_TVALID,
			<<Name>>_TREADY => <<Name>>_TREADY,
			<<Name>>_TKEEP => <<Name>>_TKEEP,
			<<Name>>_TSTRB => <<Name>>_TSTRB,
			<<Name>>_TUSER => <<Name>>_TUSER, 
			<<Name>>_TLAST => <<Name>>_TLAST, 
			<<Name>>_TID => <<Name>>_TID, 
			<<Name>>_TDEST => <<Name>>_TDEST,
<<end generate>>

<<generate for HWTOPICSPUB>>
			<<Name>>_TDATA => <<Name>>_TDATA, 
			<<Name>>_TVALID => <<Name>>_TVALID,
			<<Name>>_TREADY => <<Name>>_TREADY,
			<<Name>>_TKEEP => <<Name>>_TKEEP,
			<<Name>>_TSTRB => <<Name>>_TSTRB,
			<<Name>>_TUSER => <<Name>>_TUSER, 
			<<Name>>_TLAST => <<Name>>_TLAST, 
			<<Name>>_TID => <<Name>>_TID, 
			<<Name>>_TDEST => <<Name>>_TDEST,
<<end generate>>

			memif_hwt2mem_v_din     => memif_hwt2mem_v_din,
			memif_hwt2mem_v_full_n  => memif_hwt2mem_v_full_n,
			memif_hwt2mem_v_write   => memif_hwt2mem_v_write,

			memif_mem2hwt_v_dout    => memif_mem2hwt_v_dout,
			memif_mem2hwt_v_empty_n => memif_mem2hwt_v_empty_n,
			memif_mem2hwt_v_read    => memif_mem2hwt_v_read,

			<<RTIMPRESETMAPPING>>
	);
<<end if>>

<<if VIVADO2021==True>>
	rt_imp_inst : rt_imp
		port map (
			ap_clk => HWT_Clk,

			hwt_signal => HWT_Signal,
			osif_sw2hw_dout    => osif_sw2hw_v_dout,
			osif_sw2hw_empty_n => osif_sw2hw_v_empty_n,
			osif_sw2hw_read    => osif_sw2hw_v_read,

			osif_hw2sw_din     => osif_hw2sw_v_din,
			osif_hw2sw_full_n  => osif_hw2sw_v_full_n,
			osif_hw2sw_write   => osif_hw2sw_v_write,

<<generate for HWTOPICSSUB>>
			<<Name>>_TDATA => <<Name>>_TDATA, 
			<<Name>>_TVALID => <<Name>>_TVALID,
			<<Name>>_TREADY => <<Name>>_TREADY,
			<<Name>>_TKEEP => <<Name>>_TKEEP,
			<<Name>>_TSTRB => <<Name>>_TSTRB,
			<<Name>>_TUSER => <<Name>>_TUSER, 
			<<Name>>_TLAST => <<Name>>_TLAST, 
			<<Name>>_TID => <<Name>>_TID, 
			<<Name>>_TDEST => <<Name>>_TDEST,
<<end generate>>

<<generate for HWTOPICSPUB>>
			<<Name>>_TDATA => <<Name>>_TDATA, 
			<<Name>>_TVALID => <<Name>>_TVALID,
			<<Name>>_TREADY => <<Name>>_TREADY,
			<<Name>>_TKEEP => <<Name>>_TKEEP,
			<<Name>>_TSTRB => <<Name>>_TSTRB,
			<<Name>>_TUSER => <<Name>>_TUSER, 
			<<Name>>_TLAST => <<Name>>_TLAST, 
			<<Name>>_TID => <<Name>>_TID, 
			<<Name>>_TDEST => <<Name>>_TDEST,
<<end generate>>

			memif_hwt2mem_din     => memif_hwt2mem_v_din,
			memif_hwt2mem_full_n  => memif_hwt2mem_v_full_n,
			memif_hwt2mem_write   => memif_hwt2mem_v_write,

			memif_mem2hwt_dout    => memif_mem2hwt_v_dout,
			memif_mem2hwt_empty_n => memif_mem2hwt_v_empty_n,
			memif_mem2hwt_read    => memif_mem2hwt_v_read,

			<<RTIMPRESETMAPPING>>
	);
<<end if>>	
end architecture;
