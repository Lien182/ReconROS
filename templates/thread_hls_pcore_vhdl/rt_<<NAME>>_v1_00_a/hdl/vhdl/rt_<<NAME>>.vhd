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
		HWT_Signal : in  std_logic;

		DEBUG : out std_logic_vector(135 downto 0);

		debug_port : out std_logic_vector(31 downto 0)
	);
<<if RECONFIGURABLE==True>>
	end entity rt_reconf;
	
	architecture implementation of rt_reconf is
<<end if>>
<<if RECONFIGURABLE==False>>
	end entity rt_<<NAME>>;
	
	architecture implementation of rt_<<NAME>> is
<<end if>>
	component rt_imp is
		port (
			ap_clk : in std_logic;
			ap_rst : in std_logic;

			osif_sw2hw_v_dout    : in std_logic_vector (31 downto 0);
			osif_sw2hw_v_empty_n : in std_logic;
			osif_sw2hw_v_read    : out std_logic;

			osif_hw2sw_v_din     : out std_logic_vector (31 downto 0);
			osif_hw2sw_v_full_n  : in std_logic;
			osif_hw2sw_v_write   : out std_logic;

			memif_hwt2mem_v_din     : out std_logic_vector (31 downto 0);
			memif_hwt2mem_v_full_n  : in std_logic;
			memif_hwt2mem_v_write   : out std_logic;

			memif_mem2hwt_v_dout    : in std_logic_vector (31 downto 0);
			memif_mem2hwt_v_empty_n : in std_logic;
			memif_mem2hwt_v_read    : out std_logic
		);
  	end component;

	signal osif_sw2hw_v_dout    : std_logic_vector(31 downto 0);
	signal osif_sw2hw_v_empty_n : std_logic;
	signal osif_sw2hw_v_read    : std_logic;

	signal osif_hw2sw_v_din     : std_logic_vector(31 downto 0);
	signal osif_hw2sw_v_full_n  : std_logic;
	signal osif_hw2sw_v_write   : std_logic;

	signal memif_hwt2mem_v_din     : std_logic_vector(31 downto 0);
	signal memif_hwt2mem_v_full_n  : std_logic;
	signal memif_hwt2mem_v_write   : std_logic;

	signal memif_mem2hwt_v_dout    : std_logic_vector(31 downto 0);
	signal memif_mem2hwt_v_empty_n : std_logic;
	signal memif_mem2hwt_v_read    : std_logic;
begin
	osif_sw2hw_v_dout    <= OSIF_Sw2Hw_Data;
	osif_sw2hw_v_empty_n <= not OSIF_Sw2Hw_Empty;
	OSIF_Sw2Hw_RE        <= osif_sw2hw_v_read;

	OSIF_Hw2Sw_Data      <= osif_hw2sw_v_din;
	osif_hw2sw_v_full_n  <= not OSIF_Hw2Sw_Full;
	OSIF_Hw2Sw_WE        <= osif_hw2sw_v_write;

	MEMIF_Hwt2Mem_Data      <= memif_hwt2mem_v_din;
	memif_hwt2mem_v_full_n  <= not MEMIF_Hwt2Mem_Full;
	MEMIF_Hwt2Mem_WE        <= memif_hwt2mem_v_write;

	memif_mem2hwt_v_dout    <= MEMIF_Mem2Hwt_Data;
	memif_mem2hwt_v_empty_n <= not MEMIF_Mem2Hwt_Empty;
	MEMIF_Mem2Hwt_RE        <= memif_mem2hwt_v_read;

	DEBUG(135 downto 104) <= osif_sw2hw_v_dout;
	DEBUG(103) <= not osif_sw2hw_v_empty_n;
	DEBUG(102) <= osif_sw2hw_v_read;
	DEBUG(101 downto 70) <= osif_hw2sw_v_din;
	DEBUG(69) <= not osif_hw2sw_v_full_n;
	DEBUG(68) <= osif_hw2sw_v_write;
	DEBUG(67 downto 36) <= memif_hwt2mem_v_din;
	DEBUG(35) <= not memif_hwt2mem_v_full_n;
	DEBUG(34) <= memif_hwt2mem_v_write;
	DEBUG(33 downto 2) <= memif_mem2hwt_v_dout;
	DEBUG(1) <= not memif_mem2hwt_v_empty_n;
	DEBUG(0) <= memif_mem2hwt_v_read;

	rt_imp_inst : rt_imp
		port map (
			ap_clk => HWT_Clk,
			ap_rst => HWT_Rst,

			osif_sw2hw_v_dout    => osif_sw2hw_v_dout,
			osif_sw2hw_v_empty_n => osif_sw2hw_v_empty_n,
			osif_sw2hw_v_read    => osif_sw2hw_v_read,

			osif_hw2sw_v_din     => osif_hw2sw_v_din,
			osif_hw2sw_v_full_n  => osif_hw2sw_v_full_n,
			osif_hw2sw_v_write   => osif_hw2sw_v_write,

			memif_hwt2mem_v_din     => memif_hwt2mem_v_din,
			memif_hwt2mem_v_full_n  => memif_hwt2mem_v_full_n,
			memif_hwt2mem_v_write   => memif_hwt2mem_v_write,

			memif_mem2hwt_v_dout    => memif_mem2hwt_v_dout,
			memif_mem2hwt_v_empty_n => memif_mem2hwt_v_empty_n,
			memif_mem2hwt_v_read    => memif_mem2hwt_v_read
	);	
end architecture;
