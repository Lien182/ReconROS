--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - MEMIF Arbiter
--
--   project:      ReconOS
--   author:       Christoph R??thing, University of Paderborn
--   description:  The arbiter connects the different HWTs
--                 to the memory system of ReconOS. It acts as an
--                 arbiter and controls the the memory access.
--
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

<<if TOOL=="ise">>
library reconos_v3_01_a;
use reconos_v3_01_a.reconos_pkg.all;
<<end if>>


<<if TOOL=="vivado">>
library reconos_v3_01_a;
use reconos_v3_01_a.reconos_pkg.all;
<<end if>>

entity reconos_memif_arbiter is
	--
	-- Generic definitions
	--
	--   C_NUM_HWTS - number of hardware threads
	--
	--   C_MEMIF_DATA_WIDTH - width of the memif
	--
	generic (
		C_NUM_HWTS : integer := 1;

		C_MEMIF_DATA_WIDTH : integer := 32
	);

	--
	-- Port defintions
	--
	--   MEMIF_Hwt2Mem_#i#_In_/MEMIF_Mem2Hwt_#i#_In_ - fifo signal inputs
	--
	--   MEMIF_Hwt2Mem_Out_/MEMIF_Mem2Hwt_Out_ - fifo signal outputs
	--   
	--   SYS_Clk - system clock
	--   SYS_Rst - system reset
	--
	port (
		<<generate for SLOTS>>
		MEMIF_Hwt2Mem_<<Id>>_In_Data  : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		MEMIF_Hwt2Mem_<<Id>>_In_Empty : in  std_logic;
		MEMIF_Hwt2Mem_<<Id>>_In_RE    : out std_logic;
		<<end generate>>

		<<generate for SLOTS>>
		MEMIF_Mem2Hwt_<<Id>>_In_Data  : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		MEMIF_Mem2Hwt_<<Id>>_In_Full  : in  std_logic;
		MEMIF_Mem2Hwt_<<Id>>_In_WE    : out std_logic;
		<<end generate>>

		MEMIF_Hwt2Mem_Out_Data  : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		MEMIF_Hwt2Mem_Out_Empty : out std_logic;
		MEMIF_Hwt2Mem_Out_RE    : in  std_logic;

		MEMIF_Mem2Hwt_Out_Data  : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		MEMIF_Mem2Hwt_Out_Full  : out std_logic;
		MEMIF_Mem2Hwt_Out_WE    : in  std_logic;

		SYS_Clk : in std_logic;
		SYS_Rst : in std_logic
	);
end entity reconos_memif_arbiter;


architecture imp of reconos_memif_arbiter is
	--
	-- Internal state machine
	--
	--   state_type - vhdl type of the states
	--   state      - instantiation of the state
	--
	type state_type is (STATE_WAIT,STATE_ARBITRATE,
	                    STATE_CMD,STATE_ADDR,STATE_PROCESS);
	signal state : state_type := STATE_WAIT;

	--
	-- Internal signals for round robin arbiter
	--
	--   req  - masked request vector
	--   msk  - mask to disable previous grants
	--   msb  - most significant bit of req
	--   grnt - grant vector to multiplex signals
	--   orr  - override for full and empty signals
	--
	signal req  : std_logic_vector(C_NUM_HWTS - 1 downto 0) := (others => '0');
	signal msk  : std_logic_vector(C_NUM_HWTS - 1 downto 0) := (others => '1');
	signal msb  : std_logic_vector(C_NUM_HWTS - 1 downto 0) := (others => '0');
	signal grnt : std_logic_vector(C_NUM_HWTS - 1 downto 0) := (others => '0');
	signal orr  : std_logic := '1';

	--
	-- Internal signals
	--
	--   mem_count - counter of transferred bytes
	--
	signal mem_count : unsigned(C_MEMIF_LENGTH_WIDTH - 1 downto 0) := (others => '0');

	--
	-- Signals used for usage of multiplexed signals
	--
	--   hwt2mem_/mem2hwt_ - multiplexed fifo signals
	--
	signal hwt2mem_data  : std_logic_vector(C_MEMIF_DATA_WIDTH -1 downto 0);
	signal hwt2mem_empty : std_logic;
	signal mem2hwt_full  : std_logic;
begin

	-- == Assignment of input signals =====================================

	<<generate for SLOTS>>
	req(<<_i>>) <= not MEMIF_Hwt2Mem_<<Id>>_In_Empty and msk(<<_i>>);
	<<end generate>>

	msb <= req and std_logic_vector(unsigned(not(req)) + 1);


	-- == Process definitions =============================================

	--
	-- Arbitrate fifos based on requests and snoop
	--
	--   A state machine to implement a round robin arbiter. The arbiter
	--   snoops on the fifos to figure out the end of a transaction.
	--
	arb : process(SYS_Clk,SYS_Rst) is
		variable i : integer range 1 to C_NUM_HWTS;
	begin
		if SYS_Rst = '1' then
			msk <= (others => '1');
			grnt <= (others => '0');

			state <= STATE_WAIT;
		elsif rising_edge(SYS_Clk) then
			case state is
				when STATE_WAIT =>
					if req = (req'Range => '0') then
						msk <= (others => '1');
					else
						state <= STATE_ARBITRATE;
					end if;

				when STATE_ARBITRATE =>
					grnt <= msb;
					msk <= msk and (not msb);

					state <= STATE_CMD;

				when STATE_CMD =>
					if MEMIF_Hwt2Mem_Out_RE = '1' and hwt2mem_empty = '0' then
						mem_count <= unsigned(hwt2mem_data(C_MEMIF_LENGTH_RANGE));

						state <= STATE_ADDR;
					end if;

				when STATE_ADDR =>
					if MEMIF_Hwt2Mem_Out_RE = '1' and hwt2mem_empty = '0' then
						state <= STATE_PROCESS;
					end if;

				when STATE_PROCESS =>
					if    (MEMIF_Hwt2Mem_Out_RE = '1' and hwt2mem_empty = '0')
					   or (MEMIF_Mem2Hwt_Out_WE = '1' and mem2hwt_full = '0') then
						mem_count <= mem_count - 4;

						if mem_count - 4 = 0 then
							state <= STATE_WAIT;

							grnt <= (others => '0');
						end if;
					end if;

				when others =>
			end case;
		end if;
	end process arb;


	-- == Multiplexing signals ============================================

	orr <= '1' when state = STATE_WAIT else
	       '1' when state = STATE_ARBITRATE else
	       '0';

	hwt2mem_data <=
	  <<generate for SLOTS>>
	  (MEMIF_Hwt2Mem_<<Id>>_In_Data and (MEMIF_Hwt2Mem_<<Id>>_In_Data'Range => grnt(<<_i>>))) or
	  <<end generate>>
	  (C_MEMIF_DATA_WIDTH - 1 downto 0 => '0');

	hwt2mem_empty <=
	  <<generate for SLOTS>>
	  (MEMIF_Hwt2Mem_<<Id>>_In_Empty and grnt(<<_i>>)) or
	  <<end generate>>
	  orr;

	mem2hwt_full <=
	  <<generate for SLOTS>>
	  (MEMIF_Mem2Hwt_<<Id>>_In_Full and grnt(<<_i>>)) or
	  <<end generate>>
	  orr;

	MEMIF_Hwt2Mem_Out_Data  <= hwt2mem_data;
	MEMIF_Hwt2Mem_Out_Empty <= hwt2mem_empty;
	MEMIF_Mem2Hwt_Out_Full  <=  mem2hwt_full;
	<<generate for SLOTS>>
	MEMIF_Hwt2Mem_<<Id>>_In_RE   <= MEMIF_Hwt2Mem_Out_RE and grnt(<<_i>>);
	MEMIF_Mem2Hwt_<<Id>>_In_Data <= MEMIF_Mem2Hwt_Out_Data;
	MEMIF_Mem2Hwt_<<Id>>_In_WE   <= MEMIF_Mem2Hwt_Out_WE and grnt(<<_i>>);
	<<end generate>>

end architecture imp;
