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
--   author:       Christoph Rüthing, University of Paderborn
--   description:  A memory controller connecting the memory fifos with
--                 the axi bus of the system.
--
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library reconos_v3_01_a;
use reconos_v3_01_a.reconos_pkg.all;

entity user_logic is
	--
	-- Port definitions
	--
	--   MEMIF_Hwt2Mem_In_/MEMIF_Mem2Hwt_In_ - fifo signal inputs
	--
	--   BUS2IP_/IP2BUS_ - axi ipif signals
	--
	port (
		MEMIF_Hwt2Mem_In_Data  : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		MEMIF_Hwt2Mem_In_Empty : in  std_logic;
		MEMIF_Hwt2Mem_In_RE    : out std_logic;

		MEMIF_Mem2Hwt_In_Data  : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		MEMIF_Mem2Hwt_In_Full  : in  std_logic;
		MEMIF_Mem2Hwt_In_WE    : out std_logic;

		BUS2IP_Clk             : in  std_logic;
		BUS2IP_Resetn          : in  std_logic;
		IP2BUS_Mst_Addr        : out std_logic_vector(31 downto 0);
		IP2BUS_Mst_BE          : out std_logic_vector(3 downto 0);
		IP2BUS_Mst_Length      : out std_logic_vector(11 downto 0);
		IP2BUS_Mst_Type        : out std_logic;
		IP2BUS_Mst_Lock        : out std_logic;
		IP2BUS_Mst_Reset       : out std_logic;
		BUS2IP_Mst_CmdAck      : in  std_logic;
		BUS2IP_Mst_Cmplt       : in  std_logic;
		BUS2IP_Mst_Error       : in  std_logic;
		IP2BUS_MstRd_Req       : out std_logic;
		BUS2IP_MstRd_D         : in  std_logic_vector(31 downto 0);
		BUS2IP_MstRd_Rem       : in  std_logic_vector(3 downto 0);
		BUS2IP_MstRd_Sof_N     : in  std_logic;
		BUS2IP_MstRd_Eof_N     : in  std_logic;
		BUS2IP_MstRd_Src_Rdy_N : in  std_logic;
		BUS2IP_MstRd_Src_Dsc_N : in  std_logic;
		IP2BUS_MstRd_Dst_Rdy_N : out std_logic;
		IP2BUS_MstRd_Dst_Dsc_N : out std_logic;
		IP2BUS_MstWr_Req       : out std_logic;
		IP2BUS_MstWr_D         : out std_logic_vector(31 downto 0);
		IP2BUS_MstWr_Rem       : out std_logic_vector(3 downto 0);
		IP2BUS_MstWr_Sof_N     : out std_logic;
		IP2BUS_MstWr_Eof_N     : out std_logic;
		IP2BUS_MstWr_Src_Rdy_N : out std_logic;
		IP2BUS_MstWr_Src_Dsc_N : out std_logic;
		BUS2IP_MstWr_Dst_Rdy_N : in  std_logic;
		BUS2IP_MstWr_Dst_Dsc_N : in  std_logic
	);
end entity user_logic;

architecture imp of user_logic is
	--
	-- Internal state machine
	--
	--
	--   state_type - vhdl type of the states
	--   state      - instantiation of the state
	--
	type state_type is (STATE_READ_CMD,STATE_READ_ADDR,
	                    STATE_PROCESS_WRITE_0,STATE_PROCESS_WRITE_1,
	                    STATE_PROCESS_READ_0,STATE_PROCESS_READ_1,
	                    STATE_CMPLT);
	signal state : state_type := STATE_READ_CMD;

	--
	-- Internal signals
	--
	--   mem_addr - received address from hwt
	--
	--   mem_op     - operation to perform
	--   mem_length - number of bytes to transfer
	--   mem_count  - counter of transferred bytes
	--
	signal mem_addr : std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0) := (others => '0');

	signal mem_op     : std_logic_vector(C_MEMIF_OP_WIDTH - 1 downto 0) := (others => '0');
	signal mem_length : unsigned(C_MEMIF_LENGTH_WIDTH - 1 downto 0) := (others => '0');
	signal mem_count  : unsigned(C_MEMIF_LENGTH_WIDTH - 1 downto 0) := (others => '0');

	signal rd_req, wr_req : std_logic;

begin

	-- == Static assignments of signals ===================================

	IP2BUS_Mst_BE    <= (others => '1');
	IP2BUS_Mst_Type  <= '1';
	IP2BUS_Mst_Lock  <= '0';
	IP2BUS_Mst_Reset <= '0';
	
	IP2BUS_MstRd_Dst_Dsc_N <= '0';
	IP2BUS_MstWr_Src_Dsc_N <= '1';
	IP2BUS_MstWr_Rem       <= (others => '0');


	-- == Process definitions =============================================

	--
	-- Process handling reading from and writing to the memory.
	--
	--   Reads data from the memory and writes it into the memif fifos or
	--   writes data form the memif fifo the the memory.
	--
	rdwr : process(BUS2IP_Clk,BUS2IP_Resetn) is
	begin
		if BUS2IP_Resetn = '0' then
			state <= STATE_READ_CMD;
		elsif rising_edge(BUS2IP_Clk) then
			case state is
				when STATE_READ_CMD =>
					if MEMIF_Hwt2Mem_In_Empty = '0' then
						mem_op <= MEMIF_Hwt2Mem_In_Data(C_MEMIF_OP_RANGE);
						mem_length <= unsigned(MEMIF_Hwt2Mem_In_Data(C_MEMIF_LENGTH_RANGE));

						mem_count <= unsigned(MEMIF_Hwt2Mem_In_Data(C_MEMIF_LENGTH_RANGE));

						state <= STATE_READ_ADDR;
					end if;

				when STATE_READ_ADDR =>
					if MEMIF_Hwt2Mem_In_Empty = '0' then
						mem_addr <= MEMIF_Hwt2Mem_In_Data;

						case mem_op is
							when MEMIF_CMD_READ =>
								state <= STATE_PROCESS_READ_0;

							when MEMIF_CMD_WRITE =>
								state <= STATE_PROCESS_WRITE_0;

							when others =>
						end case;
					end if;

				when STATE_PROCESS_WRITE_0 =>
					if BUS2IP_Mst_CmdAck = '1' then
						state <= STATE_PROCESS_WRITE_1;
					end if;

				when STATE_PROCESS_WRITE_1 =>
					if BUS2IP_MstWr_Dst_Rdy_N = '0' and MEMIF_Hwt2Mem_In_Empty = '0' then
						mem_count <= mem_count - 4;

						if mem_count - 4 = 0 then
							state <= STATE_CMPLT;
						end if;
					end if;

				when STATE_PROCESS_READ_0 =>
					if BUS2IP_Mst_CmdAck = '1' then
						state <= STATE_PROCESS_READ_1;
					end if;

				when STATE_PROCESS_READ_1 =>
					if BUS2IP_MstRd_Src_Rdy_N = '0' and MEMIF_Mem2Hwt_In_Full = '0' then
						mem_count <= mem_count - 4;

						if mem_count - 4 = 0 then
							state <= STATE_CMPLT;
						end if;
					end if;

				when STATE_CMPLT =>
					if BUS2IP_Mst_Cmplt = '1' then
						state <= STATE_READ_CMD;
					end if;

				when others =>
			end case;
		end if;
	end process rdwr;


	-- == Multiplexing signals ============================================

	IP2Bus_Mst_Addr   <= mem_addr;
	IP2BUS_Mst_Length <= std_logic_vector(mem_length(11 downto 0));

	IP2BUS_MstRd_Req       <= '1' when state = STATE_PROCESS_READ_0 else '0';
	IP2BUS_MstRd_Dst_Rdy_N <= MEMIF_Mem2Hwt_In_Full when state = STATE_PROCESS_READ_1 else '1';

	IP2BUS_MstWr_D         <= MEMIF_Hwt2Mem_In_Data;
	IP2BUS_MstWr_Req       <= '1' when state = STATE_PROCESS_WRITE_0 else '0';
	IP2BUS_MstWr_Src_Rdy_N <= MEMIF_Hwt2Mem_In_Empty when state = STATE_PROCESS_WRITE_1 else '1';
	IP2BUS_MstWr_Sof_N     <= '0' when mem_count = mem_length else '1';
	IP2BUS_MstWr_Eof_N     <= '0' when mem_count - 4 = 0 else '1';

	MEMIF_Hwt2Mem_In_RE <= not BUS2IP_MstWr_Dst_Rdy_N when state = STATE_PROCESS_WRITE_1 else
	                       '1'                        when state = STATE_READ_CMD else
	                       '1'                        when state = STATE_READ_ADDR else
	                       '0';

	MEMIF_Mem2Hwt_In_Data <= BUS2IP_MstRd_D;
	MEMIF_Mem2Hwt_In_WE   <= not BUS2IP_MstRd_Src_Rdy_N when state = STATE_PROCESS_READ_1 else '0';

end architecture imp;