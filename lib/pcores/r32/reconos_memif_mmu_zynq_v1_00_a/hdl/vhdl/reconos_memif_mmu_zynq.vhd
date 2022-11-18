--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - MEMIF MMU 
--
--   project:      ReconOS
--   author:       Christoph Rüthing, University of Paderborn
--   description:  The memory management unit enables virtual address
--                 support. Therefore, it performs page table walks,
--                 manages a TLB for faster translation and handles
--                 page fault via the proc control unit.
--
-- ======================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library reconos_v3_01_a;
use reconos_v3_01_a.reconos_pkg.all;

library reconos_memif_mmu_zynq_v1_00_a;
use reconos_memif_mmu_zynq_v1_00_a.tlb;

entity reconos_memif_mmu_zynq is
	--
	-- Generic definitions
	--
	--   C_TLB_SIZE - size of the tlb
	--
	--   C_MEMIF_DATA_WIDTH - width of the memif
	--
	generic (
		C_TLB_SIZE : integer := 128;

		C_MEMIF_DATA_WIDTH : integer := 32
	);

	--
	-- Port definitions
	--
	--   MEMIF_Hwt2Mem_In_/MEMIF_Mem2Hwt_In_ - fifo signal inputs
	--   MEMIF_Hwt2Mem_Out_/MEMIF_Mem2Hwt_Out_ - fifo signal outputs
	--
	--   MMU_Pgf        - interrupt output if page fault happend
	--   MMU_Fault_Addr - fault address of page fault
	--   MMU_Retry      - retry signal after page fault processed
	--   MMU_Pgd        - base address of l1 page table
	--
	--   SYS_Clk - system clock
	--   SYS_Rst - system reset
	--
	port (
		MEMIF_Hwt2Mem_In_Data  : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		MEMIF_Hwt2Mem_In_Empty : in  std_logic;
		MEMIF_Hwt2Mem_In_RE    : out std_logic;

		MEMIF_Mem2Hwt_In_Data  : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		MEMIF_Mem2Hwt_In_Full  : in  std_logic;
		MEMIF_Mem2Hwt_In_WE    : out std_logic;

		MEMIF_Hwt2Mem_Out_Data  : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		MEMIF_Hwt2Mem_Out_Empty : out std_logic;
		MEMIF_Hwt2Mem_Out_RE    : in  std_logic;

		MEMIF_Mem2Hwt_Out_Data  : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		MEMIF_Mem2Hwt_Out_Full  : out std_logic;
		MEMIF_Mem2Hwt_Out_WE    : in  std_logic;

		MMU_Pgf        : out std_logic;
		MMU_Fault_Addr : out std_logic_vector(31 downto 0);
		MMU_Retry      : in  std_logic;
		MMU_Pgd        : in  std_logic_vector(31 downto 0);

		SYS_Clk : in std_logic;
		SYS_Rst : in std_logic
	);
end entity reconos_memif_mmu_zynq;

architecture imp of reconos_memif_mmu_zynq is
	ATTRIBUTE X_INTERFACE_INFO : STRING;
	ATTRIBUTE X_INTERFACE_PARAMETER : STRING;

	ATTRIBUTE X_INTERFACE_INFO of SYS_Clk: SIGNAL is "xilinx.com:signal:clock:1.0 SYS_Clk CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of SYS_Clk: SIGNAL is "ASSOCIATED_RESET SYS_Rst, ASSOCIATED_BUSIF MEMIF_Hwt2Mem_In:MEMIF_Mem2Hwt_In:MEMIF_Mem2Hwt_Out:MEMIF_Hwt2Mem_Out";

	ATTRIBUTE X_INTERFACE_INFO of SYS_Rst: SIGNAL is "xilinx.com:signal:reset:1.0 SYS_Rst RST";
	ATTRIBUTE X_INTERFACE_PARAMETER of SYS_Rst: SIGNAL is "POLARITY ACTIVE_HIGH";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Hwt2Mem_In_Data:   SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 MEMIF_Hwt2Mem_In FIFO_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Hwt2Mem_In_Empty:  SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 MEMIF_Hwt2Mem_In FIFO_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Hwt2Mem_In_RE:     SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 MEMIF_Hwt2Mem_In FIFO_S_RE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Mem2Hwt_In_Data:   SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_Mem2Hwt_In FIFO_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Mem2Hwt_In_Full:   SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_Mem2Hwt_In FIFO_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Mem2Hwt_In_WE:     SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_Mem2Hwt_In FIFO_M_WE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Mem2Hwt_Out_Data:  SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_Mem2Hwt_Out FIFO_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Mem2Hwt_Out_Full:  SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_Mem2Hwt_Out FIFO_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Mem2Hwt_Out_WE:    SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_Mem2Hwt_Out FIFO_M_WE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Hwt2Mem_Out_Data:  SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 MEMIF_Hwt2Mem_Out FIFO_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Hwt2Mem_Out_Empty: SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 MEMIF_Hwt2Mem_Out FIFO_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_Hwt2Mem_Out_RE:    SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 MEMIF_Hwt2Mem_Out FIFO_S_RE";
	--
	-- Internal state machine
	--
	--   state_type - vhdl type of the states
	--   state      - instatntiation of the state
	--
	type state_type is (STATE_READ_CMD,STATE_READ_ADDR,
	                    STATE_WRITE_CMD,STATE_WRITE_ADDR,
	                    STATE_READ_L1_0,STATE_READ_L1_1,STATE_READ_L1_2,
	                    STATE_READ_L2_0,STATE_READ_L2_1,STATE_READ_L2_2,
	                    STATE_TLB_READ,STATE_TLB_WRITE,STATE_PAGE_FAULT,STATE_PROCESS);
	signal state : state_type := STATE_READ_CMD;

	--
	-- Internal signals
	--
	--   mem_cmd, mem_addr - received command and address from hwt
	--
	--   mem_count - counter of transferred bytes
	--
	signal mem_cmd, mem_addr : std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0) := (others => '0');

	signal mem_count : unsigned(C_MEMIF_LENGTH_WIDTH - 1 downto 0) := (others => '0');

	--
	-- Signals used for page table walk
	--
	--   l1_table_addr   - base address of l1 page table
	--   l1_descr_addr   - address of l1 page table entry
	--   l2_table_addr   - base address of l2 page table
	--   l2_descr_addr   - address of l2 page table entry
	--   small_page_addr - address of physical page
	--   physical_addr   - translated physical address
	--
	--   For detailed information of how the different addresses are
	--   calculated, take a look into the technical reference manual.
	--
	signal l1_table_addr   : std_logic_vector(31 downto 14);
	signal l1_descr_addr   : std_logic_vector(31 downto 0);
	signal l2_table_addr   : std_logic_vector(31 downto 10);
	signal l2_descr_addr   : std_logic_vector(31 downto 0);
	signal small_page_addr : std_logic_vector(31 downto 12);
	signal physical_addr   : std_logic_vector(31 downto 0);

	--
	-- Signals for the tlb
	--
	--   tlb_addr - data output of the physical page address
	--   tlb_we   - write enable for the tlb
	--   tlb_hit  - result of query
	--
	signal tlb_addr : std_logic_vector(19 downto 0);
	signal tlb_we   : std_logic;
	signal tlb_hit  : std_logic;

begin

	-- == Page table walk addresses========================================

	l1_table_addr <= MMU_Pgd(31 downto 14);
	l1_descr_addr <= l1_table_addr(31 downto 14) & mem_addr(31 downto 20) & "00";
	l2_descr_addr <= l2_table_addr(31 downto 10) & mem_addr(19 downto 12) & "00";
	physical_addr <= small_page_addr(31 downto 12) & mem_addr(11 downto 0);


	-- == Process definitions =============================================

	--
	-- Implements an mmu for the zynq platform
	--
	--   To translate a virtual address to a physical one, the mmu needs to
	--   perform a page table walk. See the signal description above.
	--
	mmu : process(SYS_Clk,SYS_Rst) is
	begin
		if SYS_Rst = '1' then
			state <= STATE_READ_CMD;
		elsif rising_edge(SYS_Clk) then
			case state is
				when STATE_READ_CMD =>
					if MEMIF_Hwt2Mem_In_Empty = '0' then
						mem_cmd <= MEMIF_Hwt2Mem_In_Data;
						mem_count <= unsigned(MEMIF_Hwt2Mem_In_Data(C_MEMIF_LENGTH_RANGE));

						state <= STATE_READ_ADDR;
					end if;

				when STATE_READ_ADDR =>
					if MEMIF_Hwt2Mem_In_Empty = '0' then
						mem_addr <= MEMIF_Hwt2Mem_In_Data;

						state <= STATE_TLB_READ;
					end if;

				when STATE_TLB_READ =>
					if tlb_hit = '1' then
						small_page_addr <= tlb_addr;

						state <= STATE_WRITE_CMD;
					else
						state <= STATE_READ_L1_0;
					end if;

				when STATE_READ_L1_0 =>
					if MEMIF_Hwt2Mem_Out_RE = '1' then
						state <= STATE_READ_L1_1;
					end if;

				when STATE_READ_L1_1 =>
					if MEMIF_Hwt2Mem_Out_RE = '1' then
						state <= STATE_READ_L1_2;
					end if;

				when STATE_READ_L1_2 =>
					if MEMIF_Mem2Hwt_Out_WE = '1' then
						l2_table_addr <= MEMIF_Mem2Hwt_Out_Data(31 downto 10);

						if MEMIF_Mem2Hwt_Out_Data(1 downto 0) = "00" then
							state <= STATE_PAGE_FAULT;
						else
							state <= STATE_READ_L2_0;
						end if;
					end if;

				when STATE_READ_L2_0 =>
					if MEMIF_Hwt2Mem_Out_RE = '1' then
						state <= STATE_READ_L2_1;
					end if;

				when STATE_READ_L2_1 =>
					if MEMIF_Hwt2Mem_Out_RE = '1' then
						state <= STATE_READ_L2_2;
					end if;

				when STATE_READ_L2_2 =>
					if MEMIF_Mem2Hwt_Out_WE = '1' then
						small_page_addr <= MEMIF_Mem2Hwt_Out_Data(31 downto 12);

						if MEMIF_Mem2Hwt_Out_Data(1 downto 0) = "00" then
							state <= STATE_PAGE_FAULT;
						else
							tlb_we <= '1';

							state <= STATE_TLB_WRITE;
						end if;
					end if;

				when STATE_TLB_WRITE =>
					tlb_we <= '0';

					state <= STATE_WRITE_CMD;
					
				when STATE_WRITE_CMD =>
					if MEMIF_Hwt2Mem_Out_RE = '1' then
						state <= STATE_WRITE_ADDR;
					end if;

				when STATE_WRITE_ADDR =>
					if MEMIF_Hwt2Mem_Out_RE = '1' then
						state <= STATE_PROCESS;
					end if;

				when STATE_PROCESS =>
					if    (MEMIF_Hwt2Mem_Out_RE = '1' and MEMIF_Hwt2Mem_In_Empty = '0')
					   or (MEMIF_Mem2Hwt_Out_WE = '1' and MEMIF_Mem2Hwt_In_Full = '0') then
						mem_count <= mem_count - 4;

						if mem_count - 4 = 0 then
							state <= STATE_READ_CMD;
						end if;
					end if;

				when STATE_PAGE_FAULT =>
					if MMU_Retry = '1' then
						state <= STATE_READ_L1_0;
					end if;

				when others =>
			end case;
		end if;
	end process mmu;


	-- == Multiplexing signals ============================================

	MEMIF_Hwt2Mem_Out_Data  <= MEMIF_Hwt2Mem_In_Data when state = STATE_PROCESS else
	                           x"00000004"           when state = STATE_READ_L1_0 else
	                           l1_descr_addr         when state = STATE_READ_L1_1 else
	                           x"00000004"           when state = STATE_READ_L2_0 else
	                           l2_descr_addr         when state = STATE_READ_L2_1 else
	                           mem_cmd               when state = STATE_WRITE_CMD else
	                           physical_addr         when state = STATE_WRITE_ADDR else
	                           x"00000000";

	MEMIF_Hwt2Mem_Out_Empty <= MEMIF_Hwt2Mem_In_Empty when state = STATE_PROCESS else
	                           '0'                    when state = STATE_READ_L1_0 else
	                           '0'                    when state = STATE_READ_L1_1 else
	                           '0'                    when state = STATE_READ_L2_0 else
	                           '0'                    when state = STATE_READ_L2_1 else
	                           '0'                    when state = STATE_WRITE_CMD else
	                           '0'                    when state = STATE_WRITE_ADDR else
	                           '1';

	MEMIF_Hwt2Mem_In_RE     <= MEMIF_Hwt2Mem_Out_RE when state = STATE_PROCESS else 
	                           '1'                  when state = STATE_READ_CMD else
	                           '1'                  when state = STATE_READ_ADDR else
	                           '0';

	MEMIF_Mem2Hwt_In_Data  <= MEMIF_Mem2Hwt_Out_Data;
	MEMIF_Mem2Hwt_Out_Full <= MEMIF_Mem2Hwt_In_Full when state = STATE_PROCESS else '0';
	MEMIF_Mem2Hwt_In_WE    <= MEMIF_Mem2Hwt_Out_WE when state = STATE_PROCESS else '0';


	-- == Assigning mmu ports =============================================

	MMU_Pgf <= '1' when state = STATE_PAGE_FAULT else '0';
	MMU_Fault_Addr <= mem_addr;


	-- == TLB =============================================================

	tlb_gen : if C_TLB_SIZE > 0 generate
		tlb : entity work.reconos_memif_mmu_zynq_tlb
			generic map (
				C_TLB_SIZE  => C_TLB_SIZE,
				C_TAG_SIZE  => 20,
				C_DATA_SIZE => 20 
			)
			port map (
				TLB_Tag => mem_addr(31 downto 12),
				TLB_DI  => small_page_addr(31 downto 12),
				TLB_DO  => tlb_addr,
				TLB_WE  => tlb_we,
				TLB_Hit => tlb_hit,
				TLB_Clk => SYS_Clk,
				TLB_Rst => SYS_Rst
			);
	end generate;

end architecture imp;
