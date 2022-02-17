--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/64
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
----------------------------------------------------------------------------------
-- Company:  CEG UPB
-- Engineer: Christoph Rüthing
--           Lennart Clausing
-- 
-- Create Date: 06/12/2019 07:04:11 PM
-- Design Name: 
-- Module Name: reconos_memif_mmu_usp
-- Project Name: ReconOS64
-- Target Devices: Zynq UltraScale+
-- Tool Versions: 2018.2
-- Description:    The memory management unit enables virtual address
--                 support. Therefore, it performs page table walks,
--                 manages a TLB for faster translation and handles
--                 page fault via the proc control unit.
--
--                 This MMU does a translation starting at level 1
--                 with 39-bit VA addressing on a 4kB granule layout
--
--                 see pg. K7-7289 of Appx K7 of ARM DDI 0487D.b / ID042519
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Revision 0.02 - added auto padding of mem_cmd
-- Revision 0.03 - added early return of PT walk for block descriptors at l1/l2
-- Additional Comments:
-- 
----------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library reconos_v3_01_a;
use reconos_v3_01_a.reconos_pkg.all;

entity reconos_memif_mmu_usp is
	--
	-- Generic definitions
	--
	--   C_TLB_SIZE - size of the tlb
	--
	--   C_MEMIF_DATA_WIDTH - width of the memif
	--
	generic (
		C_TLB_SIZE : integer := 128
      --C_MEMIF_DATA_WIDTH : integer := 64    --no point in this, since the MEMIF64 has to have 64bit in order to transfer the whole address resolving anyway
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
		MEMIF64_Hwt2Mem_In_Data  : in  std_logic_vector(63 downto 0);
		MEMIF64_Hwt2Mem_In_Empty : in  std_logic;
		MEMIF64_Hwt2Mem_In_RE    : out std_logic;

		MEMIF64_Mem2Hwt_In_Data  : out std_logic_vector(63 downto 0);
		MEMIF64_Mem2Hwt_In_Full  : in  std_logic;
		MEMIF64_Mem2Hwt_In_WE    : out std_logic;

		MEMIF64_Hwt2Mem_Out_Data  : out std_logic_vector(63 downto 0);
		MEMIF64_Hwt2Mem_Out_Empty : out std_logic;
		MEMIF64_Hwt2Mem_Out_RE    : in  std_logic;

		MEMIF64_Mem2Hwt_Out_Data  : in  std_logic_vector(63 downto 0);
		MEMIF64_Mem2Hwt_Out_Full  : out std_logic;
		MEMIF64_Mem2Hwt_Out_WE    : in  std_logic;

		MMU_Pgf        : out std_logic;
		MMU_Fault_Addr : out std_logic_vector(63 downto 0);
		MMU_Retry      : in  std_logic;
		MMU_Pgd        : in  std_logic_vector(63 downto 0);

		SYS_Clk : in std_logic;
		SYS_Rst : in std_logic
	);
end entity reconos_memif_mmu_usp;


architecture addr39 of reconos_memif_mmu_usp is

	-- Declare port attributes for the Vivado IP Packager
	ATTRIBUTE X_INTERFACE_INFO : STRING;
	ATTRIBUTE X_INTERFACE_PARAMETER : STRING;

	ATTRIBUTE X_INTERFACE_INFO of SYS_Clk: SIGNAL is "xilinx.com:signal:clock:1.0 SYS_Clk CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of SYS_Clk: SIGNAL is "ASSOCIATED_RESET SYS_Rst, ASSOCIATED_BUSIF MEMIF64_Hwt2Mem_In:MEMIF64_Mem2Hwt_In:MEMIF64_Mem2Hwt_Out:MEMIF64_Hwt2Mem_Out";

	ATTRIBUTE X_INTERFACE_INFO of SYS_Rst: SIGNAL is "xilinx.com:signal:reset:1.0 SYS_Rst RST";
	ATTRIBUTE X_INTERFACE_PARAMETER of SYS_Rst: SIGNAL is "POLARITY ACTIVE_HIGH";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_In_Data:   SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Hwt2Mem_In FIFO64_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_In_Empty:  SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Hwt2Mem_In FIFO64_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_In_RE:     SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Hwt2Mem_In FIFO64_S_RE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_In_Data:   SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Mem2Hwt_In FIFO64_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_In_Full:   SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Mem2Hwt_In FIFO64_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_In_WE:     SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Mem2Hwt_In FIFO64_M_WE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_Out_Data:  SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Mem2Hwt_Out FIFO64_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_Out_Full:  SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Mem2Hwt_Out FIFO64_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_Out_WE:    SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Mem2Hwt_Out FIFO64_M_WE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_Out_Data:  SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Hwt2Mem_Out FIFO64_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_Out_Empty: SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Hwt2Mem_Out FIFO64_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_Out_RE:    SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Hwt2Mem_Out FIFO64_S_RE";

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
	                    STATE_READ_L3_0,STATE_READ_L3_1,STATE_READ_L3_2,
	                    STATE_TLB_READ,STATE_TLB_WRITE,
	                    STATE_PAGE_FAULT,
	                    STATE_PROCESS);
	                    
	signal state : state_type := STATE_READ_CMD;

	--
	-- Internal signals
	--
	--   mem_cmd, mem_addr - received command and address from hwt
	--
	--   mem_count - counter of transferred BYTES
	--
	signal mem_cmd, mem_addr : std_logic_vector(63 downto 0) := (others => '0');

	signal mem_count : unsigned(C_MEMIF_LENGTH_WIDTH - 1 downto 0) := (others => '0');

	--
	-- Signals used for page table walk
	--
	--   pgd_base_addr   - page global directory from linux
	--                     Equals first table, so no TTBR is queried
	-- 
    --   l1_descr_addr   - address of l1 page table entry
	--   l1_table_result - base address of l1 page table
    --
    --   l2_descr_addr   - address of l2 page table entry
	--   l2_table_result - base address of l2 page table
	--
    --   l3_descr_addr   - address of l3 page table entry
	--   l3_table_result - base address of l3 page table
	--   
	--   block_descr     - returned block address
    --                      typically from l3 lookup, but ARM64 also
    --                      allows for earlier stages to return block descriptors
    --
	--   physical_addr   - translated physical address
	--
	--   For detailed information of how the different addresses are
	--   calculated, take a look into the technical reference manual.
	--
	signal pgd_base_addr   : std_logic_vector(47 downto 12) := (others => '0');
    
    signal l1_descr_addr   : std_logic_vector(47 downto 0) := (others => '0');
	signal l1_table_result : std_logic_vector(63 downto 0) := (others => '0');

	signal l2_descr_addr   : std_logic_vector(47 downto 0) := (others => '0');
	signal l2_table_result : std_logic_vector(63 downto 0) := (others => '0');
	
    signal l3_descr_addr   : std_logic_vector(47 downto 0) := (others => '0');
-- 	signal l3_table_result : std_logic_vector(63 downto 0) := (others => '0');
	
	signal block_descr     : std_logic_vector(47 downto 12) := (others => '0');

	signal physical_addr   : std_logic_vector(47 downto 0) := (others => '0');

	--
	-- Signals for the tlb
	--
	--   tlb_addr - data output of the physical page address
	--   tlb_we   - write enable for the tlb
	--   tlb_hit  - result of query
	--
	signal tlb_addr : std_logic_vector(35 downto 0) := (others => '0');
	-- (35 downto 0) to store the 36 bits of output address(47:12)
	signal tlb_we   : std_logic := '0';
	signal tlb_hit  : std_logic := '0';

begin

	-- == Page table walk addresses========================================

	pgd_base_addr(47 downto 12) <= MMU_Pgd(47 downto 12);
	
	l1_descr_addr(47 downto 0) <= pgd_base_addr(47 downto 12) & mem_addr(38 downto 30) & "000";
	--now do l1 lookup
	
	l2_descr_addr(47 downto 0) <= l1_table_result(47 downto 12) & mem_addr(29 downto 21) & "000";
	--now do l2 lookup
	
	l3_descr_addr(47 downto 0) <= l2_table_result(47 downto 12) & mem_addr(20 downto 12) & "000";
	--now do l3 lookup
	
	physical_addr <= block_descr(47 downto 12) & mem_addr(11 downto 0);


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
					if MEMIF64_Hwt2Mem_In_Empty = '0' then
						mem_cmd <= MEMIF64_Hwt2Mem_In_Data;
						mem_count <= unsigned(MEMIF64_Hwt2Mem_In_Data(C_MEMIF_LENGTH_RANGE));

						state <= STATE_READ_ADDR;
					end if;

				when STATE_READ_ADDR =>
					if MEMIF64_Hwt2Mem_In_Empty = '0' then
						mem_addr <= MEMIF64_Hwt2Mem_In_Data;

						state <= STATE_TLB_READ;
					end if;

				when STATE_TLB_READ =>
					if tlb_hit = '1' then
                        block_descr(47 downto 12) <= tlb_addr(35 downto 0);
                        --l3_table_result <= "0000000000000000" & tlb_addr(35 downto 0) & "000000000000";
                        --l3_table_result(47 downto 12) <= tlb_addr(35 downto 0);

						state <= STATE_WRITE_CMD;
					else
						state <= STATE_READ_L1_0;
					end if;

				when STATE_READ_L1_0 =>
					if MEMIF64_Hwt2Mem_Out_RE = '1' then
						state <= STATE_READ_L1_1;
					end if;

				when STATE_READ_L1_1 =>
					if MEMIF64_Hwt2Mem_Out_RE = '1' then
						state <= STATE_READ_L1_2;
					end if;

				when STATE_READ_L1_2 =>
					if MEMIF64_Mem2Hwt_Out_WE = '1' then
						l1_table_result <= MEMIF64_Mem2Hwt_Out_Data(63 downto 0);
                        -- Bit [0]: 0=invalid / 1=valid
                        -- Bit [1]: 0=Block descriptor / 1=table descriptor
                        -- 00->invalid, 10->invalid, 01->valid block, 11->valid table
                        if MEMIF64_Mem2Hwt_Out_Data(0) /= '1' then
                            state <= STATE_PAGE_FAULT;
                        else
                            if MEMIF64_Mem2Hwt_Out_Data(1) = '0' then
                                block_descr(47 downto 12) <= MEMIF64_Mem2Hwt_Out_Data(47 downto 30) & mem_addr(29 downto 12);
                                tlb_we <= '1';
                                state <= STATE_TLB_WRITE;
                            else
                                state <= STATE_READ_L2_0;
                            end if;
                        end if;
					end if;

                when STATE_READ_L2_0 =>
                    if MEMIF64_Hwt2Mem_Out_RE = '1' then
                        state <= STATE_READ_L2_1;
                    end if;

                when STATE_READ_L2_1 =>
                    if MEMIF64_Hwt2Mem_Out_RE = '1' then
                        state <= STATE_READ_L2_2;
                    end if;

                when STATE_READ_L2_2 =>
                    if MEMIF64_Mem2Hwt_Out_WE = '1' then
                        l2_table_result <= MEMIF64_Mem2Hwt_Out_Data(63 downto 0);
                        -- Bit [0]: 0=invalid / 1=valid
                        -- Bit [1]: 0=Block descriptor / 1=table descriptor
                        -- 00->invalid, 10->invalid, 01->valid block, 11->valid table
                        if MEMIF64_Mem2Hwt_Out_Data(0) /= '1' then
                            state <= STATE_PAGE_FAULT;
                        else
                            if MEMIF64_Mem2Hwt_Out_Data(1) = '0' then
                                block_descr(47 downto 12) <= MEMIF64_Mem2Hwt_Out_Data(47 downto 21) & mem_addr(20 downto 12);
                                tlb_we <= '1';
                                state <= STATE_TLB_WRITE;
                            else
                                state <= STATE_READ_L3_0;
                            end if;
                        end if;
                    end if;

				when STATE_READ_L3_0 =>
					if MEMIF64_Hwt2Mem_Out_RE = '1' then
						state <= STATE_READ_L3_1;
					end if;

				when STATE_READ_L3_1 =>
					if MEMIF64_Hwt2Mem_Out_RE = '1' then
						state <= STATE_READ_L3_2;
					end if;

				when STATE_READ_L3_2 =>
					if MEMIF64_Mem2Hwt_Out_WE = '1' then
-- 						l3_table_result <= MEMIF64_Mem2Hwt_Out_Data(63 downto 0);
						if MEMIF64_Mem2Hwt_Out_Data(0) /= '1' then
                            state <= STATE_PAGE_FAULT;
                        else
                            block_descr(47 downto 12) <= MEMIF64_Mem2Hwt_Out_Data(47 downto 12);
                            tlb_we <= '1';
                            state <= STATE_TLB_WRITE;
                        end if;
					end if;

				when STATE_TLB_WRITE =>
					tlb_we <= '0';
					state <= STATE_WRITE_CMD;
					
				when STATE_WRITE_CMD =>
					if MEMIF64_Hwt2Mem_Out_RE = '1' then
						state <= STATE_WRITE_ADDR;
					end if;

				when STATE_WRITE_ADDR =>
					if MEMIF64_Hwt2Mem_Out_RE = '1' then
						state <= STATE_PROCESS;
					end if;

				when STATE_PROCESS =>
					if    (MEMIF64_Hwt2Mem_Out_RE = '1' and MEMIF64_Hwt2Mem_In_Empty = '0')
					   or (MEMIF64_Mem2Hwt_Out_WE = '1' and MEMIF64_Mem2Hwt_In_Full = '0') then
						mem_count <= mem_count - 8;

						if mem_count - 8 = 0 then
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
	                               
    MEMIF64_Hwt2Mem_Out_Data  <= MEMIF64_Hwt2Mem_In_Data  when state = STATE_PROCESS else
                                  x"0000000000000008"     when state = STATE_READ_L1_0 else
                                  x"0000" & l1_descr_addr when state = STATE_READ_L1_1 else
                                  x"0000000000000008"     when state = STATE_READ_L2_0 else
                                  x"0000" & l2_descr_addr when state = STATE_READ_L2_1 else
                                  x"0000000000000008"     when state = STATE_READ_L3_0 else
                                  x"0000" & l3_descr_addr when state = STATE_READ_L3_1 else
                                  mem_cmd                 when state = STATE_WRITE_CMD else
                                  x"0000" & physical_addr when state = STATE_WRITE_ADDR else
                                  x"0000000000000000";

	MEMIF64_Hwt2Mem_Out_Empty <= MEMIF64_Hwt2Mem_In_Empty when state = STATE_PROCESS else
	                               '0'                    when state = STATE_READ_L1_0 else
	                               '0'                    when state = STATE_READ_L1_1 else
	                               '0'                    when state = STATE_READ_L2_0 else
	                               '0'                    when state = STATE_READ_L2_1 else
	                               '0'                    when state = STATE_READ_L3_0 else
                                   '0'                    when state = STATE_READ_L3_1 else
	                               '0'                    when state = STATE_WRITE_CMD else
	                               '0'                    when state = STATE_WRITE_ADDR else
	                               '1';

	MEMIF64_Hwt2Mem_In_RE     <= MEMIF64_Hwt2Mem_Out_RE when state = STATE_PROCESS else 
	                               '1'                  when state = STATE_READ_CMD else
	                               '1'                  when state = STATE_READ_ADDR else
	                               '0';

	MEMIF64_Mem2Hwt_In_Data  <= MEMIF64_Mem2Hwt_Out_Data;
	
	MEMIF64_Mem2Hwt_Out_Full <= MEMIF64_Mem2Hwt_In_Full when state = STATE_PROCESS 
	                                                     else '0';
	                                                
	MEMIF64_Mem2Hwt_In_WE    <= MEMIF64_Mem2Hwt_Out_WE when state = STATE_PROCESS 
	                                                    else '0';


	-- == Assigning mmu ports =============================================

	MMU_Pgf <= '1' when state = STATE_PAGE_FAULT 
	                else '0';
	                
	MMU_Fault_Addr <= mem_addr;


	-- == TLB =============================================================

	tlb_gen : if C_TLB_SIZE > 0 generate
		tlb : entity work.reconos_memif_mmu_usp_tlb
			generic map (
				C_TLB_SIZE  => C_TLB_SIZE,
				C_TAG_SIZE  => 27,
				C_DATA_SIZE => 36
			)
			port map (
				TLB_Tag => mem_addr(38 downto 12),
				TLB_DI  => block_descr(47 downto 12),
				TLB_DO  => tlb_addr,
				TLB_WE  => tlb_we,
				TLB_Hit => tlb_hit,
				TLB_Clk => SYS_Clk,
				TLB_Rst => SYS_Rst
			);
	end generate;

end architecture addr39;
