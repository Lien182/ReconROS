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
--                 support. Therefore it performs page table walks,
--                 manages a TLB for faster translation and handles
--                 page fault via the proc control unit. 
--
-- ======================================================================


library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_misc.all;


entity reconos_memif_mmu_microblaze is
	generic (
		C_CTRL_FIFO_WIDTH    : integer := 32;
		
		C_MEMIF_LENGTH_WIDTH : integer := 24;
		
		C_TLB_SIZE           : integer := 128
	);
	port (
		-- Input FIFO ports from the HWTs (via burst converter and transaction control)
		CTRL_FIFO_In_Data    : in  std_logic_vector(C_CTRL_FIFO_WIDTH - 1 downto 0);
		CTRL_FIFO_In_Fill    : in  std_logic_vector(15 downto 0);
		CTRL_FIFO_In_Empty   : in  std_logic;
		CTRL_FIFO_In_RE      : out std_logic;
		
		-- Output FIFO ports to memory controller
		CTRL_FIFO_Out_Data    : out std_logic_vector(C_CTRL_FIFO_WIDTH - 1 downto 0);
		CTRL_FIFO_Out_Fill    : out std_logic_vector(15 downto 0);
		CTRL_FIFO_Out_Empty   : out std_logic;
		CTRL_FIFO_Out_RE      : in  std_logic;

		-- Seperate control and data FIFOs (emulated) for page table walks
		CTRL_FIFO_Mmu_Data    : out std_logic_vector(C_CTRL_FIFO_WIDTH - 1 downto 0);
		CTRL_FIFO_Mmu_Fill    : out std_logic_vector(15 downto 0);
		CTRL_FIFO_Mmu_Empty   : out std_logic;
		CTRL_FIFO_Mmu_RE      : in  std_logic;

		MEMIF_FIFO_Mmu_Data    : in  std_logic_vector(C_CTRL_FIFO_WIDTH - 1 downto 0);
		MEMIF_FIFO_Mmu_Rem     : out std_logic_vector(15 downto 0);
		MEMIF_FIFO_Mmu_Full    : out std_logic;
		MEMIF_FIFO_Mmu_WE      : in  std_logic;		
 
		-- MMU ports
		MMU_Pgf         : out std_logic;
		MMU_Fault_addr  : out std_logic_vector(31 downto 0);
		MMU_Retry       : in  std_logic;
		MMU_Pgd         : in  std_logic_vector(31 downto 0);
		MMU_Tlb_Hits    : out std_logic_vector(31 downto 0);
		MMU_Tlb_Misses  : out std_logic_vector(31 downto 0);
		
		MMU_Clk : in std_logic;
		MMU_Rst : in std_logic;

		DEBUG_DATA : out std_logic_vector(203 downto 0)
	);
end entity reconos_memif_mmu_microblaze;

architecture implementation of reconos_memif_mmu_microblaze is

	-- Declare port attributes for the Vivado IP Packager
	ATTRIBUTE X_INTERFACE_INFO : STRING;
	ATTRIBUTE X_INTERFACE_PARAMETER : STRING;

	ATTRIBUTE X_INTERFACE_INFO of MMU_Clk: SIGNAL is "xilinx.com:signal:clock:1.0 MMU_Clk CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of MMU_Clk: SIGNAL is "ASSOCIATED_RESET MMU_Rst, ASSOCIATED_BUSIF CTRL_FIFO_In:CTRL_FIFO_Out:CTRL_FIFO_Mmu:MEMIF_FIFO_Mmu";

	ATTRIBUTE X_INTERFACE_INFO of MMU_Rst: SIGNAL is "xilinx.com:signal:reset:1.0 MMU_Rst RST";
	ATTRIBUTE X_INTERFACE_PARAMETER of MMU_Rst: SIGNAL is "POLARITY ACTIVE_HIGH";

	ATTRIBUTE X_INTERFACE_INFO of CTRL_FIFO_In_Data:   SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 CTRL_FIFO_In FIFO_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of CTRL_FIFO_In_Empty:  SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 CTRL_FIFO_In FIFO_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of CTRL_FIFO_In_RE:     SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 CTRL_FIFO_In FIFO_S_RE";

	ATTRIBUTE X_INTERFACE_INFO of CTRL_FIFO_Out_Data:  SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 CTRL_FIFO_Out FIFO_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of CTRL_FIFO_Out_Empty: SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 CTRL_FIFO_Out FIFO_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of CTRL_FIFO_Out_RE:    SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 CTRL_FIFO_Out FIFO_S_RE";

	ATTRIBUTE X_INTERFACE_INFO of CTRL_FIFO_Mmu_Data:  SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 CTRL_FIFO_Mmu FIFO_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of CTRL_FIFO_Mmu_Empty: SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 CTRL_FIFO_Mmu FIFO_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of CTRL_FIFO_Mmu_RE:    SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 CTRL_FIFO_Mmu FIFO_S_RE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF_FIFO_Mmu_Data: SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_FIFO_Mmu FIFO_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_FIFO_Mmu_Full: SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_FIFO_Mmu FIFO_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF_FIFO_Mmu_WE:   SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 MEMIF_FIFO_Mmu FIFO_M_WE";

	constant C_MEMIF_CMD_WIDTH : integer := C_CTRL_FIFO_WIDTH - C_MEMIF_LENGTH_WIDTH;

	signal ctrl_in_re      : std_logic;

	signal ctrl_out_data    : std_logic_vector(C_CTRL_FIFO_WIDTH - 1 downto 0);
	signal ctrl_out_fill    : std_logic_vector(15 downto 0);
	signal ctrl_out_empty   : std_logic;
	
	signal ctrl_mmu_data    : std_logic_vector(C_CTRL_FIFO_WIDTH - 1 downto 0);
	signal ctrl_mmu_fill    : std_logic_vector(15 downto 0);
	signal ctrl_mmu_empty   : std_logic;

	-- MMU signals
	type STATE_TYPE is (WAIT_REQUEST, READ_CMD, READ_ADDR,
	                    READ_L1_ENTRY_0, READ_L1_ENTRY_1, READ_L1_ENTRY_2,
	                    READ_L2_ENTRY_0, READ_L2_ENTRY_1, READ_L2_ENTRY_2,
	                    WRITE_CMD, WRITE_ADDR, PAGE_FAULT);
	signal state : STATE_TYPE;
	
	signal pgf        : std_logic;
	signal tlb_hits   : std_logic_vector(31 downto 0);
	signal tlb_misses : std_logic_vector(31 downto 0);
	
	-- these signals contain the received request data unchanged
	signal ctrl_cmd      : std_logic_vector(C_MEMIF_CMD_WIDTH - 1 downto 0);
	signal ctrl_length   : std_logic_vector(C_MEMIF_LENGTH_WIDTH - 1 downto 0);
	signal ctrl_addr     : std_logic_vector(C_CTRL_FIFO_WIDTH - 1 downto 0);

	signal l1_table_addr       : std_logic_vector(31 downto 0); -- address of the level 1 page table
	signal l1_descriptor_addr  : std_logic_vector(31 downto 0); -- address of the level 1 page table entry
	signal l2_table_addr       : std_logic_vector(31 downto 0); -- address of the level 2 page table
	signal l2_descriptor_addr  : std_logic_vector(31 downto 0); -- address of the level 2 page table entry
	signal small_page_addr     : std_logic_vector(31 downto 0); -- page table entry
	signal physical_addr       : std_logic_vector(31 downto 0); -- physical address
	
	signal tlb_hit  : std_logic;
	signal tlb_tag  : std_logic_vector(19 downto 0);
	signal tlb_do   : std_logic_vector(19 downto 0);
	signal tlb_di   : std_logic_vector(19 downto 0);
	signal tlb_we   : std_logic;

	signal clk : std_logic;
	signal rst : std_logic;

begin
	DEBUG_DATA(0) <= '1' when state = WAIT_REQUEST else '0';
	DEBUG_DATA(1) <= '1' when state = READ_CMD else '0';
	DEBUG_DATA(2) <= '1' when state = READ_ADDR else '0';
	DEBUG_DATA(3) <= '1' when state = READ_L1_ENTRY_0 else '0';
	DEBUG_DATA(4) <= '1' when state = READ_L1_ENTRY_1 else '0';
	DEBUG_DATA(5) <= '1' when state = READ_L1_ENTRY_2 else '0';
	DEBUG_DATA(6) <= '1' when state = READ_L2_ENTRY_0 else '0';
	DEBUG_DATA(7) <= '1' when state = READ_L2_ENTRY_1 else '0';
	DEBUG_DATA(8) <= '1' when state = READ_L2_ENTRY_2 else '0';
	DEBUG_DATA(9) <= '1' when state = WRITE_CMD else '0';
	DEBUG_DATA(10) <= '1' when state = WRITE_ADDR else '0';
	DEBUG_DATA(11) <= '1' when state = PAGE_FAULT else '0';
	DEBUG_DATA(203 downto 172) <= l1_table_addr;
	DEBUG_DATA(171 downto 140) <= l1_descriptor_addr;
	DEBUG_DATA(139 downto 108) <= l2_table_addr;
	DEBUG_DATA(107 downto 76) <= l2_descriptor_addr;
	DEBUG_DATA(75 downto 44) <= small_page_addr;
	DEBUG_DATA(43 downto 12) <= physical_addr;

	clk <= MMU_Clk;
	rst <= MMU_Rst;

	CTRL_FIFO_In_RE <= ctrl_in_re;

	CTRL_FIFO_Out_Data  <= ctrl_out_data;
	CTRL_FIFO_Out_Fill  <= ctrl_out_fill;
	CTRL_FIFO_Out_Empty <= ctrl_out_empty;

	CTRL_FIFO_Mmu_Data  <= ctrl_mmu_data;
	CTRL_FIFO_Mmu_Fill  <= ctrl_mmu_fill;
	CTRL_FIFO_Mmu_Empty <= ctrl_mmu_empty;

	MEMIF_FIFO_Mmu_Rem   <= X"1111";
	MEMIF_FIFO_Mmu_Full  <= '0';

	MMU_Pgf         <= pgf;
	MMU_Fault_Addr  <= ctrl_addr;
	MMU_Tlb_Hits    <= tlb_hits;
	MMU_Tlb_Misses  <= tlb_misses;


	-- some address calculations based on the page table architecture
	-- for detailed information look into the TRM on page 80
	l1_table_addr       <= MMU_Pgd;
	l1_descriptor_addr  <= "00" & l1_table_addr(29 downto 12) & ctrl_addr(31 downto 22) & "00";
	l2_descriptor_addr  <= "00" & l2_table_addr(29 downto 12) & ctrl_addr(21 downto 12) & "00";
	physical_addr       <= small_page_addr(31 downto 12) & ctrl_addr(11 downto 0);


	mmu_proc : process(clk,rst) is
	begin
		if rst = '1' then
			state <= WAIT_REQUEST;

			ctrl_cmd       <= (others => '0');
			ctrl_length    <= (others => '0');
			ctrl_addr      <= (others => '0');

			ctrl_out_empty <= '1';
			ctrl_out_fill  <= (others => '0');
			ctrl_out_data  <= (others => '0');

			ctrl_in_re     <= '0';
			
			ctrl_mmu_empty <= '1';
			ctrl_mmu_fill  <= (others => '0');
			ctrl_mmu_data  <= (others => '0');

			pgf            <= '0';
			tlb_hits       <= (others => '0');
			tlb_misses     <= (others => '0');
		elsif rising_edge(clk) then
			tlb_we <= '0';

			case state is
				when WAIT_REQUEST =>
					-- start reading if there are 2 word in FIFO
					--if CTRL_FIFO_In_Empty = '0' and CTRL_FIFO_In_Fill >= X"0001" then
					ctrl_in_re <= '1';
					state <= READ_CMD;
					--end if;
				
				when READ_CMD =>
					-- read cmd and length
					if CTRL_FIFO_In_Empty = '0' then
						ctrl_cmd <= CTRL_FIFO_In_Data(31 downto C_MEMIF_LENGTH_WIDTH);
						ctrl_length <= CTRL_FIFO_In_Data(C_MEMIF_LENGTH_WIDTH - 1 downto 0);

						state <= READ_ADDR;
					end if;
				
				when READ_ADDR =>
					-- read address 
					if CTRL_FIFO_In_Empty = '0' then
						ctrl_addr <= CTRL_FIFO_In_Data;
						ctrl_in_re <= '0';

						state <= READ_L1_ENTRY_0;
					end if;

				when READ_L1_ENTRY_0 =>
					if tlb_hit = '1' then
						small_page_addr(31 downto 12) <= tlb_do;

						ctrl_out_empty <= '0';
						ctrl_out_fill  <= X"0001";
						ctrl_out_data <= ctrl_cmd & ctrl_length;

						tlb_hits <= tlb_hits + 1;

						state <= WRITE_CMD;
					else
						-- write command to memory controller
						ctrl_mmu_empty <= '0';
						ctrl_mmu_fill  <= X"0001";
						ctrl_mmu_data  <= X"00000004";
						
						if CTRL_FIFO_Mmu_RE = '1' and ctrl_mmu_empty = '0' then
							ctrl_mmu_fill <= X"0000";
							ctrl_mmu_data <= l1_descriptor_addr;

							tlb_misses <= tlb_misses + 1;

							state <= READ_L1_ENTRY_1;
						end if;
					end if;

				when READ_L1_ENTRY_1 =>
					if CTRL_FIFO_Mmu_RE = '1' and ctrl_mmu_empty = '0' then
						ctrl_mmu_empty <= '1';
						ctrl_mmu_fill  <= X"0000";
						
						state <= READ_L1_ENTRY_2;
					end if;

				when READ_L1_ENTRY_2 =>
					if MEMIF_FIFO_Mmu_WE = '1' then
						l2_table_addr <= MEMIF_FIFO_Mmu_Data;

						if or_reduce(MEMIF_FIFO_Mmu_Data) = '0' then
							pgf <= '1';
							state <= PAGE_FAULT;
						else
							state <= READ_L2_ENTRY_0;
						end if;
					end if;

				when READ_L2_ENTRY_0 =>
					ctrl_mmu_empty <= '0';
					ctrl_mmu_fill  <= X"0001";
					ctrl_mmu_data  <= X"00000004";
					
					if CTRL_FIFO_Mmu_RE = '1' and ctrl_mmu_empty = '0' then
						ctrl_mmu_fill <= X"0000";
						ctrl_mmu_data <= l2_descriptor_addr;

						state <= READ_L2_ENTRY_1;
					end if;

				when READ_L2_ENTRY_1 =>
					if CTRL_FIFO_Mmu_RE = '1' and ctrl_mmu_empty = '0' then
						ctrl_mmu_empty <= '1';
						ctrl_mmu_fill  <= X"0000";
						
						state <= READ_L2_ENTRY_2;
					end if;

				when READ_L2_ENTRY_2 =>
					if MEMIF_FIFO_Mmu_WE = '1' then
						small_page_addr <= MEMIF_FIFO_Mmu_Data;
						
						if MEMIF_FIFO_Mmu_Data(1) = '0' then
							pgf <= '1';
							state <= PAGE_FAULT;
						else
							ctrl_out_empty <= '0';
							ctrl_out_fill  <= X"0001";
							
							ctrl_out_data <= ctrl_cmd & ctrl_length;
							
							tlb_we <= '1';

							state <= WRITE_CMD;
						end if;
					end if;

				when WRITE_CMD =>
					if CTRL_FIFO_Out_RE = '1' then
						ctrl_out_fill <= X"0000";
						
						ctrl_out_data <= physical_addr;

						state <= WRITE_ADDR;
					end if;
				
				when WRITE_ADDR =>
					if CTRL_FIFO_Out_RE = '1' then
						ctrl_out_empty <= '1';
						ctrl_out_fill  <= X"0000";

						state <= WAIT_REQUEST;
					end if;

				when PAGE_FAULT =>
					pgf <= '0';
					
					if MMU_Retry = '1' then
						pgf <= '0';

						state <= READ_L1_ENTRY_0;
					end if; 
			end case;
		end if;
	end process mmu_proc;


	tlb_tag <= ctrl_addr(31 downto 12);
	tlb_di  <= small_page_addr(31 downto 12);

	tlb_gen : if C_TLB_SIZE > 0 generate
		tlb : entity work.reconos_memif_mmu_microblaze_tlb
			generic map (
				C_TLB_SIZE  => C_TLB_SIZE,
				C_TAG_SIZE  => 20,
				C_DATA_SIZE => 20 
			)
			port map (
				TLB_Tag => tlb_tag,
				TLB_DI  => tlb_di,
				TLB_DO  => tlb_do,
				TLB_WE  => tlb_we,
				TLB_Hit => tlb_hit,
				TLB_Clk => clk,
				TLB_Rst => rst
			);
	end generate;
		
end architecture implementation;
