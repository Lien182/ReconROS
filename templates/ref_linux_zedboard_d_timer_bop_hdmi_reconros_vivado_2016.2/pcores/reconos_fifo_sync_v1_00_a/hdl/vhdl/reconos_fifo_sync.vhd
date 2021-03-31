--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - FIFO
--
--   project:      ReconOS
--   author:       Christoph Rüthing, University of Paderborn
--   description:  A simple unidirectional and synchronous FIFO.
--
-- ======================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity reconos_fifo_sync is
	--
	-- Definition of the fifo generics
	--
	--   C_FIFO_DATA_WIDTH - width of the data words
	--   C_FIFO_ADDR_WIDTH - address width (2^C_FIFO_ADDR_WIDTH elements)
	--
	--   C_USE_ALMOST   - enables/disables the almost signals
	--   C_USE_FILLREMM - use fill/remaining signals
	--   C_FIFO_AEMPTY  - limit for almost empty
	--   C_FIFO_AFULL   - limit for almost full
	--
	generic (
		C_FIFO_DATA_WIDTH : integer := 32;
		C_FIFO_ADDR_WIDTH : integer := 2;

		C_USE_ALMOST    : boolean := false;
		C_USE_FILL_REMM : boolean := false;
		C_FIFO_AEMPTY   : integer := 2;
		C_FIFO_AFULL    : integer := 2
	);

	--
	-- Definition of the fifo ports
	--
	--   FIFO_S_Data   - data to read
	--   FIFO_S_Fill   - number of elements currently stored
	--   FIFO_S_Empty  - indicates if empty
	--   FIFO_S_AEmpty - indicates if almost empty (see C_FIFO_AEMPTY)
	--   FIFO_S_RE     - read enable signal
	--
	--   FIFO_M_Data   - data to write
	--   FIFO_M_Remm   - number of elements free to store
	--   FIFO_M_Full   - indicates if full
	--   FIFO_M_AFull  - indicates if almost full (see C_FIFO_AFULL)
	--   FIFO_M_WE     - write enable signal
	--
	--   FIFO_Clk      - clock signal
	--   FIFO_Rst      - asynchronous reset
	--   FIFO_Has_Data - interrupt signal if fifo has data
	--
	port (
		FIFO_S_Data   : out std_logic_vector(C_FIFO_DATA_WIDTH - 1 downto 0);
		FIFO_S_Fill   : out std_logic_vector(C_FIFO_ADDR_WIDTH downto 0);
		FIFO_S_Empty  : out std_logic;
		FIFO_S_AEmpty : out std_logic;
		FIFO_S_RE     : in  std_logic;

		FIFO_M_Data   : in  std_logic_vector(C_FIFO_DATA_WIDTH - 1 downto 0);
		FIFO_M_Remm   : out std_logic_vector(C_FIFO_ADDR_WIDTH downto 0);
		FIFO_M_Full   : out std_logic;
		FIFO_M_AFull  : out std_logic;
		FIFO_M_WE     : in  std_logic;

		FIFO_Clk      : in  std_logic;
		FIFO_Rst      : in  std_logic;
		FIFO_Has_Data : out std_logic
	);
end entity reconos_fifo_sync;


architecture imp of reconos_fifo_sync is
	--
	-- Internal constants
	--
	--   C_FIFO_DEPTH - number of elements to store
	--
	constant C_FIFO_DEPTH : integer := 2 ** C_FIFO_ADDR_WIDTH;

	--
	-- Internal ram to store data
	--
	--   The internal ram is modelled according to the XST User Guide and
	--   will be synthesized as distributed ram. Block ram might be more
	--   efficient use of resources but the fifo is typically rather
	--   small and the block ram delay is hard to handle.
	--
	--   ram_type - vhdl type of the ram
	--   ram      - instantiation of the ram
	--
	type ram_type is array (0 to C_FIFO_DEPTH - 1)
	                       of std_logic_vector(C_FIFO_DATA_WIDTH - 1 downto 0);
	signal ram : ram_type;

	--
	-- Internal pointers used to store state
	--
	--   The internal counters represent the state of the fifo. The read
	--   pointer always points at the active word to read and the write
	--   pointer to the next free memory location. To handle full and
	--   empty conditions, the counters are one bit wider than the
	--   address and the extra bit is used to distinguish full and empty.
	--
	--   rdbin, wrbin - read and write counters
	--   rdptr, wrptr - read and write pointers to address the ram
	--
	signal rdbin, wrbin : unsigned(C_FIFO_ADDR_WIDTH downto 0) := (others => '0');
	signal rdptr, wrptr : unsigned(C_FIFO_ADDR_WIDTH - 1 downto 0) := (others => '0');

	--
	-- Status signals
	--
	--   fill, remm - Fill and remaining
	--
	--   empty, aempty - empty signals
	--   full, afull   - full signals
	--
	signal fill, remm : unsigned(C_FIFO_ADDR_WIDTH downto 0) := (others => '0');

	signal empty, aempty : std_logic;
	signal full, afull   : std_logic;
begin

	-- == Asynchronous calculations =======================================

	rdptr <= rdbin(C_FIFO_ADDR_WIDTH - 1 downto 0);
	wrptr <= wrbin(C_FIFO_ADDR_WIDTH - 1 downto 0);
	
	fill <= wrbin - rdbin;
	remm  <= C_FIFO_DEPTH - fill;
	
	empty  <= '1' when fill = 0 else '0';
	aempty <= '1' when fill <= C_FIFO_AEMPTY else '0';

	full  <= '1' when fill = C_FIFO_DEPTH else '0';
	afull <= '1' when fill >= C_FIFO_DEPTH - C_FIFO_AFULL else '0';


	-- == Process definitions =============================================

	--
	-- Read process
	--
	--   Reading from the fifo by incrementing read counter
	--	
	rd_proc : process(FIFO_Clk,FIFO_Rst) is
	begin
		if FIFO_Rst = '1' then
			rdbin <= (others => '0');
		elsif rising_edge(FIFO_Clk) then
			if FIFO_S_RE = '1' and not empty = '1' then
				rdbin <= rdbin + 1;
			end if;
		end if;
	end process rd_proc;

	--
	-- Write process
	--
	--   Writing to the fifo by incrementing write counter and writing
	--   data to internal ram.
	--	
	wr_proc : process(FIFO_Clk,FIFO_Rst) is
	begin
		if FIFO_Rst = '1' then
			wrbin <= (others => '0');
		elsif rising_edge(FIFO_Clk) then
			if FIFO_M_WE = '1' and not full = '1' then
				ram(to_integer(wrptr)) <= FIFO_M_Data;
				wrbin <= wrbin + 1;
			end if;
		end if;
	end process wr_proc;
	

	-- == Output port assignment ==========================================
	
	FIFO_S_Fill   <= std_logic_vector(fill);
	FIFO_S_Empty  <= empty;
	FIFO_S_AEmpty <= aempty;
	FIFO_S_Data   <= ram(to_integer(rdptr));
	
	FIFO_M_Remm   <= std_logic_vector(remm);
	FIFO_M_Full   <= full;
	FIFO_M_AFull  <= afull;

	FIFO_Has_Data <= not empty;

end architecture imp;

