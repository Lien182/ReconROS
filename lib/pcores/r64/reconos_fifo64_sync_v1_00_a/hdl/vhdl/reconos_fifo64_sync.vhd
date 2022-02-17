--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/64
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - FIFO
--
--   project:      ReconOS64
--   author:       see AUTHORS
--
--   description:  A simple unidirectional and synchronous FIFO.
--
-- ======================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity reconos_fifo64_sync is
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
		C_FIFO_DATA_WIDTH : integer := 64;
		C_FIFO_ADDR_WIDTH : integer := 2;

		C_USE_ALMOST    : boolean := false;
		C_USE_FILL_REMM : boolean := false;
		C_FIFO_AEMPTY   : integer := 2;
		C_FIFO_AFULL    : integer := 2
	);

	--
	-- Definition of the fifo ports
	--
	--   FIFO64_S_Data   - data to read
	--   FIFO64_S_Fill   - number of elements currently stored
	--   FIFO64_S_Empty  - indicates if empty
	--   FIFO64_S_AEmpty - indicates if almost empty (see C_FIFO_AEMPTY)
	--   FIFO64_S_RE     - read enable signal
	--
	--   FIFO64_M_Data   - data to write
	--   FIFO64_M_Remm   - number of elements free to store
	--   FIFO64_M_Full   - indicates if full
	--   FIFO64_M_AFull  - indicates if almost full (see C_FIFO_AFULL)
	--   FIFO64_M_WE     - write enable signal
	--
	--   FIFO64_Clk      - clock signal
	--   FIFO_Rst      - asynchronous reset
	--   FIFO_Has_Data - interrupt signal if fifo has data
	--
	port (
		FIFO64_S_Data   : out std_logic_vector(C_FIFO_DATA_WIDTH - 1 downto 0);
		FIFO64_S_Fill   : out std_logic_vector(C_FIFO_ADDR_WIDTH downto 0);
		FIFO64_S_Empty  : out std_logic;
		FIFO64_S_AEmpty : out std_logic;
		FIFO64_S_RE     : in  std_logic;

		FIFO64_M_Data   : in  std_logic_vector(C_FIFO_DATA_WIDTH - 1 downto 0);
		FIFO64_M_Remm   : out std_logic_vector(C_FIFO_ADDR_WIDTH downto 0);
		FIFO64_M_Full   : out std_logic;
		FIFO64_M_AFull  : out std_logic;
		FIFO64_M_WE     : in  std_logic;

		FIFO64_Clk      : in  std_logic;
		FIFO_Rst      : in  std_logic;
		FIFO_Has_Data : out std_logic
	);
end entity reconos_fifo64_sync;


architecture imp of reconos_fifo64_sync is

	-- Declare port attributes for the Vivado IP Packager
	ATTRIBUTE X_INTERFACE_INFO : STRING;
	ATTRIBUTE X_INTERFACE_PARAMETER : STRING;

	ATTRIBUTE X_INTERFACE_INFO of FIFO_Has_Data: SIGNAL is "xilinx.com:signal:interrupt:1.0 FIFO_Has_Data INTERRUPT";
	ATTRIBUTE X_INTERFACE_PARAMETER of FIFO_Has_Data: SIGNAL is "SENSITIVITY LEVEL_HIGH";

	ATTRIBUTE X_INTERFACE_INFO of FIFO64_Clk: SIGNAL is "xilinx.com:signal:clock:1.0 FIFO64_Clk CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of FIFO64_Clk: SIGNAL is "ASSOCIATED_BUSIF FIFO64_M:FIFO64_S";

	ATTRIBUTE X_INTERFACE_INFO of FIFO_Rst: SIGNAL is "xilinx.com:signal:reset:1.0 FIFO_Rst RST";
	ATTRIBUTE X_INTERFACE_PARAMETER of FIFO_Rst: SIGNAL is "POLARITY ACTIVE_HIGH";

	ATTRIBUTE X_INTERFACE_INFO of FIFO64_M_Data:     SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 FIFO64_M FIFO64_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of FIFO64_M_Full:     SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 FIFO64_M FIFO64_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of FIFO64_M_WE:       SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 FIFO64_M FIFO64_M_WE";

	ATTRIBUTE X_INTERFACE_INFO of FIFO64_S_Data:     SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 FIFO64_S FIFO64_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of FIFO64_S_Empty:    SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 FIFO64_S FIFO64_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of FIFO64_S_RE:       SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 FIFO64_S FIFO64_S_RE";

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
	rd_proc : process(FIFO64_Clk,FIFO_Rst) is
	begin
		if FIFO_Rst = '1' then
			rdbin <= (others => '0');
		elsif rising_edge(FIFO64_Clk) then
			if FIFO64_S_RE = '1' and not empty = '1' then
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
	wr_proc : process(FIFO64_Clk,FIFO_Rst) is
	begin
		if FIFO_Rst = '1' then
			wrbin <= (others => '0');
		elsif rising_edge(FIFO64_Clk) then
			if FIFO64_M_WE = '1' and not full = '1' then
				ram(to_integer(wrptr)) <= FIFO64_M_Data;
				wrbin <= wrbin + 1;
			end if;
		end if;
	end process wr_proc;
	

	-- == Output port assignment ==========================================
	
	FIFO64_S_Fill   <= std_logic_vector(fill);
	FIFO64_S_Empty  <= empty;
	FIFO64_S_AEmpty <= aempty;
	FIFO64_S_Data   <= ram(to_integer(rdptr));
	
	FIFO64_M_Remm   <= std_logic_vector(remm);
	FIFO64_M_Full   <= full;
	FIFO64_M_AFull  <= afull;

	FIFO_Has_Data <= not empty;

end architecture imp;

