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
--   author:       see AUTHORS
--   description:  A simple unidirectional and asynchronous FIFO.
--
-- ======================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity reconos_fifo_async is
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
	--   FIFO_S_Clk    - clock of the slave port
	--   FIFO_S_Data   - data to read
	--   FIFO_S_Fill   - number of elements currently stored
	--   FIFO_S_Empty  - indicates if empty
	--   FIFO_S_AEmpty - indicates if almost empty (see C_FIFO_AEMPTY)
	--   FIFO_S_RE     - read enable signal
	--
	--   FIFO_M_Clk    - clock of the master port
	--   FIFO_M_Data   - data to write
	--   FIFO_M_Remm   - number of elements free to store
	--   FIFO_M_Full   - indicates if full
	--   FIFO_M_AFull  - indicates if almost full (see C_FIFO_AFULL)
	--   FIFO_M_WE     - write enable signal
	--
	--   FIFO_Rst      - asynchronous reset
	--   FIFO_Has_Data - interrupt signal if fifo has data
	--
	port (
		FIFO_S_Clk    : in  std_logic;
		FIFO_S_Data   : out std_logic_vector(C_FIFO_DATA_WIDTH - 1 downto 0);
		FIFO_S_Fill   : out std_logic_vector(C_FIFO_ADDR_WIDTH downto 0);
		FIFO_S_Empty  : out std_logic;
		FIFO_S_AEmpty : out std_logic;
		FIFO_S_RE     : in  std_logic;

		FIFO_M_Clk    : in  std_logic;
		FIFO_M_Data   : in  std_logic_vector(C_FIFO_DATA_WIDTH - 1 downto 0);
		FIFO_M_Remm   : out std_logic_vector(C_FIFO_ADDR_WIDTH downto 0);
		FIFO_M_Full   : out std_logic;
		FIFO_M_AFull  : out std_logic;
		FIFO_M_WE     : in  std_logic;

		FIFO_Rst      : in  std_logic;
		FIFO_Has_Data : out std_logic
	);
end entity reconos_fifo_async;


architecture imp of reconos_fifo_async is

	-- Declare port attributes for the Vivado IP Packager
	ATTRIBUTE X_INTERFACE_INFO : STRING;
	ATTRIBUTE X_INTERFACE_PARAMETER : STRING;

	ATTRIBUTE X_INTERFACE_INFO of FIFO_Has_Data: SIGNAL is "xilinx.com:signal:interrupt:1.0 FIFO_Has_Data INTERRUPT";
	ATTRIBUTE X_INTERFACE_PARAMETER of FIFO_Has_Data: SIGNAL is "SENSITIVITY LEVEL_HIGH";

	ATTRIBUTE X_INTERFACE_INFO of FIFO_M_Clk: SIGNAL is "xilinx.com:signal:clock:1.0 FIFO_M_Clk CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of FIFO_M_Clk: SIGNAL is "ASSOCIATED_BUSIF FIFO_M";

	ATTRIBUTE X_INTERFACE_INFO of FIFO_S_Clk: SIGNAL is "xilinx.com:signal:clock:1.0 FIFO_S_Clk CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of FIFO_S_Clk: SIGNAL is "ASSOCIATED_BUSIF FIFO_S";

	ATTRIBUTE X_INTERFACE_INFO of FIFO_Rst: SIGNAL is "xilinx.com:signal:reset:1.0 FIFO_Rst RST";
	ATTRIBUTE X_INTERFACE_PARAMETER of FIFO_Rst: SIGNAL is "POLARITY ACTIVE_HIGH";

	ATTRIBUTE X_INTERFACE_INFO of FIFO_M_Data:     SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 FIFO_M FIFO_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of FIFO_M_Full:     SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 FIFO_M FIFO_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of FIFO_M_WE:       SIGNAL is "cs.upb.de:reconos:FIFO_M:1.0 FIFO_M FIFO_M_WE";

	ATTRIBUTE X_INTERFACE_INFO of FIFO_S_Data:     SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 FIFO_S FIFO_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of FIFO_S_Empty:    SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 FIFO_S FIFO_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of FIFO_S_RE:       SIGNAL is "cs.upb.de:reconos:FIFO_S:1.0 FIFO_S FIFO_S_RE";

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
	--   The counters are synchronized to the other clock domain via
	--   gray code conversion and a two stage synchronizer.
	--
	--   rdbin, wrbin - read and write counters
	--   rdgry, wrgry - gray code conversion of the binary counters
	--   rdptr, wrptr - read and write pointers to address the ram
	--
	--   rdgry_sync0, rdgry_sync, rdbin_sync - synchronized read pointers
	--   wrgry_sync0, wrgry_sync, wrbin_sync - synchronized write pointers
	--
	signal rdbin, wrbin : unsigned(C_FIFO_ADDR_WIDTH downto 0) := (others => '0');
	signal rdgry, wrgry : unsigned(C_FIFO_ADDR_WIDTH downto 0) := (others => '0');
	signal rdptr, wrptr : unsigned(C_FIFO_ADDR_WIDTH - 1 downto 0) := (others => '0');
	
	signal rdgry_sync0, rdgry_sync, rdbin_sync : unsigned(C_FIFO_ADDR_WIDTH downto 0) := (others => '0');
	signal wrgry_sync0, wrgry_sync, wrbin_sync : unsigned(C_FIFO_ADDR_WIDTH downto 0) := (others => '0');

	--
	-- Synchronized signals to read or write clock
	--
	--   rdfill, rdremm - fill and remaining in read clock
	--   wrfill, wrremm - fill and remaining in write clock
	--
	--   rdempty, rdaempty - empty signals in read clock
	--   wrfull, wrafull   - full signals in write clock
	--
	signal rdfill, rdremm : unsigned(C_FIFO_ADDR_WIDTH downto 0) := (others => '0');
	signal wrfill, wrremm : unsigned(C_FIFO_ADDR_WIDTH downto 0) := (others => '0');

	signal rdempty, rdaempty : std_logic;
	signal wrfull, wrafull   : std_logic;
begin

	-- == Asynchronous calculations =======================================

	rdptr <= rdbin(C_FIFO_ADDR_WIDTH - 1 downto 0);
	wrptr <= wrbin(C_FIFO_ADDR_WIDTH - 1 downto 0);
	
	rdfill <= wrbin_sync - rdbin;
	rdremm  <= C_FIFO_DEPTH - rdfill;
	
	wrfill <= wrbin - rdbin_sync;
	wrremm  <= C_FIFO_DEPTH - wrfill;
	
	rdempty  <= '1' when rdfill = 0 else '0';
	rdaempty <= '1' when rdfill <= C_FIFO_AEMPTY else '0';

	wrfull  <= '1' when wrfill = C_FIFO_DEPTH else '0';
	wrafull <= '1' when wrfill >= C_FIFO_DEPTH - C_FIFO_AFULL else '0';
	
	
	rdgry <= (rdbin srl 1) xor rdbin;
	wrgry <= (wrbin srl 1) xor wrbin;
	
	rdbin_sync <= (rdbin_sync srl 1) xor rdgry_sync;
	wrbin_sync <= (wrbin_sync srl 1) xor wrgry_sync;


	-- == Process definitions =============================================

	--
	-- Synchronize write counter to read clock
	--
	--   A two stage synchronizer to cross the different clocks
	--		
	rd_sync : process(FIFO_S_Clk,FIFO_Rst) is
	begin
		if FIFO_Rst = '1' then
			wrgry_sync0 <= (others => '0');
			wrgry_sync <= (others => '0');
		elsif rising_edge(FIFO_S_Clk) then
			wrgry_sync0 <= wrgry;
			wrgry_sync <= wrgry_sync0;
		end if;
	end process rd_sync;
	
	--
	-- Synchronize read counter to write clock
	--
	--   A two stage synchronizer to cross the different clocks
	--		
	wr_sync : process(FIFO_M_Clk,FIFO_Rst) is
	begin
		if FIFO_Rst = '1' then
			rdgry_sync0 <= (others => '0');
			rdgry_sync <= (others => '0');
		elsif rising_edge(FIFO_M_Clk) then
			rdgry_sync0 <= rdgry;
			rdgry_sync <= rdgry_sync0;
		end if;
	end process wr_sync;

	--
	-- Read process
	--
	--   Reading from the fifo by incrementing read counter
	--	
	rd_proc : process(FIFO_S_Clk,FIFO_Rst) is
	begin
		if FIFO_Rst = '1' then
			rdbin <= (others => '0');
		elsif rising_edge(FIFO_S_Clk) then
			if FIFO_S_RE = '1' and not rdempty = '1' then
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
	wr_proc : process(FIFO_M_Clk,FIFO_Rst) is
	begin
		if FIFO_Rst = '1' then
			wrbin <= (others => '0');
		elsif rising_edge(FIFO_M_Clk) then
			if FIFO_M_WE = '1' and not wrfull = '1' then
				ram(to_integer(wrptr)) <= FIFO_M_Data;
				wrbin <= wrbin + 1;
			end if;
		end if;
	end process wr_proc;
	

	-- == Output port assignment ==========================================
	
	FIFO_S_Fill   <= std_logic_vector(rdfill);
	FIFO_S_Empty  <= rdempty;
	FIFO_S_AEmpty <= rdaempty;
	FIFO_S_Data   <= ram(to_integer(rdptr));
	
	FIFO_M_Remm   <= std_logic_vector(wrremm);
	FIFO_M_Full   <= wrfull;
	FIFO_M_AFull  <= wrafull;

	FIFO_Has_Data <= not rdempty;

end architecture imp;

