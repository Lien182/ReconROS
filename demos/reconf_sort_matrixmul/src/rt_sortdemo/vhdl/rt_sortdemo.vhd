library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.math_real.all;

library reconos_v3_01_a;
use reconos_v3_01_a.reconos_pkg.all;

entity rt_reconf is
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
		
		DEBUG : out std_logic_vector(110 downto 0)
	);
end entity rt_reconf;

architecture sortdemo of rt_reconf is
	type STATE_TYPE is (
					STATE_INIT,
					STATE_GET_ADDR,STATE_READ,STATE_SORTING,
					STATE_WRITE,STATE_ACK,STATE_THREAD_EXIT);

	component bubble_sorter is
		generic (
			G_LEN    : integer := 512;  -- number of words to sort
			G_AWIDTH : integer := 9;  -- in bits
			G_DWIDTH : integer := 32  -- in bits
		);

		port (
			clk   : in std_logic;
			reset : in std_logic;
			-- local ram interface
			o_RAMAddr : out std_logic_vector(0 to G_AWIDTH-1);
			o_RAMData : out std_logic_vector(0 to G_DWIDTH-1);
			i_RAMData : in  std_logic_vector(0 to G_DWIDTH-1);
			o_RAMWE   : out std_logic;
			start     : in  std_logic;
			done      : out std_logic
		);
  	end component;
	
	-- The sorting application reads 'C_LOCAL_RAM_SIZE' 32-bit words into the local RAM,
	-- from a given address (send in a message box), sorts them and writes them back into main memory.

	-- IMPORTANT: define size of local RAM here!!!! 
	constant C_LOCAL_RAM_SIZE          : integer := 2048;
	constant C_LOCAL_RAM_ADDRESS_WIDTH : integer := integer(ceil(log2(real(C_LOCAL_RAM_SIZE))));
	constant C_LOCAL_RAM_SIZE_IN_BYTES : integer := 4*C_LOCAL_RAM_SIZE;

	type LOCAL_MEMORY_T is array (0 to C_LOCAL_RAM_SIZE-1) of std_logic_vector(31 downto 0);
	
	constant MBOX_RECV  : std_logic_vector(31 downto 0) := x"00000000";
	constant MBOX_SEND  : std_logic_vector(31 downto 0) := x"00000001";

	signal addr     : std_logic_vector(31 downto 0);
	signal len      : std_logic_vector(31 downto 0);
	signal state    : STATE_TYPE;
	signal i_osif   : i_osif_t;
	signal o_osif   : o_osif_t;
	signal i_memif  : i_memif_t;
	signal o_memif  : o_memif_t;
	signal i_ram    : i_ram_t;
	signal o_ram    : o_ram_t;

	signal o_RAMAddr_sorter : std_logic_vector(0 to C_LOCAL_RAM_ADDRESS_WIDTH-1);
	signal o_RAMData_sorter : std_logic_vector(0 to 31);
	signal o_RAMWE_sorter   : std_logic;
	signal i_RAMData_sorter : std_logic_vector(0 to 31);

	signal o_RAMAddr_reconos   : std_logic_vector(0 to C_LOCAL_RAM_ADDRESS_WIDTH-1);
	signal o_RAMAddr_reconos_2 : std_logic_vector(0 to 31);
	signal o_RAMData_reconos   : std_logic_vector(0 to 31);
	signal o_RAMWE_reconos     : std_logic;
	signal i_RAMData_reconos   : std_logic_vector(0 to 31);
	
	constant o_RAMAddr_max : std_logic_vector(0 to C_LOCAL_RAM_ADDRESS_WIDTH-1) := (others=>'1');

	shared variable local_ram : LOCAL_MEMORY_T;

	signal ignore   : std_logic_vector(31 downto 0);

	signal sort_start : std_logic := '0';
	signal sort_done  : std_logic := '0';
	signal status : std_logic_vector(31 downto 0);
begin
	DEBUG(0) <= '1' when state = STATE_INIT else '0';
	DEBUG(1) <= '1' when state = STATE_GET_ADDR else '0';
	DEBUG(2) <= '1' when state = STATE_READ else '0';
	DEBUG(3) <= '1' when state = STATE_SORTING else '0';
	DEBUG(4) <= '1' when state = STATE_WRITE else '0';
	DEBUG(5) <= '1' when state = STATE_ACK else '0';
	DEBUG(6) <= '1' when state = STATE_THREAD_EXIT else '0';
	DEBUG(38 downto 7) <= OSIF_Sw2Hw_Data;
	DEBUG(39) <= OSIF_Sw2Hw_Empty;
	DEBUG(71 downto 40) <= MEMIF_Mem2Hwt_Data;
	DEBUG(72) <= MEMIF_Mem2Hwt_Empty;
	DEBUG(104 downto 73) <= o_memif.hwt2mem_data;
	DEBUG(105) <= o_memif.hwt2mem_we;
	DEBUG(106) <= i_memif.hwt2mem_full;
	DEBUG(110 downto 107) <= conv_std_logic_vector(i_memif.step, 4);

	-- local dual-port RAM
	local_ram_ctrl_1 : process (HWT_Clk) is
	begin
		if (rising_edge(HWT_Clk)) then
			if (o_RAMWE_reconos = '1') then
				local_ram(conv_integer(unsigned(o_RAMAddr_reconos))) := o_RAMData_reconos;
			else
				i_RAMData_reconos <= local_ram(conv_integer(unsigned(o_RAMAddr_reconos)));
			end if;
		end if;
	end process;
			
	local_ram_ctrl_2 : process (HWT_Clk) is
	begin
		if (rising_edge(HWT_Clk)) then		
			if (o_RAMWE_sorter = '1') then
				local_ram(conv_integer(unsigned(o_RAMAddr_sorter))) := o_RAMData_sorter;
			else
				i_RAMData_sorter <= local_ram(conv_integer(unsigned(o_RAMAddr_sorter)));
			end if;
		end if;
	end process;
	

	-- instantiate bubble_sorter module
	sorter_i : bubble_sorter
		generic map (
			G_LEN     => C_LOCAL_RAM_SIZE,
			G_AWIDTH  => C_LOCAL_RAM_ADDRESS_WIDTH,
			G_DWIDTH  => 32
		)
		port map (
			clk       => HWT_Clk,
			reset     => HWT_Rst,
			o_RAMAddr => o_RAMAddr_sorter,
			o_RAMData => o_RAMData_sorter,
			i_RAMData => i_RAMData_sorter,
			o_RAMWE   => o_RAMWE_sorter,
			start     => sort_start,
			done      => sort_done
	);

	-- ReconOS initilization
	osif_setup (
		i_osif,
		o_osif,
		OSIF_Sw2Hw_Data,
		OSIF_Sw2Hw_Empty,
		OSIF_Sw2Hw_RE,
		OSIF_Hw2Sw_Data,
		OSIF_Hw2Sw_Full,
		OSIF_Hw2Sw_WE
	);

	memif_setup (
		i_memif,
		o_memif,
		MEMIF_Mem2Hwt_Data,
		MEMIF_Mem2Hwt_Empty,
		MEMIF_Mem2Hwt_RE,
		MEMIF_Hwt2Mem_Data,
		MEMIF_Hwt2Mem_Full,
		MEMIF_Hwt2Mem_WE
	);
	
	ram_setup (
		i_ram,
		o_ram,
		o_RAMAddr_reconos_2,
		o_RAMData_reconos,
		i_RAMData_reconos,
		o_RAMWE_reconos
	);
	
	o_RAMAddr_reconos(0 to C_LOCAL_RAM_ADDRESS_WIDTH-1) <= o_RAMAddr_reconos_2((32-C_LOCAL_RAM_ADDRESS_WIDTH) to 31);
		
	-- os and memory synchronisation state machine
	reconos_fsm: process (HWT_Clk,HWT_Rst,o_osif,o_memif,o_ram) is
		variable done : boolean;
	begin
		if HWT_Rst = '1' then
			osif_reset(o_osif);
			memif_reset(o_memif);
			ram_reset(o_ram);
			state <= STATE_INIT;
			done := False;
			addr <= (others => '0');
			len <= (others => '0');
			sort_start <= '0';
		elsif rising_edge(HWT_Clk) then
			case state is
				when STATE_INIT =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						osif_read(i_osif, o_osif, ignore, done);
						if done then
							state <= STATE_GET_ADDR;
						end if;
					end if;

				-- get address via mbox: the data will be copied from this address to the local ram in the next states
				when STATE_GET_ADDR =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						osif_mbox_tryget(i_osif, o_osif, MBOX_RECV, addr, status, done);
						if done then
							if status = x"00000000" then
								state <= STATE_GET_ADDR;
							else
								len               <= conv_std_logic_vector(C_LOCAL_RAM_SIZE_IN_BYTES,32);
								state             <= STATE_READ;
							end if;
						end if;
					end if;
				
				-- copy data from main memory to local memory
				when STATE_READ =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						memif_read(i_ram,o_ram,i_memif,o_memif,addr(31 downto 2) & "00",X"00000000",len,done);
						if done then
							sort_start <= '1';
							state <= STATE_SORTING;
						end if;
					end if;

				-- sort the words in local RAM
				when STATE_SORTING =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						sort_start <= '0';
						--o_ram.addr <= (others => '0');
						if sort_done = '1' then
							len    <= conv_std_logic_vector(C_LOCAL_RAM_SIZE_IN_BYTES,32);
							--state  <= STATE_WRITE_REQ;
							state  <= STATE_WRITE;
						end if;
					end if;
					
				-- copy data from local memory to main memory
				when STATE_WRITE =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						memif_write(i_ram,o_ram,i_memif,o_memif,X"00000000",addr,len,done);
						if done then
							state <= STATE_ACK;
						end if;
					end if;
				
				-- send mbox that signals that the sorting is finished
				when STATE_ACK =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						osif_mbox_put(i_osif, o_osif, MBOX_SEND, addr, ignore, done);
						if done then state <= STATE_GET_ADDR; end if;
					end if;

				-- thread exit
				when STATE_THREAD_EXIT =>
					osif_thread_exit(i_osif,o_osif);
			
			end case;
		end if;
	end process;
	
end architecture sortdemo;
