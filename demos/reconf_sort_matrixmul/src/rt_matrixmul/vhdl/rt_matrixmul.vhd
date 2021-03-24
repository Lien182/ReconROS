library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.math_real.all;

-- library proc_common_v3_00_a;
-- use proc_common_v3_00_a.proc_common_pkg.all;

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

end rt_reconf;

architecture matrixmul of rt_reconf is
	type STATE_TYPE is (
		STATE_INIT,
		STATE_GET_ADDR2MADDRS,
		STATE_READ_MADDRS,
		STATE_READ_MATRIX_B,
		STATE_READ_MATRIX_ROW_FROM_A,
		STATE_MULTIPLY_MATRIX_ROW,
		STATE_WRITE_MATRIX_ROW_TO_C,
		STATE_ACK,
		STATE_THREAD_EXIT
	);
	
	component matrixmultiplier is
		generic (
			G_LINE_LEN_MATRIX : integer := 128;
			G_RAM_DATA_WIDTH  : integer := 32;

			G_RAM_SIZE_MATRIX_A_C       : integer := 128;
			G_RAM_ADDR_WIDTH_MATRIX_A_C : integer := 7;

			G_RAM_SIZE_MATRIX_B       : integer := 16384;
			G_RAM_ADDR_WIDTH_MATRIX_B : integer := 14
		);
		port (
			clk	  : in  std_logic;
			reset : in  std_logic;
			start : in  std_logic;
			done  : out std_logic;
			
			o_RAM_A_Addr : out std_logic_vector(0 to G_RAM_ADDR_WIDTH_MATRIX_A_C - 1);
			i_RAM_A_Data : in  std_logic_vector(0 to G_RAM_DATA_WIDTH - 1);
			
			o_RAM_B_Addr : out std_logic_vector(0 to G_RAM_ADDR_WIDTH_MATRIX_B - 1);
			i_RAM_B_Data : in  std_logic_vector(0 to G_RAM_DATA_WIDTH - 1);
			
			o_RAM_C_Addr : out std_logic_vector(0 to G_RAM_ADDR_WIDTH_MATRIX_A_C - 1);
			o_RAM_C_Data : out std_logic_vector(0 to G_RAM_DATA_WIDTH - 1);
			o_RAM_C_WE   : out std_logic
		);
	end component;
	
	constant C_LINE_LEN_MATRIX : integer := 128;
	
	-- const for matrixes A and C
	constant C_LOCAL_RAM_SIZE_MATRIX_A_C          : integer := C_LINE_LEN_MATRIX;
	constant C_LOCAL_RAM_ADDR_WIDTH_MATRIX_A_C    : integer := integer(ceil(log2(real(C_LOCAL_RAM_SIZE_MATRIX_A_C))));
	constant C_LOCAL_RAM_SIZE_IN_BYTES_MATRIX_A_C : integer := 4 * C_LOCAL_RAM_SIZE_MATRIX_A_C;
	type LOCAL_MEMORY_TYPE_MATRIX_A_C is array(0 to C_LOCAL_RAM_SIZE_MATRIX_A_C - 1) of std_logic_vector(31 downto 0);
	
	-- const for matrix B
	constant C_LOCAL_RAM_SIZE_MATRIX_B            : integer := C_LINE_LEN_MATRIX*C_LINE_LEN_MATRIX;
	constant C_LOCAL_RAM_ADDR_WIDTH_MATRIX_B      : integer := integer(ceil(log2(real(C_LOCAL_RAM_SIZE_MATRIX_B))));
	constant C_LOCAL_RAM_SIZE_IN_BYTES_MATRIX_B   : integer := 4 * C_LOCAL_RAM_SIZE_MATRIX_B;
	type LOCAL_MEMORY_TYPE_MATRIX_B is array(0 to C_LOCAL_RAM_SIZE_MATRIX_B   - 1) of std_logic_vector(31 downto 0);
	
	-- communication with microblaze core
	constant MBOX_RECV : std_logic_vector(31 downto 0) := x"00000000";
	constant MBOX_SEND : std_logic_vector(31 downto 0) := x"00000001";
	signal ignore : std_logic_vector(31 downto 0);
	
	-- maddr is an acronym for "matrix address" (address that points to a matrix)
	constant C_MADDRS : integer	:= 3;
	type MADDR_BOX_TYPE is array(0 to C_MADDRS-1) of std_logic_vector(31 downto 0);
	-- container for adresses pointing to the first element of matrixes A, B and C
	signal maddrs : MADDR_BOX_TYPE;
	-- points to pointers to the matrixes
	signal addr2maddrs : std_logic_vector(31 downto 0);
	
	-- temporary signals
	signal temp_addr_A : std_logic_vector(31 downto 0);
	signal temp_addr_C : std_logic_vector(31 downto 0);
	
	-- fsm state
	signal state : STATE_TYPE;
	
	-- additional data for memif interfaces
	signal len_data_MATRIX_A_C : std_logic_vector(31 downto 0);
	signal len_data_MATRIX_B   : std_logic_vector(31 downto 0);
	
	-- osif, memif and different local BRAM interfaces
	signal i_osif  : i_osif_t;
	signal o_osif  : o_osif_t;
	signal i_memif : i_memif_t;
	signal o_memif : o_memif_t;
	signal i_ram_A : i_ram_t;
	signal o_ram_A : o_ram_t;
	signal i_ram_B : i_ram_t;
	signal o_ram_B : o_ram_t;
	signal i_ram_C : i_ram_t;
	signal o_ram_C : o_ram_t;
	
	signal o_RAM_A_Addr_reconos   : std_logic_vector(0 to C_LOCAL_RAM_ADDR_WIDTH_MATRIX_A_C - 1);
	signal o_RAM_A_Addr_reconos_2 : std_logic_vector(0 to 31);
	signal o_RAM_A_Data_reconos   : std_logic_vector(0 to 31);
	signal o_RAM_A_WE_reconos     : std_logic;
	signal i_RAM_A_Data_reconos   : std_logic_vector(0 to 31);
	
	signal o_RAM_B_Addr_reconos   : std_logic_vector(0 to C_LOCAL_RAM_ADDR_WIDTH_MATRIX_B   - 1);
	signal o_RAM_B_Addr_reconos_2 : std_logic_vector(0 to 31);
	signal o_RAM_B_Data_reconos   : std_logic_vector(0 to 31);
	signal o_RAM_B_WE_reconos     : std_logic;
	signal i_RAM_B_Data_reconos   : std_logic_vector(0 to 31);
	
	signal o_RAM_C_Addr_reconos   : std_logic_vector(0 to C_LOCAL_RAM_ADDR_WIDTH_MATRIX_A_C - 1);
	signal o_RAM_C_Addr_reconos_2 : std_logic_vector(0 to 31);
	signal o_RAM_C_Data_reconos   : std_logic_vector(0 to 31);
	signal o_RAM_C_WE_reconos     : std_logic;
	signal i_RAM_C_Data_reconos   : std_logic_vector(0 to 31);
	
	signal o_RAM_A_Addr_mul : std_logic_vector(0 to C_LOCAL_RAM_ADDR_WIDTH_MATRIX_A_C - 1);
	signal i_RAM_A_Data_mul : std_logic_vector(0 to 31);
	
	signal o_RAM_B_Addr_mul : std_logic_vector(0 to C_LOCAL_RAM_ADDR_WIDTH_MATRIX_B - 1);
	signal i_RAM_B_Data_mul : std_logic_vector(0 to 31);
	
	signal o_RAM_C_Addr_mul : std_logic_vector(0 to C_LOCAL_RAM_ADDR_WIDTH_MATRIX_A_C - 1);
	signal o_RAM_C_Data_mul : std_logic_vector(0 to 31);
	signal o_RAM_C_WE_mul   : std_logic;
	
	shared variable local_ram_a : LOCAL_MEMORY_TYPE_MATRIX_A_C;
	shared variable local_ram_b : LOCAL_MEMORY_TYPE_MATRIX_B;
	shared variable local_ram_c : LOCAL_MEMORY_TYPE_MATRIX_A_C;
	
	signal multiplier_start : std_logic;
	signal multiplier_done  : std_logic;
	signal status : std_logic_vector(31 downto 0);
begin
	-- local BRAM read and write access
	local_ram_ctrl_1 : process (HWT_Clk) is
	begin
		if (rising_edge(HWT_Clk)) then
			if (o_RAM_A_WE_reconos = '1') then
				local_ram_A(conv_integer(unsigned(o_RAM_A_Addr_reconos))) := o_RAM_A_Data_reconos;
			end if;
			if (o_RAM_B_WE_reconos = '1') then
				local_ram_B(conv_integer(unsigned(o_RAM_B_Addr_reconos))) := o_RAM_B_Data_reconos;
			end if;
			if (o_RAM_C_WE_reconos = '0') then
				i_RAM_C_Data_reconos <= local_ram_C(conv_integer(unsigned(o_RAM_C_Addr_reconos)));
			end if;
		end if;
	end process;
	
	local_ram_ctrl_2 : process (HWT_Clk) is
	begin
		if (rising_edge(HWT_Clk)) then		
			if (o_RAM_C_WE_mul = '1') then
				local_ram_C(conv_integer(unsigned(o_RAM_C_Addr_mul))) := o_RAM_C_Data_mul;
			else
				i_RAM_A_Data_mul <= local_ram_A(conv_integer(unsigned(o_RAM_A_Addr_mul)));
				i_RAM_B_Data_mul <= local_ram_B(conv_integer(unsigned(o_RAM_B_Addr_mul)));
			end if;
		end if;
	end process;
	
	-- the matrix multiplication module
	matrixmultiplier_i : matrixmultiplier
		generic map(
			G_LINE_LEN_MATRIX => C_LINE_LEN_MATRIX,
			G_RAM_DATA_WIDTH  => 32,
			
			G_RAM_SIZE_MATRIX_A_C       => C_LOCAL_RAM_SIZE_MATRIX_A_C,
			G_RAM_ADDR_WIDTH_MATRIX_A_C => C_LOCAL_RAM_ADDR_WIDTH_MATRIX_A_C,
			
			G_RAM_SIZE_MATRIX_B        => C_LOCAL_RAM_SIZE_MATRIX_B,
			G_RAM_ADDR_WIDTH_MATRIX_B  => C_LOCAL_RAM_ADDR_WIDTH_MATRIX_B
		)
		port map(
			clk   => HWT_Clk,
			reset => HWT_Rst,
			start => multiplier_start,
			done  => multiplier_done,
			
			o_RAM_A_Addr => o_RAM_A_Addr_mul,
			i_RAM_A_Data => i_RAM_A_Data_mul,
			
			o_RAM_B_Addr => o_RAM_B_Addr_mul,
			i_RAM_B_Data => i_RAM_B_Data_mul,
			
			o_RAM_C_Addr => o_RAM_C_Addr_mul,
			o_RAM_C_Data => o_RAM_C_Data_mul,
			o_RAM_C_WE   => o_RAM_C_WE_mul
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
		i_ram_A,
		o_ram_A,
		o_RAM_A_Addr_reconos_2,
		o_RAM_A_Data_reconos,
		i_RAM_A_Data_reconos,
		o_RAM_A_WE_reconos
	);
	
	ram_setup (
		i_ram_B,
		o_ram_B,
		o_RAM_B_Addr_reconos_2,
		o_RAM_B_Data_reconos,
		i_RAM_B_Data_reconos,
		o_RAM_B_WE_reconos
	);
	
	ram_setup (
		i_ram_C,
		o_ram_C,
		o_RAM_C_Addr_reconos_2,
		o_RAM_C_Data_reconos,
		i_RAM_C_Data_reconos,
		o_RAM_C_WE_reconos
	);
	
	o_RAM_A_Addr_reconos(0 to C_LOCAL_RAM_ADDR_WIDTH_MATRIX_A_C - 1) <= o_RAM_A_Addr_reconos_2((32-C_LOCAL_RAM_ADDR_WIDTH_MATRIX_A_C) to 31);
	o_RAM_B_Addr_reconos(0 to C_LOCAL_RAM_ADDR_WIDTH_MATRIX_B - 1)   <= o_RAM_B_Addr_reconos_2((32-C_LOCAL_RAM_ADDR_WIDTH_MATRIX_B  ) to 31);
	o_RAM_C_Addr_reconos(0 to C_LOCAL_RAM_ADDR_WIDTH_MATRIX_A_C - 1) <= o_RAM_C_Addr_reconos_2((32-C_LOCAL_RAM_ADDR_WIDTH_MATRIX_A_C) to 31);
	
	-- os and memory synchronisation state machine
	reconos_fsm	: process(HWT_Clk,HWT_Rst,o_osif,o_memif,o_ram_a,o_ram_b,o_ram_c) is
		variable done            : boolean;
		variable addr_pos        : integer;
		variable calculated_rows : integer;
	begin
		if HWT_Rst = '1' then
			osif_reset(o_osif);
			memif_reset(o_memif);
			ram_reset(o_ram_A);
			ram_reset(o_ram_B);
			ram_reset(o_ram_C);
			
			multiplier_start <= '0';
			done := false;
			
			calculated_rows	:= 0;
			
			len_data_MATRIX_A_C <= conv_std_logic_vector(C_LOCAL_RAM_SIZE_IN_BYTES_MATRIX_A_C, 32);
			len_data_MATRIX_B   <= conv_std_logic_vector(C_LOCAL_RAM_SIZE_IN_BYTES_MATRIX_B  , 32);
			-- important to know:
			-- maddrs(0) = C, maddrs(1) = B, maddrs(2) = A
			addr2maddrs <= (others => '0');
			addr_pos    := C_MADDRS - 1;
			for i in 0 to (C_MADDRS - 1) loop
				maddrs(i) <= (others => '0');
			end loop;
			
			temp_addr_A <= (others => '0');
			temp_addr_C <= (others => '0');
			
			state <= STATE_INIT;
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
							state <= STATE_GET_ADDR2MADDRS;
						end if;
					end if;

				-- Get address pointing to the addresses pointing to the 3 matrixes via FSL.
				when STATE_GET_ADDR2MADDRS =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						--osif_mbox_get(i_osif, o_osif, MBOX_RECV, addr2maddrs, done);
						osif_mbox_tryget(i_osif, o_osif, MBOX_RECV, addr2maddrs, status, done);
						if (done) then
							if status = x"00000000" then
								state <= STATE_GET_ADDR2MADDRS;
							else
								addr2maddrs <= addr2maddrs(31 downto 2) & "00";
								addr_pos := C_MADDRS - 1;
								state <= STATE_READ_MADDRS;
							end if;
						end if;
					end if;
				
				-- Read addresses pointing to input matrixes A, B and output matrix C from main memory.
				when STATE_READ_MADDRS =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						memif_read_word(i_memif, o_memif, addr2maddrs, maddrs(addr_pos), done);
						if done then
							if (addr_pos = 0) then
								state <= STATE_READ_MATRIX_B;
							else
								addr_pos := addr_pos - 1;
								addr2maddrs <= conv_std_logic_vector(unsigned(addr2maddrs) + 4, 32);
							end if;
						end if;
					end if;
				
				-- Read matrix B from main memory.
				when STATE_READ_MATRIX_B =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						memif_read(i_ram_B, o_ram_B, i_memif, o_memif, maddrs(1), X"00000000", len_data_MATRIX_B, done);
						if done then
							temp_addr_A <= maddrs(2);
							temp_addr_C <= maddrs(0);
							state <= STATE_READ_MATRIX_ROW_FROM_A;
						end if;
					end if;
				
				-- Read a row of matrix A.
				when STATE_READ_MATRIX_ROW_FROM_A =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						memif_read(i_ram_A, o_ram_A, i_memif, o_memif, temp_addr_A, X"00000000", len_data_MATRIX_A_C, done);
						if done then
							multiplier_start <= '1';
							state <= STATE_MULTIPLY_MATRIX_ROW;
						end if;
					end if;
				
				-- Multiply row of matrix A with matrix B.
				when STATE_MULTIPLY_MATRIX_ROW =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						multiplier_start <= '0';
						if (multiplier_done = '1') then
							calculated_rows := calculated_rows + 1;
							state <= STATE_WRITE_MATRIX_ROW_TO_C;
						end if;
					end if;
				
				-- Write multiplication result (row of matrix C) to main memory.
				when STATE_WRITE_MATRIX_ROW_TO_C =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						memif_write(i_ram_C, o_ram_C, i_memif, o_memif, X"00000000", temp_addr_C, len_data_MATRIX_A_C, done);
						if (done) then
							if (calculated_rows < C_LINE_LEN_MATRIX) then
								-- Calculate new temporary addresses
								-- => to fetch next matrix row of matrix A
								-- => to store calculated values to next matrix row of matrix C
								temp_addr_A <= conv_std_logic_vector(unsigned(temp_addr_A) + C_LINE_LEN_MATRIX*4, 32);
								temp_addr_C <= conv_std_logic_vector(unsigned(temp_addr_C) + C_LINE_LEN_MATRIX*4, 32);
								state <= STATE_READ_MATRIX_ROW_FROM_A;
							else
								state <= STATE_ACK;
							end if;
						end if;
					end if;
				
				-- We finished calculating matrix multiplication A * B = C.
				when STATE_ACK =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						osif_mbox_put(i_osif, o_osif, MBOX_SEND, maddrs(addr_pos), ignore, done);
						if (done) then
							calculated_rows	:= 0;
							addr_pos := C_MADDRS - 1;
							temp_addr_A <= (others => '0');
							temp_addr_C	 <= (others => '0');
							state <= STATE_GET_ADDR2MADDRS;
						end if;
					end if;
				
				-- Terminate hardware thread.
				when STATE_THREAD_EXIT =>
					osif_thread_exit(i_osif, o_osif);
			end case;
		end if;
	end process;
	
end architecture matrixmul;
