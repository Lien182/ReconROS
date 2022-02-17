--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/64
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
----------------------------------------------------------------------------------
-- Company:     University of Paderborn
-- Engineer:    see AUTHORS
-- 
-- Create Date:     05/10/2019 01:33:41 PM
-- Design Name:     VHDL Package - ReconOS
-- Module Name:     reconos64_pkg
-- Project Name:    ReconOS64
-- Target Devices:  Zynq Ultrascale+
-- Tool Versions:   Vivado 2018.2
-- Description:     The entire ReconOS package with type definitions and
--                  hardware OS services in VHDL
-- Dependencies: 
-- 
-- Revision:
-- Revision 1.02 - altered OSIF data width to 64bit
-- Revision 1.01 - altered MEMIF data width to 64bit
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------



library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package reconos_pkg is

	-- == Constant definitions ============================================

	--
	-- General constants
	--
	--   C_OSIF_DATA_WIDTH  - width of the osif
	--   C_MEMIF_DATA_WIDTH - width of the memif
	--
	--   C_MEMIF_CHUNK_WORDS  - size of one memory request in words
	--                          (a request might be split up to meet this)
	--   C_MEMIF_CHUNK_BYTES  - chunk size in bytes
	--   C_MEMIF_CHUNK_WIDTH  - width of chunk (log2 C_MEMIF_CHUNK_BYTES)
	--   C_MEMIF_LENGTH_WIDTH - width of the length in command word
	--   C_MEMIF_OP_WIDTH     - width of the operation in command word
	--
	constant C_OSIF_DATA_WIDTH  : integer := 64;
	constant C_MEMIF_DATA_WIDTH : integer := 64;
	

	constant C_MEMIF_CHUNK_WORDS  : integer := 32;
	constant C_MEMIF_CHUNK_BYTES  : integer := C_MEMIF_CHUNK_WORDS * 8;
	constant C_MEMIF_CHUNK_WIDTH  : integer := 8;
	constant C_MEMIF_LENGTH_WIDTH : integer := 56;
	constant C_MEMIF_OP_WIDTH     : integer := 8;

	--
	-- "Constants" for easier handling of ranges
	--
	--   C_MEMIF_LENGTH_RANGE - range of the length in command word
	--   C_MEMIF_OP_RANGE     - range of the operation in command word
	--   C_MEMIF_CHUNK_RANGE  - range of chunk offset
	--
	subtype C_MEMIF_LENGTH_RANGE is natural range C_MEMIF_LENGTH_WIDTH - 1 downto 0;
	subtype C_MEMIF_OP_RANGE is natural range C_MEMIF_DATA_WIDTH - 1 downto C_MEMIF_DATA_WIDTH - C_MEMIF_OP_WIDTH;
	subtype C_MEMIF_CHUNK_RANGE is natural range C_MEMIF_CHUNK_WIDTH - 1 downto 0;

	--
	-- Definition of osif commands
	--
	--   self-describing
	--
	constant OSIF_CMD_THREAD_GET_INIT_DATA  : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000A0";
	constant OSIF_CMD_THREAD_GET_STATE_ADDR : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000A1";
	constant OSIF_CMD_THREAD_EXIT           : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000A2";
	constant OSIF_CMD_THREAD_YIELD          : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000A3";
	constant OSIF_CMD_THREAD_CLEAR_SIGNAL   : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000A4";
	constant OSIF_CMD_SEM_POST              : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000B0";
	constant OSIF_CMD_SEM_WAIT              : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000B1";
	constant OSIF_CMD_MUTEX_LOCK            : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000C0";
	constant OSIF_CMD_MUTEX_UNLOCK          : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000C1";
	constant OSIF_CMD_MUTEX_TRYLOCK         : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000C2";
	constant OSIF_CMD_COND_WAIT             : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000D0";
	constant OSIF_CMD_COND_SIGNAL           : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000D1";
	constant OSIF_CMD_COND_BROADCAST        : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000D2";
	constant OSIF_CMD_MBOX_GET              : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000F0";
	constant OSIF_CMD_MBOX_PUT              : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000F1";
	constant OSIF_CMD_MBOX_TRYGET           : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000F2";
	constant OSIF_CMD_MBOX_TRYPUT           : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000F3";
	constant OSIF_CMD_MASK                  : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000FF";
	constant OSIF_CMD_YIELD_MASK            : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"0000000080000000";

	constant OSIF_SIGNAL_THREAD_START       : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"0000000001000000";
	constant OSIF_SIGNAL_THREAD_RESUME      : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"0000000001000001";

	constant OSIF_INTERRUPTED               : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0) := x"00000000000000FF";

	--
	-- Definition of memif commands
	--
	--   self-describing
	--
	constant MEMIF_CMD_READ  : std_logic_vector(C_MEMIF_OP_WIDTH - 1 downto 0) := x"00";
	constant MEMIF_CMD_WRITE : std_logic_vector(C_MEMIF_OP_WIDTH - 1 downto 0) := x"F0";


	-- == Type definitions ================================================

	--
	-- Type definitions of i_osif_t and o_osif_t
	--
	--   sw2hw_/hw2sw_ - fifo signals
	--
	--   step  - internal state of the osif
	--   void  - void bit free to use
	--
	type i_osif_t is record
		sw2hw_data   : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		sw2hw_empty  : std_logic;
		hw2sw_full   : std_logic;

		step         : integer range 0 to 15;
		void         : std_logic;
	end record;

	type o_osif_t is record
		sw2hw_re   : std_logic;
		hw2sw_data : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		hw2sw_we   : std_logic;

		step       : integer range 0 to 15;
		void       : std_logic;
	end record;

	--
	-- Type definitions of i_memif_t and o_memif_t
	--
	--   mem2hwt_/hwt2mem_ - fifo signals
	--
	--   step  - internal state of the osif
	--
	type i_memif_t is record
		mem2hwt_data   : std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		mem2hwt_empty  : std_logic;
		hwt2mem_full   : std_logic;

		step           : integer range 0 to 15;
	end record;

	type o_memif_t is record
		mem2hwt_re   : std_logic;
		hwt2mem_data : std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		hwt2mem_we   : std_logic;

		step         : integer range 0 to 15;
	end record;
	
	--
	-- Type definitions of i_ram_t and o_ram_t
	--
	--   ram_  - ram signals
	--
	--   count    - byte counter of written bytes
	--   mem_addr - remote address of main memory
	--
	type i_ram_t is record
		ram_addr : unsigned(63 downto 0);
		ram_data : std_logic_vector(63 downto 0);

		remm     : unsigned(63 downto 0);
		mem_addr : unsigned(63 downto 0);
	end record;
	
	type o_ram_t is record
		ram_addr : unsigned(63 downto 0);
		ram_data : std_logic_vector(63 downto 0);
		ram_we   : std_logic;

		remm     : unsigned(63 downto 0);
		mem_addr : unsigned(63 downto 0);
	end record;


	-- == Reconos functions ===============================================

	--
	-- Assigns signals to the osif record. This function must be called
	-- asynchronously in the main entity including the os-fsm.
	--
	--   i_osif       - i_osif_t record
	--   o_osif       - o_osif_t_record
	--   sw2hw_data   - OSIF_FIFO_Sw2Hw_Data
	--   sw2hw_empty  - OSIF_FIFO_Sw2Hw_Empty
	--   sw2hw_re     - OSIF_FIFO_Sw2Hw_RE
	--   hw2sw_data   - OSIF_FIFO_Hw2Sw_Data
	--   hw2sw_full   - OSIF_FIFO_Hw2Sw_Full
	--   hw2sw_we     - OSIF_FIFO_Hw2Sw_WE
	--
	procedure osif_setup (
		signal i_osif       : out i_osif_t;
		signal o_osif       : in  o_osif_t;
		signal sw2hw_data   : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal sw2hw_empty  : in  std_logic;
		signal sw2hw_re     : out std_logic;
		signal hw2sw_data   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal hw2sw_full   : in  std_logic;
		signal hw2sw_we     : out std_logic
	);

	--	
	-- Resets the osif signals to a default state. This function should be called
	-- on reset of the os-fsm.
	--
	--   o_osif - o_osif_t record
	--
	procedure osif_reset (
		signal o_osif : out o_osif_t
	);

	--
	-- Assigns signals to the memif record. This function must be called
	-- asynchronously in the main entity including the os-fsm.
	--
	--   i_memif        - i_memif_t record
	--   o_memif        - o_memif_t record
	--   mem2hwt_data   - MEMIF_FIFO_Mem2Hwt_Data
	--   mem2hwt_empty  - MEMIF_FIFO_Mem2Hwt_Empty
	--   mem2hwt_re     - MEMIF_FIFO_Mem2Hwt_RE
	--   hwt2mem_data   - MEMIF_FIFO_Hwt2Mem_Data
	--   hwt2mem_full   - MEMIF_FIFO_Hwt2Mem_Full
	--   hwt2mem_we     - MEMIF_FIFO_Hwt2Mem_WE
	--
	procedure memif_setup (
		signal i_memif        : out i_memif_t;
		signal o_memif        : in  o_memif_t;
		signal mem2hwt_data   : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal mem2hwt_empty  : in  std_logic;
		signal mem2hwt_re     : out std_logic;
		signal hwt2mem_data   : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal hwt2mem_full   : in  std_logic;
		signal hwt2mem_we     : out std_logic
	);

	--
	-- Resets the memif signals to a default state. This function should be called
	-- on reset of the os-fsm.
	--
	--   o_memif - o_memif_t record
	--
	procedure memif_reset (
		signal o_memif : out o_memif_t
	);

	--
	-- Assigns signals to the memif record. This function must be called
	-- asynchronously in the main entity including the os-fsm.
	--
	--   i_ram      - i_ram_t record
	--   o_ram      - o_ram_t record
	--   ram_addr   - address signal of the local ram
	--   ram_i_data - input data signal of the local ram
	--   ram_o_data - output data signal of the local ram
	--   ram_we     - write enable signal of the local ram
	--
	procedure ram_setup (
		signal i_ram      : out i_ram_t;
		signal o_ram      : in  o_ram_t;
		signal ram_addr   : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal ram_i_data : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal ram_o_data : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal ram_we     : out std_logic
	);

	--
	-- Resets the RAM signals to a default state. This function should be called
	-- on reset of the os-fsm.
	--
	--   o_ram - o_ram_t record
	--
	procedure ram_reset (
		signal o_ram  : out o_ram_t
	);

	--
	-- Reads a single word from the osif.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   word   - word read from the osif
	--   done   - indicates when read finished
	--
	procedure osif_read (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		signal word   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- Writes a single word into the osif
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   word   - word to write int the osif
	--   done   - indicates when write finished
	-- 
	procedure osif_write (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		word          : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- ONLY FOR INTERNAL USE
	--
	-- Issues a system call with no arguments and a no result.
	--
	--   i_osif  - i_osif_t record
	--   o_osif  - o_osif_t record
	--   call_id - id of the system call
	--   done    - indicates when system call finished
	--
	procedure osif_call_0_0 (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		call_id       : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- ONLY FOR INTERNAL USE
	--
	-- Issues a system call with no arguments and a single result.
	--
	--   i_osif  - i_osif_t record
	--   o_osif  - o_osif_t record
	--   call_id - id of the system call
	--   ret0    - result of the system call
	--   done    - indicates when system call finished
	--
	procedure osif_call_0_1 (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		call_id       : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal ret0   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- ONLY FOR INTERNAL USE
	--
	-- Issues a system call with one argument and a single result.
	--
	--   i_osif  - i_osif_t record
	--   o_osif  - o_osif_t record
	--   call_id - id of the system call
	--   arg0    - argument of the system call
	--   ret0    - result of the system call
	--   done    - indicates when system call finished
	--
	procedure osif_call_1_1 (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		call_id       : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		arg0          : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal ret0   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- ONLY FOR INTERNAL USE
	--
	-- Issues a system call with one arguments and two results.
	--
	--   i_osif  - i_osif_t record
	--   o_osif  - o_osif_t record
	--   call_id - id of the system call
	--   arg0    - argument of the system call
	--   ret1    - first result of the system call
	--   ret2    - second result of the system call
	--   done    - indicates when system call finished
	--
	procedure osif_call_1_2 (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		call_id       : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		arg0          : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal ret0   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal ret1   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	-- ONLY FOR INTERNAL USE
	--
	-- Issues a system call with two arguments and a single result.
	--
	--   i_osif  - i_osif_t record
	--   o_osif  - o_osif_t record
	--   call_id - id of the system call
	--   arg0    - first argument of the system call
	--   arg1    - second argument of the system call
	--   ret0    - result of the system call
	--   done    - indicates when system call finished
	--
	procedure osif_call_2_1 (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		call_id       : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		arg0          : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		arg1          : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal ret0   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- Posts the semaphore specified by handle.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   handle - index representing the resource in the resource array
	--   result - result of the osif call
	--   done   - indicates when call finished
	--
	procedure osif_sem_post (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- Waits for the semaphore specified by handle.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   handle - index representing the resource in the resource array
	--   result - result of the osif call
	--   done   - indicates when call finished
	--
	procedure osif_sem_wait (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- Locks the mutex specified by handle.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   handle - index representing the resource in the resource array
	--   result - result of the osif call
	--   done   - indicates when call finished
	--
	procedure osif_mutex_lock (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	-- Unlocks the mutex specified by handle.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   handle - index representing the resource in the resource array
	--   result - result of the osif call
	--   done   - indicates when call finished
	--
	procedure osif_mutex_unlock (
		signal i_osif  : in  i_osif_t;
		signal o_osif  : out o_osif_t;
		handle         : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result  : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done  : out boolean
	);

	--
	-- Tries to lock the mutex specified by handle and returns if successful or not.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   handle - index representing the resource in the resource array
	--   result - result of the osif call
	--   done   - indicates when call finished
	--
	procedure osif_mutex_trylock (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- Waits for the condition variable specified by handle.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   handle - index representing the resource in the resource array
	--   result - result of the osif call
	--   done   - indicates when call finished
	--
	procedure osif_cond_wait (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		cond_handle   : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		mutex_handle  : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- Signals a single thread waiting on the condition variable specified by handle.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   handle - index representing the resource in the resource array
	--   result - result of the osif call
	--   done   - indicates when call finished
	--
	procedure osif_cond_signal (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- Signals all threads waiting on the condition variable specified by handle.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   handle - index representing the resource in the resource array
	--   result - result of the osif call
	--   done   - indicates when call finished
	--
	procedure osif_cond_broadcast (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);
	
	--
	-- Puts a single word into the mbox specified by handle.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   handle - index representing the resource in the resource array
	--   word   - word to write into the mbox
	--   result - result of the osif call
	--   done   - indicates when call finished
	--
	procedure osif_mbox_put (
		signal i_osif  : in  i_osif_t;
		signal o_osif  : out o_osif_t;
		handle         : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		word           : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result  : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done  : out boolean
	);

	--
	-- Reads a single word from the mbox specified by handle.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   handle - index representing the resource in the resource array
	--   word   - word read from the mbox
	--   done   - indicates when call finished
	--
	procedure osif_mbox_get (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal word   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- Tries to put a single word into the mbox specified by handle but does not
	-- blocks until the mbox gets populated.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   handle - index representing the resource in the resource array
	--   word   - word to write into the mbox
	--   result - indicates if word was written into the mbox
	--   done   - indicates when call finished
	--
	procedure osif_mbox_tryput (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		word          : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- Tries to read a single word from the mbox specified by handle but does not
	-- blocks until the mbox gets free.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   handle - index representing the resource in the resource array
	--   word   - word read from the mbox
	--   result - indicates if a word was read from the mbox
	--   done   - indicates when call finished
	--
	procedure osif_mbox_tryget (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal word   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- Gets the pointer to the initialization data of the ReconOS thread
	-- specified by reconos_hwt_setinitdata.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--   init   - the pointer to the initialization data
	--   done   - indicated when call finished
	--
	procedure osif_get_init_data (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		signal init   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	);

	--
	-- Terminates the current ReconOS thread.
	--
	--   i_osif - i_osif_t record
	--   o_osif - o_osif_t record
	--
	procedure osif_thread_exit (
		signal i_osif  : in  i_osif_t;
		signal o_osif  : out o_osif_t
	);
	
	
	-- supports 39-bit addressing
	
	--
    -- Writes a double word into the main memory.
    --
    --   i_memif - i_memif_t record
    --   o_memif - o_memif_t record
    --   addr    - address of the main memory to write
    --   data    - double word to write into the main memory
    --   done    - indicates that the call finished
    --
    procedure memif_write_word (
        signal i_memif : in  i_memif_t;
        signal o_memif : out o_memif_t;
        addr           : in  std_logic_vector(63 downto 0);
        data           : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
        variable done  : out boolean
    );
	
	--
    -- Reads a double word from the main memory.
    --
    --   i_memif - i_memif_t record
    --   o_memif - o_memif_t record
    --   addr    - address of the main memory to read from
    --   data    - word read from the main memory
    --   done    - indicates that the call finished
    --
    procedure memif_read_word (
        signal i_memif : in  i_memif_t;
        signal o_memif : out o_memif_t;
        addr           : in  std_logic_vector(63 downto 0);
        signal data    : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
        variable done  : out boolean
    );

	--
 	-- Writes several words from the local ram into main memory. Therefore,
 	-- divides a large request into smaller ones of length at most
 	-- MEMIF_CHUNK_BYTES and splits request at page borders to guarantee
 	-- correct address translation.
	--
	--   i_ram    - i_ram_t record
	--   o_ram    - o_ram_t record
	--   i_memif  - i_memif_t record
	--   o_memif  - o_memif_t record
	--   src_addr - start address to read from the local ram
	--   dst_addr - start address to write into the main memory
	--   len      - number of bytes to transmit (bytes)
	--   done     - indicates that the call finished
	--
	procedure memif_write (
		signal i_ram   : in  i_ram_t;
		signal o_ram   : out o_ram_t;
		signal i_memif : in  i_memif_t;
		signal o_memif : out o_memif_t;
		src_addr       : in  std_logic_vector(63 downto 0);
		dst_addr       : in  std_logic_vector(63 downto 0);
		len            : in  std_logic_vector(31 downto 0);
		variable done  : out boolean
	);

	--
 	-- Reads several words from the main memory into the local ram. Therefore,
 	-- divides a large request into smaller ones of length at most
 	-- MEMIF_CHUNK_BYTES and splits request at page borders to guarantee
 	-- correct address translation.
	--
	--   i_ram    - i_ram_t record
	--   o_ram    - o_ram_t record
	--   i_memif  - i_memif_t record
	--   o_memif  - o_memif_t record
	--   src_addr - start address to read from the main memory
	--   dst_addr - start address to write into the local ram
	--   len      - number of bytes to transmit (bytes)
	--   done     - indicates that the call finished
	--
	procedure memif_read (
		signal i_ram   : in  i_ram_t;
		signal o_ram   : out o_ram_t;
		signal i_memif : in  i_memif_t;
		signal o_memif : out o_memif_t;
		src_addr       : in  std_logic_vector(63 downto 0);
		dst_addr       : in  std_logic_vector(63 downto 0);
		len            : in  std_logic_vector(31 downto 0);
		variable done  : out boolean
	);
	
end package reconos_pkg;

package body reconos_pkg is

	--
	-- @see header
	--
	procedure osif_setup (
		signal i_osif       : out i_osif_t;
		signal o_osif       : in  o_osif_t;
		signal sw2hw_data   : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal sw2hw_empty  : in  std_logic;
		signal sw2hw_re     : out std_logic;
		signal hw2sw_data   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal hw2sw_full   : in  std_logic;
		signal hw2sw_we     : out std_logic
	) is begin
		i_osif.sw2hw_data  <= sw2hw_data;
		i_osif.sw2hw_empty <= sw2hw_empty;
		sw2hw_re           <= o_osif.sw2hw_re;

		hw2sw_data         <= o_osif.hw2sw_data;
		i_osif.hw2sw_full  <= hw2sw_full;
		hw2sw_we           <= o_osif.hw2sw_we;

		i_osif.step        <= o_osif.step;
		i_osif.void        <= o_osif.void;
	end procedure osif_setup;

	--
	-- @see header
	--
	procedure osif_reset (
		signal o_osif  : out o_osif_t
	) is begin
		o_osif.sw2hw_re   <= '0';
		o_osif.hw2sw_data <= (others => '0');
		o_osif.hw2sw_we   <= '0';

		o_osif.step       <= 0;
		o_osif.void       <= '0';
	end procedure osif_reset;

	--
	-- @see header
	--
	procedure memif_setup (
		signal i_memif        : out i_memif_t;
		signal o_memif        : in  o_memif_t;
		signal mem2hwt_data   : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal mem2hwt_empty  : in  std_logic;
		signal mem2hwt_re     : out std_logic;
		signal hwt2mem_data   : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal hwt2mem_full   : in  std_logic;
		signal hwt2mem_we     : out std_logic
	) is begin
		i_memif.mem2hwt_data  <= mem2hwt_data;
		i_memif.mem2hwt_empty <= mem2hwt_empty;
		mem2hwt_re            <= o_memif.mem2hwt_re;

		hwt2mem_data          <= o_memif.hwt2mem_data;
		i_memif.hwt2mem_full  <= hwt2mem_full;
		hwt2mem_we            <= o_memif.hwt2mem_we;

		i_memif.step          <= o_memif.step;
	end procedure memif_setup;

	--
	-- @see header
	--
	procedure memif_reset (
		signal o_memif  : out o_memif_t
	) is begin
		o_memif.mem2hwt_re   <= '0';
		o_memif.hwt2mem_data <= (others => '0');
		o_memif.hwt2mem_we   <= '0';

		o_memif.step         <= 0;
	end procedure memif_reset;

	--
	-- @see header
	--
	procedure ram_setup (
		signal i_ram      : out i_ram_t;
		signal o_ram      : in  o_ram_t;
		signal ram_addr   : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal ram_i_data : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal ram_o_data : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal ram_we     : out std_logic
	) is begin
		ram_addr       <= std_logic_vector(o_ram.ram_addr);
		i_ram.ram_addr <= o_ram.ram_addr;
		ram_i_data     <= o_ram.ram_data;
		i_ram.ram_data <= ram_o_data;
		ram_we         <= o_ram.ram_we;

		i_ram.remm     <= o_ram.remm;
		i_ram.mem_addr <= o_ram.mem_addr;
	end procedure ram_setup;

	--
	-- @see header
	--
	procedure ram_reset (
		signal o_ram  : out o_ram_t
	) is begin
		o_ram.ram_addr <= (others => '0');
		o_ram.ram_data <= (others => '0');
		o_ram.ram_we   <= '0';
		o_ram.remm     <= (others => '0');
		o_ram.mem_addr <= (others => '0');
	end procedure ram_reset;

	--
	-- @see header
	--
	procedure osif_read (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		signal word   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		done := False;
		
		case i_osif.step is
			when 0 =>
				o_osif.sw2hw_re <= '1';

				o_osif.step <= 1;

			when 1 =>
				if i_osif.sw2hw_empty = '0' then
					word <= i_osif.sw2hw_data;
					o_osif.sw2hw_re <= '0';

					o_osif.step <= 2;
				end if;

			when others =>
					done := True;
					o_osif.step <= 0;

		end case;
	end procedure osif_read;

	--
	-- @see header
	--	
	procedure osif_write (
		signal i_osif  : in  i_osif_t;
		signal o_osif  : out o_osif_t;
		word           : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done  : out boolean
	) is begin
		done := False;

		case i_osif.step is
			when 0 =>
				o_osif.hw2sw_we <= '1';
				o_osif.hw2sw_data <= word;

				o_osif.step <= 1;

			when 1 =>
				if i_osif.hw2sw_full = '0' then
					o_osif.hw2sw_we <= '0';

					o_osif.step <= 2;
				end if;

			when others =>
					done := True;
					o_osif.step <= 0;

		end case;
	end procedure osif_write;

	--
	-- @see header
	--
	procedure osif_call_0_0 (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		call_id       : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		done := False;

		case i_osif.step is
			when 0 =>
				o_osif.hw2sw_we <= '1';
				o_osif.hw2sw_data <= call_id;

				o_osif.step <= 1;

			when 1 =>
				if i_osif.hw2sw_full = '0' then
					o_osif.hw2sw_we <= '0';

					o_osif.step <= 2;
				end if;

			when others =>
					done := True;
					o_osif.step <= 0;

		end case;
	end procedure osif_call_0_0;

	--
	-- @see header
	--
	procedure osif_call_0_1 (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		call_id       : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal ret0   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		done := False;

		case i_osif.step is
			when 0 =>
				o_osif.hw2sw_we <= '1';
				o_osif.hw2sw_data <= call_id;

				o_osif.step <= 1;

			when 1 =>
				if i_osif.hw2sw_full = '0' then
					o_osif.hw2sw_we <= '0';
					o_osif.sw2hw_re <= '1';

					o_osif.step <= 2;
				end if;
				
			when 2 =>
				if i_osif.sw2hw_empty = '0' then
					ret0 <= i_osif.sw2hw_data;
					o_osif.sw2hw_re <= '0';

					o_osif.step <= 3;
				end if;

			when others =>
					done := True;
					o_osif.step <= 0;

		end case;
	end procedure osif_call_0_1;

	--
	-- @see header
	--
	procedure osif_call_1_1 (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		call_id       : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		arg0          : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal ret0   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		done := False;

		case i_osif.step is
			when 0 =>
				o_osif.hw2sw_we <= '1';
				o_osif.hw2sw_data <= call_id;

				o_osif.step <= 1;

			when 1 =>
				if i_osif.hw2sw_full = '0' then
					o_osif.hw2sw_data <= arg0;

					o_osif.step <= 2;
				end if;

			when 2 =>
				if i_osif.hw2sw_full = '0' then
					o_osif.hw2sw_we <= '0';
					o_osif.sw2hw_re <= '1';

					o_osif.step <= 3;
				end if;

			when 3 =>
				if i_osif.sw2hw_empty = '0' then
					ret0 <= i_osif.sw2hw_data;
					o_osif.sw2hw_re <= '0';

					o_osif.step <= 4;
				end if;

			when others =>
					done := True;
					o_osif.step <= 0;

		end case;
	end procedure osif_call_1_1;

	--
	-- @see header
	--
	procedure osif_call_1_2 (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		call_id       : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		arg0          : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal ret0   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal ret1   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		done := False;

		case i_osif.step is
			when 0 =>
				o_osif.hw2sw_we <= '1';
				o_osif.hw2sw_data <= call_id;

				o_osif.step <= 1;

			when 1 =>
				if i_osif.hw2sw_full = '0' then
					o_osif.hw2sw_data <= arg0;

					o_osif.step <= 2;
				end if;

			when 2 =>
				if i_osif.hw2sw_full = '0' then
					o_osif.hw2sw_we <= '0';
					o_osif.sw2hw_re <= '1';

					o_osif.step <= 3;
				end if;
				
			when 3 =>
				if i_osif.sw2hw_empty = '0' then
					ret0 <= i_osif.sw2hw_data;

					o_osif.step <= 4;
				end if;
				
			when 4 =>
				if i_osif.sw2hw_empty = '0' then
					ret1 <= i_osif.sw2hw_data;
					o_osif.sw2hw_re <= '0';

					o_osif.step <= 5;
				end if;

			when others =>
					done := True;
					o_osif.step <= 0;
		end case;
	end procedure osif_call_1_2;

	--
	-- @see header
	--
	procedure osif_call_2_1 (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		call_id       : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		arg0          : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		arg1          : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal ret0   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		done := False;

		case i_osif.step is
			when 0 =>
				o_osif.hw2sw_we <= '1';
				o_osif.hw2sw_data <= call_id;

				o_osif.step <= 1;

			when 1 =>
				if i_osif.hw2sw_full = '0' then
					o_osif.hw2sw_data <= arg0;

					o_osif.step <= 2;
				end if;

			when 2 =>
				if i_osif.hw2sw_full = '0' then
					o_osif.hw2sw_data <= arg1;

					o_osif.step <= 3;
				end if;

			when 3 =>
				if i_osif.hw2sw_full = '0' then
					o_osif.hw2sw_we <= '0';
					o_osif.sw2hw_re <= '1';

					o_osif.step <= 4;
				end if;

			when 4 =>
				if i_osif.sw2hw_empty = '0' then
					ret0 <= i_osif.sw2hw_data;
					o_osif.sw2hw_re <= '0';

					o_osif.step <= 5;
				end if;

			when others =>
				done := True;
				o_osif.step <= 0;
		end case;
	end procedure osif_call_2_1;

	--
	-- @see header
	--
	procedure osif_sem_post (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		osif_call_1_1(i_osif, o_osif, OSIF_CMD_SEM_POST, handle, result, done);
	end procedure osif_sem_post;

	--
	-- @see header
	--
	procedure osif_sem_wait (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		osif_call_1_1(i_osif, o_osif, OSIF_CMD_SEM_WAIT, handle, result, done);
	end procedure osif_sem_wait;

	--
	-- @see header
	--
	procedure osif_mutex_lock (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		osif_call_1_1(i_osif, o_osif, OSIF_CMD_MUTEX_LOCK, handle, result, done);
	end procedure osif_mutex_lock;

	--
	-- @see header
	--
	procedure osif_mutex_unlock (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		osif_call_1_1(i_osif, o_osif, OSIF_CMD_MUTEX_UNLOCK, handle, result, done);
	end procedure osif_mutex_unlock;

	--
	-- @see header
	--
	procedure osif_mutex_trylock (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		osif_call_1_1(i_osif, o_osif, OSIF_CMD_MUTEX_TRYLOCK, handle, result, done);
	end procedure osif_mutex_trylock;

	--
	-- @see header
	--
	procedure osif_cond_wait (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		cond_handle   : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		mutex_handle  : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		osif_call_2_1(i_osif, o_osif, OSIF_CMD_COND_WAIT, cond_handle, mutex_handle, result, done);
	end procedure osif_cond_wait;

	--
	-- @see header
	--
	procedure osif_cond_signal (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		osif_call_1_1(i_osif, o_osif, OSIF_CMD_COND_SIGNAL, handle, result, done);
	end procedure osif_cond_signal;

	--
	-- @see header
	--
	procedure osif_cond_broadcast (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		osif_call_1_1(i_osif, o_osif, OSIF_CMD_COND_BROADCAST, handle, result, done);
	end procedure osif_cond_broadcast;

	--
	-- @see header
	--
	procedure osif_mbox_put (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		word          : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		osif_call_2_1(i_osif, o_osif, OSIF_CMD_MBOX_PUT, handle, word, result, done);
	end procedure osif_mbox_put;

	--
	-- @see header
	--
	procedure osif_mbox_get (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal word   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		osif_call_1_1(i_osif, o_osif, OSIF_CMD_MBOX_GET, handle, word, done);
	end procedure osif_mbox_get;

	--
	-- @see header
	--
	procedure osif_mbox_tryput (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		word          : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		osif_call_2_1(i_osif, o_osif, OSIF_CMD_MBOX_TRYPUT, handle, word, result, done);
	end procedure osif_mbox_tryput;

	--
	-- @see header
	--
	procedure osif_mbox_tryget (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		handle        : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal word   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal result : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		osif_call_1_2(i_osif, o_osif, OSIF_CMD_MBOX_TRYGET, handle, word, result, done);
	end procedure osif_mbox_tryget;

	--
	-- @see header
	--
	procedure osif_get_init_data (
		signal i_osif : in  i_osif_t;
		signal o_osif : out o_osif_t;
		signal init   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable done : out boolean
	) is begin
		osif_call_0_1(i_osif, o_osif, OSIF_CMD_THREAD_GET_INIT_DATA, init, done);
	end procedure osif_get_init_data;

	--
	-- @see header
	--
	procedure osif_thread_exit (
		signal i_osif  : in  i_osif_t;
		signal o_osif  : out o_osif_t
	) is begin
		case i_osif.step is
			when 0 =>
				o_osif.hw2sw_we <= '1';
				o_osif.hw2sw_data <= OSIF_CMD_THREAD_EXIT;

				o_osif.step <= 1;

			when 1 =>
				if i_osif.hw2sw_full = '0' then
					o_osif.hw2sw_we <= '0';

					o_osif.step <= 2;
				end if;

			when others =>
		end case;
	end procedure osif_thread_exit;


-- changed to 39-bit addressing

	procedure memif_write_word (
		signal i_memif  : in  i_memif_t;
		signal o_memif  : out o_memif_t;
		addr            : in  std_logic_vector(63 downto 0);
		data            : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		variable done   : out boolean
	) is begin
		done := False;

		case i_memif.step is
			when 0 =>
				o_memif.hwt2mem_we <= '1';
				o_memif.hwt2mem_data <= MEMIF_CMD_WRITE & X"00000000000008";

				o_memif.step <= 1;

			when 1 =>
				if i_memif.hwt2mem_full = '0' then
				    -- 64bit aligned
					o_memif.hwt2mem_data <= addr(63 downto 3) & "000";

					o_memif.step <= 2;
				end if;

			when 2 =>
				if i_memif.hwt2mem_full = '0' then
					o_memif.hwt2mem_data <= data;

					o_memif.step <= 3;
				end if;

			when 3 =>
				if i_memif.hwt2mem_full = '0' then
					o_memif.hwt2mem_we <= '0';

					o_memif.step <= 4;
				end if;

			when others =>
					done := True;
					o_memif.step <= 0;

		end case;
	end procedure memif_write_word;
	
	
        procedure memif_read_word (
                signal i_memif  : in  i_memif_t;
                signal o_memif  : out o_memif_t;
                addr            : in  std_logic_vector(63 downto 0);
                signal data     : out  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
                variable done   : out boolean
        ) is begin
                done := False;

                case i_memif.step is
                    when 0 =>
                        o_memif.hwt2mem_we <= '1';
                        o_memif.hwt2mem_data <= MEMIF_CMD_READ & X"00000000000008";

                        o_memif.step <= 1;

                    when 1 =>
                        if i_memif.hwt2mem_full = '0' then
                            -- 64bit aligned
                            o_memif.hwt2mem_data <= addr(63 downto 3) & "000";

                            o_memif.step <= 2;
                        end if;

                    when 2 =>
                        if i_memif.hwt2mem_full = '0' then
                            o_memif.hwt2mem_we <= '0';
                            o_memif.mem2hwt_re <= '1';

                            o_memif.step <= 3;
                        end if;

                    when 3 =>
                        if i_memif.mem2hwt_empty = '0' then
                            data <= i_memif.mem2hwt_data;
                            o_memif.mem2hwt_re <= '0';

                            o_memif.step <= 4;
                        end if;

                    when others =>
                            done := True;
                            o_memif.step <= 0;

                end case;
        end procedure memif_read_word;
    

	procedure memif_write (
		signal i_ram    : in  i_ram_t;
		signal o_ram    : out o_ram_t;
		signal i_memif  : in  i_memif_t;
		signal o_memif  : out o_memif_t;
		src_addr        : in  std_logic_vector(63 downto 0);
		dst_addr        : in  std_logic_vector(63 downto 0);
		len             : in  std_logic_vector(31 downto 0);
		variable done   : out boolean
	) is
		variable to_border, to_remm : unsigned(C_MEMIF_LENGTH_WIDTH - 1 downto 0);
	begin
		done := False;

		case i_memif.step is
			when 0 =>
				o_memif.hwt2mem_we <= '0';
				
				o_ram.mem_addr <= unsigned(dst_addr(63 downto 3) & "000");
				o_ram.remm(31 downto 0) <= unsigned(len);

				o_ram.ram_addr <= unsigned(src_addr);

				o_memif.step <= 1;

			when 1 =>
				o_memif.hwt2mem_we <= '1';

				to_border := to_unsigned(C_MEMIF_CHUNK_BYTES, C_MEMIF_LENGTH_WIDTH) - i_ram.mem_addr(C_MEMIF_CHUNK_RANGE);
				to_remm := i_ram.remm(C_MEMIF_LENGTH_RANGE);
				if to_remm < to_border then
					o_memif.hwt2mem_data <= MEMIF_CMD_WRITE & std_logic_vector(to_remm);
				else
					o_memif.hwt2mem_data <= MEMIF_CMD_WRITE & std_logic_vector(to_border);
				end if;

				o_memif.step <= 2;

			when 2 =>
				if i_memif.hwt2mem_full = '0' then
					o_memif.hwt2mem_data <= std_logic_vector(i_ram.mem_addr);

					o_memif.step <= 3;
				end if;

			when 3 =>
				if i_memif.hwt2mem_full = '0' then
					o_memif.hwt2mem_we <= '0';

					o_memif.step <= 4;
				end if;

			when 4 =>
				o_ram.ram_addr <= i_ram.ram_addr + 1;

				o_memif.step <= 5;
				
			when 5 =>
				o_memif.hwt2mem_we <= '1';
				o_memif.hwt2mem_data <= i_ram.ram_data;
					
				o_ram.ram_addr <= i_ram.ram_addr + 1;
					
				o_memif.step <= 6;
				
			when 6 =>
				if i_memif.hwt2mem_full = '0' then
					o_memif.hwt2mem_data <= i_ram.ram_data;
					
					o_ram.ram_addr <= i_ram.ram_addr + 1;
				
					o_ram.mem_addr <= i_ram.mem_addr + 8;
					o_ram.remm <= i_ram.remm - 8;
					
					if (i_ram.mem_addr + 8) mod C_MEMIF_CHUNK_BYTES = 0 then
						o_memif.hwt2mem_we <= '0';
						
						o_ram.ram_addr <= i_ram.ram_addr - 1;
						
						o_memif.step <= 1;
					end if;
								
					if i_ram.remm - 8 = 0 then
						o_memif.hwt2mem_we <= '0';
						
						o_memif.step <= 8;
					end if;
				else
					o_memif.hwt2mem_we <= '0';
				
					o_ram.ram_addr <= i_ram.ram_addr - 2;
				
					o_memif.step <= 7;
				end if;
				
			when 7 =>
				if i_memif.hwt2mem_full = '0' then
					o_memif.step <= 4;
				end if;

			when others =>
				o_memif.hwt2mem_we <= '0';
			
				o_memif.step <= 0;
				done := True;

		end case;
	end procedure memif_write;

	
	procedure memif_read (
		signal i_ram    : in  i_ram_t;
		signal o_ram    : out o_ram_t;
		signal i_memif  : in  i_memif_t;
		signal o_memif  : out o_memif_t;
		src_addr        : in  std_logic_vector(63 downto 0);
		dst_addr        : in  std_logic_vector(63 downto 0);
		len             : in  std_logic_vector(31 downto 0);
		variable done   : out boolean
	) is
		variable to_border, to_remm : unsigned(C_MEMIF_LENGTH_WIDTH - 1 downto 0);
	begin
		done := False;

		case i_memif.step is
			when 0 =>
                                -- 64bit align
				o_ram.mem_addr <= unsigned(src_addr(63 downto 3) & "000");
				o_ram.remm(31 downto 0) <= unsigned(len);

				o_ram.ram_addr <= unsigned(dst_addr) - 1;

				o_memif.step <= 1;

			when 1 =>
				o_ram.ram_we <= '0';
			
				to_border := to_unsigned(C_MEMIF_CHUNK_BYTES, C_MEMIF_LENGTH_WIDTH) - i_ram.mem_addr(C_MEMIF_CHUNK_RANGE);
				to_remm := i_ram.remm(C_MEMIF_LENGTH_RANGE);
				if to_remm < to_border then
					o_memif.hwt2mem_we <= '1';
					o_memif.hwt2mem_data <= MEMIF_CMD_READ & std_logic_vector(to_remm);
				else
					o_memif.hwt2mem_we <= '1';
					o_memif.hwt2mem_data <= MEMIF_CMD_READ & std_logic_vector(to_border);
				end if;

				o_memif.step <= 2;

			when 2 =>
				if i_memif.hwt2mem_full = '0' then
					o_memif.hwt2mem_data <= std_logic_vector(i_ram.mem_addr);

					o_memif.step <= 3;
				end if;

			when 3 =>
				if i_memif.hwt2mem_full = '0' then
					o_memif.hwt2mem_we <= '0';
					o_memif.mem2hwt_re <= '1';
				
					o_memif.step <= 4;
				end if;

			when 4 =>
				if i_memif.mem2hwt_empty = '0' then
					o_ram.ram_we <= '1';
					o_ram.ram_data <= i_memif.mem2hwt_data;
					
					o_ram.ram_addr <= i_ram.ram_addr + 1;
					o_ram.mem_addr <= i_ram.mem_addr + 8;
					o_ram.remm <= i_ram.remm - 8;
					
					if (i_ram.mem_addr + 8) mod C_MEMIF_CHUNK_BYTES = 0 then
						o_memif.mem2hwt_re <= '0';
					
						o_memif.step <= 1;
					end if;
					
					if i_ram.remm - 8 = 0 then
						o_memif.mem2hwt_re <= '0';
						
						o_memif.step <= 5;
					end if;
				end if;
				
			when others =>
				o_ram.ram_we <= '0';
				
				o_memif.step <= 0;
				done := true;

		end case;
	end procedure memif_read;
	
end package body reconos_pkg;
