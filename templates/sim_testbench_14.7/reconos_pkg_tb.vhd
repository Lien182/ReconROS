--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        VHDL Package - ReconOS
--
--   project:      ReconOS
--   author:       Enno Lübbers, University of Paderborn
--                 Andreas Agne, University of Paderborn
--                 Christoph Rüthing, University of Paderborn
--   description:  The entire ReconOS package with type definitions and
--                 hardware OS services in VHDL
--
--   fixme:        osif read is not correct if it is not empty
--                 same problem might occur for write
--
-- ======================================================================


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library reconos_v3_01_a;
use reconos_v3_01_a.reconos_pkg.all;

package reconos_pkg_tb is

	-- == Constant definitions ============================================

	--
	-- General constants
	--
	--   C_CLK_PRD          - Clock period to simulate
	--   C_OSIF_ADDR_WIDTH  - Address width of the osif
	--   C_MEMIF_ADDR_WIDTH - Address width of memif
	--
	constant C_CLK_PRD : time := 10 ns;
	constant C_OSIF_ADDR_WIDTH : integer := 2;
	constant C_MEMIF_ADDR_WIDTH : integer := 6;
	constant C_SYSTEM_RAM_SIZE_WORDS : integer := 2048;


	-- == Type definitions ================================================

	--
	-- Type definitions of tb_osif_t and tb_o_osif_t
	--
	--   s_/m_ - fifo signals
	--
	type tb_i_osif_t is record
		hw2sw_data   : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		hw2sw_empty  : std_logic;

		sw2hw_full   : std_logic;
	end record;

	type tb_o_osif_t is record
		hw2sw_re   : std_logic;
		sw2hw_data : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		sw2hw_we   : std_logic;
	end record;

	--
	-- Type definitions of tb_i_memif_t and tb_o_memif_t
	--
	--   s_/m_ - fifo signals
	--
	type tb_i_memif_t is record
		hwt2mem_data   : std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		hwt2mem_empty  : std_logic;

		mem2hwt_full   : std_logic;
	end record;

	type tb_o_memif_t is record
		hwt2mem_re   : std_logic;
		mem2hwt_data : std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		mem2hwt_we   : std_logic;
	end record;


	-- == ReconOS testbench functions =====================================

	--
	-- Assigns signals to the osif record. This function must be called
	-- asynchronously in the main entity including the os-fsm.
	--
	--   tb_i_osif       - tb_i_osif_t record
	--   tb_o_osif       - tb_o_osif_t_record
	--   hw2sw_data   - OSIF_FIFO_Hw2Sw_Data
	--   hw2sw_empty  - OSIF_FIFO_Hw2Sw_Empty
	--   hw2sw_re     - OSIF_FIFO_Hw2Sw_RE
	--   sw2hw_data   - OSIF_FIFO_Sw2Hw_Data
	--   sw2hw_full   - OSIF_FIFO_Sw2Hw_Full
	--   sw2hw_we     - OSIF_FIFO_Sw2Hw_WE
	--
	procedure tb_osif_setup (
		signal tb_i_osif    : out tb_i_osif_t;
		signal tb_o_osif    : in  tb_o_osif_t;
		signal hw2sw_data   : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal hw2sw_empty  : in  std_logic;
		signal hw2sw_re     : out std_logic;
		signal sw2hw_data   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal sw2hw_full   : in  std_logic;
		signal sw2hw_we     : out std_logic
	);

	--	
	-- Resets the osif signals to a default state. This function should be called
	-- on reset of the os-fsm.
	--
	--   zb_o_osif - tb_o_osif_t record
	--
	procedure tb_osif_reset (
		signal tb_o_osif : out tb_o_osif_t
	);

	--
	-- Assigns signals to the memif record. This function must be called
	-- asynchronously in the main entity including the os-fsm.
	--
	--   tb_i_memif     - tb_i_memif_t record
	--   tb_o_memif     - tb_o_memif_t record
	--   hwt2mem_data   - MEMIF_FIFO_Hwt2Mem_Data
	--   hwt2mem_empty  - MEMIF_FIFO_Hwt2Mem_Empty
	--   hwt2mem_re     - MEMIF_FIFO_Hwt2Mem_RE
	--   mem2hwt_data   - MEMIF_FIFO_Mem2Hwt_Data
	--   mem2hwt_full   - MEMIF_FIFO_Mem2Hwt_Full
	--   mem2hwt_we     - MEMIF_FIFO_Mem2Hwt_WE
	--
	procedure tb_memif_setup (
		signal tb_i_memif     : out tb_i_memif_t;
		signal tb_o_memif     : in  tb_o_memif_t;
		signal hwt2mem_data   : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal hwt2mem_empty  : in  std_logic;
		signal hwt2mem_re     : out std_logic;
		signal mem2hwt_data   : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal mem2hwt_full   : in  std_logic;
		signal mem2hwt_we     : out std_logic
	);

	--
	-- Resets the memif signals to a default state. This function should be called
	-- on reset of the os-fsm.
	--
	--   tb_o_memif - tb_o_memif_t record
	--
	procedure tb_memif_reset (
		signal tb_o_memif : out tb_o_memif_t
	);

	--
	-- Sends the start command to the hardwarw thread.
	--
	--   tb_i_memif     - tb_i_memif_t record
	--   tb_o_memif     - tb_o_memif_t record
	--
	procedure tb_osif_thread_start (
		signal tb_i_osif : in  tb_i_osif_t;
		signal tb_o_osif : out tb_o_osif_t
	);

	--
	-- Reacts to an mbox get of the hardware thread.
	--
	--   tb_i_memif     - tb_i_memif_t record
	--   tb_o_memif     - tb_o_memif_t record
	--
	procedure tb_osif_mbox_get (
		signal tb_i_osif : in  tb_i_osif_t;
		signal tb_o_osif : out tb_o_osif_t;
		word             : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0)
	);

	--
	-- Reacts to an mbox put of the hardware thread.
	--
	--   tb_i_memif     - tb_i_memif_t record
	--   tb_o_memif     - tb_o_memif_t record
	--
	procedure tb_osif_mbox_put (
		signal tb_i_osif : in  tb_i_osif_t;
		signal tb_o_osif : out tb_o_osif_t
	);


end package reconos_pkg_tb;

package body reconos_pkg_tb is

	-- == Util functions ==================================================

	function hex (
		slv : std_logic_vector
	) return string is
		variable len  : integer;
		variable str  : string(1 to 16);
	begin
		len := (slv'left + 1) / 4;

		for i in 0 to len - 1 loop
			case slv(4 * i + 3 downto 4 * i) is
				when x"0" => str(len - i) := '0';
				when x"1" => str(len - i) := '1';
				when x"2" => str(len - i) := '2';
				when x"3" => str(len - i) := '3';
				when x"4" => str(len - i) := '4';
				when x"5" => str(len - i) := '5';
				when x"6" => str(len - i) := '6';
				when x"7" => str(len - i) := '7';
				when x"8" => str(len - i) := '8';
				when x"9" => str(len - i) := '9';
				when x"A" => str(len - i) := 'a';
				when x"B" => str(len - i) := 'b';
				when x"C" => str(len - i) := 'c';
				when x"D" => str(len - i) := 'd';
				when x"E" => str(len - i) := 'e';
				when x"F" => str(len - i) := 'f';
				when others => str(len - i) := '?';
			end case;
		end loop;

		return str(1 to len);
	end;


	--
	-- @see header
	--
	procedure tb_osif_setup (
		signal tb_i_osif    : out tb_i_osif_t;
		signal tb_o_osif    : in  tb_o_osif_t;
		signal hw2sw_data   : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal hw2sw_empty  : in  std_logic;
		signal hw2sw_re     : out std_logic;
		signal sw2hw_data   : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		signal sw2hw_full   : in  std_logic;
		signal sw2hw_we     : out std_logic
	) is begin
		tb_i_osif.hw2sw_data   <= hw2sw_data;
		tb_i_osif.hw2sw_empty  <= hw2sw_empty;
		hw2sw_re               <= tb_o_osif.hw2sw_re;

		sw2hw_data             <= tb_o_osif.sw2hw_data;
		tb_i_osif.sw2hw_full   <= sw2hw_full;
		sw2hw_we               <= tb_o_osif.sw2hw_we;
	end procedure tb_osif_setup;

	--
	-- @see header
	--
	procedure tb_osif_reset (
		signal tb_o_osif : out tb_o_osif_t
	) is begin
		tb_o_osif.hw2sw_re   <= '0';
		tb_o_osif.sw2hw_data <= (others => '0');
		tb_o_osif.sw2hw_we   <= '0';
	end procedure tb_osif_reset;

	--
	-- @see header
	--
	procedure tb_memif_setup (
		signal tb_i_memif     : out tb_i_memif_t;
		signal tb_o_memif     : in  tb_o_memif_t;
		signal hwt2mem_data   : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal hwt2mem_empty  : in  std_logic;
		signal hwt2mem_re     : out std_logic;
		signal mem2hwt_data   : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
		signal mem2hwt_full   : in  std_logic;
		signal mem2hwt_we     : out std_logic
	) is begin
		tb_i_memif.hwt2mem_data   <= hwt2mem_data;
		tb_i_memif.hwt2mem_empty  <= hwt2mem_empty;
		hwt2mem_re                <= tb_o_memif.hwt2mem_re;

		mem2hwt_data              <= tb_o_memif.mem2hwt_data;
		tb_i_memif.mem2hwt_full   <= mem2hwt_full;
		mem2hwt_we                <= tb_o_memif.mem2hwt_we;
	end procedure tb_memif_setup;

	--
	-- @see header
	--
	procedure tb_memif_reset (
		signal tb_o_memif : out tb_o_memif_t
	) is begin
		tb_o_memif.hwt2mem_re   <= '0';
		tb_o_memif.mem2hwt_data <= (others => '0');
		tb_o_memif.mem2hwt_we   <= '0';
	end procedure tb_memif_reset;

	--
	-- @see header
	--
	procedure tb_osif_thread_start (
		signal tb_i_osif : in  tb_i_osif_t;
		signal tb_o_osif : out tb_o_osif_t
	) is begin
		tb_o_osif.sw2hw_data <= OSIF_SIGNAL_THREAD_START;
		tb_o_osif.sw2hw_we <= '1';

		wait for C_CLK_PRD;
	end procedure tb_osif_thread_start;

	--
	-- @see header
	--
	procedure tb_osif_mbox_get (
		signal tb_i_osif : in  tb_i_osif_t;
		signal tb_o_osif : out tb_o_osif_t;
		word             : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0)
	) is
		variable cmd    : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable handle : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	begin
		tb_o_osif.hw2sw_re <= '1';

		wait until tb_i_osif.hw2sw_empty = '0';
		wait for C_CLK_PRD;

		cmd := tb_i_osif.hw2sw_data;
		assert cmd = OSIF_CMD_MBOX_GET
		  report "[ReconOS Testbench] " &
		         "ERROR: Wrong command received: " &
		         "0x" & hex(cmd) & " but expected 0x" & hex(OSIF_CMD_MBOX_GET)
		  severity failure;

		wait until tb_i_osif.hw2sw_empty = '0';
		wait for C_CLK_PRD;

		tb_o_osif.hw2sw_re <= '0';

		tb_o_osif.sw2hw_we <= '1';
		tb_o_osif.sw2hw_data <= word;

		handle := tb_i_osif.hw2sw_data;
		report "[ReconOS Testbench] " &
		       "OSIF-Call mbox_get (0x" & hex(handle) & "): 0x" & hex(word);

		wait until tb_i_osif.sw2hw_full = '0';
		wait for C_CLK_PRD;

		tb_o_osif.sw2hw_we <= '0';

		wait for C_CLK_PRD;
	end procedure tb_osif_mbox_get;

	--
	-- @see header
	--
	procedure tb_osif_mbox_put(
		signal tb_i_osif : in  tb_i_osif_t;
		signal tb_o_osif : out tb_o_osif_t
	) is
		variable cmd    : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable handle : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		variable word   : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	begin
		tb_o_osif.hw2sw_re <= '1';

		wait until tb_i_osif.hw2sw_empty = '0';
		wait for C_CLK_PRD;

		cmd := tb_i_osif.hw2sw_data;
		assert cmd = OSIF_CMD_MBOX_PUT
		  report "[ReconOS Testbench] " &
		         "ERROR: Wrong command received: " &
		         "0x" & hex(cmd) & " but expected 0x" & hex(OSIF_CMD_MBOX_PUT)
		  severity failure;

		wait until tb_i_osif.hw2sw_empty = '0';
		wait for C_CLK_PRD;

		handle := tb_i_osif.hw2sw_data;

		wait until tb_i_osif.hw2sw_empty = '0';
		wait for C_CLK_PRD;

		word := tb_i_osif.hw2sw_data;
		report "[ReconOS Testbench] " &
		       "OSIF-Call mbox_put (0x" & hex(handle) & "): 0x" & hex(word);

		tb_o_osif.hw2sw_re <= '0';

		tb_o_osif.sw2hw_we <= '1';
		tb_o_osif.sw2hw_data <= word;

		wait until tb_i_osif.sw2hw_full = '0';
		wait for C_CLK_PRD;

		tb_o_osif.sw2hw_we <= '0';

		wait for C_CLK_PRD;
	end procedure tb_osif_mbox_put;


end package body reconos_pkg_tb;