--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - PROC_CONTROL - Proc control implementation
--
--   project:      ReconOS
--   author:       Christoph Rüthing, University of Paderborn
--   description:  The Proc Conrol is used to control the different
--                 hardware parts through a single interface. It allows
--                 to reset the HWTs seperately and asynchronously and
--                 configures the MMU. To provide its functionality it
--                 has several registers.
--                 Register Definition (as seen from Bus):
--                   Reg0: Number of HWT-Slots (OSIFS) - Read only
--                   # all MMU related stuff
--                   Reg1: PGD address - Read / Write
--                   Reg2: Page fault address (only valid on interrupt)
--                         read to clear interrupt, write after handling
--                   Reg3: TLB hits - Read only
--                   Reg4: TLB misses - Read only
--                   # resets
--                   Reg5: ReconOS reset (reset everything) - Write only
--                   Reg6: HWT reset (multiple registers) - Write only
--                         | x , x-1, ... | x-32 , x-33, ... 0 |
--                   Reg7: HWT signal - Write only
--                         | x , x-1, ... | x-32 , x-33, ... 0 |
--
--                   Page fault handling works the following:
--                     1.) MMU raises MMU_Pgf
--                     2.) Proc control raises PROC_Pgf_Int
--                     3.) CPU clears interrupt by reading register 2
--                     4.) CPU handles page fault and acknowledges this
--                         by writing to register 2
--                     5.) Proc control informs MMU by raising MMU_Ready
--                         that the page fault has been handled
--
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_misc.all;

<<if TOOL=="ise">>
library proc_common_v3_00_a;
use proc_common_v3_00_a.proc_common_pkg.all;
<<end if>>

entity user_logic is
	generic (
		-- Proc Control parameters
		C_NUM_HWTS     : integer := 1;
	
		-- Bus protocol parameters
		C_NUM_REG      : integer   := 1;
		C_SLV_DWIDTH   : integer   := 32
	);
  port (
		-- PROC control ports
		PROC_Hwt_Rst     : out std_logic_vector(C_NUM_HWTS - 1 downto 0);
		PROC_Hwt_Signal  : out std_logic_vector(C_NUM_HWTS - 1 downto 0);
		PROC_Sys_Rst     : out std_logic;
		PROC_Pgf_Int     : out std_logic;

		-- MMU related ports
		MMU_Pgf          : in  std_logic;
		MMU_Fault_Addr   : in  std_logic_vector(31 downto 0);
		MMU_Retry        : out std_logic;
		MMU_Pgd          : out std_logic_vector(31 downto 0);
		MMU_Tlb_Hits     : in  std_logic_vector(31 downto 0);
		MMU_Tlb_Misses   : in  std_logic_vector(31 downto 0);

		-- Bus protocol ports
		Bus2IP_Clk       : in  std_logic;
		Bus2IP_Resetn    : in  std_logic;
		Bus2IP_Data      : in  std_logic_vector(C_SLV_DWIDTH-1 downto 0);
		Bus2IP_BE        : in  std_logic_vector(C_SLV_DWIDTH/8-1 downto 0);
		Bus2IP_RdCE      : in  std_logic_vector(C_NUM_REG-1 downto 0);
		Bus2IP_WrCE      : in  std_logic_vector(C_NUM_REG-1 downto 0);
		IP2Bus_Data      : out std_logic_vector(C_SLV_DWIDTH-1 downto 0);
		IP2Bus_RdAck     : out std_logic;
		IP2Bus_WrAck     : out std_logic;
		IP2Bus_Error     : out std_logic
	);
end entity user_logic;


architecture imp of user_logic is

	constant NUM_HWT_REGS : integer := ((C_NUM_HWTS - 1) / C_SLV_DWIDTH) + 1;

	type PGF_INT_STATE_TYPE is (WAIT_PGF, WAIT_CLEAR, WAIT_READY);
	signal pgf_int_state : PGF_INT_STATE_TYPE;

	type SYS_RESET_STATE_TYPE is (WAIT_RST, PERF_RST);
	signal sys_reset_state   : SYS_RESET_STATE_TYPE;
	signal sys_reset_counter : std_logic_vector(3 downto 0);

	signal pgd                 : std_logic_vector(31 downto 0);
	signal fault_addr          : std_logic_vector(31 downto 0);
	signal tlb_hits            : std_logic_vector(31 downto 0);
	signal tlb_misses          : std_logic_vector(31 downto 0);
	signal sys_reset           : std_logic;
	signal hwt_reset           : std_logic_vector(C_NUM_HWTS - 1 downto 0);
	signal hwt_signal          : std_logic_vector(C_NUM_HWTS - 1 downto 0);

	signal hwt_reset_reg       : std_logic_vector(NUM_HWT_REGS * C_SLV_DWIDTH - 1 downto 0);
	signal hwt_signal_reg      : std_logic_vector(NUM_HWT_REGS * C_SLV_DWIDTH - 1 downto 0);

	-- Signals for user logic slave model s/w accessible register
	signal slv_reg_write_sel   : std_logic_vector(C_NUM_REG - 1 downto 0);
	signal slv_reg_read_sel    : std_logic_vector(C_NUM_REG - 1 downto 0);
	signal slv_ip2bus_data     : std_logic_vector(C_SLV_DWIDTH - 1 downto 0);
	signal slv_read_ack        : std_logic;
	signal slv_write_ack       : std_logic;
	
	signal clk : std_logic;
	signal rst : std_logic;

begin

	clk <= Bus2IP_Clk;
	rst <= not Bus2IP_Resetn;
	
	-- Dive bus signals
	IP2Bus_Data  <= slv_ip2bus_data;

	slv_reg_write_sel <= Bus2IP_WrCE;
	slv_reg_read_sel  <= Bus2IP_RdCE;
	slv_read_ack      <= or_reduce(Bus2IP_RdCE);
	slv_write_ack     <= or_reduce(Bus2IP_WrCE);
	
	IP2Bus_WrAck <= slv_write_ack;
	IP2Bus_RdAck <= slv_read_ack;
	IP2Bus_Error <= '0';


	-- proc realted signals
	fault_addr   <= MMU_Fault_Addr;
	tlb_hits     <= MMU_Tlb_Hits;
	tlb_misses   <= MMU_Tlb_Misses;

	PROC_Hwt_Rst <= hwt_reset;
	PROC_Hwt_Signal <= hwt_signal;
	PROC_Sys_Rst <= sys_reset;

	hwt_reset  <= hwt_reset_reg(C_NUM_HWTS - 1 downto 0);
	hwt_signal <= hwt_signal_reg(C_NUM_HWTS - 1 downto 0);

	MMU_Pgd <= pgd;


	-- page fault handlig (for details see description above)
	pgf_int_proc : process(clk,rst) is
	begin
		if rst = '1' or sys_reset = '1' then
			PROC_Pgf_Int <= '0';
			pgf_int_state <= WAIT_PGF;
		elsif rising_edge(clk) then
			MMU_Retry <= '0';

			case pgf_int_state is
				when WAIT_PGF =>
					if MMU_Pgf = '1' then
						PROC_Pgf_Int <= '1';
						pgf_int_state <= WAIT_CLEAR;
					end if;

				when WAIT_CLEAR =>
					-- reading from page_fault_addr register
					if slv_reg_read_sel(C_NUM_REG - 3) = '1' then
						PROC_Pgf_Int <= '0';
						pgf_int_state <= WAIT_READY;
					end if;

				when WAIT_READY =>
					-- writing to page_fault_addr register
					if slv_reg_write_sel(C_NUM_REG - 3) = '1' then
						MMU_Retry <= '1';
						pgf_int_state <= WAIT_PGF;
					end if;
			end case;
		end if;
	end process pgf_int_proc;


	hwt_reset_proc : process(clk,rst) is
	begin
		if rst = '1' or sys_reset = '1' then
			hwt_reset_reg <= (others => '1');
		elsif rising_edge(clk) then
			-- writing to hwt_reset
			-- ignoring byte enable
			for i in 0 to NUM_HWT_REGS - 1 loop
				if slv_reg_write_sel(NUM_HWT_REGS * 2 - i - 1) = '1' then
					hwt_reset_reg(32 * i + 31 downto 32 * i) <= Bus2IP_Data;
				end if;
			end loop;
		end if;
	end process hwt_reset_proc;


	hwt_signal_proc : process(clk,rst) is
	begin
		if rst = '1' or sys_reset = '1' then
			hwt_signal_reg <= (others => '0');
		elsif rising_edge(clk) then
			-- writing to hwt_signal
			-- ignoring byte enable
			for i in 0 to NUM_HWT_REGS - 1 loop
				if slv_reg_write_sel(NUM_HWT_REGS - i - 1) = '1' then
					hwt_signal_reg(32 * i + 31 downto 32 * i) <= Bus2IP_Data;
				end if;
			end loop;
		end if;
	end process hwt_signal_proc;


	sys_reset_proc : process(clk,rst) is
	begin
		if rst = '1' then
			sys_reset <= '1';
			sys_reset_state <= PERF_RST;
			sys_reset_counter <= (others => '0');
		elsif rising_edge(clk) then
			sys_reset <= '0';

			case sys_reset_state is
				when WAIT_RST =>
					if slv_reg_write_sel(C_NUM_REG - 6) = '1' then
						sys_reset_state <= PERF_RST;

						sys_reset <= '1';
						sys_reset_counter <= (others => '0');
					end if;
				when PERF_RST =>
					sys_reset <= '1';
					sys_reset_counter <= sys_reset_counter + 1;

					if and_reduce(sys_reset_counter) = '1' then
						sys_reset_state <= WAIT_RST;
					end if;
			end case;
		end if;
	end process sys_reset_proc;


	pgd_proc : process(clk,rst) is
	begin
		if rst = '1' or sys_reset = '1' then
			pgd <= (others => '0');
		else
			if rising_edge(clk) then
				if slv_reg_write_sel(C_NUM_REG - 2) = '1' then
					pgd <= Bus2IP_Data;
				end if;
			end if;
		end if;
	end process pgd_proc;


	bus_reg_read_proc : process(slv_reg_read_sel) is
	begin
		case slv_reg_read_sel(C_NUM_REG - 1 downto C_NUM_REG - 6) is
			when "100000" => slv_ip2bus_data <= CONV_STD_LOGIC_VECTOR(C_NUM_HWTS, C_SLV_DWIDTH);
			when "010000" => slv_ip2bus_data <= pgd;
			when "001000" => slv_ip2bus_data <= fault_addr;
			when "000100" => slv_ip2bus_data <= tlb_hits;
			when "000010" => slv_ip2bus_data <= tlb_misses;
			when others => slv_ip2bus_data <= (others => '0');
		end case;
	end process bus_reg_read_proc;

end imp;
