<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library reconos_v3_01_a;
use reconos_v3_01_a.reconos_fifo_sync;
use reconos_v3_01_a.reconos_pkg.all;
use reconos_v3_01_a.reconos_pkg_tb.all;

entity testbench is
end testbench;

architecture testbench of testbench is	
	signal sys_clk : std_logic;
	signal hwt_clk : std_logic;
	signal sys_rst : std_logic;

	signal osif_sw2hw_s_data   : std_logic_vector(31 downto 0);
	signal osif_sw2hw_s_fill   : std_logic_vector(C_OSIF_ADDR_WIDTH downto 0);
	signal osif_sw2hw_s_empty  : std_logic;
	signal osif_sw2hw_s_re     : std_logic;
	signal osif_sw2hw_m_data   : std_logic_vector(31 downto 0);
	signal osif_sw2hw_m_remm   : std_logic_vector(C_OSIF_ADDR_WIDTH downto 0);
	signal osif_sw2hw_m_full   : std_logic;
	signal osif_sw2hw_m_we     : std_logic;

	signal osif_hw2sw_s_data   : std_logic_vector(31 downto 0);
	signal osif_hw2sw_s_fill   : std_logic_vector(C_OSIF_ADDR_WIDTH downto 0);
	signal osif_hw2sw_s_empty  : std_logic;
	signal osif_hw2sw_s_re     : std_logic;
	signal osif_hw2sw_m_data   : std_logic_vector(31 downto 0);
	signal osif_hw2sw_m_remm   : std_logic_vector(C_OSIF_ADDR_WIDTH downto 0);
	signal osif_hw2sw_m_full   : std_logic;
	signal osif_hw2sw_m_we     : std_logic;

	signal memif_mem2hwt_s_data   : std_logic_vector(31 downto 0);
	signal memif_mem2hwt_s_fill   : std_logic_vector(C_MEMIF_ADDR_WIDTH downto 0);
	signal memif_mem2hwt_s_empty  : std_logic;
	signal memif_mem2hwt_s_re     : std_logic;
	signal memif_mem2hwt_m_data   : std_logic_vector(31 downto 0);
	signal memif_mem2hwt_m_remm   : std_logic_vector(C_MEMIF_ADDR_WIDTH downto 0);
	signal memif_mem2hwt_m_full   : std_logic;
	signal memif_mem2hwt_m_we     : std_logic;
	
	signal memif_hwt2mem_s_data   : std_logic_vector(31 downto 0);
	signal memif_hwt2mem_s_fill   : std_logic_vector(C_MEMIF_ADDR_WIDTH downto 0);
	signal memif_hwt2mem_s_empty  : std_logic;
	signal memif_hwt2mem_s_re     : std_logic;
	signal memif_hwt2mem_m_data   : std_logic_vector(31 downto 0);
	signal memif_hwt2mem_m_remm   : std_logic_vector(C_MEMIF_ADDR_WIDTH downto 0);
	signal memif_hwt2mem_m_full   : std_logic;
	signal memif_hwt2mem_m_we     : std_logic;

	signal hwt_signal : std_logic;
	
	type mem_type is array (0 to C_SYSTEM_RAM_SIZE_WORDS - 1)
	                       of std_logic_vector(31 downto 0);
								  
	function init_mem return mem_type is
		variable mem : mem_type;
	begin
		for i in 0 to C_SYSTEM_RAM_SIZE_WORDS - 1 loop
			mem(i) := std_logic_vector(to_unsigned(C_SYSTEM_RAM_SIZE_WORDS - i, 32));
		end loop;
		
		return mem;
	end function init_mem;
	
	signal mem : mem_type := init_mem;

	signal mem_state : integer := 0;
	signal mem_addr  : unsigned(31 downto 0) := (others => '0');
	signal mem_count : unsigned(C_MEMIF_LENGTH_WIDTH - 1 downto 0) := (others => '0');
	signal mem_op    : std_logic_vector(C_MEMIF_OP_WIDTH - 1 downto 0) := (others => '0');
begin

	sys_clk_process :process
   begin
		sys_clk <= '0';
		wait for C_CLK_PRD / 2;
		sys_clk <= '1';
		wait for C_CLK_PRD / 2;
	end process;
	
	hwt_clk_process :process
   begin
		hwt_clk <= '0';
		wait for C_CLK_PRD;
		hwt_clk <= '1';
		wait for C_CLK_PRD;
	end process;
	

	osif_sw2hw : entity reconos_v3_01_a.reconos_fifo_sync
		generic map (
			C_FIFO_DATA_WIDTH => 32,
			C_FIFO_ADDR_WIDTH => C_OSIF_ADDR_WIDTH
		)

		port map (
			FIFO_S_Data => osif_sw2hw_s_data,
			FIFO_S_Fill => osif_sw2hw_s_fill,
			FIFO_S_Empty => osif_sw2hw_s_empty,
			FIFO_S_RE => osif_sw2hw_s_re,
			FIFO_M_Data => osif_sw2hw_m_data,
			FIFO_M_Remm => osif_sw2hw_m_remm,
			FIFO_M_Full => osif_sw2hw_m_full,
			FIFO_M_WE => osif_sw2hw_m_we,
			FIFO_Clk => sys_clk,
			FIFO_Rst => sys_rst
		);

	osif_hw2sw : entity reconos_v3_01_a.reconos_fifo_sync
		generic map (
			C_FIFO_DATA_WIDTH => 32,
			C_FIFO_ADDR_WIDTH => C_OSIF_ADDR_WIDTH
		)

		port map (
			FIFO_S_Data => osif_hw2sw_s_data,
			FIFO_S_Fill => osif_hw2sw_s_fill,
			FIFO_S_Empty => osif_hw2sw_s_empty,
			FIFO_S_RE => osif_hw2sw_s_re,
			FIFO_M_Data => osif_hw2sw_m_data,
			FIFO_M_Remm => osif_hw2sw_m_remm,
			FIFO_M_Full => osif_hw2sw_m_full,
			FIFO_M_WE => osif_hw2sw_m_we,
			FIFO_Clk => sys_clk,
			FIFO_Rst => sys_rst
		);

	memif_mem2hwt : entity reconos_v3_01_a.reconos_fifo_sync
		generic map (
			C_FIFO_DATA_WIDTH => 32,
			C_FIFO_ADDR_WIDTH => C_MEMIF_ADDR_WIDTH
		)

		port map (
			FIFO_S_Data => memif_mem2hwt_s_data,
			FIFO_S_Fill => memif_mem2hwt_s_fill,
			FIFO_S_Empty => memif_mem2hwt_s_empty,
			FIFO_S_RE => memif_mem2hwt_s_re,
			FIFO_M_Data => memif_mem2hwt_m_data,
			FIFO_M_Remm => memif_mem2hwt_m_remm,
			FIFO_M_Full => memif_mem2hwt_m_full,
			FIFO_M_WE => memif_mem2hwt_m_we,
			FIFO_Clk => sys_clk,
			FIFO_Rst => sys_rst
		);

	memif_hwt2mem : entity reconos_v3_01_a.reconos_fifo_sync
		generic map (
			C_FIFO_DATA_WIDTH => 32,
			C_FIFO_ADDR_WIDTH => C_MEMIF_ADDR_WIDTH
		)

		port map (
			FIFO_S_Data => memif_hwt2mem_s_data,
			FIFO_S_Fill => memif_hwt2mem_s_fill,
			FIFO_S_Empty => memif_hwt2mem_s_empty,
			FIFO_S_RE => memif_hwt2mem_s_re,
			FIFO_M_Data => memif_hwt2mem_m_data,
			FIFO_M_Remm => memif_hwt2mem_m_remm,
			FIFO_M_Full => memif_hwt2mem_m_full,
			FIFO_M_WE => memif_hwt2mem_m_we,
			FIFO_Clk => sys_clk,
			FIFO_Rst => sys_rst
		);

	hwt : entity work.rt_<<THREAD>>
		port map (
			OSIF_Sw2Hw_Data => osif_sw2hw_s_data,
			OSIF_Sw2Hw_Empty => osif_sw2hw_s_empty,
			OSIF_Sw2Hw_RE => osif_sw2hw_s_re,
			OSIF_Hw2Sw_Data => osif_hw2sw_m_data,
			OSIF_Hw2Sw_Full => osif_hw2sw_m_full,
			OSIF_Hw2Sw_WE => osif_hw2sw_m_we,
			MEMIF_Mem2Hwt_Data => memif_mem2hwt_s_data,
			MEMIF_Mem2Hwt_Empty => memif_mem2hwt_s_empty,
			MEMIF_Mem2Hwt_RE => memif_mem2hwt_s_re,
			MEMIF_Hwt2Mem_Data => memif_hwt2mem_m_data,
			MEMIF_Hwt2Mem_Full => memif_hwt2mem_m_full,
			MEMIF_Hwt2Mem_WE => memif_hwt2mem_m_we,
			HWT_Clk => sys_clk,
			HWT_Rst => sys_rst,
			HWT_Signal => hwt_signal
		);
		
	stimulus : entity work.stimulus
		port map (
			OSIF_Hw2Sw_Data => osif_hw2sw_s_data,
			OSIF_Hw2Sw_Empty => osif_hw2sw_s_empty,
			OSIF_Hw2Sw_RE => osif_hw2sw_s_re,
			OSIF_Sw2Hw_Data => osif_sw2hw_m_data,
			OSIF_Sw2Hw_Full => osif_sw2hw_m_full,
			OSIF_Sw2Hw_WE => osif_sw2hw_m_we,
			SYS_Rst => sys_rst,
			HWT_Signal => hwt_signal
		);


	memif_stub : process(sys_clk,sys_rst)
	begin
		if sys_rst = '1' then
			mem_state <= 0;
			memif_hwt2mem_s_re <= '0';
			memif_mem2hwt_m_we <= '0';
		elsif rising_edge(sys_clk) then
			case mem_state is
				when 0 =>
					memif_mem2hwt_m_we <= '0';
					memif_hwt2mem_s_re <= '1';

					mem_state <= 1;

				when 1 =>
					if memif_hwt2mem_s_empty = '0' then
						mem_op <= memif_hwt2mem_s_data(C_MEMIF_DATA_WIDTH - 1 downto C_MEMIF_DATA_WIDTH - C_MEMIF_OP_WIDTH);
						mem_count <= unsigned(memif_hwt2mem_s_data(C_MEMIF_LENGTH_WIDTH - 1 downto 0));

						mem_state <= 2;
					end if;

				when 2 =>
					if memif_hwt2mem_s_empty = '0' then
						mem_addr <= unsigned(memif_hwt2mem_s_data);

						memif_hwt2mem_s_re <= '0';
						mem_state <= 3;
					end if;

				when 3 =>
					case mem_op is
						when MEMIF_CMD_WRITE =>
							memif_hwt2mem_s_re <= '1';

						when others =>
					end case;

					mem_state <= 4;

				when 4 =>
					case mem_op is
						when MEMIF_CMD_READ =>
							if memif_mem2hwt_m_full = '0' then
								memif_mem2hwt_m_data <= mem(to_integer(mem_addr / 4));
								memif_mem2hwt_m_we <= '1';

								mem_addr <= mem_addr + 4;
								mem_count <= mem_count - 4;

								if mem_count - 4 = 0 then
									mem_state <= 5;
								end if;
							end if;

						when MEMIF_CMD_WRITE =>
							if memif_hwt2mem_s_empty = '0' then
								mem(to_integer(mem_addr / 4)) <= memif_hwt2mem_s_data;

								mem_addr <= mem_addr + 4;
								mem_count <= mem_count - 4;

								if mem_count - 4 = 0 then
									memif_hwt2mem_s_re <= '0';

									mem_state <= 0;
								end if;
							end if;

						when others =>
					end case;
					
				when 5 => 
					if memif_mem2hwt_m_full = '0' then
						memif_mem2hwt_m_we <= '0';
						
						mem_state <= 0;
					end if;

				when others =>
			end case;
		end if;

	end process memif_stub;

end testbench;

