--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - INTC - INTC implementation
--
--   project:      ReconOS
--   author:       Christoph Rüthing, University of Paderborn
--   description:  A simple interrupt controller with variable number of
--                 inputs to connect the RECONOS_AXI_FIFO-interrupts to
--                 the processor.
--
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_misc.all;

entity reconos_osif_intc_user_logic is
	generic (
		-- INTC parameters
		C_NUM_INTERRUPTS   : integer := 1;
	
		-- Bus protocol parameters
		C_NUM_REG      : integer   := 1;
		C_SLV_DWIDTH   : integer   := 32
	);
  port (
		OSIF_INTC_in   : in  std_logic_vector(C_NUM_INTERRUPTS - 1 downto 0);
		OSIF_INTC_out  : out std_logic;
  
		-- Bus protocol ports
		Bus2IP_Clk      : in  std_logic;
		Bus2IP_Resetn   : in  std_logic;
		Bus2IP_Data     : in  std_logic_vector(C_SLV_DWIDTH-1 downto 0);
		Bus2IP_BE       : in  std_logic_vector(C_SLV_DWIDTH/8-1 downto 0);
		Bus2IP_RdCE     : in  std_logic_vector(C_NUM_REG-1 downto 0);
		Bus2IP_WrCE     : in  std_logic_vector(C_NUM_REG-1 downto 0);
		IP2Bus_Data     : out std_logic_vector(C_SLV_DWIDTH-1 downto 0);
		IP2Bus_RdAck    : out std_logic;
		IP2Bus_WrAck    : out std_logic;
		IP2Bus_Error    : out std_logic
	);
end entity reconos_osif_intc_user_logic;


architecture imp of reconos_osif_intc_user_logic is
	
	-- padding to fill unused interrupts in interrupt_reg
	signal pad   : std_logic_vector(C_SLV_DWIDTH * C_NUM_REG - C_NUM_INTERRUPTS - 1 downto 0);

	signal interrupt_masked     : std_logic_vector(C_NUM_INTERRUPTS - 1 downto 0);
	signal interrupt_enable     : std_logic_vector(C_NUM_INTERRUPTS - 1 downto 0);
	signal interrupt_reg        : std_logic_vector(C_NUM_REG * 32 - 1 downto 0);
	signal interrupt_enable_reg : std_logic_vector(C_NUM_REG * 32 - 1 downto 0);

	-- Signals for user logic slave model s/w accessible register example
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
	pad <= (others => '0');
	
	interrupt_enable <= interrupt_enable_reg(C_NUM_INTERRUPTS - 1 downto 0);

	interrupt_masked <= OSIF_INTC_in and interrupt_enable;
	-- interrupt register only contains enabled interrupts
	interrupt_reg <= pad & interrupt_masked;
	OSIF_INTC_Out <= or_reduce(interrupt_masked);

	--    Bus2IP_WrCE/Bus2IP_RdCE   Memory Mapped Register
	--                     "1000"   C_BASEADDR + 0x0
	--                     "0100"   C_BASEADDR + 0x4
	--                     "0010"   C_BASEADDR + 0x8
	--                     "0001"   C_BASEADDR + 0xC

	-- Example code to drive IP to Bus signals
	IP2Bus_Data  <= slv_ip2bus_data when slv_read_ack = '1' else (others => '0');

	slv_reg_write_sel <= Bus2IP_WrCE;
	slv_reg_read_sel  <= Bus2IP_RdCE;
	slv_write_ack     <= or_reduce(Bus2IP_WrCE);
	slv_read_ack      <= or_reduce(Bus2IP_RdCE);
	
	IP2Bus_WrAck <= slv_write_ack;
	IP2Bus_RdAck <= slv_read_ack;
	IP2Bus_Error <= '0';


	int_enable_reg_proc : process(clk,rst) is
	begin
		if rst = '1' then
			interrupt_enable_reg <= (others => '0');
		elsif rising_edge(clk) then
			for i in 0 to C_NUM_REG - 1 loop
				if slv_reg_write_sel(C_NUM_REG - 1 - i) = '1' then
					interrupt_enable_reg(32 * i + 31 downto 32 * i) <= Bus2IP_Data;
				end if;
			end loop;
		end if;
	end process int_enable_reg_proc;

	bus_read_reg_proc : process(slv_reg_read_sel) is
	begin
		for i in 0 to C_NUM_REG - 1 loop
			if slv_reg_read_sel(C_NUM_REG - 1 - i) = '1' then
				slv_ip2bus_data <= interrupt_reg(32 * i + 31 downto 32 * i);
			end if;
		end loop;
	end process bus_read_reg_proc;

end imp;
