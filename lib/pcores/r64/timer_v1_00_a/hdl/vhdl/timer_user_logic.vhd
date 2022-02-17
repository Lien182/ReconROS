--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - Timer
--
--   project:      ReconOS
--   author:       Christoph Rüthing, University of Paderborn
--   description:  A simple timer providing a accurate time for both
--                 hardware and software via direct signals and memory
--                 mapped registers on the AXI-interface.
--
-- ======================================================================


<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity timer_user_logic is
	generic (
		-- Bus protocol parameters
		C_NUM_REG      : integer   := 1;
		C_SLV_DWIDTH   : integer   := 32
	);
	port (
		-- Timer ports
		T_COUNTER   : out std_logic_vector(31 downto 0);
		T_RST       : in  std_logic;

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
end entity timer_user_logic;

architecture implementation of timer_user_logic is
	signal counter : std_logic_vector(C_SLV_DWIDTH-1 downto 0) := x"00000000";
	signal step_counter : std_logic_vector(C_SLV_DWIDTH-1 downto 0) := x"00000000";
	signal step : std_logic_vector(C_SLV_DWIDTH-1 downto 0) := x"00000000";
begin
	IP2Bus_WrAck   <= Bus2IP_WrCE(0) or Bus2IP_WrCE(1);
	IP2Bus_RdAck   <= Bus2IP_RdCE(0) or Bus2IP_WrCE(1);
	IP2Bus_Error   <= '0';

	IP2Bus_Data  <= counter;
	T_COUNTER    <= counter;

	counter_proc : process(Bus2IP_Clk) is
	begin
		if rising_edge(Bus2IP_Clk) then
			step_counter <= step_counter + 1;

			if step_counter = step then
				counter <= counter + 1;
				step_counter <= (others => '0');
			end if;

			if Bus2IP_WrCE = "10" or T_RST = '1' then
				counter <= (others => '0');
				step_counter <= (others => '0');
			end if;

			if Bus2IP_WrCE = "01" then
				step <= Bus2IP_Data;
			end if;
		end if;
	end process counter_proc;

end implementation;
