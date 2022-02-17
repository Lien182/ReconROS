library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spi_master is
	generic (
		G_SM_CLK_PRD  : time := 10ns;
		G_SPI_CLK_PRD : time := 400ns;

		G_DATA_LEN : integer := 8
	);
	port (
		SPI_SCLK : out std_logic;
		SPI_MOSI : out std_logic;
		SPI_MISO : in  std_logic;
		SPI_SSn  : out std_logic;

		SM_TxData : in  std_logic_vector(G_DATA_LEN - 1 downto 0);
		SM_RxData : out std_logic_vector(G_DATA_LEN - 1 downto 0);
		SM_Start  : in  std_logic;
		SM_Ready  : out std_logic;
		SM_Conti  : in  std_logic;
		SM_Clk    : in  std_logic
	);
end entity spi_master;

architecture implementation of spi_master is
	constant C_CLK_DIV_COUNT : integer := G_SPI_CLK_PRD / G_SM_CLK_PRD / 2 - 1;
	signal sclk_count : unsigned(31 downto 0) := (others => '0');
	signal sclk       : std_logic := '0';

	signal miso_s_0, miso_s_1 : std_logic;

	type STATE_TYPE is (STATE_WAIT, STATE_PROC);
	signal state : STATE_TYPE := STATE_WAIT;

	signal data_count : unsigned(31 downto 0) := (others => '0');
	signal txdata, rxdata : std_logic_vector(G_DATA_LEN - 1 downto 0);

	signal conti : std_logic;
begin

	ctrl_proc : process (SM_Clk) is
	begin
		if rising_edge(SM_Clk) then
			case state is
				when STATE_WAIT =>
					data_count <= (others => '0');
					sclk_count <= (others => '0');

					if SM_Start = '1' then
						txdata <= SM_TxData;
						conti <= SM_Conti;
						
						state <= STATE_PROC;
					end if;

				when STATE_PROC =>
					sclk_count <= sclk_count + 1;

					if sclk_count = C_CLK_DIV_COUNT then
						sclk       <= not sclk;
						sclk_count <= (others => '0');

						if sclk = '1' then
							txdata <= txdata(G_DATA_LEN - 2 downto 0) & '0';
						else
							if data_count = G_DATA_LEN then
								sclk <= '0';

								state <= STATE_WAIT;
							else
								rxdata <= rxdata(G_DATA_LEN - 2 downto 0) & miso_s_0;

								data_count <= data_count + 1;
							end if;
						end if;
					end if;
				when others =>
			end case;
		end if;
	end process ctrl_proc;

	sync_proc : process (SM_Clk) is
	begin
		if rising_edge(SM_Clk) then
			miso_s_1 <= SPI_MISO;
			miso_s_0 <= miso_s_1;
		end if;
	end process sync_proc;

	SPI_SCLK <= sclk;
	SPI_MOSI <= txdata(G_DATA_LEN - 1);
	SPI_SSn  <= '0' when state = STATE_PROC else
	            '0' when conti = '1' else
	            '1';

	SM_RxData <= rxdata;
	SM_Ready  <= '1' when state = STATE_WAIT else '0';

end architecture implementation;