--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - MEMIF MMU - TLB
--
--   project:      ReconOS
--   author:       Christoph R??thing, University of Paderborn
--   description:  The TLB (translation lookaside buffer) caches the last
--                 address translations for faster access.
--
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_misc.all;
use ieee.math_real.all;

<<if TOOL=="ise">>
library proc_common_v3_00_a;
use proc_common_v3_00_a.proc_common_pkg.all;
<<end if>>

entity tlb is
	generic (
		C_TLB_SIZE  : integer := 128;
		C_TAG_SIZE  : integer := 20;
		C_DATA_SIZE : integer := 32
	);
	port (
		-- TLB ports
		TLB_Tag : in  std_logic_vector(C_TAG_SIZE - 1 downto 0);
		TLB_DI  : in  std_logic_vector(C_DATA_SIZE - 1 downto 0);
		TLB_DO  : out std_logic_vector(C_DATA_SIZE - 1 downto 0);
		TLB_WE  : in  std_logic;
		TLB_Hit : out std_logic;
		
		TLB_Clk : in std_logic;
		TLB_Rst : in std_logic
	);
end entity tlb;

architecture implementation of tlb is
	
	signal do  : std_logic_vector(C_DATA_SIZE - 1 downto 0);
	signal hit : std_logic;

	type TAG_MEM_T  is array (0 to C_TLB_SIZE - 1) of std_logic_vector(C_TAG_SIZE - 1 downto 0);
	type DATA_MEM_T  is array (0 to C_TLB_SIZE - 1) of std_logic_vector(C_DATA_SIZE - 1 downto 0);

	signal valid    : std_logic_vector(0 to C_TLB_SIZE - 1);
	signal tag_mem  : TAG_MEM_T;
	signal data_mem : DATA_MEM_T;
	
	--signal wrptr : std_logic_vector(clog2(C_TLB_SIZE) - 1 downto 0); 
	signal wrptr : std_logic_vector(integer(ceil(log2(real(C_TLB_SIZE)))) - 1 downto 0); 
	

begin

	TLB_DO  <= do;
	TLB_Hit <= hit;


	write_proc : process(TLB_Clk,TLB_Rst) is
	begin
		if TLB_Rst = '1' then
			wrptr <= (others => '0');
			valid <= (others => '0');
		elsif rising_edge(TLB_Clk) then
			if TLB_WE = '1' then
				tag_mem(CONV_INTEGER(wrptr)) <= TLB_Tag;
				data_mem(CONV_INTEGER(wrptr)) <= TLB_DI;
				
				valid(CONV_INTEGER(wrptr)) <= '1';
				
				wrptr <= wrptr + 1;
			end if;
		end if;
	end process write_proc;


	read_proc : process(TLB_Tag,data_mem,valid,tag_mem) is
	begin
		hit <= '0';
		do  <= (others => '0');

		-- loop over all tlb entries and take the first hit
		for i in 0 to C_TLB_SIZE - 1 loop
			if valid(i) = '1' and tag_mem(i) = TLB_Tag then
				hit <= '1';
				do  <= data_mem(i);
				exit;
			end if;
		end loop;
	end process read_proc;

		
end architecture implementation;
