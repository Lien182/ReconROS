--
-- bubble_sorter.vhd
-- Bubble sort module. Sequentially sorts the contents of an attached 
-- single-port block RAM.
--
-- Author:     Enno Luebbers   <luebbers@reconos.de>
-- Date:       28.09.2007
--
-- This file is part of the ReconOS project <http://www.reconos.de>.
-- University of Paderborn, Computer Engineering Group.
--
-- (C) Copyright University of Paderborn 2007.
--

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_ARITH.all;
use IEEE.STD_LOGIC_UNSIGNED.all;
use IEEE.NUMERIC_STD.all;

---- Uncomment the following library declaration if instantiating
---- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity bubble_sorter is

  generic (
    G_LEN    : integer := 2048;         -- number of words to sort
    G_AWIDTH : integer := 11;           -- in bits
    G_DWIDTH : integer := 32            -- in bits
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
end bubble_sorter;

architecture Behavioral of bubble_sorter is

  type state_t is (STATE_IDLE, STATE_LOAD_A, STATE_LOAD_B, STATE_LOAD_WAIT_A, STATE_LOAD_WAIT_B, STATE_COMPARE, STATE_WRITE, STATE_LOAD_NEXT, STATE_START_OVER);

  signal state : state_t := STATE_IDLE;

  signal ptr     : natural range 0 to G_LEN-1;  --std_logic_vector(0 to C_AWIDTH-1);
  signal ptr_max : natural range 0 to G_LEN-1;
  signal a       : std_logic_vector(0 to G_DWIDTH-1);
  signal b       : std_logic_vector(0 to G_DWIDTH-1);
  signal low     : std_logic_vector(0 to G_DWIDTH-1);
  signal high    : std_logic_vector(0 to G_DWIDTH-1);
  signal swap    : boolean;
  signal swapped : boolean;

begin

  -- set RAM address
  o_RAMAddr <= std_logic_vector(TO_UNSIGNED(ptr, G_AWIDTH));

  -- concurrent signal assignments
  swap <= true when a > b else false;   -- should a and b be swapped?
  low  <= b    when swap  else a;       -- lower value of a and b
  high <= a    when swap  else b;       -- higher value of a and b

  -- sorting state machine
  sort_proc              : process(clk, reset)
    variable ptr_max_new : natural range 0 to G_LEN-1;  -- number of items left to sort
  begin

    if reset = '1' then
      ptr       <= 0;
      ptr_max   <= G_LEN-1;
      ptr_max_new := G_LEN-1;
      o_RAMData <= (others => '0');
      o_RAMWE   <= '0';
      done      <= '0';
      swapped   <= false;
      a         <= (others => '0');
      b         <= (others => '0');
		state     <= STATE_IDLE;
    elsif rising_edge(clk) then

      o_RAMWE   <= '0';
      o_RAMData <= (others => '0');

      case state is

        when STATE_IDLE        =>
          done      <= '0';
          ptr       <= 0;
          ptr_max   <= G_LEN-1;
          ptr_max_new := G_LEN-1;
          o_RAMData <= (others => '0');
          o_RAMWE   <= '0';
          swapped   <= false;
          -- start sorting on 'start' signal
          if start = '1' then
            state   <= STATE_LOAD_WAIT_A;
          end if;

          -- increase address (for B), wait for A to appear on RAM outputs
        when STATE_LOAD_WAIT_A =>
          ptr   <= ptr + 1;
          state <= STATE_LOAD_A;

          -- wait for B to appear on RAM outputs
        when STATE_LOAD_WAIT_B =>
          state <= STATE_LOAD_B;

          -- read A value from RAM
        when STATE_LOAD_A =>
          a     <= i_RAMData;
          state <= STATE_LOAD_B;

          -- read B value from RAM
        when STATE_LOAD_B =>
          b     <= i_RAMData;
          state <= STATE_COMPARE;

          -- compare A and B and act accordingly
        when STATE_COMPARE =>
          -- if A is higher than B
          if swap then
            -- write swapped values back
            ptr         <= ptr - 1;      -- back to writing
            o_RAMData   <= low;          -- write low value
            o_RAMWE     <= '1';
            swapped     <= true;
            state       <= STATE_WRITE;
          else
            if ptr < ptr_max then
                                         -- generate addres for next value for b
              a         <= b;
              ptr       <= ptr + 1;
              state     <= STATE_LOAD_WAIT_B;
            else
              -- if we swapped something then
              if swapped then
                                         -- start over
                ptr     <= 0;
                ptr_max <= ptr_max_new;  -- sort up to last swapped value
                swapped <= false;
                state   <= STATE_LOAD_WAIT_A;
              else
                                         -- else we're done
                done    <= '1';
                state   <= STATE_IDLE;
              end if;
            end if;
          end if;

          -- write high value
        when STATE_WRITE =>
          ptr_max_new := ptr;           -- save location of last swapped value
          ptr       <= ptr + 1;
          o_RAMData <= high;
          o_RAMWE   <= '1';
          if ptr < ptr_max-1 then
            state   <= STATE_LOAD_NEXT;
          else
            -- if we swapped something then
            if swapped then
              -- start over
              state <= STATE_START_OVER;
            else
              -- else we're done
              done  <= '1';
              state <= STATE_IDLE;
            end if;
          end if;

          -- load next B value
        when STATE_LOAD_NEXT =>
          ptr   <= ptr + 1;
          state <= STATE_LOAD_WAIT_B;

          -- start from beginning
        when STATE_START_OVER =>
          ptr     <= 0;
          ptr_max <= ptr_max_new;       -- sort up to last swapped value
          swapped <= false;
          state   <= STATE_LOAD_WAIT_A;

        when others =>
          state <= STATE_IDLE;

      end case;

    end if;
  end process;

end Behavioral;
