--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/64
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
-- Company:  CEG UPB
-- Engineer: Christoph Rüthing
--           Lennart Clausing
--           Felix Jentzsch
-- 
-- Module Name:    reconos_memif_memory_controller
-- Project Name:   ReconOS64
-- Target Devices: Zynq UltraScale+
-- Tool Versions:  2018.2
-- Description:    A memory controller connecting the memory fifos with
--                 the axi bus of the system.
-- 
-- Dependencies:        "..._axi.vhd" submodule contains all AXI/user logic
-- 
-- Revision:            -1.0 First working 64-bit version
--                      -1.1 Burst transfer support
--                      -1.2 Added WVALID throttling if (HLS-based) HWT sends data too slowly
--
-- Additional Comments: -Based on Vivado AXI master template (create new peripheral wizard)
--                      -Supports variable burst length up to specified maximum
--                      -MEMIF read/write functions handle alignment to chunk borders to avoid crossing 4K boundaries with a burst
--                      -Designed to connect to 128bit HPC0 slave port of PS via Interconnect (or Smartconnect) with width conversion
-- 
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library reconos_v3_01_a;
use reconos_v3_01_a.reconos_pkg.all;

entity reconos64_memif_axicontroller_v0_91_M00_AXI is
	generic (
		-- Users to add parameters here
         C_MEMIF_DATA_WIDTH : integer   := 64;
		-- User parameters ends

		-- Burst Length. Supports 1, 2, 4, 8, 16, 32, 64, 128, 256 burst lengths
		C_M_AXI_BURST_LEN	: integer	:= 16;
		-- Thread ID Width
		C_M_AXI_ID_WIDTH	: integer	:= 1;
		-- Width of Address Bus
		C_M_AXI_ADDR_WIDTH	: integer	:= 40;
		-- Width of Data Bus
		C_M_AXI_DATA_WIDTH	: integer	:= 64;
		-- Width of User Write Address Bus
		C_M_AXI_AWUSER_WIDTH	: integer	:= 2;
		-- Width of User Read Address Bus
		C_M_AXI_ARUSER_WIDTH	: integer	:= 2;
		-- Width of User Write Data Bus
		C_M_AXI_WUSER_WIDTH	: integer	:= 2;
		-- Width of User Read Data Bus
		C_M_AXI_RUSER_WIDTH	: integer	:= 2;
		-- Width of User Response Bus
		C_M_AXI_BUSER_WIDTH	: integer	:= 2
	);
	port (
		-- Users to add ports here
		MEMIF64_Hwt2Mem_In_Data  : in  std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
        MEMIF64_Hwt2Mem_In_Empty : in  std_logic;
        MEMIF64_Hwt2Mem_In_RE    : out std_logic;

        MEMIF64_Mem2Hwt_In_Data  : out std_logic_vector(C_MEMIF_DATA_WIDTH - 1 downto 0);
        MEMIF64_Mem2Hwt_In_Full  : in  std_logic;
        MEMIF64_Mem2Hwt_In_WE    : out std_logic;

        DEBUG                    : out std_logic_vector(63 downto 0);
		-- User ports ends

		-- Global Clock Signal.
		M_AXI_ACLK	: in std_logic;
		-- Global Reset Singal. This Signal is Active Low
		M_AXI_ARESETN	: in std_logic;
		
		-- Master Interface Write Address ID
		M_AXI_AWID	: out std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
		-- Master Interface Write Address
		M_AXI_AWADDR	: out std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
		-- Burst length. The burst length gives the exact number of transfers in a burst
		M_AXI_AWLEN	: out std_logic_vector(7 downto 0);
		-- Burst size. This signal indicates the size of each transfer in the burst
		M_AXI_AWSIZE	: out std_logic_vector(2 downto 0);
		-- Burst type. The burst type and the size information, 
    -- determine how the address for each transfer within the burst is calculated.
		M_AXI_AWBURST	: out std_logic_vector(1 downto 0);
		-- Lock type. Provides additional information about the
    -- atomic characteristics of the transfer.
		M_AXI_AWLOCK	: out std_logic;
		-- Memory type. This signal indicates how transactions
    -- are required to progress through a system.
		M_AXI_AWCACHE	: out std_logic_vector(3 downto 0);
		-- Protection type. This signal indicates the privilege
    -- and security level of the transaction, and whether
    -- the transaction is a data access or an instruction access.
		M_AXI_AWPROT	: out std_logic_vector(2 downto 0);
		-- Quality of Service, QoS identifier sent for each write transaction.
		M_AXI_AWQOS	: out std_logic_vector(3 downto 0);
		-- Optional User-defined signal in the write address channel.
		M_AXI_AWUSER	: out std_logic_vector(C_M_AXI_AWUSER_WIDTH-1 downto 0);
		-- Write address valid. This signal indicates that
    -- the channel is signaling valid write address and control information.
		M_AXI_AWVALID	: out std_logic;
		-- Write address ready. This signal indicates that
    -- the slave is ready to accept an address and associated control signals
		M_AXI_AWREADY	: in std_logic;
		
		-- Master Interface Write Data.
		M_AXI_WDATA	: out std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
		-- Write strobes. This signal indicates which byte
    -- lanes hold valid data. There is one write strobe
    -- bit for each eight bits of the write data bus.
		M_AXI_WSTRB	: out std_logic_vector(C_M_AXI_DATA_WIDTH/8-1 downto 0);
		-- Write last. This signal indicates the last transfer in a write burst.
		M_AXI_WLAST	: out std_logic;
		-- Optional User-defined signal in the write data channel.
		M_AXI_WUSER	: out std_logic_vector(C_M_AXI_WUSER_WIDTH-1 downto 0);
		-- Write valid. This signal indicates that valid write
    -- data and strobes are available
		M_AXI_WVALID	: out std_logic;
		-- Write ready. This signal indicates that the slave
    -- can accept the write data.
		M_AXI_WREADY	: in std_logic;
		
		-- Master Interface Write Response.
		M_AXI_BID	: in std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
		-- Write response. This signal indicates the status of the write transaction.
		M_AXI_BRESP	: in std_logic_vector(1 downto 0);
		-- Optional User-defined signal in the write response channel
		M_AXI_BUSER	: in std_logic_vector(C_M_AXI_BUSER_WIDTH-1 downto 0);
		-- Write response valid. This signal indicates that the
    -- channel is signaling a valid write response.
		M_AXI_BVALID	: in std_logic;
		-- Response ready. This signal indicates that the master
    -- can accept a write response.
		M_AXI_BREADY	: out std_logic;
		
		-- Master Interface Read Address.
		M_AXI_ARID	: out std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
		-- Read address. This signal indicates the initial
    -- address of a read burst transaction.
		M_AXI_ARADDR	: out std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
		-- Burst length. The burst length gives the exact number of transfers in a burst
		M_AXI_ARLEN	: out std_logic_vector(7 downto 0);
		-- Burst size. This signal indicates the size of each transfer in the burst
		M_AXI_ARSIZE	: out std_logic_vector(2 downto 0);
		-- Burst type. The burst type and the size information, 
    -- determine how the address for each transfer within the burst is calculated.
		M_AXI_ARBURST	: out std_logic_vector(1 downto 0);
		-- Lock type. Provides additional information about the
    -- atomic characteristics of the transfer.
		M_AXI_ARLOCK	: out std_logic;
		-- Memory type. This signal indicates how transactions
    -- are required to progress through a system.
		M_AXI_ARCACHE	: out std_logic_vector(3 downto 0);
		-- Protection type. This signal indicates the privilege
    -- and security level of the transaction, and whether
    -- the transaction is a data access or an instruction access.
		M_AXI_ARPROT	: out std_logic_vector(2 downto 0);
		-- Quality of Service, QoS identifier sent for each read transaction
		M_AXI_ARQOS	: out std_logic_vector(3 downto 0);
		-- Optional User-defined signal in the read address channel.
		M_AXI_ARUSER	: out std_logic_vector(C_M_AXI_ARUSER_WIDTH-1 downto 0);
		-- Write address valid. This signal indicates that
    -- the channel is signaling valid read address and control information
		M_AXI_ARVALID	: out std_logic;
		-- Read address ready. This signal indicates that
    -- the slave is ready to accept an address and associated control signals
		M_AXI_ARREADY	: in std_logic;
		
		-- Read ID tag. This signal is the identification tag
    -- for the read data group of signals generated by the slave.
		M_AXI_RID	: in std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
		-- Master Read Data
		M_AXI_RDATA	: in std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
		-- Read response. This signal indicates the status of the read transfer
		M_AXI_RRESP	: in std_logic_vector(1 downto 0);
		-- Read last. This signal indicates the last transfer in a read burst
		M_AXI_RLAST	: in std_logic;
		-- Optional User-defined signal in the read address channel.
		M_AXI_RUSER	: in std_logic_vector(C_M_AXI_RUSER_WIDTH-1 downto 0);
		-- Read valid. This signal indicates that the channel
    -- is signaling the required read data.
		M_AXI_RVALID	: in std_logic;
		-- Read ready. This signal indicates that the master can
    -- accept the read data and response information.
		M_AXI_RREADY	: out std_logic
	);
end reconos64_memif_axicontroller_v0_91_M00_AXI;

architecture implementation of reconos64_memif_axicontroller_v0_91_M00_AXI is

	-- function called clogb2 that returns an integer which has the
	-- value of the ceiling of the log base 2
	function clogb2 (bit_depth : integer) return integer is            
	 	variable depth  : integer := bit_depth;                               
	 	variable count  : integer := 1;                                       
	 begin                                                                   
	 	 for clogb2 in 1 to bit_depth loop  -- Works for up to 32 bit integers
	      if (bit_depth <= 2) then                                           
	        count := 1;                                                      
	      else                                                               
	        if(depth <= 1) then                                              
	 	       count := count;                                                
	 	     else                                                             
	 	       depth := depth / 2;                                            
	          count := count + 1;                                            
	 	     end if;                                                          
	 	   end if;                                                            
	   end loop;                                                             
	   return(count);        	                                              
	 end;                                                                    

	-- C_TRANSACTIONS_NUM is the width of the index counter for
	-- number of beats in a burst write or burst read transaction.
	 constant  C_TRANSACTIONS_NUM : integer := clogb2(C_M_AXI_BURST_LEN-1);
	 
	-- Burst length for transactions, in C_M_AXI_DATA_WIDTHs.
	-- Non-2^n lengths will eventually cause bursts across 4K address boundaries.
	-- constant  C_MASTER_LENGTH  : integer := 12;
	-- total number of burst transfers is master length divided by burst length and burst size
	--                                          |-    32bit  -| |- counter width for number of bytes per burst -|
	-- constant  C_NO_BURSTS_REQ  : integer := (C_MASTER_LENGTH-clogb2((C_M_AXI_BURST_LEN*C_M_AXI_DATA_WIDTH/8)-1));
	signal master_length  : integer := 12;
	signal no_bursts_req  : integer := (master_length - clogb2((C_M_AXI_BURST_LEN*C_M_AXI_DATA_WIDTH/8)-1));

	-- Example State machine to initialize counter, initialize write transactions, 
	-- initialize read transactions and comparison of read data with the 
	-- written data words.
	type state is ( IDLE,      -- This state initiates AXI4Lite transaction  
	 							-- after the state machine changes state to INIT_WRITE
	 							-- when there is 0 to 1 transition on INIT_AXI_TXN
	 				INIT_WRITE,   -- This state initializes write transaction,
	 							-- once writes are done, the state machine 
	 							-- changes state to INIT_READ 
	 				INIT_READ,    -- This state initializes read transaction
	 							-- once reads are done, the state machine 
	 							-- changes state to INIT_COMPARE 
	 				INIT_COMPARE);-- This state issues the status of comparison 
	 							-- of the written data with the read data
	 							
	type state_type is (STATE_READ_CMD,
	                     STATE_READ_ADDR,
                         STATE_PROCESS_WRITE_0,
                         STATE_PROCESS_WRITE_1,
                         STATE_PROCESS_READ_0,
                         STATE_PROCESS_READ_1,
                         STATE_CMPLT);

	signal mst_exec_state  : state ; 
	signal memif_state     : state_type; --new SM
	
	 -- target memory address (read&write)
    signal mem_addr   : std_logic_vector(C_M_AXI_ADDR_WIDTH- 1 downto 0);
	signal mem_op     : std_logic_vector(C_MEMIF_OP_WIDTH - 1 downto 0) := (others => '0');
    signal mem_length_bytes : unsigned(C_MEMIF_LENGTH_WIDTH - 1 downto 0) := (others => '0');
    signal mem_count_bytes  : unsigned(C_MEMIF_LENGTH_WIDTH - 1 downto 0) := (others => '0');

	-- AXI4FULL signals
	--AXI4 internal temp signals
	signal axi_awaddr	: std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
	signal axi_awvalid	: std_logic;
	signal axi_wdata	: std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
	signal axi_wlast	: std_logic;
	signal axi_wvalid	: std_logic;
	signal axi_bready	: std_logic;
	signal axi_araddr	: std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
	signal axi_arvalid	: std_logic;
	signal axi_rready	: std_logic;
	
	--write beat count in a burst
	signal write_index_in_burst	: std_logic_vector(C_TRANSACTIONS_NUM downto 0);
	--read beat count in a burst
	signal read_index_in_burst	: std_logic_vector(C_TRANSACTIONS_NUM downto 0);
	
	--size of C_M_AXI_BURST_LEN length burst in bytes
	signal burst_size_bytes	: std_logic_vector(C_TRANSACTIONS_NUM+2 downto 0);
	signal burst_size_words	: std_logic_vector(C_TRANSACTIONS_NUM downto 0);
	
	--The burst counters are used to track the number of burst transfers of C_M_AXI_BURST_LEN burst length needed to transfer 2^C_MASTER_LENGTH bytes of data.
	signal write_burst_counter	: std_logic_vector(63 downto 0);
	signal read_burst_counter	: std_logic_vector(63 downto 0);
	
	signal start_single_burst_write	: std_logic;
	signal start_single_burst_read	: std_logic;
	
	--signal writes_done : std_logic;
	--signal reads_done	 : std_logic;

	signal burst_write_active	: std_logic;
	signal burst_read_active	: std_logic;

	--Interface response error flags
	signal write_resp_error	: std_logic;
	signal read_resp_error	: std_logic;
	signal wnext	: std_logic;
	signal rnext	: std_logic;
	
	signal init_txn_pulse	: std_logic;

begin
	-- I/O Connections assignments
    DEBUG(2 downto 0)   <= std_logic_vector(to_unsigned(state_type'POS(memif_state), 3));
    DEBUG(3)            <= MEMIF64_Hwt2Mem_In_Empty;
	DEBUG(4)            <= MEMIF64_Mem2Hwt_In_Full;
	DEBUG(5)            <= write_resp_error;
	DEBUG(6)            <= read_resp_error;
    DEBUG(20 downto 13) <= write_burst_counter(7 downto 0);
    DEBUG(28 downto 21) <= read_burst_counter(7 downto 0);
    DEBUG(29)           <= start_single_burst_write;
    DEBUG(30)           <= start_single_burst_read;
    DEBUG(33)           <= burst_write_active;
    DEBUG(34)           <= burst_read_active;
    DEBUG(38+C_TRANSACTIONS_NUM downto 38) <= write_index_in_burst;

	--I/O Connections. Write Address (AW)
	M_AXI_AWID    	<= (others => '0');
	--The AXI address is a concatenation of the target base address + active offset range
	--M_AXI_AWADDR	defined by mem_addr
	M_AXI_AWADDR	<= std_logic_vector(unsigned(mem_addr) + unsigned(axi_awaddr));
	--Burst LENgth is number of transaction beats, minus 1
	--M_AXI_AWLEN	<= std_logic_vector( to_unsigned(C_M_AXI_BURST_LEN - 1, 8) );
	M_AXI_AWLEN	    <= std_logic_vector( to_unsigned( (to_integer(mem_length_bytes) /8) -1, 8) );
	--Size should be C_M_AXI_DATA_WIDTH, in 2^SIZE bytes, otherwise narrow bursts are used
	M_AXI_AWSIZE	<= std_logic_vector( to_unsigned(clogb2((C_M_AXI_DATA_WIDTH/8)-1), 3) );
	--INCR burst type is usually used, except for keyhole bursts
	M_AXI_AWBURST	<= "01";
	M_AXI_AWLOCK	<= '0';
	--Update value to 4'b0011 if coherent accesses to be used via the Zynq ACP port. Not Allocated, Modifiable, not Bufferable. Not Bufferable since this example is meant to test memory, not intermediate cache. 
	M_AXI_AWCACHE	<= "1111"; -- coherent transactions
	M_AXI_AWPROT	<= "010";  -- unprivileged, non-secure, data access
	M_AXI_AWQOS	    <= x"0";
	M_AXI_AWUSER	<= "01";   -- mark accessed memory as inner shareable (required at least for ACP port..)
	M_AXI_AWVALID	<= axi_awvalid;
	
	--Write Data(W)
	M_AXI_WDATA	    <= axi_wdata;
	--All bursts are complete and aligned in this example
	M_AXI_WSTRB	    <= (others => '1');
	M_AXI_WLAST	    <= axi_wlast;
	M_AXI_WUSER	    <= "01";   -- mark accessed memory as inner shareable (required at least for ACP port..)
	M_AXI_WVALID	<= axi_wvalid and (not MEMIF64_Hwt2Mem_In_Empty); -- throttle if no data is available
	
	--Write Response (B)
	M_AXI_BREADY	<= axi_bready;
	
	--Read Address (AR)
	M_AXI_ARID	    <= (others => '0');
	--M_AXI_ARADDR	defined by mem_addr
	M_AXI_ARADDR	<= std_logic_vector(unsigned(mem_addr) + unsigned(axi_araddr));
	--Burst LENgth is number of transaction beats, minus 1
	--M_AXI_ARLEN	<= std_logic_vector( to_unsigned(C_M_AXI_BURST_LEN - 1, 8) );
	M_AXI_ARLEN	    <= std_logic_vector( to_unsigned( (to_integer(mem_length_bytes) /8) -1, 8) );
	--Size should be C_M_AXI_DATA_WIDTH, in 2^n bytes, otherwise narrow bursts are used
	M_AXI_ARSIZE	<= std_logic_vector( to_unsigned( clogb2((C_M_AXI_DATA_WIDTH/8)-1),3 ));
	--INCR burst type is usually used, except for keyhole bursts
	M_AXI_ARBURST	<= "01";
	M_AXI_ARLOCK	<= '0';
	--Update value to 4'b0011 if coherent accesses to be used via the Zynq ACP port. Not Allocated, Modifiable, not Bufferable. Not Bufferable since this example is meant to test memory, not intermediate cache. 
	M_AXI_ARCACHE	<= "1111"; -- coherent transactions
	M_AXI_ARPROT	<= "010";  -- unprivileged, non-secure, data access
	M_AXI_ARQOS	    <= x"0";
	M_AXI_ARUSER	<= "01";   -- mark accessed memory as inner shareable (required at least for ACP port..)
	M_AXI_ARVALID	<= axi_arvalid;
	
	--Read and Read Response (R)
	M_AXI_RREADY	<= axi_rready;

	--Burst size in bytes
	--Modified to support variable burst length
	--burst_size_bytes	<= std_logic_vector( to_unsigned((C_M_AXI_BURST_LEN * (C_M_AXI_DATA_WIDTH/8)),C_TRANSACTIONS_NUM+3) );
	burst_size_bytes	<= std_logic_vector( to_unsigned( (to_integer(mem_length_bytes)), C_TRANSACTIONS_NUM+3) );
	burst_size_words    <= std_logic_vector( to_unsigned( (to_integer(mem_length_bytes) /8), C_TRANSACTIONS_NUM+1) );

	----------------------
	--Write Address Channel
	----------------------

	-- The purpose of the write address channel is to request the address and 
	-- command information for the entire transaction.  It is a single beat
	-- of information.

	-- The AXI4 Write address channel in this example will continue to initiate
	-- write commands as fast as it is allowed by the slave/interconnect.
	-- The address will be incremented on each accepted address transaction,
	-- by burst_size_byte to point to the next address. 

	  process(M_AXI_ACLK)
	  begin
	    if (rising_edge (M_AXI_ACLK)) then
	      if (M_AXI_ARESETN = '0' or init_txn_pulse = '1') then
	        axi_awvalid <= '0';
	      else
	        -- If previously not valid , start next transaction            
	        if (axi_awvalid = '0' and start_single_burst_write = '1') then
	          axi_awvalid <= '1';
	          -- Once asserted, VALIDs cannot be deasserted, so axi_awvalid
	          -- must wait until transaction is accepted                   
	        elsif (M_AXI_AWREADY = '1' and axi_awvalid = '1') then
	          axi_awvalid <= '0';
	        else
	          axi_awvalid <= axi_awvalid;
	        end if;
	      end if;
	    end if;
	  end process;

	-- Next address after AWREADY indicates previous address acceptance    
	  process(M_AXI_ACLK)
	  begin
	    if (rising_edge (M_AXI_ACLK)) then
	      if (M_AXI_ARESETN = '0' or init_txn_pulse = '1') then
	        axi_awaddr <= (others => '0');
	      else
	        if (M_AXI_AWREADY= '1' and axi_awvalid = '1') then
	          axi_awaddr <= std_logic_vector(unsigned(axi_awaddr) + unsigned(burst_size_bytes));
	        end if;
	      end if;
	    end if;
	  end process;


	----------------------
	--Write Data Channel
	----------------------

	--The write data will continually try to push write data across the interface.

	--The amount of data accepted will depend on the AXI slave and the AXI
	--Interconnect settings, such as if there are FIFOs enabled in interconnect.

	--Note that there is no explicit timing relationship to the write address channel.
	--The write channel has its own throttling flag, separate from the AW channel.

	--Synchronization between the channels must be determined by the user.

	--The simpliest but lowest performance would be to only issue one address write
	--and write data burst at a time.

	--In this example they are kept in sync by using the same address increment
	--and burst sizes. Then the AW and W channels have their transactions measured
	--with threshold counters as part of the user logic, to make sure neither 
	--channel gets too far ahead of each other.

	--Forward movement occurs when the write channel is valid and ready

	  wnext <= M_AXI_WREADY and (axi_wvalid and (not MEMIF64_Hwt2Mem_In_Empty)); -- throttle if no data is available

	-- WVALID logic, similar to the axi_awvalid always block above                      
	  process(M_AXI_ACLK)
	  begin
	    if (rising_edge (M_AXI_ACLK)) then
	      if (M_AXI_ARESETN = '0' or init_txn_pulse = '1') then
	        axi_wvalid <= '0';
	      else
	        if (axi_wvalid = '0' and start_single_burst_write = '1') then
	          -- If previously not valid, start next transaction                        
	          axi_wvalid <= '1';
	          --     /* If WREADY and too many writes, throttle WVALID                  
	          --      Once asserted, VALIDs cannot be deasserted, so WVALID             
	          --      must wait until burst is complete with WLAST */                   
	        elsif (wnext = '1' and axi_wlast = '1') then
	          axi_wvalid <= '0';
	        else
	          axi_wvalid <= axi_wvalid;
	        end if;
	      end if;
	    end if;
	  end process;

	--WLAST generation on the MSB of a counter underflow                                
	-- WVALID logic, similar to the axi_awvalid always block above                      
	  process(M_AXI_ACLK)
	  begin
	    if (rising_edge (M_AXI_ACLK)) then
	      if (M_AXI_ARESETN = '0' or init_txn_pulse = '1') then
	        axi_wlast <= '0';
	        -- axi_wlast is asserted when the write index
	        -- count reaches the penultimate count to synchronize
	        -- with the last write data when write_index_in_burst is b1111
	        -- elsif (&(write_index_in_burst[C_TRANSACTIONS_NUM-1:1])&& ~write_index_in_burst[0] && wnext)
		  else
			-- Modified to support variable burst length
			--if ((((write_index_in_burst = std_logic_vector(to_unsigned(C_M_AXI_BURST_LEN-2,C_TRANSACTIONS_NUM+1))) and C_M_AXI_BURST_LEN >= 2) and wnext = '1') or (C_M_AXI_BURST_LEN = 1)) then
		    if ((((write_index_in_burst = std_logic_vector(unsigned(burst_size_words) - 2)) and unsigned(burst_size_words) >= 2) and wnext = '1') or (unsigned(burst_size_words) = 1)) then
	          axi_wlast <= '1';
	          -- Deassrt axi_wlast when the last write data has been
	          -- accepted by the slave with a valid response
	        elsif (wnext = '1') then
	          axi_wlast <= '0';
	        elsif (axi_wlast = '1' and C_M_AXI_BURST_LEN = 1) then
	          axi_wlast <= '0';
	        end if;
	      end if;
	    end if;
	  end process;

	-- Burst length counter. Uses extra counter register bit to indicate terminal
	-- count to reduce decode logic */
	  process(M_AXI_ACLK)
	  begin
	    if (rising_edge (M_AXI_ACLK)) then
	      if (M_AXI_ARESETN = '0' or start_single_burst_write = '1' or init_txn_pulse = '1') then
	        write_index_in_burst <= (others => '0');
		  else
		  -- Modified to support variable burst length
			--if (wnext = '1' and (write_index_in_burst /= std_logic_vector(to_unsigned(C_M_AXI_BURST_LEN-1,C_TRANSACTIONS_NUM+1)))) then
			if (wnext = '1' and (write_index_in_burst /= std_logic_vector(unsigned(burst_size_words) - 1))) then
	          write_index_in_burst <= std_logic_vector(unsigned(write_index_in_burst) + 1);
	        end if;
	      end if;
	    end if;
	  end process;
	                                                             
	------------------------------
	--Write Response (B) Channel
	------------------------------

	--The write response channel provides feedback that the write has committed
	--to memory. BREADY will occur when all of the data and the write address
	--has arrived and been accepted by the slave.

	--The write issuance (number of outstanding write addresses) is started by 
	--the Address Write transfer, and is completed by a BREADY/BRESP.

	--While negating BREADY will eventually throttle the AWREADY signal, 
	--it is best not to throttle the whole data channel this way.

	--The BRESP bit [1] is used indicate any errors from the interconnect or
	--slave for the entire write burst. This example will capture the error 
	--into the ERROR output. 

	  process(M_AXI_ACLK)                                             
	  begin                                                                 
	    if (rising_edge (M_AXI_ACLK)) then                                  
	      if (M_AXI_ARESETN = '0' or init_txn_pulse = '1') then                                    
	        axi_bready <= '0';                                              
	        -- accept/acknowledge bresp with axi_bready by the master       
	        -- when M_AXI_BVALID is asserted by slave                       
	      else                                                              
	        if (M_AXI_BVALID = '1' and axi_bready = '0') then               
	          axi_bready <= '1';                                            
	          -- deassert after one clock cycle                             
	        elsif (axi_bready = '1') then                                   
	          axi_bready <= '0';                                            
	        end if;                                                         
	      end if;                                                           
	    end if;                                                             
	  end process;                                                          
	                                                                        
	                                                                        
	--Flag any write response errors                                        
	  write_resp_error <= axi_bready and M_AXI_BVALID and M_AXI_BRESP(1);   

	------------------------------
	--Read Address Channel
	------------------------------

	--The Read Address Channel (AW) provides a similar function to the
	--Write Address channel- to provide the tranfer qualifiers for the burst.

	--In this example, the read address increments in the same
	--manner as the write address channel.

	  process(M_AXI_ACLK)										  
	  begin                                                              
	    if (rising_edge (M_AXI_ACLK)) then                               
	      if (M_AXI_ARESETN = '0' or init_txn_pulse = '1') then                                 
	        axi_arvalid <= '0';                                          
	     -- If previously not valid , start next transaction             
	      else                                                           
	        if (axi_arvalid = '0' and start_single_burst_read = '1') then
	          axi_arvalid <= '1';                                        
	        elsif (M_AXI_ARREADY = '1' and axi_arvalid = '1') then       
	          axi_arvalid <= '0';                                        
	        end if;                                                      
	      end if;                                                        
	    end if;                                                          
	  end process;                                                       
	                                                                     
	-- Next address after ARREADY indicates previous address acceptance  
	  process(M_AXI_ACLK)
	  begin
	    if (rising_edge (M_AXI_ACLK)) then
	      if (M_AXI_ARESETN = '0' or init_txn_pulse = '1' ) then
	        axi_araddr <= (others => '0');
	      else
	        if (M_AXI_ARREADY = '1' and axi_arvalid = '1') then
	          axi_araddr <= std_logic_vector(unsigned(axi_araddr) + unsigned(burst_size_bytes));
	        end if;
	      end if;
	    end if;
	  end process;


	----------------------------------
	--Read Data (and Response) Channel
	----------------------------------

	 -- Forward movement occurs when the channel is valid and ready   
	  rnext <= M_AXI_RVALID and axi_rready;
	                                                                                                                          
	-- Burst length counter. Uses extra counter register bit to indicate    
	-- terminal count to reduce decode logic                                
	--   process(M_AXI_ACLK)
	--   begin
	--     if (rising_edge (M_AXI_ACLK)) then
	--       if (M_AXI_ARESETN = '0' or start_single_burst_read = '1' or init_txn_pulse = '1') then
	--         read_index_in_burst <= (others => '0');
	-- 	  else
	-- 		-- Modified to support variable burst length
	-- 		--if (rnext = '1' and (read_index_in_burst <= std_logic_vector(to_unsigned(C_M_AXI_BURST_LEN-1,C_TRANSACTIONS_NUM+1)))) then
	-- 		if (rnext = '1' and (read_index_in_burst <= std_logic_vector(unsigned(burst_size_words) - 1))) then
	--           read_index_in_burst <= std_logic_vector(unsigned(read_index_in_burst) + 1);
	--         end if;
	--       end if;
	--     end if;
	--   end process;

	--/*                                                                    
	-- The Read Data channel returns the results of the read request        
	--                                                                      
	-- In this example the data checker is always able to accept            
	-- more data, so no need to throttle the RREADY signal                  
	-- */                                                                   
	  process(M_AXI_ACLK)                                                   
	  begin                                                                 
	    if (rising_edge (M_AXI_ACLK)) then                                  
	      if (M_AXI_ARESETN = '0' or init_txn_pulse = '1') then             
	        axi_rready <= '0';                                              
	     -- accept/acknowledge rdata/rresp with axi_rready by the master    
	      -- when M_AXI_RVALID is asserted by slave                         
	      else                                                   
	        if (M_AXI_RVALID = '1') then                         
	          if (M_AXI_RLAST = '1' and axi_rready = '1') then   
	            axi_rready <= '0';                               
	           else                                              
	             axi_rready <= '1';                              
	          end if;                                            
	        end if;                                              
	      end if;                                                
	    end if;                                                  
	  end process;                                               
	                                                                                                                  
	--Flag any read response errors                                         
	  read_resp_error <= axi_rready and M_AXI_RVALID and M_AXI_RRESP(1);    

	----------------------------------
	--Example design throttling
	----------------------------------

	-- For maximum port throughput, this user example code will try to allow
	-- each channel to run as independently and as quickly as possible.

	-- However, there are times when the flow of data needs to be throtted by
	-- the user application. This example application requires that data is
	-- not read before it is written and that the write channels do not
	-- advance beyond an arbitrary threshold (say to prevent an 
	-- overrun of the current read address by the write address).

	-- From AXI4 Specification, 13.13.1: "If a master requires ordering between 
	-- read and write transactions, it must ensure that a response is received 
	-- for the previous transaction before issuing the next transaction."

	-- This example accomplishes this user application throttling through:
	-- -Reads wait for writes to fully complete
	-- -Address writes wait when not read + issued transaction counts pass 
	-- a parameterized threshold
	-- -Writes wait when a not read + active data burst count pass 
	-- a parameterized threshold

	 -- write_burst_counter counter keeps track with the number of burst transaction initiated             
	 -- against the number of burst transactions the master needs to initiate                                    
	  process(M_AXI_ACLK)
	  begin
	    if (rising_edge (M_AXI_ACLK)) then
	      if (M_AXI_ARESETN = '0' or init_txn_pulse = '1') then
	        write_burst_counter <= (others => '0');
	      else
	        if (M_AXI_AWREADY = '1' and axi_awvalid = '1') then
	          if (write_burst_counter(no_bursts_req) = '0')then
	            -- a new burst will start, since 2^C_NO_BURST_REQ bursts have not been reached
	            write_burst_counter <= std_logic_vector(unsigned(write_burst_counter) + 1);
	          end if;
	        end if;
	      end if;
	    end if;
	  end process;
	                                                                                                             
	 -- read_burst_counter counter keeps track with the number of burst transaction initiated                    
	 -- against the number of burst transactions the master needs to initiate                                    
	  process(M_AXI_ACLK)                                                                                        
	  begin                                                                                                      
	    if (rising_edge (M_AXI_ACLK)) then                                                                       
	      if (M_AXI_ARESETN = '0' or init_txn_pulse = '1') then                                                                         
	        read_burst_counter <= (others => '0');                                                               
	      else                                                                                                   
	        if (M_AXI_ARREADY = '1' and axi_arvalid = '1') then                                                  
	          if (read_burst_counter(no_bursts_req) = '0')then                                                 
	            read_burst_counter <= std_logic_vector(unsigned(read_burst_counter) + 1);                        
	          end if;                                                                                            
	        end if;                                                                                              
	      end if;                                                                                                
	    end if;                                                                                                  
	  end process;                                                                                               


	  main: process(M_AXI_ACLK) is
          begin
              if M_AXI_ARESETN = '0' then
                  memif_state <= STATE_READ_CMD;
                  
              elsif rising_edge(M_AXI_ACLK) then
                  case memif_state is
                      when STATE_READ_CMD =>
                          if MEMIF64_Hwt2Mem_In_Empty = '0' then
                              mem_op <= MEMIF64_Hwt2Mem_In_Data(C_MEMIF_OP_RANGE);
                              
                              mem_length_bytes <= unsigned(MEMIF64_Hwt2Mem_In_Data(C_MEMIF_LENGTH_RANGE)); -- fixed during transaction for AXI logic
                              mem_count_bytes <= unsigned(MEMIF64_Hwt2Mem_In_Data(C_MEMIF_LENGTH_RANGE)); -- is counted down during operation
                              
                              memif_state <= STATE_READ_ADDR;
                              
                              init_txn_pulse <= '1';
                          end if;
      
                      when STATE_READ_ADDR =>
                          init_txn_pulse <= '0';
                          
                          if MEMIF64_Hwt2Mem_In_Empty = '0' then 
                              mem_addr <= MEMIF64_Hwt2Mem_In_Data((C_M_AXI_ADDR_WIDTH-1) downto 0);
      
                              case mem_op is
                                  when MEMIF_CMD_READ =>
                                      memif_state <= STATE_PROCESS_READ_0;
      
                                  when MEMIF_CMD_WRITE =>
                                      memif_state <= STATE_PROCESS_WRITE_0;
      
                                  when others =>
                              end case;
                          end if;
      
                      when STATE_PROCESS_WRITE_0 =>
                            if (axi_awvalid = '0' and start_single_burst_write = '0' and burst_write_active = '0' and MEMIF64_Hwt2Mem_In_Empty = '0') then 
                                start_single_burst_write <= '1';
                            else
                                start_single_burst_write <= '0'; --Negate to generate a pulse
                                
                                if(start_single_burst_write = '1') then
                                   memif_state  <= STATE_PROCESS_WRITE_1;
                                end if;
                            end if;
      
                      when STATE_PROCESS_WRITE_1 =>
                          if wnext = '1' and MEMIF64_Hwt2Mem_In_Empty = '0' then
                              mem_count_bytes <= mem_count_bytes - 8;
      
                              if mem_count_bytes - 8 = 0 then
                                  memif_state <= STATE_CMPLT;
                              end if;
                          end if;
      
                      when STATE_PROCESS_READ_0 =>
                              if (axi_arvalid = '0' and burst_read_active = '0' and start_single_burst_read = '0' and MEMIF64_Mem2Hwt_In_Full = '0') then
                                start_single_burst_read <= '1';
                              else
                                start_single_burst_read <= '0'; --Negate to generate a pulse
                                if(start_single_burst_read = '1') then
                                      memif_state <= STATE_PROCESS_READ_1;
                                end if;
                              end if;

                      when STATE_PROCESS_READ_1 =>
                          if rnext = '1' and MEMIF64_Mem2Hwt_In_Full = '0' then
                              mem_count_bytes <= mem_count_bytes - 8;
      
                              if mem_count_bytes - 8 = 0 then
                                  memif_state <= STATE_CMPLT;
                              end if;
                          end if;
      
                      when STATE_CMPLT =>
                              memif_state <= STATE_READ_CMD;
      
                      when others =>
                  end case;
              end if;
          end process main;
          
          
       -- == Multiplexing signals ============================================
    	axi_wdata              <= MEMIF64_Hwt2Mem_In_Data;

        MEMIF64_Hwt2Mem_In_RE <= M_AXI_WREADY when memif_state = STATE_PROCESS_WRITE_1 else
                                 '1'          when memif_state = STATE_READ_CMD else
                                 '1'          when memif_state = STATE_READ_ADDR else
                                 '0';
      
        MEMIF64_Mem2Hwt_In_Data <= M_AXI_RDATA;
        MEMIF64_Mem2Hwt_In_WE   <= (M_AXI_RVALID and axi_rready) when memif_state = STATE_PROCESS_READ_1 else '0';
	                                                                                                                                                                                                           
	  -- burst_write_active signal is asserted when there is a burst write transaction                           
	  -- is initiated by the assertion of start_single_burst_write. burst_write_active                           
	  -- signal remains asserted until the burst write is accepted by the slave                                  
	  process(M_AXI_ACLK)                                                                                        
	  begin                                                                                                      
	    if (rising_edge (M_AXI_ACLK)) then                                                                       
	      if (M_AXI_ARESETN = '0' or init_txn_pulse = '1') then                                                                         
	        burst_write_active <= '0';                                                                           
	                                                                                                             
	       --The burst_write_active is asserted when a write burst transaction is initiated                      
	      else                                                                                                   
	        if (start_single_burst_write = '1') then                                                             
	          burst_write_active <= '1';                                                                         
	        elsif (M_AXI_BVALID = '1' and axi_bready = '1') then                                                 
	          burst_write_active <= '0';                                                                         
	        end if;                                                                                              
	      end if;                                                                                                
	    end if;                                                                                                  
	  end process;                                                                                               
	                                                                                                             
	 -- Check for last write completion.                                                                         
	                                                                                                             
	 -- This logic is to qualify the last write count with the final write                                       
	 -- response. This demonstrates how to confirm that a write has been                                         
	 -- committed.                                                                                                                                                                                                       
	--   process(M_AXI_ACLK)                                                                                        
	--   begin                                                                                                      
	--     if (rising_edge (M_AXI_ACLK)) then                                                                       
	--       if (M_AXI_ARESETN = '0' or init_txn_pulse = '1') then                                                                         
	--        writes_done <= '0';                                                                                   
	--       --The reads_done should be associated with a rready response                                           
	--       --elsif (M_AXI_RVALID && axi_rready && (read_burst_counter == {(C_NO_BURSTS_REQ-1){1}}) && axi_rlast)  
	--       else                                                                                                   
	--         --if (M_AXI_BVALID = '1' and (write_burst_counter(C_NO_BURSTS_REQ) = '1') and axi_bready = '1') then   
    --         if (M_AXI_BVALID = '1' and (write_burst_counter(no_bursts_req) = '1')) then --LC: test without this-> or(unsigned(write_burst_counter) >= unsigned(tx_length_todo(63-(C_TRANSACTIONS_NUM) downto C_TRANSACTIONS_NUM))))) then
	--           writes_done <= '1';                                                                                
	--         end if;                                                                                              
	--       end if;                                                                                                
	--     end if;                                                                                                  
	--   end process;                                                                                               
	                                                                                                             
	  -- burst_read_active signal is asserted when there is a burst write transaction                            
	  -- is initiated by the assertion of start_single_burst_write. start_single_burst_read                      
	  -- signal remains asserted until the burst read is accepted by the master                                  
	  process(M_AXI_ACLK)
	  begin
	    if (rising_edge (M_AXI_ACLK)) then
	      if (M_AXI_ARESETN = '0' or init_txn_pulse = '1') then
	        burst_read_active <= '0';

	       --The burst_write_active is asserted when a write burst transaction is initiated
	      else
	        if (start_single_burst_read = '1')then
	          burst_read_active <= '1';
	        elsif (M_AXI_RVALID = '1' and axi_rready = '1' and M_AXI_RLAST = '1') then
	          burst_read_active <= '0';
	        end if;
	      end if;
	    end if;
	  end process;
	                                                                                                             
	 -- Check for last read completion.      

	 -- This logic is to qualify the last read count with the final read                                         
	 -- response. This demonstrates how to confirm that a read has been                                          
	 -- committed.                                                                                               
	                                                                                                             
	--   process(M_AXI_ACLK)
	--   begin
	--     if (rising_edge (M_AXI_ACLK)) then                                                                       
	--       if (M_AXI_ARESETN = '0' or init_txn_pulse = '1') then            
	--         reads_done <= '0';
	--         --The reads_done should be associated with a rready response
	--         --elsif (M_AXI_RVALID && axi_rready && (read_burst_counter == {(C_NO_BURSTS_REQ-1){1}}) && axi_rlast)
	--       else
	--         if (M_AXI_RVALID = '1' and axi_rready = '1' and (read_index_in_burst = std_logic_vector (to_unsigned(C_M_AXI_BURST_LEN-1,C_TRANSACTIONS_NUM+1))) and (read_burst_counter(no_bursts_req) = '1')) then
	--           reads_done <= '1';
	--         end if;
	--       end if;
	--     end if;
	--   end process;

end implementation;
