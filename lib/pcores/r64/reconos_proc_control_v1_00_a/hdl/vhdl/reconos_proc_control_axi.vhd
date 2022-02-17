--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/64
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - PROC_CONTROL
--
--   project:      ReconOS64
--   author:       see AUTHORS
--
--   description:  proc_control AXI module 
--                 connecting kernel driver to hardware
-- 
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_misc.all;


entity ProcControlAXI_S00_AXI is
	generic (
		-- Users to add parameters here
        C_NUM_HWTS     : integer := 1;
		-- User parameters ends
		-- Do not modify the parameters beyond this line

		-- Width of S_AXI data bus
		C_S_AXI_DATA_WIDTH	: integer	:= 64;
		-- Width of S_AXI address bus
		C_S_AXI_ADDR_WIDTH	: integer	:= 6
	);
	port (
		-- Users to add ports here
		
		
        -- PROC control ports
        PROC_Hwt_Rst     : out std_logic_vector(C_NUM_HWTS - 1 downto 0);
        PROC_Hwt_Signal  : out std_logic_vector(C_NUM_HWTS - 1 downto 0);
        PROC_Sys_Rst     : out std_logic;
        PROC_Pgf_Int     : out std_logic;

        -- MMU related ports
        MMU_Pgf          : in  std_logic;
        MMU_Fault_Addr   : in  std_logic_vector(63 downto 0);
        MMU_Retry        : out std_logic;
        MMU_Pgd          : out std_logic_vector(63 downto 0);
        MMU_Tlb_Hits     : in  std_logic_vector(63 downto 0);
        MMU_Tlb_Misses   : in  std_logic_vector(63 downto 0);
        
        
		-- User ports ends
		-- Do not modify the ports beyond this line

		-- Global Clock Signal
		S_AXI_ACLK	: in std_logic;
		-- Global Reset Signal. This Signal is Active LOW
		S_AXI_ARESETN	: in std_logic;
		-- Write address (issued by master, acceped by Slave)
		S_AXI_AWADDR	: in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		-- Write channel Protection type. This signal indicates the
    		-- privilege and security level of the transaction, and whether
    		-- the transaction is a data access or an instruction access.
		S_AXI_AWPROT	: in std_logic_vector(2 downto 0);
		-- Write address valid. This signal indicates that the master signaling
    		-- valid write address and control information.
		S_AXI_AWVALID	: in std_logic;
		-- Write address ready. This signal indicates that the slave is ready
    		-- to accept an address and associated control signals.
		S_AXI_AWREADY	: out std_logic;
		-- Write data (issued by master, acceped by Slave) 
		S_AXI_WDATA	: in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		-- Write strobes. This signal indicates which byte lanes hold
    		-- valid data. There is one write strobe bit for each eight
    		-- bits of the write data bus.    
		S_AXI_WSTRB	: in std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
		-- Write valid. This signal indicates that valid write
    		-- data and strobes are available.
		S_AXI_WVALID	: in std_logic;
		-- Write ready. This signal indicates that the slave
    		-- can accept the write data.
		S_AXI_WREADY	: out std_logic;
		-- Write response. This signal indicates the status
    		-- of the write transaction.
		S_AXI_BRESP	: out std_logic_vector(1 downto 0);
		-- Write response valid. This signal indicates that the channel
    		-- is signaling a valid write response.
		S_AXI_BVALID	: out std_logic;
		-- Response ready. This signal indicates that the master
    		-- can accept a write response.
		S_AXI_BREADY	: in std_logic;
		-- Read address (issued by master, acceped by Slave)
		S_AXI_ARADDR	: in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		-- Protection type. This signal indicates the privilege
    		-- and security level of the transaction, and whether the
    		-- transaction is a data access or an instruction access.
		S_AXI_ARPROT	: in std_logic_vector(2 downto 0);
		-- Read address valid. This signal indicates that the channel
    		-- is signaling valid read address and control information.
		S_AXI_ARVALID	: in std_logic;
		-- Read address ready. This signal indicates that the slave is
    		-- ready to accept an address and associated control signals.
		S_AXI_ARREADY	: out std_logic;
		-- Read data (issued by slave)
		S_AXI_RDATA	: out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		-- Read response. This signal indicates the status of the
    		-- read transfer.
		S_AXI_RRESP	: out std_logic_vector(1 downto 0);
		-- Read valid. This signal indicates that the channel is
    		-- signaling the required read data.
		S_AXI_RVALID	: out std_logic;
		-- Read ready. This signal indicates that the master can
    		-- accept the read data and response information.
		S_AXI_RREADY	: in std_logic
	);
end ProcControlAXI_S00_AXI;

architecture arch_imp of ProcControlAXI_S00_AXI is

    -- number of AXI_DATA_WIDTH-wide registers for each bitwise signalling (reset/signal)
    constant NUM_HWT_REGS : integer := ((C_NUM_HWTS - 1) / C_S_AXI_DATA_WIDTH) + 1;

	type PGF_INT_STATE_TYPE is (WAIT_PGF, WAIT_CLEAR, WAIT_READY);
	signal pgf_int_state : PGF_INT_STATE_TYPE;

	type SYS_RESET_STATE_TYPE is (WAIT_RST, PERF_RST);
	signal sys_reset_state   : SYS_RESET_STATE_TYPE;
	signal sys_reset_counter : std_logic_vector(3 downto 0);
	
    type BUS_WAIT_STATE_TYPE is (IDLE, WAIT_DATA);
    --signal pgd_state         : BUS_WAIT_STATE_TYPE;
    --signal pgd_hi_state      : BUS_WAIT_STATE_TYPE;
    signal hwt_reset_state   : BUS_WAIT_STATE_TYPE;
    signal hwt_signal_state  : BUS_WAIT_STATE_TYPE;

	--signal pgd                 : std_logic_vector(63 downto 0);
	signal mmu_retry_flag      : std_logic;
	signal fault_addr          : std_logic_vector(63 downto 0);
	signal tlb_hits            : std_logic_vector(63 downto 0);
	signal tlb_misses          : std_logic_vector(63 downto 0);
	signal sys_reset           : std_logic;
	signal hwt_reset           : std_logic_vector(C_NUM_HWTS - 1 downto 0);
	signal hwt_signal          : std_logic_vector(C_NUM_HWTS - 1 downto 0);

	signal hwt_reset_reg       : std_logic_vector(NUM_HWT_REGS * C_S_AXI_DATA_WIDTH - 1 downto 0);
	signal hwt_signal_reg      : std_logic_vector(NUM_HWT_REGS * C_S_AXI_DATA_WIDTH - 1 downto 0);



	-- AXI4LITE signals
	signal axi_awaddr	: std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
	signal axi_awready	: std_logic;
	signal axi_wready	: std_logic;
	signal axi_bresp	: std_logic_vector(1 downto 0);
	signal axi_bvalid	: std_logic;
	signal axi_araddr	: std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
	signal axi_arready	: std_logic;
	signal axi_rdata	: std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal axi_rresp	: std_logic_vector(1 downto 0);
	signal axi_rvalid	: std_logic;


	-- Example-specific design signals
	-- local parameter for addressing 32 bit / 64 bit C_S_AXI_DATA_WIDTH
	-- ADDR_LSB is used for addressing 32/64 bit registers/memories
	-- ADDR_LSB = 2 for 32 bits (n downto 2)
	-- ADDR_LSB = 3 for 64 bits (n downto 3)
	constant ADDR_LSB  : integer := (C_S_AXI_DATA_WIDTH/32)+ 1;
	constant OPT_MEM_ADDR_BITS : integer := 2; --LSB_Bit, so 2 for 8 registers
	------------------------------------------------
	---- Signals for user logic register space example
	--------------------------------------------------
	---- Number of Slave Registers 8
	signal slv_reg0    	: std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal slv_reg1    	: std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal slv_reg2    	: std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal slv_reg3    	: std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal slv_reg4    	: std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal slv_reg5    	: std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal slv_reg6    	: std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal slv_reg7    	: std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal slv_reg_rden	: std_logic;
	signal slv_reg_wren	: std_logic;
	signal reg_data_out	: std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal byte_index	: integer;
	signal aw_en	    : std_logic;

begin

    slv_reg0        <= std_logic_vector(to_unsigned(C_NUM_HWTS, C_S_AXI_DATA_WIDTH));

    -- copied read addresses:
    
    -- proc related signals
	fault_addr      <= MMU_Fault_Addr;
	tlb_hits        <= MMU_Tlb_Hits;
	tlb_misses      <= MMU_Tlb_Misses;
	
	MMU_Retry       <= mmu_retry_flag;

	PROC_Hwt_Rst    <= hwt_reset;
	PROC_Hwt_Signal <= hwt_signal;
	PROC_Sys_Rst    <= sys_reset;

	hwt_reset       <= hwt_reset_reg(C_NUM_HWTS - 1 downto 0);
	hwt_signal      <= hwt_signal_reg(C_NUM_HWTS - 1 downto 0);

	-- attach Pgd output directly to slave registers and remove respective processes from old 32bit implementation
	--MMU_Pgd         <= pgd;
	MMU_Pgd         <= slv_reg1;

	-- I/O Connections assignments
	S_AXI_AWREADY	<= axi_awready;
	S_AXI_WREADY	<= axi_wready;
	S_AXI_BRESP	<= axi_bresp;
	S_AXI_BVALID	<= axi_bvalid;
	S_AXI_ARREADY	<= axi_arready;
	S_AXI_RDATA	<= axi_rdata;
	S_AXI_RRESP	<= axi_rresp;
	S_AXI_RVALID	<= axi_rvalid;
	
	
	-- Implement axi_awready generation
	-- axi_awready is asserted for one S_AXI_ACLK clock cycle when both
	-- S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_awready is
	-- de-asserted when reset is low.
	process (S_AXI_ACLK)
	begin
	  if rising_edge(S_AXI_ACLK) then 
	    if S_AXI_ARESETN = '0' then
	      axi_awready <= '0';
	      aw_en <= '1';
	    else
	      if (axi_awready = '0' and S_AXI_AWVALID = '1' and S_AXI_WVALID = '1' and aw_en = '1') then
	        -- slave is ready to accept write address when
	        -- there is a valid write address and write data
	        -- on the write address and data bus. This design 
	        -- expects no outstanding transactions. 
	        axi_awready <= '1';
	        elsif (S_AXI_BREADY = '1' and axi_bvalid = '1') then
	            aw_en <= '1';
	        	axi_awready <= '0';
	      else
	        axi_awready <= '0';
	      end if;
	    end if;
	  end if;
	end process;


	-- Implement axi_awaddr latching
	-- This process is used to latch the address when both 
	-- S_AXI_AWVALID and S_AXI_WVALID are valid. 
	process (S_AXI_ACLK)
	begin
	  if rising_edge(S_AXI_ACLK) then 
	    if S_AXI_ARESETN = '0' then
	      axi_awaddr <= (others => '0');
	    else
	      if (axi_awready = '0' and S_AXI_AWVALID = '1' and S_AXI_WVALID = '1' and aw_en = '1') then
	        -- Write Address latching
	        axi_awaddr <= S_AXI_AWADDR;
	      end if;
	    end if;
	  end if;                   
	end process; 


	-- Implement axi_wready generation
	-- axi_wready is asserted for one S_AXI_ACLK clock cycle when both
	-- S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_wready is 
	-- de-asserted when reset is low. 
	process (S_AXI_ACLK)
	begin
	  if rising_edge(S_AXI_ACLK) then 
	    if S_AXI_ARESETN = '0' then
	      axi_wready <= '0';
	    else
	      if (axi_wready = '0' and S_AXI_WVALID = '1' and S_AXI_AWVALID = '1' and aw_en = '1') then
	          -- slave is ready to accept write data when 
	          -- there is a valid write address and write data
	          -- on the write address and data bus. This design 
	          -- expects no outstanding transactions.           
	          axi_wready <= '1';
	      else
	        axi_wready <= '0';
	      end if;
	    end if;
	  end if;
	end process; 


	-- Implement memory mapped register select and write logic generation
	-- The write data is accepted and written to memory mapped registers when
	-- axi_awready, S_AXI_WVALID, axi_wready and S_AXI_WVALID are asserted. Write strobes are used to
	-- select byte enables of slave registers while writing.
	-- These registers are cleared when reset (active low) is applied.
	-- Slave register write enable is asserted when valid address and data are available
	-- and the slave is ready to accept the write address and write data.
	slv_reg_wren <= axi_wready and S_AXI_WVALID and axi_awready and S_AXI_AWVALID ;
	incoming_write : process (S_AXI_ACLK)
	
        variable loc_addr :std_logic_vector(OPT_MEM_ADDR_BITS downto 0); 
	
	begin
	
	  if rising_edge(S_AXI_ACLK) then 
	    if S_AXI_ARESETN = '0' then
        --slv_reg0 <= (others => '0'); --readonly
	      slv_reg1 <= (others => '0');
	      slv_reg2 <= (others => '0');
	      slv_reg3 <= (others => '0');
	      slv_reg4 <= (others => '0');
	      slv_reg5 <= (others => '0');
	      slv_reg6 <= (others => '0');
	      slv_reg7 <= (others => '0');
	    else
	      loc_addr := axi_awaddr(ADDR_LSB + OPT_MEM_ADDR_BITS downto ADDR_LSB);
	      if (slv_reg_wren = '1') then
	        case loc_addr is
	          when b"000" =>
	            for byte_index in 0 to (C_S_AXI_DATA_WIDTH/8-1) loop
	              if ( S_AXI_WSTRB(byte_index) = '1' ) then
	                -- Respective byte enables are asserted as per write strobes
	                -- slave registor 0
                    -- reg0 is READONLY
	                --slv_reg0(byte_index*8+7 downto byte_index*8) <= S_AXI_WDATA(byte_index*8+7 downto byte_index*8);
	              end if;
	            end loop;
	          when b"001" =>
	            for byte_index in 0 to (C_S_AXI_DATA_WIDTH/8-1) loop
	              if ( S_AXI_WSTRB(byte_index) = '1' ) then
	                -- Respective byte enables are asserted as per write strobes                   
	                -- slave registor 1
	                slv_reg1(byte_index*8+7 downto byte_index*8) <= S_AXI_WDATA(byte_index*8+7 downto byte_index*8);
	              end if;
	            end loop;
	          when b"010" =>
	            for byte_index in 0 to (C_S_AXI_DATA_WIDTH/8-1) loop
	              if ( S_AXI_WSTRB(byte_index) = '1' ) then
	                -- Respective byte enables are asserted as per write strobes                   
	                -- slave registor 2
	                slv_reg2(byte_index*8+7 downto byte_index*8) <= S_AXI_WDATA(byte_index*8+7 downto byte_index*8);
	              end if;
	            end loop;
	          when b"011" =>
	            for byte_index in 0 to (C_S_AXI_DATA_WIDTH/8-1) loop
	              if ( S_AXI_WSTRB(byte_index) = '1' ) then
	                -- Respective byte enables are asserted as per write strobes                   
	                -- slave registor 3
	                slv_reg3(byte_index*8+7 downto byte_index*8) <= S_AXI_WDATA(byte_index*8+7 downto byte_index*8);
	              end if;
	            end loop;
	          when b"100" =>
	            for byte_index in 0 to (C_S_AXI_DATA_WIDTH/8-1) loop
	              if ( S_AXI_WSTRB(byte_index) = '1' ) then
	                -- Respective byte enables are asserted as per write strobes                   
	                -- slave registor 4
	                slv_reg4(byte_index*8+7 downto byte_index*8) <= S_AXI_WDATA(byte_index*8+7 downto byte_index*8);
	              end if;
	            end loop;
	          when b"101" =>
	            for byte_index in 0 to (C_S_AXI_DATA_WIDTH/8-1) loop
	              if ( S_AXI_WSTRB(byte_index) = '1' ) then
	                -- Respective byte enables are asserted as per write strobes                   
	                -- slave registor 5
	                slv_reg5(byte_index*8+7 downto byte_index*8) <= S_AXI_WDATA(byte_index*8+7 downto byte_index*8);
	              end if;
	            end loop;
	          when b"110" =>
	            for byte_index in 0 to (C_S_AXI_DATA_WIDTH/8-1) loop
	              if ( S_AXI_WSTRB(byte_index) = '1' ) then
	                -- Respective byte enables are asserted as per write strobes                   
	                -- slave registor 6
	                slv_reg6(byte_index*8+7 downto byte_index*8) <= S_AXI_WDATA(byte_index*8+7 downto byte_index*8);
	              end if;
	            end loop;
	          when b"111" =>
	            for byte_index in 0 to (C_S_AXI_DATA_WIDTH/8-1) loop
	              if ( S_AXI_WSTRB(byte_index) = '1' ) then
	                -- Respective byte enables are asserted as per write strobes                   
	                -- slave registor 7
	                slv_reg7(byte_index*8+7 downto byte_index*8) <= S_AXI_WDATA(byte_index*8+7 downto byte_index*8);
	              end if;
	            end loop;
	          when others =>
	            --slv_reg0 <= slv_reg0;
	            slv_reg1 <= slv_reg1;
	            slv_reg2 <= slv_reg2;
	            slv_reg3 <= slv_reg3;
	            slv_reg4 <= slv_reg4;
	            slv_reg5 <= slv_reg5;
	            slv_reg6 <= slv_reg6;
	            slv_reg7 <= slv_reg7;
	        end case;
	      end if;
	    end if;
	  end if;                   
	end process incoming_write; 


	-- Implement write response logic generation
	-- The write response and response valid signals are asserted by the slave 
	-- when axi_wready, S_AXI_WVALID, axi_wready and S_AXI_WVALID are asserted.  
	-- This marks the acceptance of address and indicates the status of 
	-- write transaction.
	process (S_AXI_ACLK)
	begin
	  if rising_edge(S_AXI_ACLK) then 
	    if S_AXI_ARESETN = '0' then
	      axi_bvalid  <= '0';
	      axi_bresp   <= "00"; --need to work more on the responses
	    else
	      if (axi_awready = '1' and S_AXI_AWVALID = '1' and axi_wready = '1' and S_AXI_WVALID = '1' and axi_bvalid = '0'  ) then
	        axi_bvalid <= '1';
	        axi_bresp  <= "00"; 
	      elsif (S_AXI_BREADY = '1' and axi_bvalid = '1') then   --check if bready is asserted while bvalid is high)
	        axi_bvalid <= '0';                                 -- (there is a possibility that bready is always asserted high)
	      end if;
	    end if;
	  end if;                   
	end process; 


	-- Implement axi_arready generation
	-- axi_arready is asserted for one S_AXI_ACLK clock cycle when
	-- S_AXI_ARVALID is asserted. axi_awready is 
	-- de-asserted when reset (active low) is asserted. 
	-- The read address is also latched when S_AXI_ARVALID is 
	-- asserted. axi_araddr is reset to zero on reset assertion.
	process (S_AXI_ACLK)
	begin
	  if rising_edge(S_AXI_ACLK) then 
	    if S_AXI_ARESETN = '0' then
	      axi_arready <= '0';
	      axi_araddr  <= (others => '1');
	    else
	      if (axi_arready = '0' and S_AXI_ARVALID = '1') then
	        -- indicates that the slave has acceped the valid read address
	        axi_arready <= '1';
	        -- Read Address latching 
	        axi_araddr  <= S_AXI_ARADDR;           
	      else
	        axi_arready <= '0';
	      end if;
	    end if;
	  end if;                   
	end process; 


	-- Implement axi_arvalid generation
	-- axi_rvalid is asserted for one S_AXI_ACLK clock cycle when both 
	-- S_AXI_ARVALID and axi_arready are asserted. The slave registers 
	-- data are available on the axi_rdata bus at this instance. The 
	-- assertion of axi_rvalid marks the validity of read data on the 
	-- bus and axi_rresp indicates the status of read transaction.axi_rvalid 
	-- is deasserted on reset (active low). axi_rresp and axi_rdata are 
	-- cleared to zero on reset (active low).  
	process (S_AXI_ACLK)
	begin
	  if rising_edge(S_AXI_ACLK) then
	    if S_AXI_ARESETN = '0' then
	      axi_rvalid <= '0';
	      axi_rresp  <= "00";
	    else
	      if (axi_arready = '1' and S_AXI_ARVALID = '1' and axi_rvalid = '0') then
	        -- Valid read data is available at the read data bus
	        axi_rvalid <= '1';
	        axi_rresp  <= "00"; -- 'OKAY' response
	      elsif (axi_rvalid = '1' and S_AXI_RREADY = '1') then
	        -- Read data is accepted by the master
	        axi_rvalid <= '0';
	      end if;            
	    end if;
	  end if;
	end process;


	-- Implement memory mapped register select and read logic generation
	-- Slave register read enable is asserted when valid address is available
	-- and the slave is ready to accept the read address.
	slv_reg_rden <= axi_arready and S_AXI_ARVALID and (not axi_rvalid) ;
	incoming_read : process (slv_reg0, slv_reg1, slv_reg2, slv_reg3, slv_reg4, slv_reg5, slv_reg6, slv_reg7, axi_araddr, S_AXI_ARESETN, slv_reg_rden)
	
	   variable loc_addr :std_logic_vector(OPT_MEM_ADDR_BITS downto 0);
	   
	   -- REGISTER MAP                         OLD:
	   -- slv_reg0 : NUM_HWTS
	   -- slv_reg1 : pgd 
	   -- slv_reg2 : fault_addr                pgd_hi
	   -- slv_reg3 : tlb_hits                  pgd
	   -- slv_reg4 : tlb_misses                fault_addr_hi
	   -- slv_reg5 : reset                     fault_addr
	   -- slv_reg6 : hwt_reset(bitwise)        tlb_hits
	   -- slv_reg7 : hwt_signal(bitwise)       tlb_misses
	   -- slv_reg8 : n/a                       reset
	   -- slv_reg9 : n/a                       hwt_reset (bitwise)
	   -- slv_reg10: n/a                       hwt_signal (bitwise)
	   -- slv_reg11: n/a
	   
	begin
	    -- Address decoding for reading registers
	    loc_addr := axi_araddr(ADDR_LSB + OPT_MEM_ADDR_BITS downto ADDR_LSB);
	    case loc_addr is
	      when b"000" =>
	        reg_data_out <= slv_reg0;   -- NUM_HWTS
	      when b"001" =>
	        reg_data_out <= slv_reg1;   -- pgd
	      when b"010" =>
	      -- hotfix: read directly from fault_addr signal
	        reg_data_out <= fault_addr; -- fault_addr
	      when b"011" =>
 	        reg_data_out <= slv_reg3;   -- tlb_hits
 	      when b"100" =>
 	        reg_data_out <= slv_reg4;   -- tlb_misses
 	      when b"101" =>
 	        reg_data_out <= slv_reg5;   -- reset
	      when b"110" =>
	        reg_data_out <= slv_reg6;   -- hwt_reset
	      when b"111" =>
	        reg_data_out <= slv_reg7;   -- hwt_signal
	      when others =>
	        reg_data_out  <= (others => '0');
	    end case;
	end process incoming_read; 
	

	-- Output register or memory read data
	read_response_data : process( S_AXI_ACLK ) is
	begin
	  if (rising_edge (S_AXI_ACLK)) then
	    if ( S_AXI_ARESETN = '0' ) then
	      axi_rdata  <= (others => '0');
	    else
	      if (slv_reg_rden = '1') then
	        -- When there is a valid read address (S_AXI_ARVALID) with 
	        -- acceptance of read address by the slave (axi_arready), 
	        -- output the read dada 
	        -- Read address mux
	          axi_rdata <= reg_data_out;     -- register read data
	      end if;   
	    end if;
	  end if;
	end process read_response_data;


	-- Add user logic here

    -- page fault handlig (for details see description above)
        pgf_int_proc : process(S_AXI_ACLK, S_AXI_ARESETN) is
        
            variable slv_read_reg  : std_logic_vector(OPT_MEM_ADDR_BITS downto 0);
            variable slv_write_reg : std_logic_vector(OPT_MEM_ADDR_BITS downto 0);
        
        begin
            slv_read_reg  := axi_araddr(ADDR_LSB + OPT_MEM_ADDR_BITS downto ADDR_LSB);
            slv_write_reg := axi_awaddr(ADDR_LSB + OPT_MEM_ADDR_BITS downto ADDR_LSB);
        
            if S_AXI_ARESETN = '0' or sys_reset = '1' then
                PROC_Pgf_Int <= '0';
                pgf_int_state <= WAIT_PGF;
            elsif rising_edge(S_AXI_ACLK) then
                mmu_retry_flag <= '0';
    
                case pgf_int_state is
					when WAIT_PGF =>
						-- do not enter interrupt state if MMU_Retry was just triggered, as MMU takes a cycle to deassert Pgf flag
                        if MMU_Pgf = '1' and mmu_retry_flag = '0' then
                            PROC_Pgf_Int <= '1';
                            pgf_int_state <= WAIT_CLEAR;
                        end if;
    
                    when WAIT_CLEAR =>
                        -- reading from page_fault_addr register
                        if slv_read_reg = b"010" then
                            PROC_Pgf_Int <= '0';
                            pgf_int_state <= WAIT_READY;
                        end if;    
                        --if slv_read_reg = b"0101" then
                        --    pgf_int_state <= WAIT_CLEAR_2A;
                        --elsif slv_read_reg = b"0100" then
                        --   pgf_int_state <= WAIT_CLEAR_2B;
                        --end if;
                        
                    --when WAIT_CLEAR_2A =>
                    --     -- reading from page_fault_addr_HI register
                    --    if slv_read_reg = b"0100" then
                    --        PROC_Pgf_Int <= '0';
                    --        pgf_int_state <= WAIT_READY;
                    --    end if;
                        
                   --when WAIT_CLEAR_2B =>
                   --      -- same, but if high bytes are read first
                   --      -- reading from page_fault_addr_HI register
                   --     if slv_read_reg = b"0101" then
                   --         PROC_Pgf_Int <= '0';
                   --         pgf_int_state <= WAIT_READY;
                   --     end if; 
    
                    when WAIT_READY =>
                        -- writing to page_fault_addr register
                        --if slv_write_reg = b"0101" or slv_write_reg = b"0100" then
                        --    mmu_retry_flag <= '1';
                        --    pgf_int_state <= WAIT_PGF;
                        --end if;
                        if slv_write_reg = b"010" then
                            mmu_retry_flag <= '1';
                            pgf_int_state <= WAIT_PGF;
                        end if;
                end case;
            end if;
        end process pgf_int_proc;
    
    
        hwt_reset_proc : process(S_AXI_ACLK,S_AXI_ARESETN) is
            variable slv_read_reg  : std_logic_vector(OPT_MEM_ADDR_BITS downto 0);
            variable slv_write_reg : std_logic_vector(OPT_MEM_ADDR_BITS downto 0);
        begin
            slv_read_reg  := axi_araddr(ADDR_LSB + OPT_MEM_ADDR_BITS downto ADDR_LSB);
            slv_write_reg := axi_awaddr(ADDR_LSB + OPT_MEM_ADDR_BITS downto ADDR_LSB);
            
            if S_AXI_ARESETN = '0' or sys_reset = '1' then
                hwt_reset_reg <= (others => '1');
            elsif rising_edge(S_AXI_ACLK) then
                -- writing to hwt_reset
                -- ignoring byte enable
            --    for i in 0 to NUM_HWT_REGS - 1 loop
                    -- loop through number of 32bit registers (usually 1 if <= 32 HWTs)
                    if slv_write_reg = b"110" then
            --            hwt_reset_reg(32 * i + 31 downto 32 * i) <= Bus2IP_Data;
                        hwt_reset_reg(63 downto 0) <= slv_reg6;
                    end if;
            --    end loop;
            end if;
        end process hwt_reset_proc;
    
    
        hwt_signal_proc : process(S_AXI_ACLK,S_AXI_ARESETN) is
            variable slv_read_reg  : std_logic_vector(OPT_MEM_ADDR_BITS downto 0);
            variable slv_write_reg : std_logic_vector(OPT_MEM_ADDR_BITS downto 0);
        begin
            slv_read_reg  := axi_araddr(ADDR_LSB + OPT_MEM_ADDR_BITS downto ADDR_LSB);
            slv_write_reg := axi_awaddr(ADDR_LSB + OPT_MEM_ADDR_BITS downto ADDR_LSB);
            
            if S_AXI_ARESETN = '0' or sys_reset = '1' then
                hwt_signal_reg <= (others => '0');
            elsif rising_edge(S_AXI_ACLK) then
                -- writing to hwt_signal
                -- ignoring byte enable
            --    for i in 0 to NUM_HWT_REGS - 1 loop
                -- loop through number of 32bit registers (usually 1 if <= 32 HWTs)
                    if slv_write_reg = b"111" then
            --            hwt_signal_reg(32 * i + 31 downto 32 * i) <= Bus2IP_Data;
                        hwt_signal_reg(63 downto 0) <= slv_reg7;
                    end if;
            --    end loop;
            end if;
        end process hwt_signal_proc;
    
    
        sys_reset_proc : process(S_AXI_ACLK,S_AXI_ARESETN) is
            variable slv_read_reg  : std_logic_vector(OPT_MEM_ADDR_BITS downto 0);
            variable slv_write_reg : std_logic_vector(OPT_MEM_ADDR_BITS downto 0);
        begin
            slv_read_reg  := axi_araddr(ADDR_LSB + OPT_MEM_ADDR_BITS downto ADDR_LSB);
            slv_write_reg := axi_awaddr(ADDR_LSB + OPT_MEM_ADDR_BITS downto ADDR_LSB);
            
            if S_AXI_ARESETN = '0' then
                sys_reset <= '1';
                sys_reset_state <= PERF_RST;
                sys_reset_counter <= (others => '0');
            elsif rising_edge(S_AXI_ACLK) then
                sys_reset <= '0';
    
                case sys_reset_state is
                    when WAIT_RST =>
                        if slv_write_reg = b"101" then
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

	-- User logic ends

end arch_imp;
