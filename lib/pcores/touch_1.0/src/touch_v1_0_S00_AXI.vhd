library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity touch_v1_0_S00_AXI is
	generic (
		-- Users to add parameters here

		-- User parameters ends
		-- Do not modify the parameters beyond this line

		-- Width of S_AXI data bus
		C_S_AXI_DATA_WIDTH	: integer	:= 32;
		-- Width of S_AXI address bus
		C_S_AXI_ADDR_WIDTH	: integer	:= 4
	);
	port (
		-- Users to add ports here
        TC_SCLK  : out std_logic;
        TC_MOSI  : out std_logic;
        TC_MISO  : in  std_logic := '0';
        TC_SSn   : out std_logic;
        TC_IRQ   : in  std_logic := '0';
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
end touch_v1_0_S00_AXI;

architecture arch_imp of touch_v1_0_S00_AXI is

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
	constant OPT_MEM_ADDR_BITS : integer := 1;
	------------------------------------------------
	---- Signals for user logic register space example
	--------------------------------------------------
	---- Number of Slave Registers 4
	signal slv_reg0	:std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal slv_reg1	:std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal slv_reg2	:std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal slv_reg3	:std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal slv_reg_rden	: std_logic;
	signal slv_reg_wren	: std_logic;
	signal reg_data_out	:std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal byte_index	: integer;
	
	--user defines
	
	constant C_AVG_COUNT     : integer := 32;
    constant C_AVG_COUNT_LOG : integer := 5;
	
	type STATE_TYPE is (    STATE_IRQ_START, STATE_IRQ_END, STATE_IRQ_MIDDLE,
                            STATE_START_X_0, STATE_START_X_1, STATE_READ_X_0, STATE_READ_X_1, STATE_READ_X_2, STATE_CHK_X, STATE_AVG_X,
                            STATE_START_Y_0, STATE_START_Y_1, STATE_READ_Y_0, STATE_READ_Y_1, STATE_READ_Y_2, STATE_CHK_Y, STATE_AVG_Y, STATE_SCALE );
                            
    signal state : STATE_TYPE;

    signal irq_s_0, irq_s_1 : std_logic;

    signal rb_info : unsigned(31 downto 0);

    signal sm_start, sm_ready, sm_conti : std_logic;
    signal sm_txdata, sm_rxdata         : std_logic_vector(7 downto 0);
    
    signal x_pos, y_pos           : unsigned(11 downto 0);
    signal x_pos_last, y_pos_last : unsigned(11 downto 0);
    signal x_pos_sum, y_pos_sum   : unsigned(11 + C_AVG_COUNT_LOG downto 0);
    signal pos_sum_count          : unsigned(C_AVG_COUNT_LOG downto 0);

    signal x_pos_avg, y_pos_avg : std_logic_vector(11 downto 0);
    signal x_pos_s, y_pos_s     : std_logic_vector(11 downto 0);

    signal ctrl_avg : std_logic_vector(3 downto 0);

    signal ignore, ret : std_logic_vector(31 downto 0);

    signal scale_start, scale_done, scale_idle, scale_ready : std_logic;
    signal scale_x_pos_s, scale_y_pos_s                     : std_logic_vector(11 downto 0);
    signal scale_x_pos_s_vld, scale_y_pos_s_vld             : std_logic;

    signal cycle_cnt : unsigned(19 downto 0);


    component scale is
        port (
            ap_clk   : in  std_logic;
            ap_rst   : in  std_logic;
            ap_start : in  std_logic;
            ap_done  : out std_logic;
            ap_idle  : out std_logic;
            ap_ready : out std_logic;

            x_u_V        : in std_logic_vector (11 downto 0);
            y_u_V        : in std_logic_vector (11 downto 0);
            x_s_V        : out std_logic_vector (11 downto 0);
            x_s_V_ap_vld : out std_logic;
            y_s_V        : out std_logic_vector (11 downto 0);
            y_s_V_ap_vld : out std_logic
        );
    end component;
	
	component spi_master is
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
    end component;
	

begin
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
	    else
	      if (axi_awready = '0' and S_AXI_AWVALID = '1' and S_AXI_WVALID = '1') then
	        -- slave is ready to accept write address when
	        -- there is a valid write address and write data
	        -- on the write address and data bus. This design 
	        -- expects no outstanding transactions. 
	        axi_awready <= '1';
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
	      if (axi_awready = '0' and S_AXI_AWVALID = '1' and S_AXI_WVALID = '1') then
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
	      if (axi_wready = '0' and S_AXI_WVALID = '1' and S_AXI_AWVALID = '1') then
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

	process (slv_reg0, slv_reg1, slv_reg2, slv_reg3, axi_araddr, S_AXI_ARESETN, slv_reg_rden)
	variable loc_addr :std_logic_vector(OPT_MEM_ADDR_BITS downto 0);
	begin
	    -- Address decoding for reading registers
	    loc_addr := axi_araddr(ADDR_LSB + OPT_MEM_ADDR_BITS downto ADDR_LSB);
	    case loc_addr is
	      when b"00" =>
	        reg_data_out <= slv_reg0;
	      when b"01" =>
	        reg_data_out <= slv_reg1;
	      when b"10" =>
	        reg_data_out <= slv_reg2;
	      when b"11" =>
	        reg_data_out <= slv_reg3;
	      when others =>
	        reg_data_out  <= (others => '0');
	    end case;
	end process; 

	-- Output register or memory read data
	process( S_AXI_ACLK ) is
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
	end process;


	-- Add user logic here
    osfsm_proc: process (S_AXI_ACLK,S_AXI_ARESETN) is
        begin
            if S_AXI_ARESETN = '0' then
                
                slv_reg0 <= '0' & (30 downto 0 => '0');
                slv_reg1 <= '0' & (30 downto 0 => '0');
                slv_reg2 <= (others => '0');
                slv_reg3 <= (others => '0');
                
                cycle_cnt <= (others=>'0');
                
                pos_sum_count <= (others => '0');
    
                state <= STATE_IRQ_START;
            elsif rising_edge(S_AXI_ACLK) then
                  
                 case state is 
                    when STATE_IRQ_START =>
                        x_pos_sum     <= (others => '0');
                        pos_sum_count <= (others => '0');
    
                        if irq_s_0 = '0' then
                            state <= STATE_START_X_0;
                        end if;
    
                    when STATE_START_X_0 =>
                         state <= STATE_READ_X_0;
                    when STATE_START_X_1 =>
                         state <= STATE_READ_X_1;
                    when STATE_READ_X_0 =>
                        if sm_ready = '1' then
                            state <= STATE_READ_X_1;
                        end if;
    
                    when STATE_READ_X_1 =>
                        if sm_ready = '1' then
                            x_pos(11 downto 5) <= unsigned(sm_rxdata(6 downto 0));    
                            state <= STATE_READ_X_2;
                        end if;
    
                    when STATE_READ_X_2 =>
                        if sm_ready = '1' then
                            x_pos(4 downto 0) <= unsigned(sm_rxdata(7 downto 3));
    
                            pos_sum_count <= pos_sum_count + 1;
                            if pos_sum_count = 0 then
                                state <= STATE_START_X_1;
                            elsif pos_sum_count = C_AVG_COUNT + 1 then
                                state <= STATE_IRQ_MIDDLE;
                            elsif pos_sum_count = 1 then
                                state <= STATE_AVG_X;
                            else
                                state <= STATE_CHK_X;
                            end if;
                        end if;
    
                    when STATE_CHK_X =>
                         if x_pos_last > x_pos then
                            if (x_pos_last - x_pos) > 8 then
                                x_pos_sum     <= (others => '0');
                                pos_sum_count <= (others => '0');
    
                                state <= STATE_START_X_1;
                            else
                                state <= STATE_AVG_X;
                            end if;
                        else
                            if (x_pos - x_pos_last) > 8 then
                                x_pos_sum     <= (others => '0');
                                pos_sum_count <= (others => '0');
    
                                state <= STATE_START_X_1;
                            else
                                state <= STATE_AVG_X;
                            end if;
                        end if;
    
                    when STATE_AVG_X =>
                        x_pos_last <= x_pos;
                        x_pos_sum <= x_pos_sum + x_pos;
    
                        state <= STATE_START_X_1;
    
                    when STATE_IRQ_MIDDLE =>
                        y_pos_sum     <= (others => '0');
                        pos_sum_count <= (others => '0');
    
                        if irq_s_0 = '0' then
                            state <= STATE_START_Y_0;
                        else
                            state <= STATE_IRQ_START;
                        end if;
    
                    when STATE_START_Y_0 =>
                         state <= STATE_READ_Y_0;
    
                    when STATE_START_Y_1 =>
                         state <= STATE_READ_Y_1;
    
                    when STATE_READ_Y_0 =>
                        if sm_ready = '1' then
                            state <= STATE_READ_Y_1;
                        end if;
    
                    when STATE_READ_Y_1 =>
                        if sm_ready = '1' then
                            y_pos(11 downto 5) <= unsigned(sm_rxdata(6 downto 0));
    
                            state <= STATE_READ_Y_2;
                        end if;
    
                    when STATE_READ_Y_2 =>
                        if sm_ready = '1' then
                            y_pos(4 downto 0) <= unsigned(sm_rxdata(7 downto 3));
    
                            pos_sum_count <= pos_sum_count + 1;
                            if pos_sum_count = 0 then
                                state <= STATE_START_Y_1;
                            elsif pos_sum_count = C_AVG_COUNT + 1 then
                                state <= STATE_IRQ_END;
                            elsif pos_sum_count = 1 then
                                state <= STATE_AVG_Y;
                            else
                                state <= STATE_CHK_Y;
                            end if;
                        end if;
    
                    when STATE_CHK_Y =>
                        if y_pos_last > y_pos then
                            if (y_pos_last - y_pos) > 8 then
                                y_pos_sum     <= (others => '0');
                                pos_sum_count <= (others => '0');
    
                                state <= STATE_START_Y_1;
                            else
                                state <= STATE_AVG_Y;
                            end if;
                        else
                            if (y_pos - y_pos_last) > 8 then
                                y_pos_sum     <= (others => '0');
                                pos_sum_count <= (others => '0');
    
                                state <= STATE_START_Y_1;
                            else
                                state <= STATE_AVG_Y;
                            end if;
                        end if;
    
                    when STATE_AVG_Y =>
                        y_pos_last <= y_pos;
                        y_pos_sum <= y_pos_sum + y_pos;
    
                        state <= STATE_START_Y_1;
    
                    when STATE_IRQ_END =>
                         if irq_s_0 = '0' then
                            state <= STATE_SCALE;
                        else
                            state <= STATE_IRQ_START;                                                    
                        end if;
    
                    when STATE_SCALE =>
                        if scale_done = '1' then
                            state <= STATE_IRQ_START;
                        end if;
    
                        cycle_cnt <= cycle_cnt + 1;
    
    
                        if scale_x_pos_s_vld = '1' then
                           slv_reg0 <= std_logic_vector(cycle_cnt) & scale_x_pos_s;
                        end if;
    
                        if scale_y_pos_s_vld = '1' then
                           slv_reg1 <= std_logic_vector(cycle_cnt) & scale_y_pos_s;
                        end if;   
                end case;
            end if;
        end process osfsm_proc;
        
        

        
        
--        process(state) begin
--        case state is
--          when STATE_IRQ_START  => slv_reg0  <= x"00000001";
--          when STATE_IRQ_END    => slv_reg0  <= x"00000002";
--          when STATE_IRQ_MIDDLE => slv_reg0  <= x"00000003";
--          when STATE_START_X_0  => slv_reg0  <= x"00000004";
--          when STATE_START_X_1  => slv_reg0  <= x"00000005";
--          when STATE_READ_X_0   => slv_reg0  <= x"00000006";
--          when STATE_READ_X_1   => slv_reg0  <= x"00000007";
--          when STATE_READ_X_2   => slv_reg0  <= x"00000008";
--          when STATE_CHK_X      => slv_reg0  <= x"00000009";
--          when STATE_AVG_X      => slv_reg0  <= x"0000000A";
--          when STATE_START_Y_0  => slv_reg0  <= x"0000000B";
--          when STATE_START_Y_1  => slv_reg0  <= x"0000000C";
       
--          when STATE_READ_Y_0   => slv_reg0  <= x"0000000D";
--          when STATE_READ_Y_1   => slv_reg0  <= x"0000000E";
--          when STATE_READ_Y_2   => slv_reg0  <= x"0000000F";
--          when STATE_CHK_Y      => slv_reg0  <= x"00000010";
--          when STATE_AVG_Y      => slv_reg0  <= x"00000011";
--          when STATE_SCALE      => slv_reg0  <= x"00000012";
--        end case;
--        end process;
    
        x_pos_avg <= std_logic_vector(x_pos_sum(11 + C_AVG_COUNT_LOG downto C_AVG_COUNT_LOG));
        y_pos_avg <= std_logic_vector(y_pos_sum(11 + C_AVG_COUNT_LOG downto C_AVG_COUNT_LOG));
    
        sync_proc : process(S_AXI_ACLK) is
        begin
            if rising_edge(S_AXI_ACLK) then
                irq_s_1 <= TC_IRQ;
                irq_s_0 <= irq_s_1;
            end if;
        end process sync_proc;
    
        sm_start <= '1' when state = STATE_START_X_0 else
                    '1' when state = STATE_START_X_1 else
                    '1' when state = STATE_READ_X_0 else
                    '1' when state = STATE_READ_X_1 else
                    '1' when state = STATE_START_Y_0 else
                    '1' when state = STATE_START_Y_1 else
                    '1' when state = STATE_READ_Y_0 else
                    '1' when state = STATE_READ_Y_1 else
                    '0';
    
        sm_conti <= '0' when state = STATE_READ_X_1 and pos_sum_count = C_AVG_COUNT + 1 else
                    '0' when state = STATE_READ_Y_1 and pos_sum_count = C_AVG_COUNT + 1 else
                    '1';
    
        -- Start A2 A1 A0 Mode SER PD1 PD0
        --       0  0  1                    = y pos
        --       1  0  1                    = x pos
        --                0                 = 12bit conversion
        --                1                 = 8bit conversion
        --                     0            = internal reference
        --                     1            = external reference
        --                         0   0    = power down
        --                         0   1    = power down without penirq
        --                         1   0    = reserved
        --                         1   1    = no power down
        sm_txdata <= "11010000" when state = STATE_START_X_0 else
                     "11010000" when state = STATE_READ_X_1 and pos_sum_count /= C_AVG_COUNT + 1 else
                     "10010000" when state = STATE_START_Y_0 else
                     "10010000" when state = STATE_READ_Y_1 and pos_sum_count /= C_AVG_COUNT + 1 else
                     (others => '0');
    
    
 
        sm_inst : spi_master
            generic map (
                G_SM_CLK_PRD  => 10ns,
                G_SPI_CLK_PRD => 400ns,
    
                G_DATA_LEN => 8
            )
            port map (
                SPI_SCLK => TC_SCLK,
                SPI_MOSI => TC_MOSI,
                SPI_MISO => TC_MISO,
                SPI_SSn  => TC_SSn,
    
                SM_TxData => sm_txdata,
                SM_RxData => sm_rxdata,
                SM_Start  => sm_start,
                SM_Ready  => sm_ready,
                SM_Conti  => sm_conti,
                SM_Clk    => S_AXI_ACLK
            );
    
        scale_start <= '1' when state = STATE_SCALE else '0';
    
        scale_inst : scale
            port map (
                ap_clk => S_AXI_ACLK,
                ap_rst => (not S_AXI_ARESETN),
                ap_start => scale_start,
                ap_done  => scale_done,
                ap_idle  => scale_idle,
                ap_ready => scale_ready,
                x_u_V => x_pos_avg,
                y_u_V => y_pos_avg,
                x_s_V => scale_x_pos_s,
                x_s_V_ap_vld => scale_x_pos_s_vld,
                y_s_V => scale_y_pos_s,
                y_s_V_ap_vld => scale_y_pos_s_vld
            );



	-- User logic ends

end arch_imp;
