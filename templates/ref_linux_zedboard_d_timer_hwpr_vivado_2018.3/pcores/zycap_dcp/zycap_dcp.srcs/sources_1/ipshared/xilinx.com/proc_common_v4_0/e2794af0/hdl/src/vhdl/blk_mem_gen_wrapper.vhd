-- blk_mem_gen_wrapper.vhd - entity/architecture pair
-------------------------------------------------------------------------------
--
-- ****************************************************************************
-- **  DISCLAIMER OF LIABILITY                                               **
-- **                                                                        **
-- **  This text/file contains proprietary, confidential                     **
-- **  information of Xilinx, Inc., is distributed under                     **
-- **  license from Xilinx, Inc., and may be used, copied                    **
-- **  and/or disclosed only pursuant to the terms of a valid                **
-- **  license agreement with Xilinx, Inc. Xilinx hereby                     **
-- **  grants you a license to use this text/file solely for                 **
-- **  design, simulation, implementation and creation of                    **
-- **  design files limited to Xilinx devices or technologies.               **
-- **  Use with non-Xilinx devices or technologies is expressly              **
-- **  prohibited and immediately terminates your license unless             **
-- **  covered by a separate agreement.                                      **
-- **                                                                        **
-- **  Xilinx is providing this design, code, or information                 **
-- **  "as-is" solely for use in developing programs and                     **
-- **  solutions for Xilinx devices, with no obligation on the               **
-- **  part of Xilinx to provide support. By providing this design,          **
-- **  code, or information as one possible implementation of                **
-- **  this feature, application or standard, Xilinx is making no            **
-- **  representation that this implementation is free from any              **
-- **  claims of infringement. You are responsible for obtaining             **
-- **  any rights you may require for your implementation.                   **
-- **  Xilinx expressly disclaims any warranty whatsoever with               **
-- **  respect to the adequacy of the implementation, including              **
-- **  but not limited to any warranties or representations that this        **
-- **  implementation is free from claims of infringement, implied           **
-- **  warranties of merchantability or fitness for a particular             **
-- **  purpose.                                                              **
-- **                                                                        **
-- **  Xilinx products are not intended for use in life support              **
-- **  appliances, devices, or systems. Use in such applications is          **
-- **  expressly prohibited.                                                 **
-- **                                                                        **
-- **  Any modifications that are made to the Source Code are                **
-- **  done at the users sole risk and will be unsupported.                  **
-- **  The Xilinx Support Hotline does not have access to source             **
-- **  code and therefore cannot answer specific questions related           **
-- **  to source HDL. The Xilinx Hotline support of original source          **
-- **  code IP shall only address issues and questions related               **
-- **  to the standard Netlist version of the core (and thus                 **
-- **  indirectly, the original core source).                                **
-- **                                                                        **
-- **  Copyright (c) 2008, 2009. 2010 Xilinx, Inc. All rights reserved.      **
-- **                                                                        **
-- **  This copyright and support notice must be retained as part            **
-- **  of this text at all times.                                            **
-- ****************************************************************************
--
-------------------------------------------------------------------------------
-- Filename:        blk_mem_gen_wrapper.vhd
-- Version:         v1.00a
-- Description:
--  This wrapper file performs the direct call to Block Memory Generator
--  during design implementation
-------------------------------------------------------------------------------
--
-------------------------------------------------------------------------------
-- VHDL-Standard:   VHDL'93
-------------------------------------------------------------------------------
-- Structure:
--              blk_mem_gen_wrapper.vhd
--                    |
--                    |-- blk_mem_gen_v2_7
--                    |
--                    |-- blk_mem_gen_v6_2
--
-------------------------------------------------------------------------------
-- Revision History:
--
--
-- Author:          MW
-- Revision:        $Revision: 1.1.2.69 $
-- Date:            $7/11/2008$
--
-- History:
--   MW   7/11/2008       Initial Version
--   MSH  2/26/2009       Add new blk_mem_gen version
--
--     DET     4/8/2009     EDK 11.2
-- ~~~~~~
--     - Added blk_mem_gen_v3_2 instance callout
-- ^^^^^^
--
--     DET     2/9/2010     for EDK 12.1
-- ~~~~~~
--     - Updated the the Blk Mem Gen version from blk_mem_gen_v3_2 
--       to blk_mem_gen_v3_3 (for the S6/V6 IfGen case)
-- ^^^^^^
--
--     DET     3/10/2010     For EDK 12.x
-- ~~~~~~
--   -- Per CR553307
--     - Updated the the Blk Mem Gen version from blk_mem_gen_v3_3 
--       to blk_mem_gen_v4_1 (for the S6/V6 IfGen case)
-- ^^^^^^
--
--     DET     3/17/2010     Initial
-- ~~~~~~
--    -- Per CR554253
--     - Incorporated changes to comment out FLOP_DELAY parameter from the 
--       blk_mem_gen_v4_1 instance. This parameter is on the XilinxCoreLib
--       model for blk_mem_gen_v4_1 but is declared as a TIME type for the
--       vhdl version and an integer for the verilog. 
-- ^^^^^^
--
--     DET     6/18/2010     EDK_MS2
-- ~~~~~~
--    -- Per IR565916
--     - Added constants  FAM_IS_V6_OR_S6 and FAM_IS_NOT_V6_OR_S6.
--     - Added derivative part type checks for S6 or V6.
-- ^^^^^^
--
--     DET     8/27/2010     EDK 12.4
-- ~~~~~~
--    -- Per CR573867
--     - Added the the Blk Mem Gen version blk_mem_gen_v4_3 for the S6/V6
--       and later build case.
--     - Updated method for derivative part support using new family
--       aliasing function in family_support.vhd.
--     - Incorporated an implementation to deal with unsupported FPGA
--       parts passed in on the C_FAMILY parameter.
-- ^^^^^^
--
--     DET     10/4/2010     EDK 13.1
-- ~~~~~~
--     - Updated to blk_mem_gen V5.2.
-- ^^^^^^
--
--     DET     12/8/2010     EDK 13.1
-- ~~~~~~
--    -- Per CR586109
--     - Updated to blk_mem_gen V6.1
-- ^^^^^^
--
--     DET     12/17/2010     EDK 13.1
-- ~~~~~~
--    -- Per CR587494
--     - Regressed back to blk_mem_gen V5.2
-- ^^^^^^
--
--     DET     3/2/2011     EDk 13.2
-- ~~~~~~
--    -- Per CR595473
--     - Update to use blk_mem_gen_v6_2 for s6, v6, and later.
-- ^^^^^^
--
--     DET     3/3/2011     EDK 13.2
-- ~~~~~~
--     - Removed C_ELABORATION_DIR parameter from the blk_mem_gen_v6_2
--       instance.
-- ^^^^^^
--
-------------------------------------------------------------------------------


LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

-- synopsys translate_off
--Library XilinxCoreLib;
-- synopsys translate_on

library blk_mem_gen_v8_2;
library proc_common_v4_0;
use blk_mem_gen_v8_2.all;
--use proc_common_v4_0.coregen_comp_defs.all;
use proc_common_v4_0.family_support.all;


------------------------------------------------------------------------------
-- Port Declaration
------------------------------------------------------------------------------
entity blk_mem_gen_wrapper is
   generic
      (
      -- Device Family
      c_family                 : string  := "virtex5";
         -- "Virtex2"
         -- "Virtex4"
         -- "Virtex5"
      c_xdevicefamily          : string  := "virtex5";
         -- Finest Resolution Device Family
            -- "Virtex2"
            -- "Virtex2-Pro"
            -- "Virtex4"
            -- "Virtex5"
            -- "Spartan-3A"
            -- "Spartan-3A DSP"
      c_elaboration_dir        : string := "";
      -- Memory Specific Configurations
      c_mem_type               : integer := 2;
         -- This wrapper only supports the True Dual Port RAM
         -- 0: Single Port RAM
         -- 1: Simple Dual Port RAM
         -- 2: True Dual Port RAM
         -- 3: Single Port Rom
         -- 4: Dual Port RAM
      c_algorithm              : integer := 1;
         -- 0: Selectable Primative
         -- 1: Minimum Area
      c_prim_type              : integer := 1;
         -- 0: ( 1-bit wide)
         -- 1: ( 2-bit wide)
         -- 2: ( 4-bit wide)
         -- 3: ( 9-bit wide)
         -- 4: (18-bit wide)
         -- 5: (36-bit wide)
         -- 6: (72-bit wide, single port only)
      c_byte_size              : integer := 9;   -- 8 or 9

      -- Simulation Behavior Options
      c_sim_collision_check    : string  :=  "NONE";
         -- "None"
         -- "Generate_X"
         -- "All"
         -- "Warnings_only"
      c_common_clk             : integer := 1;   -- 0, 1
      c_disable_warn_bhv_coll  : integer := 0;   -- 0, 1
      c_disable_warn_bhv_range : integer := 0;   -- 0, 1

      -- Initialization Configuration Options
      c_load_init_file         : integer := 0;
      c_init_file_name         : string  := "no_coe_file_loaded";
      c_use_default_data       : integer := 0;   -- 0, 1
      c_default_data           : string  := "0"; -- "..."

      -- Port A Specific Configurations
      c_has_mem_output_regs_a  : integer := 0;   -- 0, 1
      c_has_mux_output_regs_a  : integer := 0;   -- 0, 1
      c_write_width_a          : integer := 32;  -- 1 to 1152
      c_read_width_a           : integer := 32;  -- 1 to 1152
      c_write_depth_a          : integer := 64;  -- 2 to 9011200
      c_read_depth_a           : integer := 64;  -- 2 to 9011200
      c_addra_width            : integer := 6;   -- 1 to 24
      c_write_mode_a           : string  := "WRITE_FIRST";
         -- "Write_First"
         -- "Read_first"
         -- "No_Change"
      c_has_ena                : integer := 1;   -- 0, 1
      c_has_regcea             : integer := 0;   -- 0, 1
      c_has_ssra               : integer := 0;   -- 0, 1
      c_sinita_val             : string  := "0"; --"..."
      c_use_byte_wea           : integer := 0;   -- 0, 1
      c_wea_width              : integer := 1;   -- 1 to 128

      -- Port B Specific Configurations
      c_has_mem_output_regs_b  : integer := 0;   -- 0, 1
      c_has_mux_output_regs_b  : integer := 0;   -- 0, 1
      c_write_width_b          : integer := 32;  -- 1 to 1152
      c_read_width_b           : integer := 32;  -- 1 to 1152
      c_write_depth_b          : integer := 64;  -- 2 to 9011200
      c_read_depth_b           : integer := 64;  -- 2 to 9011200
      c_addrb_width            : integer := 6;   -- 1 to 24
      c_write_mode_b           : string  := "WRITE_FIRST";
         -- "Write_First"
         -- "Read_first"
         -- "No_Change"
      c_has_enb                : integer := 1;   -- 0, 1
      c_has_regceb             : integer := 0;   -- 0, 1
      c_has_ssrb               : integer := 0;   -- 0, 1
      c_sinitb_val             : string  := "0"; -- "..."
      c_use_byte_web           : integer := 0;   -- 0, 1
      c_web_width              : integer := 1;   -- 1 to 128

      -- Other Miscellaneous Configurations
      c_mux_pipeline_stages    : integer := 0;   -- 0, 1, 2, 3
         -- The number of pipeline stages within the MUX
         --    for both Port A and Port B
      c_use_ecc                : integer := 0;
         -- See DS512 for the limited core option selections for ECC support
      c_use_ramb16bwer_rst_bhv : integer := 0--;   --0, 1
--      c_corename               : string  := "blk_mem_gen_v2_7"
      --Uncommenting the above parameter (C_CORENAME) will cause
      --the a failure in NGCBuild!!!

      );
   port
      (
      clka    : in  std_logic;
      ssra    : in  std_logic := '0';
      dina    : in  std_logic_vector(c_write_width_a-1 downto 0) := (OTHERS => '0');
      addra   : in  std_logic_vector(c_addra_width-1   downto 0);
      ena     : in  std_logic := '1';
      regcea  : in  std_logic := '1';
      wea     : in  std_logic_vector(c_wea_width-1     downto 0) := (OTHERS => '0');
      douta   : out std_logic_vector(c_read_width_a-1  downto 0);


      clkb    : in  std_logic := '0';
      ssrb    : in  std_logic := '0';
      dinb    : in  std_logic_vector(c_write_width_b-1 downto 0) := (OTHERS => '0');
      addrb   : in  std_logic_vector(c_addrb_width-1   downto 0) := (OTHERS => '0');
      enb     : in  std_logic := '1';
      regceb  : in  std_logic := '1';
      web     : in  std_logic_vector(c_web_width-1     downto 0) := (OTHERS => '0');
      doutb   : out std_logic_vector(c_read_width_b-1  downto 0);

      dbiterr : out std_logic;
         -- Double bit error that that cannot be auto corrected by ECC
      sbiterr : out std_logic
         -- Single Bit Error that has been auto corrected on the output bus        
      );
end entity blk_mem_gen_wrapper;

architecture implementation of blk_mem_gen_wrapper is

 
   
    Constant FAMILY_TO_USE        : string  := get_root_family(C_FAMILY);  -- function from family_support.vhd
    
    
    Constant FAMILY_NOT_SUPPORTED : boolean := (equalIgnoringCase(FAMILY_TO_USE, "nofamily"));
    
    Constant FAMILY_IS_SUPPORTED  : boolean := not(FAMILY_NOT_SUPPORTED);
    
    
    --Constant FAM_IS_S3_V4_V5      : boolean := (equalIgnoringCase(FAMILY_TO_USE, "spartan3" ) or 
    --                                            equalIgnoringCase(FAMILY_TO_USE, "virtex4"  ) or 
    --                                            equalIgnoringCase(FAMILY_TO_USE, "virtex5")) and
    --                                            FAMILY_IS_SUPPORTED;
    --
    --Constant FAM_IS_NOT_S3_V4_V5  : boolean := not(FAM_IS_S3_V4_V5) and
    --                                           FAMILY_IS_SUPPORTED;
    
  --Signals added to fix MTI and XSIM issues caused by fix for VCS issues not to use "LIBRARY_SCAN = TRUE"    
    signal RDADDRECC            : STD_LOGIC_VECTOR(c_addrb_width-1 DOWNTO 0);
    signal S_AXI_AWREADY        : STD_LOGIC;
    signal S_AXI_WREADY         : STD_LOGIC;
    signal S_AXI_BID            : STD_LOGIC_VECTOR(3 DOWNTO 0);
    signal S_AXI_BRESP          : STD_LOGIC_VECTOR(1 DOWNTO 0);
    signal S_AXI_BVALID         : STD_LOGIC;
    signal S_AXI_ARREADY        : STD_LOGIC;
    signal S_AXI_RID            : STD_LOGIC_VECTOR(3 DOWNTO 0);
    signal S_AXI_RDATA          : STD_LOGIC_VECTOR(c_write_width_b-1 DOWNTO 0);
    signal S_AXI_RRESP          : STD_LOGIC_VECTOR(1 DOWNTO 0);
    signal S_AXI_RLAST          : STD_LOGIC;
    signal S_AXI_RVALID         : STD_LOGIC;
    signal S_AXI_SBITERR        : STD_LOGIC;
    signal S_AXI_DBITERR        : STD_LOGIC;
    signal S_AXI_RDADDRECC      : STD_LOGIC_VECTOR(c_addrb_width-1 DOWNTO 0);
    signal S_AXI_WSTRB          : STD_LOGIC_VECTOR(c_wea_width-1 downto 0);
    signal S_AXI_WDATA          : STD_LOGIC_VECTOR(c_write_width_a-1 downto 0);


 
begin

 
  S_AXI_WSTRB <= (others => '0'); 
  S_AXI_WDATA <= (others => '0'); 
  ------------------------------------------------------------
  -- If Generate
  --
  -- Label: GEN_NO_FAMILY
  --
  -- If Generate Description:
  --   This IfGen is implemented if an unsupported FPGA family
  -- is passed in on the C_FAMILY parameter,
  --
  ------------------------------------------------------------
  GEN_NO_FAMILY : if (FAMILY_NOT_SUPPORTED) generate
     
     
     begin
  
       
       -- synthesis translate_off
  
        
        -------------------------------------------------------------
        -- Combinational Process
        --
        -- Label: DO_ASSERTION
        --
        -- Process Description:
        -- Generate a simulation error assertion for an unsupported 
        -- FPGA family string passed in on the C_FAMILY parameter.
        --
        -------------------------------------------------------------
        DO_ASSERTION : process 
           begin
        
        
             -- Wait until second rising clock edge to issue assertion
             Wait until clka = '1';
             wait until clka = '0';
             Wait until clka = '1';
             
             -- Report an error in simulation environment
             assert FALSE report "********* UNSUPPORTED FPGA DEVICE! Check C_FAMILY parameter assignment!" 
                          severity ERROR;
  
             Wait; -- halt this process
             
           end process DO_ASSERTION; 
        
        
        
       -- synthesis translate_on
       


       
    
       -- Tie outputs to logic low
       douta   <= (others => '0');  -- : out std_logic_vector(c_read_width_a-1  downto 0);
       doutb   <= (others => '0');  -- : out std_logic_vector(c_read_width_b-1  downto 0);
       dbiterr <= '0'            ;  -- : out std_logic;
       sbiterr <= '0'            ;  -- : out std_logic
      
    
     end generate GEN_NO_FAMILY;
   
 
  ------------------------------------------------------------
  -- If Generate
  --
  -- Label: V6_S6_AND_LATER
  --
  -- If Generate Description:
  --  This IFGen Implements the Block Memeory using blk_mem_gen 5.2.
  --  This is for new cores designed and tested with FPGA 
  --  Families of Virtex-6, Spartan-6 and later.
  --
  ------------------------------------------------------------
  FAMILY_SUPPORTED: if(FAMILY_IS_SUPPORTED) generate
  begin
   
   
    -------------------------------------------------------------------------------
    -- Instantiate the generalized FIFO Generator instance
    --
    -- NOTE:
    -- DO NOT CHANGE TO DIRECT ENTITY INSTANTIATION!!!
    -- This is a Coregen Block Memory Generator Call module  
    -- for new IP BRAM implementations.
    --
    -------------------------------------------------------------------------------
    I_TRUE_DUAL_PORT_BLK_MEM_GEN : entity blk_mem_gen_v8_2.blk_mem_gen_v8_2
      generic map
        (
        --C_CORENAME                => c_corename              ,                                       
        
        -- Device Family
        C_FAMILY                    => FAMILY_TO_USE           ,     
        C_XDEVICEFAMILY             => c_xdevicefamily         ,
        C_ELABORATION_DIR           => c_elaboration_dir       ,     
                                                                                    
        ------------------     
        C_INTERFACE_TYPE            => 0                       ,
        C_USE_BRAM_BLOCK            => 0                       ,
        C_AXI_TYPE                  => 0                       ,
        C_AXI_SLAVE_TYPE            => 0                       ,
        C_HAS_AXI_ID                => 0                       ,
        C_AXI_ID_WIDTH              => 4                       ,
        ------------------ 
            
        -- Memory Specific Configurations                               
        C_MEM_TYPE                  => c_mem_type              ,     
        C_BYTE_SIZE                 => c_byte_size             ,     
        C_ALGORITHM                 => c_algorithm             ,     
        C_PRIM_TYPE                 => c_prim_type             ,     
                                                                                                      
                                                                                                      
        C_LOAD_INIT_FILE            => c_load_init_file        ,                                      
        C_INIT_FILE_NAME            => c_init_file_name        ,
        C_INIT_FILE                 => ""                      ,                                      
        C_USE_DEFAULT_DATA          => c_use_default_data      ,                                      
        C_DEFAULT_DATA              => c_default_data          ,                                      
                                                                                                   
        -- Port A Specific Configurations                                                          
        --C_RST_TYPE                  => "SYNC"                  ,  --Removed in version v8_2  
        C_HAS_RSTA                  => c_has_ssra              ,                
        C_RST_PRIORITY_A            => "CE"                    ,                             
        C_RSTRAM_A                  => 0                       ,  
        C_INITA_VAL                 => c_sinita_val            ,                               
        C_HAS_ENA                   => c_has_ena               ,                               
        C_HAS_REGCEA                => c_has_regcea            ,                               
        C_USE_BYTE_WEA              => c_use_byte_wea          ,                                
        C_WEA_WIDTH                 => c_wea_width             ,                                
        C_WRITE_MODE_A              => c_write_mode_a          ,                               
        C_WRITE_WIDTH_A             => c_write_width_a         ,                               
        C_READ_WIDTH_A              => c_read_width_a          ,                               
        C_WRITE_DEPTH_A             => c_write_depth_a         ,                               
        C_READ_DEPTH_A              => c_read_depth_a          ,                               
        C_ADDRA_WIDTH               => c_addra_width           ,                               
                                                                                                
        -- Port B Specific Configurations                             
        C_HAS_RSTB                  => c_has_ssrb              ,         
        C_RST_PRIORITY_B            => "CE"                    ,
        C_RSTRAM_B                  => 0                       ,   
        C_INITB_VAL                 => c_sinitb_val            ,         
        C_HAS_ENB                   => c_has_enb               ,         
        C_HAS_REGCEB                => c_has_regceb            ,         
        C_USE_BYTE_WEB              => c_use_byte_web          ,         
        C_WEB_WIDTH                 => c_web_width             ,         
        C_WRITE_MODE_B              => c_write_mode_b          ,         
        C_WRITE_WIDTH_B             => c_write_width_b         ,         
        C_READ_WIDTH_B              => c_read_width_b          ,         
        C_WRITE_DEPTH_B             => c_write_depth_b         ,         
        C_READ_DEPTH_B              => c_read_depth_b          ,         
        C_ADDRB_WIDTH               => c_addrb_width           ,         
        C_HAS_MEM_OUTPUT_REGS_A     => c_has_mem_output_regs_a ,                                      
        C_HAS_MEM_OUTPUT_REGS_B     => c_has_mem_output_regs_b ,         
        C_HAS_MUX_OUTPUT_REGS_A     => c_has_mux_output_regs_a ,                                                                                     
        C_HAS_MUX_OUTPUT_REGS_B     => c_has_mux_output_regs_b ,
        C_HAS_SOFTECC_INPUT_REGS_A  => 0                       ,   
        C_HAS_SOFTECC_OUTPUT_REGS_B => 0                       ,   
                                                                      
        
        -- Other Miscellaneous Configurations                         
        C_MUX_PIPELINE_STAGES       => c_mux_pipeline_stages   ,         
        C_USE_SOFTECC               => 0                       ,   
        C_USE_ECC                   => c_use_ecc               ,
        C_EN_ECC_PIPE               => 0                       ,   
                                                              
        -- Simulation Behavior Options                                           
        C_HAS_INJECTERR             => 0                       ,                                                  
        C_SIM_COLLISION_CHECK       => c_sim_collision_check   ,                    
        C_COMMON_CLK                => c_common_clk            ,                    
        C_DISABLE_WARN_BHV_COLL     => c_disable_warn_bhv_coll ,
        C_EN_SLEEP_PIN              => 0                       ,                                      
        C_DISABLE_WARN_BHV_RANGE    => c_disable_warn_bhv_range                                      
                                                                                                   
        )                                                              
                                                                       
      port map                                                          
        (                                                              
        CLKA                  => clka            ,                                           
        RSTA                  => ssra            ,                                           
        ENA                   => ena             ,                                            
        REGCEA                => regcea          ,                                         
        WEA                   => wea             ,                                            
        ADDRA                 => addra           ,                                          
        DINA                  => dina            ,                                           
        DOUTA                 => douta           ,                                          
        CLKB                  => clkb            ,                                           
        RSTB                  => ssrb            ,                                           
        ENB                   => enb             ,                                            
        REGCEB                => regceb          ,                                         
        WEB                   => web             ,                                            
        ADDRB                 => addrb           ,                                          
        DINB                  => dinb            ,                                           
        DOUTB                 => doutb           ,                                        
        INJECTSBITERR         => '0'             ,   -- input
        INJECTDBITERR         => '0'             ,   -- input
        SBITERR               => sbiterr         ,                                          
        DBITERR               => dbiterr         ,                                         
        RDADDRECC             => RDADDRECC       ,   -- output
        ECCPIPECE             => '0'             ,
        SLEEP                 => '0'             ,
      
        -- AXI BMG Input and Output Port Declarations                                                                          -- new for v6.2
                                                                                                                               -- new for v6.2
        -- AXI Global Signals                                                                                                  -- new for v6.2
        S_AClk                => '0'             ,   -- : IN  STD_LOGIC := '0';                                                -- new for v6.2
        S_ARESETN             => '0'             ,   -- : IN  STD_LOGIC := '0';                                                -- new for v6.2
                                                                                                                               -- new for v6.2
        -- AXI Full/Lite Slave Write (write side)                                                                              -- new for v6.2
        S_AXI_AWID            => "0000"          ,   -- : IN  STD_LOGIC_VECTOR(C_AXI_ID_WIDTH-1 DOWNTO 0) := (OTHERS => '0');  -- new for v6.2
        S_AXI_AWADDR          => "00000000000000000000000000000000"    ,   -- : IN  STD_LOGIC_VECTOR(31 DOWNTO 0) := (OTHERS => '0');                -- new for v6.2
        S_AXI_AWLEN           => "00000000"           ,   -- : IN  STD_LOGIC_VECTOR(7 DOWNTO 0) := (OTHERS => '0');                 -- new for v6.2
        S_AXI_AWSIZE          => "000"           ,   -- : IN  STD_LOGIC_VECTOR(2 DOWNTO 0) := (OTHERS => '0');                 -- new for v6.2
        S_AXI_AWBURST         => "00"            ,   -- : IN  STD_LOGIC_VECTOR(1 DOWNTO 0) := (OTHERS => '0');                 -- new for v6.2
        S_AXI_AWVALID         => '0'             ,   -- : IN  STD_LOGIC := '0';                                                -- new for v6.2
        S_AXI_AWREADY         => S_AXI_AWREADY   ,   -- : OUT STD_LOGIC;                                                       -- new for v6.2
        S_AXI_WDATA           => S_AXI_WDATA     ,   -- : IN  STD_LOGIC_VECTOR(C_WRITE_WIDTH_A-1 DOWNTO 0) := (OTHERS => '0'); -- new for v6.2
        S_AXI_WSTRB           => S_AXI_WSTRB     ,   -- : IN  STD_LOGIC_VECTOR(C_WEA_WIDTH-1 DOWNTO 0) := (OTHERS => '0');     -- new for v6.2
        S_AXI_WLAST           => '0'             ,   -- : IN  STD_LOGIC := '0';                                                -- new for v6.2
        S_AXI_WVALID          => '0'             ,   -- : IN  STD_LOGIC := '0';                                                -- new for v6.2
        S_AXI_WREADY          => S_AXI_WREADY    ,   -- : OUT STD_LOGIC;                                                       -- new for v6.2
        S_AXI_BID             => S_AXI_BID       ,   -- : OUT STD_LOGIC_VECTOR(C_AXI_ID_WIDTH-1 DOWNTO 0) := (OTHERS => '0');  -- new for v6.2
        S_AXI_BRESP           => S_AXI_BRESP     ,   -- : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);                                    -- new for v6.2
        S_AXI_BVALID          => S_AXI_BVALID    ,   -- : OUT STD_LOGIC;                                                       -- new for v6.2
        S_AXI_BREADY          => '0'             ,   -- : IN  STD_LOGIC := '0';                                                -- new for v6.2
                                                                                                  -- new for v6.2
        -- AXI Full/Lite Slave Read (Write side)                                                  -- new for v6.2
        S_AXI_ARID            => "0000"          ,   -- : IN  STD_LOGIC_VECTOR(C_AXI_ID_WIDTH-1 DOWNTO 0) := (OTHERS => '0');  -- new for v6.2
        S_AXI_ARADDR          => "00000000000000000000000000000000"    ,   -- : IN  STD_LOGIC_VECTOR(31 DOWNTO 0) := (OTHERS => '0');                -- new for v6.2
        S_AXI_ARLEN           => "00000000"           ,   -- : IN  STD_LOGIC_VECTOR(8-1 DOWNTO 0) := (OTHERS => '0');               -- new for v6.2
        S_AXI_ARSIZE          => "000"           ,   -- : IN  STD_LOGIC_VECTOR(2 DOWNTO 0) := (OTHERS => '0');                 -- new for v6.2
        S_AXI_ARBURST         => "00"            ,   -- : IN  STD_LOGIC_VECTOR(1 DOWNTO 0) := (OTHERS => '0');                 -- new for v6.2
        S_AXI_ARVALID         => '0'             ,   -- : IN  STD_LOGIC := '0';                                                -- new for v6.2
        S_AXI_ARREADY         => S_AXI_ARREADY   ,   -- : OUT STD_LOGIC;                                                       -- new for v6.2
        S_AXI_RID             => S_AXI_RID       ,   -- : OUT STD_LOGIC_VECTOR(C_AXI_ID_WIDTH-1 DOWNTO 0) := (OTHERS => '0');  -- new for v6.2
        S_AXI_RDATA           => S_AXI_RDATA     ,   -- : OUT STD_LOGIC_VECTOR(C_WRITE_WIDTH_B-1 DOWNTO 0);                    -- new for v6.2
        S_AXI_RRESP           => S_AXI_RRESP     ,   -- : OUT STD_LOGIC_VECTOR(2-1 DOWNTO 0);                                  -- new for v6.2
        S_AXI_RLAST           => S_AXI_RLAST     ,   -- : OUT STD_LOGIC;                                                       -- new for v6.2
        S_AXI_RVALID          => S_AXI_RVALID    ,   -- : OUT STD_LOGIC;                                                       -- new for v6.2
        S_AXI_RREADY          => '0'             ,   -- : IN  STD_LOGIC := '0';                                                -- new for v6.2
                                                                                                                               -- new for v6.2
        -- AXI Full/Lite Sideband Signals                                                                                      -- new for v6.2
        S_AXI_INJECTSBITERR   => '0'             ,   -- : IN  STD_LOGIC := '0';                                                -- new for v6.2
        S_AXI_INJECTDBITERR   => '0'             ,   -- : IN  STD_LOGIC := '0';                                                -- new for v6.2
        S_AXI_SBITERR         => S_AXI_SBITERR   ,   -- : OUT STD_LOGIC;                                                       -- new for v6.2
        S_AXI_DBITERR         => S_AXI_DBITERR   ,   -- : OUT STD_LOGIC;                                                       -- new for v6.2
        S_AXI_RDADDRECC       => S_AXI_RDADDRECC     -- : OUT STD_LOGIC_VECTOR(C_ADDRB_WIDTH-1 DOWNTO 0)                       -- new for v6.2

        );                                                              
  end generate FAMILY_SUPPORTED;
  
  
  
  
end implementation;                                                      


