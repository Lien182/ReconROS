
//*****************************************************************************
// (c) Copyright 2009 - 2010 Xilinx, Inc. All rights reserved.
//
// This file contains confidential and proprietary information
// of Xilinx, Inc. and is protected under U.S. and
// international copyright and other intellectual property
// laws.
//
// DISCLAIMER
// This disclaimer is not a license and does not grant any
// rights to the materials distributed herewith. Except as
// otherwise provided in a valid license issued to you by
// Xilinx, and to the maximum extent permitted by applicable
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
// (2) Xilinx shall not be liable (whether in contract or tort,
// including negligence, or under any other theory of
// liability) for any loss or damage of any kind or nature
// related to, arising under or in connection with these
// materials, including for any direct, or any indirect,
// special, incidental, or consequential loss or damage
// (including loss of data, profits, goodwill, or any type of
// loss or damage suffered as a result of any action brought
// by a third party) even if such damage or loss was
// reasonably foreseeable or Xilinx had been advised of the
// possibility of the same.
//
// CRITICAL APPLICATIONS
// Xilinx products are not designed or intended to be fail-
// safe, or for use in any application requiring fail-safe
// performance, such as life-support or safety devices or
// systems, Class III medical devices, nuclear facilities,
// applications related to the deployment of airbags, or any
// other applications that could lead to death, personal
// injury, or severe property or environmental damage
// (individually and collectively, "Critical
// Applications"). Customer assumes the sole risk and
// liability of any use of Xilinx products in Critical
// Applications, subject only to applicable laws and
// regulations governing limitations on product liability.
//
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
// PART OF THIS FILE AT ALL TIMES.
//
//*****************************************************************************
//   ____  ____
//  /   /\/   /
// /___/  \  /   Vendor             : Xilinx
// \   \   \/    Version            : 3.92
//  \   \        Application        : MIG
//  /   /        Filename           : DDR3_SDRAM.veo
// /___/   /\    Date Last Modified : $Date: 2011/06/02 07:17:54 $
// \   \  /  \   Date Created       : Wed May 13 2009
//  \___\/\___\
//
// Purpose          : Template file containing code that can be used as a model
//                    for instantiating a CORE Generator module in a HDL design.
// Revision History :
//*****************************************************************************

// The following must be inserted into your Verilog file for this
// core to be instantiated. Change the instance name and port connections
// (in parentheses) to your own signal names.

//----------- Begin Cut here for INSTANTIATION Template ---// INST_TAG

DDR3_SDRAM # (
  .REFCLK_FREQ                      (200),
                                       // # = 200 for all design frequencies of
                                       //         -1 speed grade devices
                                       //   = 200 when design frequency < 480 MHz
                                       //         for -2 and -3 speed grade devices.
                                       //   = 300 when design frequency >= 480 MHz
                                       //         for -2 and -3 speed grade devices.
  .MMCM_ADV_BANDWIDTH  ("OPTIMIZED"),
                                        // MMCM programming algorithm
  .CLKFBOUT_MULT_F     (6),
                                        // write PLL VCO multiplier.
  .DIVCLK_DIVIDE       (2),
                                        // write PLL VCO divisor.
  .CLKOUT_DIVIDE       (3),
                                        // VCO output divisor for fast (memory) clocks.

  .nCK_PER_CLK         (2),
                                        // # of memory CKs per fabric clock.
  .tCK                 (2500),
                                        // memory tCK paramter.
                                        // # = Clock Period.

  .DEBUG_PORT          ("OFF"),
                                        // # = "ON" Enable debug signals/controls.
                                        //   = "OFF" Disable debug signals/controls.
  .SIM_BYPASS_INIT_CAL ("OFF"),
                                        // # = "OFF" -  Complete memory init &
                                        //              calibration sequence
                                        // # = "SKIP" - Skip memory init &
                                        //              calibration sequence
                                        // # = "FAST" - Skip memory init & use
                                        //              abbreviated calib sequence
  .nCS_PER_RANK        (1),
                                        // # of unique CS outputs per Rank for
                                        // phy.
  .DQS_CNT_WIDTH       (3),
                                        // # = ceil(log2(DQS_WIDTH)).
  .RANK_WIDTH          (1),
                                        // # = ceil(log2(RANKS)).
  .BANK_WIDTH          (3),
                                        // # of memory Bank Address bits.
  .CK_WIDTH            (1),
                                        // # of CK/CK# outputs to memory.
  .CKE_WIDTH           (1),
                                        // # of CKE outputs to memory.
  .COL_WIDTH           (10),
                                        // # of memory Column Address bits.
  .CS_WIDTH            (1),
                                        // # of unique CS outputs to memory.
  .DM_WIDTH            (8),
                                        // # of Data Mask bits.
  .DQ_WIDTH            (64),
  .DQS_WIDTH           (8),
                                        // # of DQS/DQS# bits.
  .ROW_WIDTH           (13),
                                        // # of memory Row Address bits.
  .BURST_MODE          ("8"),
                                        // Burst Length (Mode Register 0).
                                        // # = "8", "4", "OTF".
  .BM_CNT_WIDTH        (2),
                                        // # = ceil(log2(nBANK_MACHS)).
  .ADDR_CMD_MODE       ("1T" ),
                                        // # = "2T", "1T".
  .ORDERING            ("STRICT"),
                                        // # = "NORM", "STRICT".
  .WRLVL               ("ON"),
                                        // # = "ON" - DDR3 SDRAM
                                        //   = "OFF" - DDR2 SDRAM.
  .PHASE_DETECT        ("ON"),
                                        // # = "ON", "OFF".
  .RTT_NOM             ("40"),
                                        // RTT_NOM (ODT) (Mode Register 1).
                                        // # = "DISABLED" - RTT_NOM disabled,
                                        //   = "120" - RZQ/2,
                                        //   = "60"  - RZQ/4,
                                        //   = "40"  - RZQ/6.
  .RTT_WR              ("OFF"),
                                        // RTT_WR (ODT) (Mode Register 2).
                                        // # = "OFF" - Dynamic ODT off,
                                        //   = "120" - RZQ/2,
                                        //   = "60"  - RZQ/4,
  .OUTPUT_DRV          ("HIGH"),
                                        // Output Driver Impedance Control (Mode Register 1).
                                        // # = "HIGH" - RZQ/7,
                                        //   = "LOW" - RZQ/6.
  .REG_CTRL            ("OFF"),
                                        // # = "ON" - RDIMMs,
                                        //   = "OFF" - Components, SODIMMs, UDIMMs.
  .nDQS_COL0           (3),
                                        // Number of DQS groups in I/O column #1.
  .nDQS_COL1           (5),
                                        // Number of DQS groups in I/O column #2.
  .nDQS_COL2           (0),
                                        // Number of DQS groups in I/O column #3.
  .nDQS_COL3           (0),
                                        // Number of DQS groups in I/O column #4.
  .DQS_LOC_COL0        (24'h020100),
                                        // DQS groups in column #1.
  .DQS_LOC_COL1        (40'h0706050403),
                                        // DQS groups in column #2.
  .DQS_LOC_COL2        (0),
                                        // DQS groups in column #3.
  .DQS_LOC_COL3        (0),
                                        // DQS groups in column #4.
  .tPRDI               (1_000_000),
                                        // memory tPRDI paramter.
  .tREFI               (7800000),
                                        // memory tREFI paramter.
  .tZQI                (128_000_000),
                                        // memory tZQI paramter.
  .ADDR_WIDTH          (27),
                                        // # = RANK_WIDTH + BANK_WIDTH
                                        //     + ROW_WIDTH + COL_WIDTH;
  .ECC                 ("OFF"),
  .ECC_TEST            ("OFF"),
  .TCQ                 (100),
  .DATA_WIDTH          (64),
                                        // # of Data (DQ) bits.
  .PAYLOAD_WIDTH       (64),
  .INTERFACE           ("AXI4"),
                                        // Port Interface.
                                        // # = UI - User Interface,
                                        //   = AXI4 - AXI4 Interface.
  // AXI related parameters
  .C_S_AXI_ID_WIDTH    (%S_AXI_ID_WIDTH),
                                        // Width of all master and slave ID signals.
  .C_S_AXI_ADDR_WIDTH  (%S_AXI_ADDR_WIDTH),
                                        // Width of S_AXI_AWADDR, S_AXI_ARADDR, M_AXI_AWADDR and
                                        // M_AXI_ARADDR for all SI/MI slots.
                                        // # = 32.
  .C_S_AXI_DATA_WIDTH  (%S_AXI_DATA_WIDTH),
                                        // Width of WDATA and RDATA on SI slot.
                                        // Must be less or equal to APP_DATA_WIDTH.
                                        // # = 32, 64, 128, 256.
  .C_S_AXI_SUPPORTS_NARROW_BURST  (%S_AXI_SUPPORTS_NARROW_BURST),
                                       // Indicates whether to instatiate upsizer
                                       // Range: 0, 1
   .C_RD_WR_ARB_ALGORITHM          ("%C_RD_WR_ARB_ALGORITHM"),
                                       // Indicates the Arbitration
                                       // Allowed values - "TDM", "ROUND_ROBIN",
                                       // "RD_PRI_REG", "RD_PRI_REG_STARVE_LIMIT"
   .CALIB_ROW_ADD      (16'h0000),// Calibration row address
   .CALIB_COL_ADD      (12'h000), // Calibration column address
   .CALIB_BA_ADD       (3'h0),    // Calibration bank address
  .RST_ACT_LOW             (1),
                           // =1 for active low reset,
                           // =0 for active high.
  .IODELAY_GRP          ("IODELAY_MIG"),
                           //to phy_top
  .INPUT_CLK_TYPE          ("DIFFERENTIAL"),
                           // input clock type DIFFERENTIAL or SINGLE_ENDED
  .STARVE_LIMIT            (2)
                           // # = 2,3,4.
  )
  u_DDR3_SDRAM (

    .sys_clk_p         (sys_clk_p),
    .sys_clk_n         (sys_clk_n),
    .clk_ref_p             (clk_ref_p),
    .clk_ref_n             (clk_ref_n),
    .ddr3_dq                     (ddr3_dq),
    .ddr3_addr                   (ddr3_addr),
    .ddr3_ba                     (ddr3_ba),
    .ddr3_ras_n                  (ddr3_ras_n),
    .ddr3_cas_n                  (ddr3_cas_n),
    .ddr3_we_n                   (ddr3_we_n),
    .ddr3_reset_n                (ddr3_reset_n),
    .ddr3_cs_n                   (ddr3_cs_n),
    .ddr3_odt                    (ddr3_odt),
    .ddr3_cke                    (ddr3_cke),
    .ddr3_dm            (ddr3_dm),
    .ddr3_dqs_p                  (ddr3_dqs_p),
    .ddr3_dqs_n                  (ddr3_dqs_n),
    .ddr3_ck_p                   (ddr3_ck_p),
    .ddr3_ck_n                   (ddr3_ck_n),
    .sda      (sda),
    .scl      (scl),
    .phy_init_done     (phy_init_done),
    .aresetn             (aresetn),
    .s_axi_awid          (s_axi_awid),
    .s_axi_awaddr        (s_axi_awaddr),
    .s_axi_awlen         (s_axi_awlen),
    .s_axi_awsize        (s_axi_awsize),
    .s_axi_awburst       (s_axi_awburst),
    .s_axi_awlock        (s_axi_awlock),
    .s_axi_awcache       (s_axi_awcache),
    .s_axi_awprot        (s_axi_awprot),
    .s_axi_awqos         (s_axi_awqos),
    .s_axi_awvalid       (s_axi_awvalid),
    .s_axi_awready       (s_axi_awready),
    .s_axi_wdata         (s_axi_wdata),
    .s_axi_wstrb         (s_axi_wstrb),
    .s_axi_wlast         (s_axi_wlast),
    .s_axi_wvalid        (s_axi_wvalid),
    .s_axi_wready        (s_axi_wready),
    .s_axi_bid           (s_axi_bid),
    .s_axi_bresp         (s_axi_bresp),
    .s_axi_bvalid        (s_axi_bvalid),
    .s_axi_bready        (s_axi_bready),
    .s_axi_arid          (s_axi_arid),
    .s_axi_araddr        (s_axi_araddr),
    .s_axi_arlen         (s_axi_arlen),
    .s_axi_arsize        (s_axi_arsize),
    .s_axi_arburst       (s_axi_arburst),
    .s_axi_arlock        (s_axi_arlock),
    .s_axi_arcache       (s_axi_arcache),
    .s_axi_arprot        (s_axi_arprot),
    .s_axi_arqos         (s_axi_arqos),
    .s_axi_arvalid       (s_axi_arvalid),
    .s_axi_arready       (s_axi_arready),
    .s_axi_rid           (s_axi_rid),
    .s_axi_rdata         (s_axi_rdata),
    .s_axi_rresp         (s_axi_rresp),
    .s_axi_rlast         (s_axi_rlast),
    .s_axi_rvalid        (s_axi_rvalid),
    .s_axi_rready        (s_axi_rready),
    .ui_clk_sync_rst     (ui_clk_sync_rst),
    .ui_clk              (ui_clk),
    .sys_rst           (sys_rst)
    );

// INST_TAG_END ------ End INSTANTIATION Template ---------

// You must compile the wrapper file DDR3_SDRAM.v when simulating
// the core, DDR3_SDRAM. When compiling the wrapper file, be sure to
// reference the XilinxCoreLib Verilog simulation library. For detailed
// instructions, please refer to the "CORE Generator Help".

    