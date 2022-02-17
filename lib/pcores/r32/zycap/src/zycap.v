//Copyright 1986-2014 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2014.2 (win64) Build 932637 Wed Jun 11 13:33:10 MDT 2014
//Date        : Fri Nov 06 11:33:12 2015
//Host        : GS-21 running 64-bit Service Pack 1  (build 7601)
//Command     : generate_target zycap.bd
//Design      : zycap
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CORE_GENERATION_INFO = "zycap,IP_Integrator,{x_ipVendor=xilinx.com,x_ipLanguage=VERILOG,numBlks=2,numReposBlks=2,numNonXlnxBlks=1,numHierBlks=0,maxHierDepth=0}" *) 
module zycap
   (M_AXI_MM2S_araddr,
    M_AXI_MM2S_arburst,
    M_AXI_MM2S_arcache,
    M_AXI_MM2S_arlen,
    M_AXI_MM2S_arprot,
    M_AXI_MM2S_arready,
    M_AXI_MM2S_arsize,
    M_AXI_MM2S_arvalid,
    M_AXI_MM2S_rdata,
    M_AXI_MM2S_rlast,
    M_AXI_MM2S_rready,
    M_AXI_MM2S_rresp,
    M_AXI_MM2S_rvalid,
    S_AXI_LITE_araddr,
    S_AXI_LITE_arready,
    S_AXI_LITE_arvalid,
    S_AXI_LITE_awaddr,
    S_AXI_LITE_awready,
    S_AXI_LITE_awvalid,
    S_AXI_LITE_bready,
    S_AXI_LITE_bresp,
    S_AXI_LITE_bvalid,
    S_AXI_LITE_rdata,
    S_AXI_LITE_rready,
    S_AXI_LITE_rresp,
    S_AXI_LITE_rvalid,
    S_AXI_LITE_wdata,
    S_AXI_LITE_wready,
    S_AXI_LITE_wvalid,
    axi_resetn,
    mm2s_introut,
    s_axi_lite_aclk);
  output [31:0]M_AXI_MM2S_araddr;
  output [1:0]M_AXI_MM2S_arburst;
  output [3:0]M_AXI_MM2S_arcache;
  output [7:0]M_AXI_MM2S_arlen;
  output [2:0]M_AXI_MM2S_arprot;
  input M_AXI_MM2S_arready;
  output [2:0]M_AXI_MM2S_arsize;
  output M_AXI_MM2S_arvalid;
  input [31:0]M_AXI_MM2S_rdata;
  input M_AXI_MM2S_rlast;
  output M_AXI_MM2S_rready;
  input [1:0]M_AXI_MM2S_rresp;
  input M_AXI_MM2S_rvalid;
  input [9:0]S_AXI_LITE_araddr;
  output S_AXI_LITE_arready;
  input S_AXI_LITE_arvalid;
  input [9:0]S_AXI_LITE_awaddr;
  output S_AXI_LITE_awready;
  input S_AXI_LITE_awvalid;
  input S_AXI_LITE_bready;
  output [1:0]S_AXI_LITE_bresp;
  output S_AXI_LITE_bvalid;
  output [31:0]S_AXI_LITE_rdata;
  input S_AXI_LITE_rready;
  output [1:0]S_AXI_LITE_rresp;
  output S_AXI_LITE_rvalid;
  input [31:0]S_AXI_LITE_wdata;
  output S_AXI_LITE_wready;
  input S_AXI_LITE_wvalid;
  input axi_resetn;
  output mm2s_introut;
  input s_axi_lite_aclk;

  wire [9:0]S_AXI_LITE_1_ARADDR;
  wire S_AXI_LITE_1_ARREADY;
  wire S_AXI_LITE_1_ARVALID;
  wire [9:0]S_AXI_LITE_1_AWADDR;
  wire S_AXI_LITE_1_AWREADY;
  wire S_AXI_LITE_1_AWVALID;
  wire S_AXI_LITE_1_BREADY;
  wire [1:0]S_AXI_LITE_1_BRESP;
  wire S_AXI_LITE_1_BVALID;
  wire [31:0]S_AXI_LITE_1_RDATA;
  wire S_AXI_LITE_1_RREADY;
  wire [1:0]S_AXI_LITE_1_RRESP;
  wire S_AXI_LITE_1_RVALID;
  wire [31:0]S_AXI_LITE_1_WDATA;
  wire S_AXI_LITE_1_WREADY;
  wire S_AXI_LITE_1_WVALID;
  wire [31:0]axi_dma_0_M_AXIS_MM2S_TDATA;
  wire axi_dma_0_M_AXIS_MM2S_TLAST;
  wire axi_dma_0_M_AXIS_MM2S_TREADY;
  wire axi_dma_0_M_AXIS_MM2S_TVALID;
  wire [31:0]axi_dma_0_M_AXI_MM2S_ARADDR;
  wire [1:0]axi_dma_0_M_AXI_MM2S_ARBURST;
  wire [3:0]axi_dma_0_M_AXI_MM2S_ARCACHE;
  wire [7:0]axi_dma_0_M_AXI_MM2S_ARLEN;
  wire [2:0]axi_dma_0_M_AXI_MM2S_ARPROT;
  wire axi_dma_0_M_AXI_MM2S_ARREADY;
  wire [2:0]axi_dma_0_M_AXI_MM2S_ARSIZE;
  wire axi_dma_0_M_AXI_MM2S_ARVALID;
  wire [31:0]axi_dma_0_M_AXI_MM2S_RDATA;
  wire axi_dma_0_M_AXI_MM2S_RLAST;
  wire axi_dma_0_M_AXI_MM2S_RREADY;
  wire [1:0]axi_dma_0_M_AXI_MM2S_RRESP;
  wire axi_dma_0_M_AXI_MM2S_RVALID;
  wire axi_dma_0_mm2s_introut;
  wire axi_dma_0_mm2s_prmry_reset_out_n;
  wire axi_resetn_1;
  wire s_axi_lite_aclk_1;

  assign M_AXI_MM2S_araddr[31:0] = axi_dma_0_M_AXI_MM2S_ARADDR;
  assign M_AXI_MM2S_arburst[1:0] = axi_dma_0_M_AXI_MM2S_ARBURST;
  assign M_AXI_MM2S_arcache[3:0] = axi_dma_0_M_AXI_MM2S_ARCACHE;
  assign M_AXI_MM2S_arlen[7:0] = axi_dma_0_M_AXI_MM2S_ARLEN;
  assign M_AXI_MM2S_arprot[2:0] = axi_dma_0_M_AXI_MM2S_ARPROT;
  assign M_AXI_MM2S_arsize[2:0] = axi_dma_0_M_AXI_MM2S_ARSIZE;
  assign M_AXI_MM2S_arvalid = axi_dma_0_M_AXI_MM2S_ARVALID;
  assign M_AXI_MM2S_rready = axi_dma_0_M_AXI_MM2S_RREADY;
  assign S_AXI_LITE_1_ARADDR = S_AXI_LITE_araddr[9:0];
  assign S_AXI_LITE_1_ARVALID = S_AXI_LITE_arvalid;
  assign S_AXI_LITE_1_AWADDR = S_AXI_LITE_awaddr[9:0];
  assign S_AXI_LITE_1_AWVALID = S_AXI_LITE_awvalid;
  assign S_AXI_LITE_1_BREADY = S_AXI_LITE_bready;
  assign S_AXI_LITE_1_RREADY = S_AXI_LITE_rready;
  assign S_AXI_LITE_1_WDATA = S_AXI_LITE_wdata[31:0];
  assign S_AXI_LITE_1_WVALID = S_AXI_LITE_wvalid;
  assign S_AXI_LITE_arready = S_AXI_LITE_1_ARREADY;
  assign S_AXI_LITE_awready = S_AXI_LITE_1_AWREADY;
  assign S_AXI_LITE_bresp[1:0] = S_AXI_LITE_1_BRESP;
  assign S_AXI_LITE_bvalid = S_AXI_LITE_1_BVALID;
  assign S_AXI_LITE_rdata[31:0] = S_AXI_LITE_1_RDATA;
  assign S_AXI_LITE_rresp[1:0] = S_AXI_LITE_1_RRESP;
  assign S_AXI_LITE_rvalid = S_AXI_LITE_1_RVALID;
  assign S_AXI_LITE_wready = S_AXI_LITE_1_WREADY;
  assign axi_dma_0_M_AXI_MM2S_ARREADY = M_AXI_MM2S_arready;
  assign axi_dma_0_M_AXI_MM2S_RDATA = M_AXI_MM2S_rdata[31:0];
  assign axi_dma_0_M_AXI_MM2S_RLAST = M_AXI_MM2S_rlast;
  assign axi_dma_0_M_AXI_MM2S_RRESP = M_AXI_MM2S_rresp[1:0];
  assign axi_dma_0_M_AXI_MM2S_RVALID = M_AXI_MM2S_rvalid;
  assign axi_resetn_1 = axi_resetn;
  assign mm2s_introut = axi_dma_0_mm2s_introut;
  assign s_axi_lite_aclk_1 = s_axi_lite_aclk;
zycap_axi_dma_0_0 axi_dma_0
       (.axi_resetn(axi_resetn_1),
        .m_axi_mm2s_aclk(s_axi_lite_aclk_1),
        .m_axi_mm2s_araddr(axi_dma_0_M_AXI_MM2S_ARADDR),
        .m_axi_mm2s_arburst(axi_dma_0_M_AXI_MM2S_ARBURST),
        .m_axi_mm2s_arcache(axi_dma_0_M_AXI_MM2S_ARCACHE),
        .m_axi_mm2s_arlen(axi_dma_0_M_AXI_MM2S_ARLEN),
        .m_axi_mm2s_arprot(axi_dma_0_M_AXI_MM2S_ARPROT),
        .m_axi_mm2s_arready(axi_dma_0_M_AXI_MM2S_ARREADY),
        .m_axi_mm2s_arsize(axi_dma_0_M_AXI_MM2S_ARSIZE),
        .m_axi_mm2s_arvalid(axi_dma_0_M_AXI_MM2S_ARVALID),
        .m_axi_mm2s_rdata(axi_dma_0_M_AXI_MM2S_RDATA),
        .m_axi_mm2s_rlast(axi_dma_0_M_AXI_MM2S_RLAST),
        .m_axi_mm2s_rready(axi_dma_0_M_AXI_MM2S_RREADY),
        .m_axi_mm2s_rresp(axi_dma_0_M_AXI_MM2S_RRESP),
        .m_axi_mm2s_rvalid(axi_dma_0_M_AXI_MM2S_RVALID),
        .m_axis_mm2s_tdata(axi_dma_0_M_AXIS_MM2S_TDATA),
        .m_axis_mm2s_tlast(axi_dma_0_M_AXIS_MM2S_TLAST),
        .m_axis_mm2s_tready(axi_dma_0_M_AXIS_MM2S_TREADY),
        .m_axis_mm2s_tvalid(axi_dma_0_M_AXIS_MM2S_TVALID),
        .mm2s_introut(axi_dma_0_mm2s_introut),
        .mm2s_prmry_reset_out_n(axi_dma_0_mm2s_prmry_reset_out_n),
        .s_axi_lite_aclk(s_axi_lite_aclk_1),
        .s_axi_lite_araddr(S_AXI_LITE_1_ARADDR),
        .s_axi_lite_arready(S_AXI_LITE_1_ARREADY),
        .s_axi_lite_arvalid(S_AXI_LITE_1_ARVALID),
        .s_axi_lite_awaddr(S_AXI_LITE_1_AWADDR),
        .s_axi_lite_awready(S_AXI_LITE_1_AWREADY),
        .s_axi_lite_awvalid(S_AXI_LITE_1_AWVALID),
        .s_axi_lite_bready(S_AXI_LITE_1_BREADY),
        .s_axi_lite_bresp(S_AXI_LITE_1_BRESP),
        .s_axi_lite_bvalid(S_AXI_LITE_1_BVALID),
        .s_axi_lite_rdata(S_AXI_LITE_1_RDATA),
        .s_axi_lite_rready(S_AXI_LITE_1_RREADY),
        .s_axi_lite_rresp(S_AXI_LITE_1_RRESP),
        .s_axi_lite_rvalid(S_AXI_LITE_1_RVALID),
        .s_axi_lite_wdata(S_AXI_LITE_1_WDATA),
        .s_axi_lite_wready(S_AXI_LITE_1_WREADY),
        .s_axi_lite_wvalid(S_AXI_LITE_1_WVALID));
zycap_icap_ctrl_0_1 icap_ctrl_0
       (.ACLK(s_axi_lite_aclk_1),
        .ARESETN(axi_dma_0_mm2s_prmry_reset_out_n),
        .S_AXIS_TDATA(axi_dma_0_M_AXIS_MM2S_TDATA),
        .S_AXIS_TLAST(axi_dma_0_M_AXIS_MM2S_TLAST),
        .S_AXIS_TREADY(axi_dma_0_M_AXIS_MM2S_TREADY),
        .S_AXIS_TVALID(axi_dma_0_M_AXIS_MM2S_TVALID));
endmodule
