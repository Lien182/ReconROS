//Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2018.3 (lin64) Build 2405991 Thu Dec  6 23:36:41 MST 2018
//Date        : Thu Aug 12 18:20:15 2021
//Host        : christian-Latitude-5591 running 64-bit Ubuntu 18.04.5 LTS
//Command     : generate_target zycap.bd
//Design      : zycap
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CORE_GENERATION_INFO = "zycap,IP_Integrator,{x_ipVendor=xilinx.com,x_ipLibrary=BlockDiagram,x_ipName=zycap,x_ipVersion=1.00.a,x_ipLanguage=VERILOG,numBlks=2,numReposBlks=2,numNonXlnxBlks=1,numHierBlks=0,maxHierDepth=0,numSysgenBlks=0,numHlsBlks=0,numHdlrefBlks=0,numPkgbdBlks=0,bdsource=USER,synth_mode=Global}" *) (* HW_HANDOFF = "zycap.hwdef" *) 
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
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARADDR" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME M_AXI_MM2S, ADDR_WIDTH 32, ARUSER_WIDTH 0, AWUSER_WIDTH 0, BUSER_WIDTH 0, CLK_DOMAIN zycap_s_axi_lite_aclk, DATA_WIDTH 32, FREQ_HZ 100000000, HAS_BRESP 0, HAS_BURST 0, HAS_CACHE 0, HAS_LOCK 0, HAS_PROT 0, HAS_QOS 0, HAS_REGION 0, HAS_RRESP 1, HAS_WSTRB 0, ID_WIDTH 0, INSERT_VIP 0, MAX_BURST_LENGTH 256, NUM_READ_OUTSTANDING 16, NUM_READ_THREADS 1, NUM_WRITE_OUTSTANDING 2, NUM_WRITE_THREADS 1, PHASE 0.000, PROTOCOL AXI4, READ_WRITE_MODE READ_ONLY, RUSER_BITS_PER_BYTE 0, RUSER_WIDTH 0, SUPPORTS_NARROW_BURST 0, WUSER_BITS_PER_BYTE 0, WUSER_WIDTH 0" *) output [31:0]M_AXI_MM2S_araddr;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARBURST" *) output [1:0]M_AXI_MM2S_arburst;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARCACHE" *) output [3:0]M_AXI_MM2S_arcache;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARLEN" *) output [7:0]M_AXI_MM2S_arlen;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARPROT" *) output [2:0]M_AXI_MM2S_arprot;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARREADY" *) input M_AXI_MM2S_arready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARSIZE" *) output [2:0]M_AXI_MM2S_arsize;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARVALID" *) output M_AXI_MM2S_arvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RDATA" *) input [31:0]M_AXI_MM2S_rdata;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RLAST" *) input M_AXI_MM2S_rlast;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RREADY" *) output M_AXI_MM2S_rready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RRESP" *) input [1:0]M_AXI_MM2S_rresp;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RVALID" *) input M_AXI_MM2S_rvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE ARADDR" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXI_LITE, ADDR_WIDTH 32, ARUSER_WIDTH 0, AWUSER_WIDTH 0, BUSER_WIDTH 0, CLK_DOMAIN zycap_s_axi_lite_aclk, DATA_WIDTH 32, FREQ_HZ 100000000, HAS_BRESP 1, HAS_BURST 1, HAS_CACHE 1, HAS_LOCK 1, HAS_PROT 1, HAS_QOS 1, HAS_REGION 1, HAS_RRESP 1, HAS_WSTRB 1, ID_WIDTH 0, INSERT_VIP 0, MAX_BURST_LENGTH 1, NUM_READ_OUTSTANDING 1, NUM_READ_THREADS 1, NUM_WRITE_OUTSTANDING 1, NUM_WRITE_THREADS 1, PHASE 0.000, PROTOCOL AXI4LITE, READ_WRITE_MODE READ_WRITE, RUSER_BITS_PER_BYTE 0, RUSER_WIDTH 0, SUPPORTS_NARROW_BURST 0, WUSER_BITS_PER_BYTE 0, WUSER_WIDTH 0" *) input [9:0]S_AXI_LITE_araddr;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE ARREADY" *) output S_AXI_LITE_arready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE ARVALID" *) input S_AXI_LITE_arvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE AWADDR" *) input [9:0]S_AXI_LITE_awaddr;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE AWREADY" *) output S_AXI_LITE_awready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE AWVALID" *) input S_AXI_LITE_awvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE BREADY" *) input S_AXI_LITE_bready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE BRESP" *) output [1:0]S_AXI_LITE_bresp;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE BVALID" *) output S_AXI_LITE_bvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE RDATA" *) output [31:0]S_AXI_LITE_rdata;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE RREADY" *) input S_AXI_LITE_rready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE RRESP" *) output [1:0]S_AXI_LITE_rresp;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE RVALID" *) output S_AXI_LITE_rvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE WDATA" *) input [31:0]S_AXI_LITE_wdata;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE WREADY" *) output S_AXI_LITE_wready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE WVALID" *) input S_AXI_LITE_wvalid;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 RST.AXI_RESETN RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME RST.AXI_RESETN, INSERT_VIP 0, POLARITY ACTIVE_LOW" *) input axi_resetn;
  (* X_INTERFACE_INFO = "xilinx.com:signal:interrupt:1.0 INTR.MM2S_INTROUT INTERRUPT" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME INTR.MM2S_INTROUT, PortWidth 1, SENSITIVITY LEVEL_HIGH" *) output mm2s_introut;
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 CLK.S_AXI_LITE_ACLK CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME CLK.S_AXI_LITE_ACLK, ASSOCIATED_BUSIF S_AXI_LITE:M_AXI_MM2S, ASSOCIATED_RESET axi_resetn, CLK_DOMAIN zycap_s_axi_lite_aclk, FREQ_HZ 100000000, INSERT_VIP 0, PHASE 0.000" *) input s_axi_lite_aclk;

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
