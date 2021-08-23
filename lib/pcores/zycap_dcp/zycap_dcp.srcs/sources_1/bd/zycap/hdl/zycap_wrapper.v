//Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2018.3 (lin64) Build 2405991 Thu Dec  6 23:36:41 MST 2018
//Date        : Thu Aug 12 18:20:15 2021
//Host        : christian-Latitude-5591 running 64-bit Ubuntu 18.04.5 LTS
//Command     : generate_target zycap_wrapper.bd
//Design      : zycap_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module zycap_wrapper
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

  wire [31:0]M_AXI_MM2S_araddr;
  wire [1:0]M_AXI_MM2S_arburst;
  wire [3:0]M_AXI_MM2S_arcache;
  wire [7:0]M_AXI_MM2S_arlen;
  wire [2:0]M_AXI_MM2S_arprot;
  wire M_AXI_MM2S_arready;
  wire [2:0]M_AXI_MM2S_arsize;
  wire M_AXI_MM2S_arvalid;
  wire [31:0]M_AXI_MM2S_rdata;
  wire M_AXI_MM2S_rlast;
  wire M_AXI_MM2S_rready;
  wire [1:0]M_AXI_MM2S_rresp;
  wire M_AXI_MM2S_rvalid;
  wire [9:0]S_AXI_LITE_araddr;
  wire S_AXI_LITE_arready;
  wire S_AXI_LITE_arvalid;
  wire [9:0]S_AXI_LITE_awaddr;
  wire S_AXI_LITE_awready;
  wire S_AXI_LITE_awvalid;
  wire S_AXI_LITE_bready;
  wire [1:0]S_AXI_LITE_bresp;
  wire S_AXI_LITE_bvalid;
  wire [31:0]S_AXI_LITE_rdata;
  wire S_AXI_LITE_rready;
  wire [1:0]S_AXI_LITE_rresp;
  wire S_AXI_LITE_rvalid;
  wire [31:0]S_AXI_LITE_wdata;
  wire S_AXI_LITE_wready;
  wire S_AXI_LITE_wvalid;
  wire axi_resetn;
  wire mm2s_introut;
  wire s_axi_lite_aclk;

  zycap zycap_i
       (.M_AXI_MM2S_araddr(M_AXI_MM2S_araddr),
        .M_AXI_MM2S_arburst(M_AXI_MM2S_arburst),
        .M_AXI_MM2S_arcache(M_AXI_MM2S_arcache),
        .M_AXI_MM2S_arlen(M_AXI_MM2S_arlen),
        .M_AXI_MM2S_arprot(M_AXI_MM2S_arprot),
        .M_AXI_MM2S_arready(M_AXI_MM2S_arready),
        .M_AXI_MM2S_arsize(M_AXI_MM2S_arsize),
        .M_AXI_MM2S_arvalid(M_AXI_MM2S_arvalid),
        .M_AXI_MM2S_rdata(M_AXI_MM2S_rdata),
        .M_AXI_MM2S_rlast(M_AXI_MM2S_rlast),
        .M_AXI_MM2S_rready(M_AXI_MM2S_rready),
        .M_AXI_MM2S_rresp(M_AXI_MM2S_rresp),
        .M_AXI_MM2S_rvalid(M_AXI_MM2S_rvalid),
        .S_AXI_LITE_araddr(S_AXI_LITE_araddr),
        .S_AXI_LITE_arready(S_AXI_LITE_arready),
        .S_AXI_LITE_arvalid(S_AXI_LITE_arvalid),
        .S_AXI_LITE_awaddr(S_AXI_LITE_awaddr),
        .S_AXI_LITE_awready(S_AXI_LITE_awready),
        .S_AXI_LITE_awvalid(S_AXI_LITE_awvalid),
        .S_AXI_LITE_bready(S_AXI_LITE_bready),
        .S_AXI_LITE_bresp(S_AXI_LITE_bresp),
        .S_AXI_LITE_bvalid(S_AXI_LITE_bvalid),
        .S_AXI_LITE_rdata(S_AXI_LITE_rdata),
        .S_AXI_LITE_rready(S_AXI_LITE_rready),
        .S_AXI_LITE_rresp(S_AXI_LITE_rresp),
        .S_AXI_LITE_rvalid(S_AXI_LITE_rvalid),
        .S_AXI_LITE_wdata(S_AXI_LITE_wdata),
        .S_AXI_LITE_wready(S_AXI_LITE_wready),
        .S_AXI_LITE_wvalid(S_AXI_LITE_wvalid),
        .axi_resetn(axi_resetn),
        .mm2s_introut(mm2s_introut),
        .s_axi_lite_aclk(s_axi_lite_aclk));
endmodule
