`timescale 1ns / 1ps
`default_nettype none

// Simulation-friendly stubs for Gowin vendor primitives used by the
// camera-to-HDMI pipeline. These provide minimal connectivity so the
// behavioral sources elaborate under Icarus Verilog.

module rPLL (
    output wire CLKOUT,
    output wire LOCK,
    output wire CLKOUTP,
    output wire CLKOUTD,
    output wire CLKOUTD3,
    input  wire RESET,
    input  wire RESET_P,
    input  wire CLKIN,
    input  wire CLKFB,
    input  wire [5:0] FBDSEL,
    input  wire [5:0] IDSEL,
    input  wire [5:0] ODSEL,
    input  wire [3:0] PSDA,
    input  wire [3:0] DUTYDA,
    input  wire [3:0] FDLY
);
    parameter string FCLKIN = "27";
    parameter string DYN_IDIV_SEL = "false";
    parameter integer IDIV_SEL = 0;
    parameter string DYN_FBDIV_SEL = "false";
    parameter integer FBDIV_SEL = 0;
    parameter string DYN_ODIV_SEL = "false";
    parameter integer ODIV_SEL = 0;
    parameter string PSDA_SEL = "0000";
    parameter string DYN_DA_EN = "false";
    parameter string DUTYDA_SEL = "0000";
    parameter bit CLKOUT_FT_DIR = 1'b0;
    parameter bit CLKOUTP_FT_DIR = 1'b0;
    parameter integer CLKOUT_DLY_STEP = 0;
    parameter integer CLKOUTP_DLY_STEP = 0;
    parameter string CLKFB_SEL = "internal";
    parameter string CLKOUT_BYPASS = "false";
    parameter string CLKOUTP_BYPASS = "false";
    parameter string CLKOUTD_BYPASS = "false";
    parameter integer DYN_SDIV_SEL = 0;
    parameter string CLKOUTD_SRC = "CLKOUT";
    parameter string CLKOUTD3_SRC = "CLKOUT";
    parameter string DEVICE = "";

    assign CLKOUT  = CLKIN;
    assign CLKOUTP = CLKIN;
    assign CLKOUTD = CLKIN;
    assign CLKOUTD3 = CLKIN;
    assign LOCK    = 1'b1;
endmodule

module CLKDIV (
    output wire CLKOUT,
    input  wire HCLKIN,
    input  wire RESETN,
    input  wire CALIB
);
    parameter string DIV_MODE = "2";
    parameter string GSREN = "false";

    assign CLKOUT = HCLKIN;
endmodule

module OSER10 (
    output wire Q,
    input  wire D0,
    input  wire D1,
    input  wire D2,
    input  wire D3,
    input  wire D4,
    input  wire D5,
    input  wire D6,
    input  wire D7,
    input  wire D8,
    input  wire D9,
    input  wire PCLK,
    input  wire FCLK,
    input  wire RESET
);
    assign Q = RESET ? 1'b0 : D0;
endmodule

module ELVDS_OBUF (
    input  wire I,
    output wire O,
    output wire OB
);
    assign O  = I;
    assign OB = ~I;
endmodule

`default_nettype wire
