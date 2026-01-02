`timescale 1ns/1ps
`default_nettype none

// Simple behavioral stubs for Gowin vendor primitives so the design
// can be simulated or linted without proprietary libraries.
module Gowin_rPLL(
    input  wire clkin,
    output wire clkout,
    output wire lock
);
    assign clkout = clkin;
    assign lock   = 1'b1;
endmodule

module Gowin_CLKDIV(
    output wire clkout,
    input  wire hclkin,
    input  wire resetn
);
    assign clkout = hclkin;
endmodule

module OSER10(
    output wire [2:0] Q,
    input  wire [2:0] D0, D1, D2, D3, D4,
    input  wire [2:0] D5, D6, D7, D8, D9,
    input  wire       PCLK,
    input  wire       FCLK,
    input  wire       RESET
);
    assign Q = D0;  // minimal stub; ignores serialization
endmodule

module ELVDS_OBUF(
    input  wire I,
    output wire O,
    output wire OB
);
    assign O  = I;
    assign OB = ~I;
endmodule

`default_nettype wire
