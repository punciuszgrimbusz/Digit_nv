`timescale 1ns / 1ps
`default_nettype none

module i2c_diag_test240 #(
    parameter integer CLK_HZ = 27000000
)(
    input  wire clk,
    input  wire resetn,

    output reg        new_frame = 1'b0,
    output reg [9:0]  frame_lines = 10'd240
);
    // ~0.5s pulse
    localparam integer PERIOD = (CLK_HZ/2);
    reg [24:0] cnt = 25'd0;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            cnt       <= 25'd0;
            new_frame <= 1'b0;
            frame_lines <= 10'd240;
        end else begin
            new_frame <= 1'b0;
            if (cnt == PERIOD-1) begin
                cnt <= 25'd0;
                new_frame <= 1'b1;      // 1 clock pulse
                frame_lines <= 10'd240; // just to be explicit
            end else begin
                cnt <= cnt + 25'd1;
            end
        end
    end
endmodule

`default_nettype wire
