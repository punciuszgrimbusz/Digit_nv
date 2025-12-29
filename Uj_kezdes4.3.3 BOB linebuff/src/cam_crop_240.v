`timescale 1ns / 1ps
`default_nettype none

module cam_crop_240 #(
    parameter integer START_SKIP_LINES = 0,
    parameter integer ACTIVE_LINES     = 240
)(
    input  wire        cam_pclk,
    input  wire        cam_resetn,

    input  wire        in_line_valid,
    input  wire        in_y_valid,
    input  wire [7:0]  in_y,
    input  wire        in_field_toggle, // tvp5150_capture.field_toggle

    output reg         out_line_valid = 1'b0,
    output reg         out_y_valid    = 1'b0,
    output reg [7:0]   out_y          = 8'd0
);

    reg in_line_valid_d     = 1'b0;
    reg in_field_toggle_d   = 1'b0;

    reg [9:0] line_idx      = 10'd0;  // active line index within field
    reg       line_enable   = 1'b0;   // latched for current line

    function is_enabled;
        input [9:0] idx;
        begin
            is_enabled = (idx >= START_SKIP_LINES) &&
                         (idx < (START_SKIP_LINES + ACTIVE_LINES));
        end
    endfunction

    always @(posedge cam_pclk or negedge cam_resetn) begin
        if (!cam_resetn) begin
            in_line_valid_d   <= 1'b0;
            in_field_toggle_d <= 1'b0;
            line_idx          <= 10'd0;
            line_enable       <= 1'b0;

            out_line_valid    <= 1'b0;
            out_y_valid       <= 1'b0;
            out_y             <= 8'd0;
        end else begin
            in_line_valid_d   <= in_line_valid;
            in_field_toggle_d <= in_field_toggle;

            out_y_valid <= 1'b0; // default

            // field boundary -> reset line counter
            if (in_field_toggle ^ in_field_toggle_d) begin
                line_idx    <= 10'd0;
                line_enable <= 1'b0;
            end

            // line start (rising edge)
            if (in_line_valid && !in_line_valid_d) begin
                line_enable <= is_enabled(line_idx);
                line_idx    <= line_idx + 10'd1;
            end

            // line valid output is gated by latched enable
            out_line_valid <= in_line_valid && line_enable;

            // y stream gated by enable too
            if (in_y_valid && line_enable) begin
                out_y       <= in_y;
                out_y_valid <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
