`timescale 1ns / 1ps
`default_nettype none

module cam_field_line_counter (
    input  wire       cam_pclk,
    input  wire       cam_resetn,
    input  wire       cam_line_valid,
    input  wire       cam_field_toggle,

    output reg [9:0]  lines_per_field_last = 10'd0,
    output reg        new_field_ready      = 1'b0
);

    reg cam_line_valid_d = 1'b0;
    reg cam_field_toggle_d = 1'b0;
    reg [9:0] line_count = 10'd0;

    always @(posedge cam_pclk or negedge cam_resetn) begin
        if (!cam_resetn) begin
            cam_line_valid_d     <= 1'b0;
            cam_field_toggle_d   <= 1'b0;
            line_count           <= 10'd0;
            lines_per_field_last <= 10'd0;
            new_field_ready      <= 1'b0;
        end else begin
            cam_line_valid_d   <= cam_line_valid;
            cam_field_toggle_d <= cam_field_toggle;
            new_field_ready    <= 1'b0;

            // count active lines on rising edge
            if (cam_line_valid && !cam_line_valid_d) begin
                line_count <= line_count + 10'd1;
            end

            // field boundary
            if (cam_field_toggle ^ cam_field_toggle_d) begin
                lines_per_field_last <= line_count;
                new_field_ready      <= 1'b1;
                line_count           <= 10'd0;
            end
        end
    end

endmodule

`default_nettype wire
