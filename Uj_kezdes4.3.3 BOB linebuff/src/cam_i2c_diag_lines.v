`timescale 1ns / 1ps
`default_nettype none

module cam_i2c_diag_lines (
    input  wire        cam_pclk,
    input  wire        cam_resetn,

    input  wire        cam_line_valid,
    input  wire        cam_field_toggle,  // tvp5150_capture.field_toggle

    input  wire        i2c_busy,          // i2c_frame_logger_arduino.busy

    output reg         new_frame = 1'b0,  // 1-cycle pulse when we actually send
    output reg  [9:0]  frame_lines = 10'd0
);

    reg cam_line_valid_d     = 1'b0;
    reg cam_field_toggle_d   = 1'b0;

    reg [9:0] lines_cnt      = 10'd0;

    reg [9:0] pending_value  = 10'd0;
    reg       pending        = 1'b0;

    always @(posedge cam_pclk or negedge cam_resetn) begin
        if (!cam_resetn) begin
            cam_line_valid_d   <= 1'b0;
            cam_field_toggle_d <= 1'b0;

            lines_cnt      <= 10'd0;
            pending_value  <= 10'd0;
            pending        <= 1'b0;

            new_frame      <= 1'b0;
            frame_lines    <= 10'd0;
        end else begin
            cam_line_valid_d   <= cam_line_valid;
            cam_field_toggle_d <= cam_field_toggle;

            new_frame <= 1'b0; // default

            // count lines (rising edge of cam_line_valid)
            if (cam_line_valid && !cam_line_valid_d) begin
                lines_cnt <= lines_cnt + 10'd1;
            end

            // field boundary -> latch result and mark pending
            if (cam_field_toggle ^ cam_field_toggle_d) begin
                pending_value <= lines_cnt;
                pending       <= 1'b1;
                lines_cnt     <= 10'd0;
            end

            // if logger free and we have pending data -> send exactly once
            if (pending && !i2c_busy) begin
                frame_lines <= pending_value;
                new_frame   <= 1'b1;
                pending     <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire
