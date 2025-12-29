`timescale 1ns / 1ps
`default_nettype none

module cam_field_diag (
    input  wire        cam_pclk,
    input  wire        cam_resetn,

    input  wire        cam_line_valid,
    input  wire        cam_field_toggle,   // from tvp5150_capture.field_toggle

    output reg  [9:0]  dbg_value = 10'd0,  // alternates: lines_per_field / pclk_ticks_low10
    output reg         new_dbg_ready = 1'b0
);
    reg cam_line_valid_d = 1'b0;
    reg cam_field_toggle_d = 1'b0;

    reg [9:0]  lines_cnt = 10'd0;
    reg [19:0] pclk_cnt  = 20'd0;

    reg [9:0]  lines_last = 10'd0;
    reg [9:0]  pclk_low10_last = 10'd0;

    reg page = 1'b0; // 0 -> lines, 1 -> period low10

    always @(posedge cam_pclk or negedge cam_resetn) begin
        if (!cam_resetn) begin
            cam_line_valid_d    <= 1'b0;
            cam_field_toggle_d  <= 1'b0;
            lines_cnt           <= 10'd0;
            pclk_cnt            <= 20'd0;
            lines_last          <= 10'd0;
            pclk_low10_last     <= 10'd0;
            dbg_value           <= 10'd0;
            new_dbg_ready       <= 1'b0;
            page                <= 1'b0;
        end else begin
            cam_line_valid_d   <= cam_line_valid;
            cam_field_toggle_d <= cam_field_toggle;

            new_dbg_ready <= 1'b0;

            // count pclk ticks continuously within field
            pclk_cnt <= pclk_cnt + 20'd1;

            // count active lines (rising edge of line_valid)
            if (cam_line_valid && !cam_line_valid_d) begin
                lines_cnt <= lines_cnt + 10'd1;
            end

            // field boundary (toggle)
            if (cam_field_toggle ^ cam_field_toggle_d) begin
                // latch stats
                lines_last      <= lines_cnt;
                pclk_low10_last <= pclk_cnt[9:0];

                // choose what to send this time
                if (!page)
                    dbg_value <= lines_cnt;
                else
                    dbg_value <= pclk_cnt[9:0];

                page          <= ~page;
                new_dbg_ready <= 1'b1;

                // reset counters for next field
                lines_cnt <= 10'd0;
                pclk_cnt  <= 20'd0;
            end
        end
    end

endmodule

`default_nettype wire
