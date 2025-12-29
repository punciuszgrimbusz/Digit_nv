// cam_frame_line_counter.v
// Frame-enként megszámolja, hány sor (line_valid falling edge) volt,
// a frame_valid jel alapján választva szét a frame-eket.

`timescale 1ns / 1ps
`default_nettype none

module cam_frame_line_counter (
    input  wire       cam_pclk,
    input  wire       cam_resetn,

    input  wire       cam_line_valid,   // aktív sor jel (TVP capture-ből)
    input  wire       cam_frame_valid,  // aktív frame jel (TVP capture-ből)

    output reg [9:0]  lines_per_frame_last, // előző TELJES frame sorainak száma
    output reg        new_frame_ready       // 1 clk-ciklus strobe, amikor frissült
);

    reg cam_line_valid_d;
    reg cam_frame_valid_d;
    reg [9:0] line_count;

    always @(posedge cam_pclk or negedge cam_resetn) begin
        if (!cam_resetn) begin
            cam_line_valid_d     <= 1'b0;
            cam_frame_valid_d    <= 1'b0;
            line_count           <= 10'd0;
            lines_per_frame_last <= 10'd0;
            new_frame_ready      <= 1'b0;
        end else begin
            // alaphelyzet: nincs strobe
            new_frame_ready   <= 1'b0;
            cam_line_valid_d  <= cam_line_valid;
            cam_frame_valid_d <= cam_frame_valid;

            // sor-számlálás: line_valid falling edge, amíg frame_valid aktív
            if (!cam_line_valid && cam_line_valid_d && cam_frame_valid) begin
                line_count <= line_count + 10'd1;
            end

            // új frame: frame_valid rising edge
            if (cam_frame_valid && !cam_frame_valid_d) begin
                // lezárjuk az előző frame-et
                lines_per_frame_last <= line_count;
                new_frame_ready      <= 1'b1;
                line_count           <= 10'd0;
            end
        end
    end

endmodule

`default_nettype wire
