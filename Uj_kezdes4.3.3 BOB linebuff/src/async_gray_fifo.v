// async_gray_fifo.v
// Simple dual-clock asynchronous FIFO using Gray-coded pointers.
// Parameters:
//   DATA_WIDTH: width of stored data
//   ADDR_WIDTH: FIFO depth = 2^ADDR_WIDTH
//
// Ports:
//   wr_clk, wr_resetn  : write clock domain / active-low reset
//   wr_en, wr_data     : write enable + data
//   wr_full            : FIFO full flag
//
//   rd_clk, rd_resetn  : read clock domain / active-low reset
//   rd_en, rd_data     : read enable + data
//   rd_empty           : FIFO empty flag

`timescale 1ns / 1ps
`default_nettype none

module async_gray_fifo #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 10            // depth = 1024
)(
    // write domain
    input  wire                     wr_clk,
    input  wire                     wr_resetn,
    input  wire                     wr_en,
    input  wire [DATA_WIDTH-1:0]    wr_data,
    output reg                      wr_full,

    // read domain
    input  wire                     rd_clk,
    input  wire                     rd_resetn,
    input  wire                     rd_en,
    output reg  [DATA_WIDTH-1:0]    rd_data,
    output reg                      rd_empty
);

    // ----------------------------------------------------------------
    // FIFO storage
    // ----------------------------------------------------------------
    localparam FIFO_DEPTH = (1 << ADDR_WIDTH);

    reg [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1];

    // ----------------------------------------------------------------
    // Binary and Gray pointers (ADDR_WIDTH+1 bits to detect full/empty)
    // ----------------------------------------------------------------
    reg [ADDR_WIDTH:0] wr_ptr_bin,  wr_ptr_bin_next;
    reg [ADDR_WIDTH:0] wr_ptr_gray, wr_ptr_gray_next;

    reg [ADDR_WIDTH:0] rd_ptr_bin,  rd_ptr_bin_next;
    reg [ADDR_WIDTH:0] rd_ptr_gray, rd_ptr_gray_next;

    // ----------------------------------------------------------------
    // Cross-domain pointer synchronizers
    // ----------------------------------------------------------------
    reg [ADDR_WIDTH:0] rd_ptr_gray_sync1, rd_ptr_gray_sync2; // into write domain
    reg [ADDR_WIDTH:0] wr_ptr_gray_sync1, wr_ptr_gray_sync2; // into read domain

    // ----------------------------------------------------------------
    // Gray coding function
    // ----------------------------------------------------------------
    function [ADDR_WIDTH:0] bin2gray;
        input [ADDR_WIDTH:0] b;
        begin
            bin2gray = (b >> 1) ^ b;
        end
    endfunction

    // ================================================================
    // WRITE DOMAIN LOGIC
    // ================================================================
    // Next-state logic for write pointer
    always @* begin
        // default: hold value
        wr_ptr_bin_next  = wr_ptr_bin;
        wr_ptr_gray_next = wr_ptr_gray;

        if (wr_en && !wr_full) begin
            wr_ptr_bin_next  = wr_ptr_bin + 1'b1;
            wr_ptr_gray_next = bin2gray(wr_ptr_bin_next);
        end
    end

    // Write pointer registers + memory write
    always @(posedge wr_clk or negedge wr_resetn) begin
        if (!wr_resetn) begin
            wr_ptr_bin       <= { (ADDR_WIDTH+1){1'b0} };
            wr_ptr_gray      <= { (ADDR_WIDTH+1){1'b0} };
        end else begin
            wr_ptr_bin       <= wr_ptr_bin_next;
            wr_ptr_gray      <= wr_ptr_gray_next;

            if (wr_en && !wr_full) begin
                mem[wr_ptr_bin[ADDR_WIDTH-1:0]] <= wr_data;
            end
        end
    end

    // Synchronize read pointer Gray into write domain
    always @(posedge wr_clk or negedge wr_resetn) begin
        if (!wr_resetn) begin
            rd_ptr_gray_sync1 <= { (ADDR_WIDTH+1){1'b0} };
            rd_ptr_gray_sync2 <= { (ADDR_WIDTH+1){1'b0} };
        end else begin
            rd_ptr_gray_sync1 <= rd_ptr_gray;
            rd_ptr_gray_sync2 <= rd_ptr_gray_sync1;
        end
    end

    // Full flag (combinational, no feedback loop)
    // Full when next write Gray equals read Gray with MSBs inverted
    wire [ADDR_WIDTH:0] wr_ptr_gray_next_comb =
        bin2gray(wr_ptr_bin + (wr_en && !wr_full ? 1'b1 : 1'b0));

    wire fifo_full_comb = (wr_ptr_gray_next_comb == {
                                ~rd_ptr_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1],
                                 rd_ptr_gray_sync2[ADDR_WIDTH-2:0]
                             });

    always @(posedge wr_clk or negedge wr_resetn) begin
        if (!wr_resetn)
            wr_full <= 1'b0;
        else
            wr_full <= fifo_full_comb;
    end

    // ================================================================
    // READ DOMAIN LOGIC
    // ================================================================
    // Next-state logic for read pointer
    always @* begin
        rd_ptr_bin_next  = rd_ptr_bin;
        rd_ptr_gray_next = rd_ptr_gray;

        if (rd_en && !rd_empty) begin
            rd_ptr_bin_next  = rd_ptr_bin + 1'b1;
            rd_ptr_gray_next = bin2gray(rd_ptr_bin_next);
        end
    end

    // Read pointer registers + data output
    always @(posedge rd_clk or negedge rd_resetn) begin
        if (!rd_resetn) begin
            rd_ptr_bin  <= { (ADDR_WIDTH+1){1'b0} };
            rd_ptr_gray <= { (ADDR_WIDTH+1){1'b0} };
            rd_data     <= { DATA_WIDTH{1'b0} };
        end else begin
            rd_ptr_bin  <= rd_ptr_bin_next;
            rd_ptr_gray <= rd_ptr_gray_next;

            if (rd_en && !rd_empty) begin
                rd_data <= mem[rd_ptr_bin[ADDR_WIDTH-1:0]];
            end
        end
    end

    // Synchronize write pointer Gray into read domain
    always @(posedge rd_clk or negedge rd_resetn) begin
        if (!rd_resetn) begin
            wr_ptr_gray_sync1 <= { (ADDR_WIDTH+1){1'b0} };
            wr_ptr_gray_sync2 <= { (ADDR_WIDTH+1){1'b0} };
        end else begin
            wr_ptr_gray_sync1 <= wr_ptr_gray;
            wr_ptr_gray_sync2 <= wr_ptr_gray_sync1;
        end
    end

    // Empty flag (combinational, no loop)
    wire [ADDR_WIDTH:0] rd_ptr_gray_next_comb =
        bin2gray(rd_ptr_bin + (rd_en && !rd_empty ? 1'b1 : 1'b0));

    wire fifo_empty_comb = (rd_ptr_gray_next_comb == wr_ptr_gray_sync2);

    always @(posedge rd_clk or negedge rd_resetn) begin
        if (!rd_resetn)
            rd_empty <= 1'b1;
        else
            rd_empty <= fifo_empty_comb;
    end

endmodule

`default_nettype wire
