// Dual-clock line buffer: 1 write port (cam_pclk), 1 read port (pix_clk).
// This should infer a dual-port block RAM instead of tons of DFFs.

module line_buffer_dc #(
    parameter H_ACTIVE = 720
)(
    // write port (camera domain)
    input  wire        wr_clk,
    input  wire        wr_en,
    input  wire [9:0]  wr_addr,
    input  wire [7:0]  wr_data,

    // read port (HDMI domain)
    input  wire        rd_clk,
    input  wire [9:0]  rd_addr,
    output reg  [7:0]  rd_data
);
    // 720 x 8-bit line buffer
    reg [7:0] mem [0:H_ACTIVE-1];

    // write side
    always @(posedge wr_clk) begin
        if (wr_en) begin
            mem[wr_addr] <= wr_data;
        end
    end

    // read side (synchronous)
    always @(posedge rd_clk) begin
        rd_data <= mem[rd_addr];
    end

endmodule
