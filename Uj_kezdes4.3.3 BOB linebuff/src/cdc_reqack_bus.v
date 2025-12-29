`timescale 1ns/1ps
`default_nettype none

module cdc_reqack_bus #(
    parameter integer W = 8
)(
    // ---------------- SRC domain ----------------
    input  wire         src_clk,
    input  wire         src_resetn,
    input  wire         src_send,        // 1-cycle pulse
    input  wire [W-1:0] src_data,
    output wire         src_busy,

    // crossing wires (internal)
    output reg  [W-1:0] bus = {W{1'b0}},
    output reg          req_tog = 1'b0,

    // ---------------- DST domain ----------------
    input  wire         dst_clk,
    input  wire         dst_resetn,
    output reg          dst_new = 1'b0,   // 1-cycle pulse
    output reg  [W-1:0] dst_data = {W{1'b0}},

    output reg          ack_tog = 1'b0
);

    // -------- SRC: wait for ACK edge --------
    reg busy_r = 1'b0;
    assign src_busy = busy_r;

    reg [2:0] ack_sync = 3'b000;
    wire ack_edge = ack_sync[2] ^ ack_sync[1];

    always @(posedge src_clk or negedge src_resetn) begin
        if (!src_resetn) begin
            busy_r   <= 1'b0;
            bus      <= {W{1'b0}};
            req_tog  <= 1'b0;
            ack_sync <= 3'b000;
        end else begin
            ack_sync <= {ack_sync[1:0], ack_tog};

            if (busy_r) begin
                if (ack_edge) busy_r <= 1'b0;
            end else begin
                if (src_send) begin
                    bus     <= src_data;
                    req_tog <= ~req_tog;
                    busy_r  <= 1'b1;
                end
            end
        end
    end

    // -------- DST: detect REQ edge, then sample bus one cycle later --------
    reg [2:0] req_sync = 3'b000;
    reg [W-1:0] bus_sync1 = {W{1'b0}}, bus_sync2 = {W{1'b0}};
    wire req_edge = req_sync[2] ^ req_sync[1];

    reg pending = 1'b0;

    always @(posedge dst_clk or negedge dst_resetn) begin
        if (!dst_resetn) begin
            req_sync  <= 3'b000;
            bus_sync1 <= {W{1'b0}};
            bus_sync2 <= {W{1'b0}};
            pending   <= 1'b0;

            dst_new   <= 1'b0;
            dst_data  <= {W{1'b0}};
            ack_tog   <= 1'b0;
        end else begin
            dst_new   <= 1'b0;

            req_sync  <= {req_sync[1:0], req_tog};
            bus_sync1 <= bus;
            bus_sync2 <= bus_sync1;

            if (req_edge) begin
                pending <= 1'b1;      // várunk 1 ciklust, hogy a bus_sync2 biztos “beérjen”
            end

            if (pending) begin
                pending  <= 1'b0;
                dst_data <= bus_sync2;
                dst_new  <= 1'b1;
                ack_tog  <= ~ack_tog;
            end
        end
    end

endmodule

`default_nettype wire
