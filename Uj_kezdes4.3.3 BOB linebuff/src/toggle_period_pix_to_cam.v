`timescale 1ns/1ps
`default_nettype none

// Méri egy "toggle" jel periódusát pix_clk ciklusokban.
// A toggle jel a cam_pclk domainből jön (cam_field_toggle),
// szinkronizáljuk pix_clk-ra, ott mérünk, majd az eredményt
// CDC-vel visszavisszük cam_pclk domainbe (logger/pager miatt).
module toggle_period_pix_to_cam #(
    parameter integer CNT_W = 32
)(
    // pix domain (mérés)
    input  wire             pix_clk,
    input  wire             pix_resetn,

    // cam domain (kimenet a pager/logger felé)
    input  wire             cam_clk,
    input  wire             cam_resetn,

    // async for pix domain (cam_clk domainből jön)
    input  wire             cam_toggle,

    output reg  [CNT_W-1:0] period_pix_cam,   // stabil érték cam domainben
    output reg              period_new_cam    // 1 clk széles pulse cam domainben
);

    // ------------------------------------------------------------
    // 1) cam_toggle -> pix_clk szinkron + edge detect
    // ------------------------------------------------------------
    reg [2:0] tog_sync_pix = 3'b000;
    always @(posedge pix_clk or negedge pix_resetn) begin
        if (!pix_resetn) tog_sync_pix <= 3'b000;
        else             tog_sync_pix <= {tog_sync_pix[1:0], cam_toggle};
    end

    wire tog_edge_pix = tog_sync_pix[2] ^ tog_sync_pix[1];

    // ------------------------------------------------------------
    // 2) pix domain period counter + latch + update toggle
    // ------------------------------------------------------------
    reg [CNT_W-1:0] cnt_pix      = {CNT_W{1'b0}};
    reg [CNT_W-1:0] latched_pix  = {CNT_W{1'b0}};
    reg             upd_tog_pix  = 1'b0;

    always @(posedge pix_clk or negedge pix_resetn) begin
        if (!pix_resetn) begin
            cnt_pix     <= {CNT_W{1'b0}};
            latched_pix <= {CNT_W{1'b0}};
            upd_tog_pix <= 1'b0;
        end else begin
            cnt_pix <= cnt_pix + {{(CNT_W-1){1'b0}},1'b1};

            if (tog_edge_pix) begin
                latched_pix <= cnt_pix;      // előző periódus
                cnt_pix     <= {CNT_W{1'b0}};
                upd_tog_pix <= ~upd_tog_pix; // jelzi: új adat
            end
        end
    end

    // ------------------------------------------------------------
    // 3) CDC pix -> cam: toggle + bus
    //    (bus két flop, toggle három flop)
    // ------------------------------------------------------------
    reg [2:0]  upd_sync_cam = 3'b000;
    reg [CNT_W-1:0] bus_sync1 = {CNT_W{1'b0}};
    reg [CNT_W-1:0] bus_sync2 = {CNT_W{1'b0}};

    wire upd_new_cam = upd_sync_cam[2] ^ upd_sync_cam[1];

    always @(posedge cam_clk or negedge cam_resetn) begin
        if (!cam_resetn) begin
            upd_sync_cam    <= 3'b000;
            bus_sync1       <= {CNT_W{1'b0}};
            bus_sync2       <= {CNT_W{1'b0}};
            period_pix_cam  <= {CNT_W{1'b0}};
            period_new_cam  <= 1'b0;
        end else begin
            upd_sync_cam <= {upd_sync_cam[1:0], upd_tog_pix};

            bus_sync1 <= latched_pix;
            bus_sync2 <= bus_sync1;

            period_new_cam <= 1'b0;
            if (upd_new_cam) begin
                period_pix_cam <= bus_sync2;
                period_new_cam <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
