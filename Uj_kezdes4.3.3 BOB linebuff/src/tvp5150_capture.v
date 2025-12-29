`timescale 1ns / 1ps
`default_nettype none

module tvp5150_capture (
    input  wire        pclk,
    input  wire        resetn,
    input  wire [7:0]  d,

    output reg         frame_valid = 1'b0,
    output reg         line_valid  = 1'b0,
    output reg [7:0]   y_out       = 8'd0,
    output reg         y_valid     = 1'b0,

    output reg         frame_start  = 1'b0,  // 1 clk pulse
    output reg         field_toggle = 1'b0   // toggles once per field (aligned to first active SAV)
);

    // sync finder: FF 00 00 XY
    reg [1:0] sync_state = 2'd0;

    // 4:2:2 phase: Cb Y Cr Y ...
    reg [1:0] pix_phase  = 2'd0;

    // previous V and F (from last accepted status byte)
    reg prev_V = 1'b1;
    reg prev_F = 1'b0;

    // NEW: field-start pending flag (set by F change and/or V fall; consumed at first active SAV)
    reg pending_field = 1'b1;     // induljon “pöccre” reset után is
    reg got_first_active = 1'b0;  // csak hogy ne maradjon örökké pending

    // status decode temporaries (kept for compatibility / debug)
    reg status_F, status_V, status_H;
    reg [3:0] pexp;
    reg       ok;

    // helper: parity check (BT.656)
    wire parity_ok = (d[7] == 1'b1) &&
                     (d[3:0] == { (d[5] ^ d[4]),
                                  (d[6] ^ d[4]),
                                  (d[6] ^ d[5]),
                                  (d[6] ^ d[5] ^ d[4]) });

    always @(posedge pclk or negedge resetn) begin
        if (!resetn) begin
            sync_state      <= 2'd0;
            pix_phase       <= 2'd0;
            frame_valid     <= 1'b0;
            line_valid      <= 1'b0;
            y_out           <= 8'd0;
            y_valid         <= 1'b0;
            frame_start     <= 1'b0;
            field_toggle    <= 1'b0;

            prev_V          <= 1'b1;
            prev_F          <= 1'b0;
            pending_field   <= 1'b1;
            got_first_active<= 1'b0;

            status_F <= 1'b0;
            status_V <= 1'b0;
            status_H <= 1'b0;
            pexp     <= 4'd0;
            ok       <= 1'b0;

        end else begin
            y_valid     <= 1'b0;
            frame_start <= 1'b0;

            // ------------------------------
            // BT.656 sync state machine
            // ------------------------------
            case (sync_state)
                2'd0: begin
                    if (d == 8'hFF)
                        sync_state <= 2'd1;
                end

                2'd1: begin
                    if (d == 8'h00)
                        sync_state <= 2'd2;
                    else if (d != 8'hFF)
                        sync_state <= 2'd0;
                end

                2'd2: begin
                    if (d == 8'h00)
                        sync_state <= 2'd3;
                    else if (d == 8'hFF)
                        sync_state <= 2'd1;
                    else
                        sync_state <= 2'd0;
                end

                2'd3: begin
                    // XY byte: [7]=1, [6]=F, [5]=V, [4]=H, [3:0]=P3..P0
                    status_F <= d[6];
                    status_V <= d[5];
                    status_H <= d[4];

                    pexp <= { (d[5] ^ d[4]),
                              (d[6] ^ d[4]),
                              (d[6] ^ d[5]),
                              (d[6] ^ d[5] ^ d[4]) };

                    ok <= parity_ok;

                    if (parity_ok) begin
                        // --- NEW: field detect robustly ---
                        // If F changes anywhere (even in blank), mark pending.
                        if (d[6] != prev_F) begin
                            prev_F        <= d[6];
                            pending_field <= 1'b1;
                        end

                        // Also keep the old V-fall trigger as a fallback (some configs keep F fixed)
                        if ((prev_V == 1'b1) && (d[5] == 1'b0)) begin
                            pending_field <= 1'b1;
                        end

                        // accepted timing word
                        if (d[4] == 1'b0) begin
                            // SAV
                            frame_valid <= (d[5] == 1'b0); // V=0 active
                            line_valid  <= (d[5] == 1'b0);
                            pix_phase   <= 2'd0;

                            // first ACTIVE SAV of a field: consume pending and toggle
                            if (d[5] == 1'b0) begin
                                got_first_active <= 1'b1;

                                if (pending_field || !got_first_active) begin
                                    frame_start   <= 1'b1;
                                    field_toggle  <= ~field_toggle;
                                    pending_field <= 1'b0;
                                end
                            end
                        end else begin
                            // EAV
                            line_valid  <= 1'b0;
                            frame_valid <= (d[5] == 1'b0);
                        end

                        // update prev_V only on valid XY
                        prev_V <= d[5];
                    end
                    // if not parity_ok: ignore as data, do NOT update prev_V/prev_F

                    sync_state <= 2'd0;
                end
            endcase

            // ---------------------------------------
            // Active video: extract Y on phase 1 and 3
            // ---------------------------------------
            if (frame_valid && line_valid && (sync_state == 2'd0)) begin
                pix_phase <= pix_phase + 2'd1;

                if ((pix_phase == 2'd1) || (pix_phase == 2'd3)) begin
                    y_out   <= d;
                    y_valid <= 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire

