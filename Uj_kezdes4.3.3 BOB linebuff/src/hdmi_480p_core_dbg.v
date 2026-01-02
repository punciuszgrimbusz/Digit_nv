`timescale 1ns / 1ps
`default_nettype none
`include "svo_defines.vh"

module hdmi_480p_core (
    input  wire        pix_clk,
    input  wire        pix_clk_5x,
    input  wire        resetn,

    // Camera / TVP5150 side
    input  wire        cam_pclk,
    input  wire        cam_resetn,
    input  wire        cam_line_valid,
    input  wire        cam_y_valid,
    input  wire [7:0]  cam_y,
    input  wire        cam_frame_toggle,   // field toggle

    // --- NEW: debug outputs (CAM domain stable) ---
    output reg  [4:0]  dbg_desc_count_cam      = 5'd0,
    output reg  [9:0]  dbg_underflow_low10_cam = 10'd0,
    output reg  [9:0]  dbg_overflow_low10_cam  = 10'd0,
    output reg  [2:0]  dbg_drop_used_cam       = 3'd0,
    output reg  [2:0]  dbg_dup_used_cam        = 3'd0,
    output reg         dbg_resync_used_cam     = 1'b0,
    output reg  [4:0]  dbg_desc_min_cam        = 5'd0,
    output reg  [4:0]  dbg_desc_max_cam        = 5'd0,
    output reg  [3:0]  dbg_marker_off_cam      = 4'd0,
    output reg         dbg_marker_found_cam    = 1'b0,

    // --- NEW: core cam-domain counterek ---
    output reg  [15:0] dbg_cam_fieldtog_cnt    = 16'd0,
    output reg  [15:0] dbg_cam_marker_inj_cnt  = 16'd0,
    output reg  [15:0] dbg_cam_desc_sent_cnt   = 16'd0,

    // HDMI TMDS outputs
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire [2:0]  tmds_d_p,
    output wire [2:0]  tmds_d_n
);

    // ------------------------------------------------------------
    // 720x480p timing @ ~27 MHz
    // ------------------------------------------------------------
    localparam integer H_ACTIVE = 720;
    localparam integer H_FP     = 16;
    localparam integer H_SYNC   = 62;
    localparam integer H_BP     = 60;
    localparam integer H_TOTAL  = H_ACTIVE + H_FP + H_SYNC + H_BP; // 858

    localparam integer V_ACTIVE = 480;
    localparam integer V_FP     = 9;
    localparam integer V_SYNC   = 6;
    localparam integer V_BP     = 30;
    localparam integer V_TOTAL  = V_ACTIVE + V_FP + V_SYNC + V_BP; // 525

    reg [10:0] h_cnt = 11'd0;
    reg [9:0]  v_cnt = 10'd0;

    always @(posedge pix_clk or negedge resetn) begin
        if (!resetn) begin
            h_cnt <= 11'd0;
            v_cnt <= 10'd0;
        end else begin
            if (h_cnt == H_TOTAL - 1) begin
                h_cnt <= 11'd0;
                if (v_cnt == V_TOTAL - 1) v_cnt <= 10'd0;
                else                      v_cnt <= v_cnt + 10'd1;
            end else begin
                h_cnt <= h_cnt + 11'd1;
            end
        end
    end

    wire de = (h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE);

    wire hsync = ~((h_cnt >= H_ACTIVE + H_FP) &&
                   (h_cnt <  H_ACTIVE + H_FP + H_SYNC));

    wire vsync = ~((v_cnt >= V_ACTIVE + V_FP) &&
                   (v_cnt <  V_ACTIVE + V_FP + V_SYNC));

    wire frame_start = (h_cnt==11'd0) && (v_cnt==10'd0);
    wire line_start_any = (h_cnt == 11'd0);

    // ------------------------------------------------------------
    // Buffers + descriptor FIFO
    // ------------------------------------------------------------
    localparam integer NUM_BUFS   = 8;
    localparam integer BUF_BITS   = 3;

    localparam integer DESC_DEPTH = 16;
    localparam integer DESC_BITS  = 4;
    localparam [DESC_BITS-1:0] DESC_MASK = 4'hF;

    localparam integer CAM_DESC_FRAME_BITS = 16;
    localparam integer CAM_DESC_Y_BITS     = 10;
    localparam integer CAM_DESC_DATA_BITS  = 1 + CAM_DESC_FRAME_BITS + CAM_DESC_Y_BITS + BUF_BITS;

    localparam integer DESC_BUF_LSB    = 0;
    localparam integer DESC_BUF_MSB    = BUF_BITS-1;
    localparam integer DESC_Y_LSB      = BUF_BITS;
    localparam integer DESC_Y_MSB      = DESC_Y_LSB + CAM_DESC_Y_BITS - 1;
    localparam integer DESC_FRAME_LSB  = DESC_Y_LSB + CAM_DESC_Y_BITS;
    localparam integer DESC_FRAME_MSB  = DESC_FRAME_LSB + CAM_DESC_FRAME_BITS - 1;
    localparam integer DESC_MARKER_BIT = CAM_DESC_DATA_BITS-1;

    // desc_bus layout: {marker, frame_id, line_y, buf_idx}

    function [7:0] onehot8;
        input [2:0] idx;
        begin
            onehot8 = (8'b0000_0001 << idx);
        end
    endfunction

    // ============================================================
    // PIX -> CAM: release handshake (no-ACK, same as stable)
    // ============================================================
    reg  [7:0] rel_mask_bus;
    reg        rel_toggle;
    reg        rel_pending;
    reg  [7:0] rel_accum;

    reg [2:0] rel_toggle_sync;
    reg [7:0] rel_mask_sync1, rel_mask_sync2;

    always @(posedge cam_pclk or negedge cam_resetn) begin
        if (!cam_resetn) begin
            rel_toggle_sync <= 3'b000;
            rel_mask_sync1  <= 8'h00;
            rel_mask_sync2  <= 8'h00;
        end else begin
            rel_toggle_sync <= {rel_toggle_sync[1:0], rel_toggle};
            rel_mask_sync1  <= rel_mask_bus;
            rel_mask_sync2  <= rel_mask_sync1;
        end
    end

    wire rel_new = rel_toggle_sync[2] ^ rel_toggle_sync[1];

    // ============================================================
    // CAMERA DOMAIN: buffer ownership + descriptor sender
    // ============================================================
    reg [7:0] free_map;
    reg [7:0] free_map_n;

    reg [BUF_BITS-1:0] wr_buf_idx;
    reg [9:0]          wr_addr;

    reg cam_line_valid_d;
    reg cam_frame_toggle_d;

    reg frame_flag_next_line;
    reg cur_line_is_frame_start_cam;

    reg [CAM_DESC_DATA_BITS-1:0] desc_bus;
    reg              desc_toggle;
    reg              desc_pending;

    reg drop_this_line;

    reg [CAM_DESC_FRAME_BITS-1:0] in_frame_id;
    reg [CAM_DESC_Y_BITS-1:0]     cur_line_y;
    reg [9:0]                     line_in_field;

    wire frame_edge = cam_frame_toggle ^ cam_frame_toggle_d;

    reg [BUF_BITS-1:0] alloc_idx;
    reg                alloc_ok;

    wire line_start = cam_line_valid && !cam_line_valid_d;
    wire line_end   = !cam_line_valid && cam_line_valid_d;

    wire [9:0] line_in_field_pre = frame_edge ? 10'd0 : line_in_field;
    always @* begin
        alloc_ok  = 1'b1;
        alloc_idx = 3'd0;
        if      (free_map[0]) alloc_idx = 3'd0;
        else if (free_map[1]) alloc_idx = 3'd1;
        else if (free_map[2]) alloc_idx = 3'd2;
        else if (free_map[3]) alloc_idx = 3'd3;
        else if (free_map[4]) alloc_idx = 3'd4;
        else if (free_map[5]) alloc_idx = 3'd5;
        else if (free_map[6]) alloc_idx = 3'd6;
        else if (free_map[7]) alloc_idx = 3'd7;
        else begin
            alloc_ok  = 1'b0;
            alloc_idx = 3'd0;
        end
    end

    always @(posedge cam_pclk or negedge cam_resetn) begin
        if (!cam_resetn) begin
            free_map                    <= 8'hFF;
            wr_buf_idx                  <= 3'd0;
            wr_addr                     <= 10'd0;
            cam_line_valid_d            <= 1'b0;
            cam_frame_toggle_d          <= 1'b0;
            frame_flag_next_line        <= 1'b0;
            cur_line_is_frame_start_cam <= 1'b0;
            desc_bus                    <= {CAM_DESC_DATA_BITS{1'b0}};
            desc_toggle                 <= 1'b0;
            desc_pending                <= 1'b0;
            drop_this_line              <= 1'b0;

            in_frame_id                 <= {CAM_DESC_FRAME_BITS{1'b0}};
            cur_line_y                  <= {CAM_DESC_Y_BITS{1'b0}};
            line_in_field               <= 10'd0;

            dbg_cam_fieldtog_cnt        <= 16'd0;
            dbg_cam_marker_inj_cnt      <= 16'd0;
            dbg_cam_desc_sent_cnt       <= 16'd0;
        end else begin
            cam_line_valid_d   <= cam_line_valid;
            cam_frame_toggle_d <= cam_frame_toggle;

            free_map_n = free_map;

            if (rel_new) begin
                free_map_n = free_map_n | rel_mask_sync2;
            end

            if (desc_pending) begin
                desc_toggle  <= ~desc_toggle;
                desc_pending <= 1'b0;
            end

            if (frame_edge) begin
                frame_flag_next_line <= 1'b1;
                dbg_cam_fieldtog_cnt <= dbg_cam_fieldtog_cnt + 16'd1;

                in_frame_id <= in_frame_id + {{(CAM_DESC_FRAME_BITS-1){1'b0}}, 1'b1};
                line_in_field <= 10'd0;
                cur_line_y    <= {CAM_DESC_Y_BITS{1'b0}};
            end

            if (line_start) begin
                wr_addr <= 10'd0;

                cur_line_is_frame_start_cam <= frame_flag_next_line | frame_edge;
                frame_flag_next_line        <= 1'b0;

                cur_line_y    <= line_in_field_pre[CAM_DESC_Y_BITS-1:0];
                line_in_field <= line_in_field_pre + 10'd1;

                if (alloc_ok) begin
                    wr_buf_idx            <= alloc_idx;
                    free_map_n[alloc_idx] <= 1'b0;
                    drop_this_line        <= 1'b0;
                end else begin
                    drop_this_line <= 1'b1;
                end
            end

            if (cam_line_valid && cam_y_valid && !drop_this_line) begin
                if (wr_addr != 10'h3FF) wr_addr <= wr_addr + 10'd1;
            end

            if (line_end) begin
                if (!drop_this_line) begin
                    desc_bus     <= {cur_line_is_frame_start_cam, in_frame_id, cur_line_y, wr_buf_idx};
                    desc_pending <= 1'b1;

                    dbg_cam_desc_sent_cnt <= dbg_cam_desc_sent_cnt + 16'd1;
                    if (cur_line_is_frame_start_cam)
                        dbg_cam_marker_inj_cnt <= dbg_cam_marker_inj_cnt + 16'd1;
                end
                drop_this_line <= 1'b0;
            end

            free_map <= free_map_n;
        end
    end

    // ------------------------------------------------------------
    // 8 dual-clock line buffers
    // ------------------------------------------------------------
    wire [7:0] line_q0, line_q1, line_q2, line_q3;
    wire [7:0] line_q4, line_q5, line_q6, line_q7;

    wire wr_active = cam_line_valid && cam_y_valid && !drop_this_line && (wr_addr < H_ACTIVE);

    wire use_buf0 = wr_active && (wr_buf_idx == 3'd0);
    wire use_buf1 = wr_active && (wr_buf_idx == 3'd1);
    wire use_buf2 = wr_active && (wr_buf_idx == 3'd2);
    wire use_buf3 = wr_active && (wr_buf_idx == 3'd3);
    wire use_buf4 = wr_active && (wr_buf_idx == 3'd4);
    wire use_buf5 = wr_active && (wr_buf_idx == 3'd5);
    wire use_buf6 = wr_active && (wr_buf_idx == 3'd6);
    wire use_buf7 = wr_active && (wr_buf_idx == 3'd7);

    wire [9:0] rd_addr = h_cnt[9:0];

    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf0 (.wr_clk(cam_pclk),.wr_en(use_buf0),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q0));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf1 (.wr_clk(cam_pclk),.wr_en(use_buf1),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q1));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf2 (.wr_clk(cam_pclk),.wr_en(use_buf2),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q2));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf3 (.wr_clk(cam_pclk),.wr_en(use_buf3),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q3));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf4 (.wr_clk(cam_pclk),.wr_en(use_buf4),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q4));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf5 (.wr_clk(cam_pclk),.wr_en(use_buf5),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q5));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf6 (.wr_clk(cam_pclk),.wr_en(use_buf6),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q6));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf7 (.wr_clk(cam_pclk),.wr_en(use_buf7),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q7));

    // ============================================================
    // PIX DOMAIN: descriptor sync + FIFO + bob + watermark correction
    // ============================================================
    reg [2:0]        desc_toggle_sync;
    reg [CAM_DESC_DATA_BITS-1:0] desc_bus_sync1;
    reg [CAM_DESC_DATA_BITS-1:0] desc_bus_sync2;

    always @(posedge pix_clk or negedge resetn) begin
        if (!resetn) begin
            desc_toggle_sync <= 3'b000;
            desc_bus_sync1   <= {CAM_DESC_DATA_BITS{1'b0}};
            desc_bus_sync2   <= {CAM_DESC_DATA_BITS{1'b0}};
        end else begin
            desc_toggle_sync <= {desc_toggle_sync[1:0], desc_toggle};
            desc_bus_sync1   <= desc_bus;
            desc_bus_sync2   <= desc_bus_sync1;
        end
    end

    wire desc_new = desc_toggle_sync[2] ^ desc_toggle_sync[1];

    reg [CAM_DESC_DATA_BITS-1:0] desc_fifo [0:DESC_DEPTH-1];
    reg [DESC_BITS-1:0] desc_wr_ptr;
    reg [DESC_BITS-1:0] desc_rd_ptr;
    reg [4:0]           desc_count;

    reg [DESC_BITS-1:0] desc_wr_ptr_n;
    reg [DESC_BITS-1:0] desc_rd_ptr_n;
    reg [4:0]           desc_count_n;

    reg                 have_any_line;
    reg                 have_any_line_n;

    reg de_d, vsync_d;
    reg repeat_phase, repeat_phase_n;

    reg [BUF_BITS-1:0] cur_buf_idx_r;
    reg [BUF_BITS-1:0] cur_buf_idx_r_n;
    reg                cur_buf_valid;
    reg                cur_buf_valid_n;

    reg [7:0] rel_accum_n;
    reg [7:0] rel_mask_bus_n;
    reg       rel_pending_n;
    reg       rel_toggle_n;

    localparam integer HIGH_WM    = 12;
    localparam integer LOW_WM     = 4;
    localparam integer SAFE_START = V_ACTIVE - 24;

    reg do_drop, do_dup;
    reg do_drop_n, do_dup_n;

    // ---- NEW debug counters (pix domain, per HDMI frame) ----
    reg [9:0] underflow_cnt, overflow_cnt;
    reg [2:0] drop_used, dup_used;
    reg       resync_used; // ebben a stabil core-ban alapból 0 (később marker-alignnál fogjuk használni)
    reg [4:0] desc_min, desc_max;

    reg [9:0] underflow_cnt_n, overflow_cnt_n;
    reg [2:0] drop_used_n, dup_used_n;
    reg       resync_used_n;
    reg [4:0] desc_min_n, desc_max_n;

    // ---- marker scan (pix domain, combinational) ----
    reg       marker_found_pix;
    reg [3:0] marker_off_pix;
    integer mi;
    reg [DESC_BITS-1:0] midx;

    always @* begin
        marker_found_pix = 1'b0;
        marker_off_pix   = 4'd0;
        for (mi = 0; mi < DESC_DEPTH; mi = mi + 1) begin
            if (!marker_found_pix && (mi < desc_count)) begin
                midx = (desc_rd_ptr + mi[DESC_BITS-1:0]) & DESC_MASK;
                if (desc_fifo[midx][DESC_MARKER_BIT]) begin
                    marker_found_pix = 1'b1;
                    marker_off_pix   = mi[3:0];
                end
            end
        end
    end

    // ---- debug snapshot bus (pix->cam) ----
    reg [63:0] dbg_bus_pix = 64'd0;
    reg        dbg_tog_pix = 1'b0;

    always @(posedge pix_clk or negedge resetn) begin
        if (!resetn) begin
            desc_wr_ptr   <= {DESC_BITS{1'b0}};
            desc_rd_ptr   <= {DESC_BITS{1'b0}};
            desc_count    <= 5'd0;

            have_any_line <= 1'b0;
            de_d          <= 1'b0;
            vsync_d       <= 1'b1;
            repeat_phase  <= 1'b0;

            cur_buf_idx_r <= 3'd0;
            cur_buf_valid <= 1'b0;

            rel_mask_bus  <= 8'h00;
            rel_toggle    <= 1'b0;
            rel_pending   <= 1'b0;
            rel_accum     <= 8'h00;

            do_drop       <= 1'b0;
            do_dup        <= 1'b0;

            underflow_cnt <= 10'd0;
            overflow_cnt  <= 10'd0;
            drop_used     <= 3'd0;
            dup_used      <= 3'd0;
            resync_used   <= 1'b0;
            desc_min      <= 5'd0;
            desc_max      <= 5'd0;

            dbg_bus_pix   <= 64'd0;
            dbg_tog_pix   <= 1'b0;

        end else begin
            // defaults
            desc_wr_ptr_n    = desc_wr_ptr;
            desc_rd_ptr_n    = desc_rd_ptr;
            desc_count_n     = desc_count;

            have_any_line_n  = have_any_line;
            de_d             <= de;
            vsync_d          <= vsync;
            repeat_phase_n   = repeat_phase;

            cur_buf_idx_r_n  = cur_buf_idx_r;
            cur_buf_valid_n  = cur_buf_valid;

            rel_accum_n      = rel_accum;
            rel_mask_bus_n   = rel_mask_bus;
            rel_pending_n    = rel_pending;
            rel_toggle_n     = rel_toggle;

            do_drop_n        = do_drop;
            do_dup_n         = do_dup;

            underflow_cnt_n  = underflow_cnt;
            overflow_cnt_n   = overflow_cnt;
            drop_used_n      = drop_used;
            dup_used_n       = dup_used;
            resync_used_n    = resync_used;
            desc_min_n       = desc_min;
            desc_max_n       = desc_max;

            // decide once per HDMI frame (vsync falling edge)
            if (!vsync && vsync_d) begin
                do_drop_n      = (desc_count > HIGH_WM);
                do_dup_n       = (desc_count < LOW_WM);
                repeat_phase_n = 1'b0;
            end

            // ship pending releases
            if (!rel_pending_n && (rel_accum_n != 8'h00)) begin
                rel_mask_bus_n = rel_accum_n;
                rel_accum_n    = 8'h00;
                rel_pending_n  = 1'b1;
            end
            if (rel_pending_n) begin
                rel_toggle_n   = ~rel_toggle_n;
                rel_pending_n  = 1'b0;
            end

            // receive descriptor from camera
            if (desc_new) begin
                have_any_line_n = 1'b1;

                if (desc_count_n < DESC_DEPTH) begin
                    desc_fifo[desc_wr_ptr_n] = desc_bus_sync2;
                    desc_wr_ptr_n            = (desc_wr_ptr_n + 1'b1) & DESC_MASK;
                    desc_count_n             = desc_count_n + 1'b1;
                end else begin
                    overflow_cnt_n = overflow_cnt_n + 10'd1;
                    rel_accum_n    = rel_accum_n | onehot8(desc_bus_sync2[DESC_BUF_MSB:DESC_BUF_LSB]);
                end
            end

            // line start in active video (DE rising)
            if (de && !de_d && (v_cnt < V_ACTIVE)) begin
                if (!repeat_phase_n) begin
                    if (desc_count_n != 0) begin
                        if (cur_buf_valid_n) rel_accum_n = rel_accum_n | onehot8(cur_buf_idx_r_n);

                        cur_buf_idx_r_n = desc_fifo[desc_rd_ptr_n][DESC_BUF_MSB:DESC_BUF_LSB];
                        cur_buf_valid_n = 1'b1;

                        if (desc_fifo[desc_rd_ptr_n][DESC_MARKER_BIT]) begin
                        cur_buf_idx_r_n = desc_fifo[desc_rd_ptr_n][BUF_BITS-1:0];
                        cur_buf_valid_n = 1'b1;

                        if (desc_fifo[desc_rd_ptr_n][BUF_BITS]) begin
                            repeat_phase_n = 1'b0;
                        end

                        desc_rd_ptr_n = (desc_rd_ptr_n + 1'b1) & DESC_MASK;
                        desc_count_n  = desc_count_n - 1'b1;
                    end else begin
                        underflow_cnt_n = underflow_cnt_n + 10'd1;
                    end
                end

                repeat_phase_n = ~repeat_phase_n;

                // min/max update
                if (desc_count_n < desc_min_n) desc_min_n = desc_count_n;
                if (desc_count_n > desc_max_n) desc_max_n = desc_count_n;
            end

            if (line_start_any && (v_cnt >= V_ACTIVE)) begin
                if ((desc_count_n != 0) && (do_drop_n || (desc_count_n > HIGH_WM))) begin
                    rel_accum_n   = rel_accum_n | onehot8(desc_fifo[desc_rd_ptr_n][DESC_BUF_MSB:DESC_BUF_LSB]);
                    rel_accum_n   = rel_accum_n | onehot8(desc_fifo[desc_rd_ptr_n][BUF_BITS-1:0]);
                    desc_rd_ptr_n = (desc_rd_ptr_n + 1'b1) & DESC_MASK;
                    desc_count_n  = desc_count_n - 1'b1;
                    do_drop_n     = 1'b0;
                    if (drop_used_n != 3'd7) drop_used_n = drop_used_n + 3'd1;
                end else if (desc_count_n < LOW_WM) begin
                    do_dup_n = 1'b0;
                    if (dup_used_n != 3'd7) dup_used_n = dup_used_n + 3'd1;
                end

                // min/max update
                if (desc_count_n < desc_min_n) desc_min_n = desc_count_n;
                if (desc_count_n > desc_max_n) desc_max_n = desc_count_n;
            end

            // per HDMI frame reset (frame_start)
            if (frame_start) begin
                underflow_cnt_n = 10'd0;
                overflow_cnt_n  = 10'd0;
                drop_used_n     = 3'd0;
                dup_used_n      = 3'd0;
                resync_used_n   = 1'b0;

                desc_min_n      = desc_count_n;
                desc_max_n      = desc_count_n;

                // snapshot bus
                dbg_bus_pix <= {
                    8'd0,                 // [63:56] reserved
                    1'b0,                 // [55] reserved
                    8'd0,                 // [54:47] reserved
                    marker_found_pix,     // [46]
                    marker_off_pix,       // [45:42]
                    desc_max_n,           // [41:37]
                    desc_min_n,           // [36:32]
                    resync_used_n,        // [31]
                    dup_used_n,           // [30:28]
                    drop_used_n,          // [27:25]
                    overflow_cnt_n,       // [24:15]
                    underflow_cnt_n,      // [14:5]
                    desc_count_n          // [4:0]
                };
                dbg_tog_pix <= ~dbg_tog_pix;
            end

            // commit
            desc_wr_ptr   <= desc_wr_ptr_n;
            desc_rd_ptr   <= desc_rd_ptr_n;
            desc_count    <= desc_count_n;

            have_any_line <= have_any_line_n;
            repeat_phase  <= repeat_phase_n;

            cur_buf_idx_r <= cur_buf_idx_r_n;
            cur_buf_valid <= cur_buf_valid_n;

            rel_mask_bus  <= rel_mask_bus_n;
            rel_toggle    <= rel_toggle_n;
            rel_pending   <= rel_pending_n;
            rel_accum     <= rel_accum_n;

            do_drop       <= do_drop_n;
            do_dup        <= do_dup_n;

            underflow_cnt <= underflow_cnt_n;
            overflow_cnt  <= overflow_cnt_n;
            drop_used     <= drop_used_n;
            dup_used      <= dup_used_n;
            resync_used   <= resync_used_n;
            desc_min      <= desc_min_n;
            desc_max      <= desc_max_n;
        end
    end

    // ------------------------------------------------------------
    // DEBUG CDC: pix -> cam snapshot bus
    // ------------------------------------------------------------
    reg [2:0]  dbg_tsync = 3'b000;
    reg [63:0] dbg_bus_sync1 = 64'd0;
    reg [63:0] dbg_bus_sync2 = 64'd0;
    wire dbg_new = dbg_tsync[2] ^ dbg_tsync[1];

    always @(posedge cam_pclk or negedge cam_resetn) begin
        if (!cam_resetn) begin
            dbg_tsync <= 3'b000;
            dbg_bus_sync1 <= 64'd0;
            dbg_bus_sync2 <= 64'd0;

            dbg_desc_count_cam      <= 5'd0;
            dbg_underflow_low10_cam <= 10'd0;
            dbg_overflow_low10_cam  <= 10'd0;
            dbg_drop_used_cam       <= 3'd0;
            dbg_dup_used_cam        <= 3'd0;
            dbg_resync_used_cam     <= 1'b0;
            dbg_desc_min_cam        <= 5'd0;
            dbg_desc_max_cam        <= 5'd0;
            dbg_marker_off_cam      <= 4'd0;
            dbg_marker_found_cam    <= 1'b0;
        end else begin
            dbg_tsync     <= {dbg_tsync[1:0], dbg_tog_pix};
            dbg_bus_sync1 <= dbg_bus_pix;
            dbg_bus_sync2 <= dbg_bus_sync1;

            if (dbg_new) begin
                dbg_desc_count_cam      <= dbg_bus_sync2[4:0];
                dbg_underflow_low10_cam <= dbg_bus_sync2[14:5];
                dbg_overflow_low10_cam  <= dbg_bus_sync2[24:15];
                dbg_drop_used_cam       <= dbg_bus_sync2[27:25];
                dbg_dup_used_cam        <= dbg_bus_sync2[30:28];
                dbg_resync_used_cam     <= dbg_bus_sync2[31];
                dbg_desc_min_cam        <= dbg_bus_sync2[36:32];
                dbg_desc_max_cam        <= dbg_bus_sync2[41:37];
                dbg_marker_off_cam      <= dbg_bus_sync2[45:42];
                dbg_marker_found_cam    <= dbg_bus_sync2[46];
            end
        end
    end

    // ------------------------------------------------------------
    // Read selected line buffer
    // ------------------------------------------------------------
    reg [7:0] cam_y_sample;
    always @* begin
        case (cur_buf_idx_r)
            3'd0: cam_y_sample = line_q0;
            3'd1: cam_y_sample = line_q1;
            3'd2: cam_y_sample = line_q2;
            3'd3: cam_y_sample = line_q3;
            3'd4: cam_y_sample = line_q4;
            3'd5: cam_y_sample = line_q5;
            3'd6: cam_y_sample = line_q6;
            3'd7: cam_y_sample = line_q7;
            default: cam_y_sample = 8'd0;
        endcase
    end

    // ------------------------------------------------------------
    // Pixel generator (grayscale)
    // ------------------------------------------------------------
    reg [7:0] y_reg;
    always @(posedge pix_clk or negedge resetn) begin
        if (!resetn) begin
            y_reg <= 8'd0;
        end else begin
            if (de) begin
                if (have_any_line && cur_buf_valid)
                    y_reg <= cam_y_sample;
                else
                    y_reg <= 8'd0;
            end else begin
                y_reg <= 8'd0;
            end
        end
    end

    wire [7:0] red   = y_reg;
    wire [7:0] green = y_reg;
    wire [7:0] blue  = y_reg;

    // ------------------------------------------------------------
    // TMDS encoding
    // ------------------------------------------------------------
    wire [9:0] tmds_b, tmds_g, tmds_r;

    svo_tmds enc_b (.clk(pix_clk), .resetn(resetn), .de(de), .ctrl({vsync, hsync}), .din(blue),  .dout(tmds_b));
    svo_tmds enc_g (.clk(pix_clk), .resetn(resetn), .de(de), .ctrl(2'b00),          .din(green), .dout(tmds_g));
    svo_tmds enc_r (.clk(pix_clk), .resetn(resetn), .de(de), .ctrl(2'b00),          .din(red),   .dout(tmds_r));

    wire [2:0] tmds_data;
    wire [2:0] tmds_d0, tmds_d1, tmds_d2, tmds_d3, tmds_d4;
    wire [2:0] tmds_d5, tmds_d6, tmds_d7, tmds_d8, tmds_d9;

    assign {tmds_d9[0], tmds_d8[0], tmds_d7[0], tmds_d6[0], tmds_d5[0],
            tmds_d4[0], tmds_d3[0], tmds_d2[0], tmds_d1[0], tmds_d0[0]} = tmds_b;

    assign {tmds_d9[1], tmds_d8[1], tmds_d7[1], tmds_d6[1], tmds_d5[1],
            tmds_d4[1], tmds_d3[1], tmds_d2[1], tmds_d1[1], tmds_d0[1]} = tmds_g;

    assign {tmds_d9[2], tmds_d8[2], tmds_d7[2], tmds_d6[2], tmds_d5[2],
            tmds_d4[2], tmds_d3[2], tmds_d2[2], tmds_d1[2], tmds_d0[2]} = tmds_r;

    OSER10 tmds_serdes [2:0] (
        .Q     (tmds_data),
        .D0    (tmds_d0), .D1(tmds_d1), .D2(tmds_d2), .D3(tmds_d3), .D4(tmds_d4),
        .D5    (tmds_d5), .D6(tmds_d6), .D7(tmds_d7), .D8(tmds_d8), .D9(tmds_d9),
        .PCLK  (pix_clk),
        .FCLK  (pix_clk_5x),
        .RESET (~resetn)
    );

    ELVDS_OBUF tmds_bufds [3:0] (
        .I  ({pix_clk, tmds_data}),
        .O  ({tmds_clk_p, tmds_d_p}),
        .OB ({tmds_clk_n, tmds_d_n})
    );

endmodule

`default_nettype wire
