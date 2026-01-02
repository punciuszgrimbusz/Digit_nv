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

    // --- debug outputs (CAM domain stable) ---
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

    // --- DEBUG V1 extra (PIX invariánsok -> CAM snapshot) ---
    output reg  [15:0] dbg_fault_sticky_cam      = 16'd0,
    output reg  [7:0]  dbg_own_map_cam           = 8'd0,    // NOTE: low8 only
    output reg  [15:0] dbg_rx_dupbuf_cnt_cam     = 16'd0,
    output reg  [15:0] dbg_rel_not_owned_cnt_cam = 16'd0,
    output reg  [7:0]  dbg_overflow_rel_lo8_cam  = 8'd0,

    // --- DEBUG V1 extra (CAM freepool telemetry) ---
    output reg  [3:0]  dbg_free_cnt_cam          = 4'd0,    // NOTE: saturated
    output reg  [3:0]  dbg_free_min_cam          = 4'd15,   // NOTE: saturated
    output reg  [3:0]  dbg_free_max_cam          = 4'd0,    // NOTE: saturated
    output reg  [15:0] dbg_alloc_fail_cnt_cam    = 16'd0,
    output reg  [15:0] dbg_rel_doublefree_cnt_cam= 16'd0,

    // --- DEBUG: last drop/dup/resync positions ---
    output reg  [15:0] dbg_last_drop_v_cam     = 16'd0,
    output reg  [15:0] dbg_last_dup_v_cam      = 16'd0,
    output reg  [15:0] dbg_last_resync_v_cam   = 16'd0,
    output reg  [15:0] dbg_last_drop_h_cam     = 16'd0,
    output reg  [15:0] dbg_last_dup_h_cam      = 16'd0,
    output reg  [15:0] dbg_last_resync_h_cam   = 16'd0,

    // --- core cam-domain counterek ---
    output reg  [15:0] dbg_cam_fieldtog_cnt    = 16'd0,
    output reg  [15:0] dbg_cam_marker_inj_cnt  = 16'd0,
    output reg  [15:0] dbg_cam_desc_sent_cnt   = 16'd0,

    // --- NEW: CAM oldali descriptor queue töltöttség (pagerhez/loghoz) ---
    output reg  [5:0]  dbg_cam_descq_cnt_cam   = 6'd0,

    // HDMI TMDS outputs
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire [2:0]  tmds_d_p,
    output wire [2:0]  tmds_d_n,

    // VSYNC toggle for diagnostics (pix -> cam domain)
    output reg         vsync_toggle_pix = 1'b0
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

    wire de     = (h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE);
    wire vblank = (v_cnt >= V_ACTIVE);

    wire hsync = ~((h_cnt >= H_ACTIVE + H_FP) &&
                   (h_cnt <  H_ACTIVE + H_FP + H_SYNC));

    wire vsync = ~((v_cnt >= V_ACTIVE + V_FP) &&
                   (v_cnt <  V_ACTIVE + V_FP + V_SYNC));

    wire frame_start = (h_cnt==11'd0) && (v_cnt==10'd0);
    wire line_start_any = (h_cnt == 11'd0);

    // ------------------------------------------------------------
    // Buffers + descriptor FIFO
    // ------------------------------------------------------------
    localparam integer NUM_BUFS   = 16;
    localparam integer BUF_BITS   = 4;

    // PIX oldali desc FIFO (elasztikusabb)
    localparam integer DESC_DEPTH = 32;
    localparam integer DESC_BITS  = 5;
    localparam [DESC_BITS-1:0] DESC_MASK = 5'h1F;

    function [15:0] onehot16;
        input [3:0] idx;
        begin
            onehot16 = (16'h0001 << idx);
        end
    endfunction

    function [4:0] popcount16;
        input [15:0] x;
        integer k;
        begin
            popcount16 = 5'd0;
            for (k = 0; k < 16; k = k + 1)
                popcount16 = popcount16 + x[k];
        end
    endfunction

    // ============================================================
    // CDC #1: PIX -> CAM release mask (16-bit)
    // ============================================================
    reg         rel_pend;
    reg [15:0]  rel_pend_data;
    wire        rel_send_pix = rel_pend && !rel_busy_pix;
    wire [15:0] rel_send_data_pix = rel_pend_data;
    wire        rel_busy_pix;

    wire [15:0] rel_bus_cdc;
    wire        rel_req_tog_cdc;
    wire        rel_ack_tog_cdc;

    wire        rel_new_cam;
    wire [15:0] rel_mask_cam;

    cdc_reqack_bus #(.W(NUM_BUFS)) u_rel_cdc (
        .src_clk    (pix_clk),
        .src_resetn (resetn),
        .src_send   (rel_send_pix),
        .src_data   (rel_send_data_pix),
        .src_busy   (rel_busy_pix),
        .bus        (rel_bus_cdc),
        .req_tog    (rel_req_tog_cdc),

        .dst_clk    (cam_pclk),
        .dst_resetn (cam_resetn),
        .dst_new    (rel_new_cam),
        .dst_data   (rel_mask_cam),
        .ack_tog    (rel_ack_tog_cdc)
    );

    // ============================================================
    // CDC #2: CAM -> PIX descriptor (frame marker + frame_id + y + idx)
    // ============================================================
    localparam integer CAM_DESC_DEPTH     = 32;
    localparam integer CAM_DESC_PTR_BITS  = 5;
    localparam integer CAM_DESC_FRAME_BITS= 16;
    localparam integer CAM_DESC_Y_BITS    = 10;
    localparam integer CAM_DESC_DATA_BITS = 1 + CAM_DESC_FRAME_BITS + CAM_DESC_Y_BITS + BUF_BITS;
    localparam [CAM_DESC_PTR_BITS-1:0] CAM_DESC_MASK = 5'h1F;

    reg  [CAM_DESC_DATA_BITS-1:0] cam_desc_fifo [0:CAM_DESC_DEPTH-1];
    reg  [CAM_DESC_PTR_BITS-1:0]  cam_desc_wr_ptr;
    reg  [CAM_DESC_PTR_BITS-1:0]  cam_desc_rd_ptr;
    reg  [5:0]                    cam_desc_count;

    wire [CAM_DESC_DATA_BITS-1:0] cam_desc_head = cam_desc_fifo[cam_desc_rd_ptr];

    wire desc_busy_cam;
    wire desc_send_cam = (cam_desc_count != 0) && !desc_busy_cam;

    wire [CAM_DESC_DATA_BITS-1:0] desc_bus_cdc;
    wire                          desc_req_tog_cdc;
    wire                          desc_ack_tog_cdc;

    wire                          desc_new_pix;
    wire [CAM_DESC_DATA_BITS-1:0] desc_data_pix;

    cdc_reqack_bus #(.W(CAM_DESC_DATA_BITS)) u_desc_cdc (
        .src_clk    (cam_pclk),
        .src_resetn (cam_resetn),
        .src_send   (desc_send_cam),
        .src_data   (cam_desc_head),
        .src_busy   (desc_busy_cam),
        .bus        (desc_bus_cdc),
        .req_tog    (desc_req_tog_cdc),

        .dst_clk    (pix_clk),
        .dst_resetn (resetn),
        .dst_new    (desc_new_pix),
        .dst_data   (desc_data_pix),
        .ack_tog    (desc_ack_tog_cdc)
    );

    // ============================================================
    // CAMERA DOMAIN: buffer ownership + descriptor producer
    // ============================================================
    reg [15:0] free_map;

    reg [BUF_BITS-1:0] wr_buf_idx;
    reg [9:0]          wr_addr;

    reg cam_line_valid_d;
    reg cam_frame_toggle_d;

    reg frame_flag_next_line;
    reg cur_line_is_frame_start_cam;

    reg drop_this_line;

    wire frame_edge = cam_frame_toggle ^ cam_frame_toggle_d;

    wire line_start = cam_line_valid && !cam_line_valid_d;
    wire line_end   = !cam_line_valid &&  cam_line_valid_d;

    localparam integer FIELD_LINES_TARGET = (V_ACTIVE/2); // 240
    reg  [9:0] line_in_field = 10'd0;

    reg  [CAM_DESC_FRAME_BITS-1:0] in_frame_id = {CAM_DESC_FRAME_BITS{1'b0}};
    reg  [CAM_DESC_Y_BITS-1:0]     cur_line_y;

    wire [9:0] line_in_field_pre = frame_edge ? 10'd0 : line_in_field;
    wire       limit_drop_pre    = (line_in_field_pre >= FIELD_LINES_TARGET);

    wire [15:0] free_map_eff = free_map | (rel_new_cam ? rel_mask_cam : 16'h0000);
    wire [4:0]  free_cnt_eff5 = popcount16(free_map_eff);

    // debug: saturáljuk 16->15-re, mert 4 bites port
    wire [3:0] free_cnt_eff4 = (free_cnt_eff5 >= 5'd15) ? 4'd15 : free_cnt_eff5[3:0];

    wire [15:0] rel_doublefree_bits = (rel_new_cam ? (free_map & rel_mask_cam) : 16'h0000);

    reg [BUF_BITS-1:0] alloc_idx;
    reg                alloc_ok;
    always @* begin
        alloc_ok  = 1'b1;
        alloc_idx = 4'd0;
        if      (free_map_eff[0])  alloc_idx = 4'd0;
        else if (free_map_eff[1])  alloc_idx = 4'd1;
        else if (free_map_eff[2])  alloc_idx = 4'd2;
        else if (free_map_eff[3])  alloc_idx = 4'd3;
        else if (free_map_eff[4])  alloc_idx = 4'd4;
        else if (free_map_eff[5])  alloc_idx = 4'd5;
        else if (free_map_eff[6])  alloc_idx = 4'd6;
        else if (free_map_eff[7])  alloc_idx = 4'd7;
        else if (free_map_eff[8])  alloc_idx = 4'd8;
        else if (free_map_eff[9])  alloc_idx = 4'd9;
        else if (free_map_eff[10]) alloc_idx = 4'd10;
        else if (free_map_eff[11]) alloc_idx = 4'd11;
        else if (free_map_eff[12]) alloc_idx = 4'd12;
        else if (free_map_eff[13]) alloc_idx = 4'd13;
        else if (free_map_eff[14]) alloc_idx = 4'd14;
        else if (free_map_eff[15]) alloc_idx = 4'd15;
        else begin
            alloc_ok  = 1'b0;
            alloc_idx = 4'd0;
        end
    end

    // calc regs (csak blocking!)
    reg [15:0]               free_map_calc;
    reg [CAM_DESC_PTR_BITS-1:0] cam_desc_wr_ptr_calc;
    reg [CAM_DESC_PTR_BITS-1:0] cam_desc_rd_ptr_calc;
    reg [5:0]                cam_desc_count_calc;

    reg pop_desc;
    reg push_req;
    reg can_push;

    always @(posedge cam_pclk or negedge cam_resetn) begin
        if (!cam_resetn) begin
            free_map                    <= 16'hFFFF;
            wr_buf_idx                  <= 4'd0;
            wr_addr                     <= 10'd0;
            cam_line_valid_d            <= 1'b0;
            cam_frame_toggle_d          <= 1'b0;
            frame_flag_next_line        <= 1'b0;
            cur_line_is_frame_start_cam <= 1'b0;
            drop_this_line              <= 1'b0;

            line_in_field               <= 10'd0;

            in_frame_id                 <= {CAM_DESC_FRAME_BITS{1'b0}};
            cur_line_y                  <= {CAM_DESC_Y_BITS{1'b0}};

            cam_desc_wr_ptr             <= {CAM_DESC_PTR_BITS{1'b0}};
            cam_desc_rd_ptr             <= {CAM_DESC_PTR_BITS{1'b0}};
            cam_desc_count              <= 6'd0;

            dbg_cam_fieldtog_cnt        <= 16'd0;
            dbg_cam_marker_inj_cnt      <= 16'd0;
            dbg_cam_desc_sent_cnt       <= 16'd0;

            dbg_free_cnt_cam            <= 4'd0;
            dbg_free_min_cam            <= 4'd15;
            dbg_free_max_cam            <= 4'd0;
            dbg_alloc_fail_cnt_cam      <= 16'd0;
            dbg_rel_doublefree_cnt_cam  <= 16'd0;

            dbg_cam_descq_cnt_cam       <= 6'd0;

        end else begin
            cam_line_valid_d   <= cam_line_valid;
            cam_frame_toggle_d <= cam_frame_toggle;

            // ---- init calc ----
            free_map_calc        = free_map;
            cam_desc_wr_ptr_calc = cam_desc_wr_ptr;
            cam_desc_rd_ptr_calc = cam_desc_rd_ptr;
            cam_desc_count_calc  = cam_desc_count;

            // release-ek mindig rákerülnek a free_map_calc-ra
            if (rel_new_cam) begin
                free_map_calc = free_map_calc | rel_mask_cam;
            end

            if (rel_new_cam && (rel_doublefree_bits != 16'h0000)) begin
                dbg_rel_doublefree_cnt_cam <= dbg_rel_doublefree_cnt_cam + 16'd1;
            end

            if (frame_edge) begin
                frame_flag_next_line <= 1'b1;
                dbg_cam_fieldtog_cnt <= dbg_cam_fieldtog_cnt + 16'd1;

                in_frame_id <= in_frame_id + {{(CAM_DESC_FRAME_BITS-1){1'b0}}, 1'b1};

                dbg_free_min_cam <= 4'd15;
                dbg_free_max_cam <= 4'd0;

                line_in_field <= 10'd0;
                cur_line_y    <= {CAM_DESC_Y_BITS{1'b0}};
            end

            if (line_start) begin
                line_in_field <= line_in_field_pre + 10'd1;

                cur_line_y <= line_in_field_pre[CAM_DESC_Y_BITS-1:0];

                wr_addr <= 10'd0;

                cur_line_is_frame_start_cam <= frame_flag_next_line | frame_edge;
                frame_flag_next_line        <= 1'b0;

                dbg_free_cnt_cam <= free_cnt_eff4;
                if (free_cnt_eff4 < dbg_free_min_cam) dbg_free_min_cam <= free_cnt_eff4;
                if (free_cnt_eff4 > dbg_free_max_cam) dbg_free_max_cam <= free_cnt_eff4;

                if (limit_drop_pre) begin
                    drop_this_line <= 1'b1;
                end else if (alloc_ok) begin
                    wr_buf_idx     <= alloc_idx;
                    drop_this_line <= 1'b0;

                    // alloc: foglalás a calc-ben
                    free_map_calc[alloc_idx] = 1'b0;
                end else begin
                    drop_this_line <= 1'b1;
                    dbg_alloc_fail_cnt_cam <= dbg_alloc_fail_cnt_cam + 16'd1;
                end
            end

            if (cam_line_valid && cam_y_valid && !drop_this_line) begin
                if (wr_addr != 10'h3FF) wr_addr <= wr_addr + 10'd1;
            end

            // ==================================================
            // PUSH/POP kezelés (pop -> push)
            // ==================================================
            pop_desc = desc_send_cam;
            push_req = line_end && !drop_this_line;

            // 1) POP előbb (slot felszabadul full esetben)
            if (pop_desc) begin
                cam_desc_rd_ptr_calc = (cam_desc_rd_ptr_calc + 1'b1) & CAM_DESC_MASK;
                cam_desc_count_calc  = cam_desc_count_calc - 1'b1;
            end

            // 2) PUSH utána
            if (line_end) begin
                if (push_req) begin
                    can_push = (cam_desc_count_calc < CAM_DESC_DEPTH);
                    if (can_push) begin
                        cam_desc_fifo[cam_desc_wr_ptr_calc] <= {cur_line_is_frame_start_cam, in_frame_id, cur_line_y, wr_buf_idx};
                        cam_desc_wr_ptr_calc                = (cam_desc_wr_ptr_calc + 1'b1) & CAM_DESC_MASK;
                        cam_desc_count_calc                 = cam_desc_count_calc + 1'b1;

                        dbg_cam_desc_sent_cnt <= dbg_cam_desc_sent_cnt + 16'd1;
                        if (cur_line_is_frame_start_cam)
                            dbg_cam_marker_inj_cnt <= dbg_cam_marker_inj_cnt + 16'd1;
                    end else begin
                        // nincs hely -> buffer vissza a poolba
                        free_map_calc = free_map_calc | onehot16(wr_buf_idx);
                    end
                end
                drop_this_line <= 1'b0;
            end

            // ---- commit (csak NB az állapot reg-ekre) ----
            free_map        <= free_map_calc;
            cam_desc_wr_ptr <= cam_desc_wr_ptr_calc;
            cam_desc_rd_ptr <= cam_desc_rd_ptr_calc;
            cam_desc_count  <= cam_desc_count_calc;

            dbg_cam_descq_cnt_cam <= cam_desc_count_calc;
        end
    end

    // ------------------------------------------------------------
    // 16 dual-clock line buffers
    // ------------------------------------------------------------
    wire [7:0] line_q0,  line_q1,  line_q2,  line_q3;
    wire [7:0] line_q4,  line_q5,  line_q6,  line_q7;
    wire [7:0] line_q8,  line_q9,  line_q10, line_q11;
    wire [7:0] line_q12, line_q13, line_q14, line_q15;

    wire wr_active = cam_line_valid && cam_y_valid && !drop_this_line && (wr_addr < H_ACTIVE);

    wire use_buf0  = wr_active && (wr_buf_idx == 4'd0);
    wire use_buf1  = wr_active && (wr_buf_idx == 4'd1);
    wire use_buf2  = wr_active && (wr_buf_idx == 4'd2);
    wire use_buf3  = wr_active && (wr_buf_idx == 4'd3);
    wire use_buf4  = wr_active && (wr_buf_idx == 4'd4);
    wire use_buf5  = wr_active && (wr_buf_idx == 4'd5);
    wire use_buf6  = wr_active && (wr_buf_idx == 4'd6);
    wire use_buf7  = wr_active && (wr_buf_idx == 4'd7);
    wire use_buf8  = wr_active && (wr_buf_idx == 4'd8);
    wire use_buf9  = wr_active && (wr_buf_idx == 4'd9);
    wire use_buf10 = wr_active && (wr_buf_idx == 4'd10);
    wire use_buf11 = wr_active && (wr_buf_idx == 4'd11);
    wire use_buf12 = wr_active && (wr_buf_idx == 4'd12);
    wire use_buf13 = wr_active && (wr_buf_idx == 4'd13);
    wire use_buf14 = wr_active && (wr_buf_idx == 4'd14);
    wire use_buf15 = wr_active && (wr_buf_idx == 4'd15);

    wire [9:0] rd_addr = (h_cnt < H_ACTIVE) ? h_cnt[9:0] : 10'd0;

    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf0  (.wr_clk(cam_pclk),.wr_en(use_buf0), .wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q0));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf1  (.wr_clk(cam_pclk),.wr_en(use_buf1), .wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q1));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf2  (.wr_clk(cam_pclk),.wr_en(use_buf2), .wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q2));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf3  (.wr_clk(cam_pclk),.wr_en(use_buf3), .wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q3));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf4  (.wr_clk(cam_pclk),.wr_en(use_buf4), .wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q4));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf5  (.wr_clk(cam_pclk),.wr_en(use_buf5), .wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q5));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf6  (.wr_clk(cam_pclk),.wr_en(use_buf6), .wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q6));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf7  (.wr_clk(cam_pclk),.wr_en(use_buf7), .wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q7));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf8  (.wr_clk(cam_pclk),.wr_en(use_buf8), .wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q8));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf9  (.wr_clk(cam_pclk),.wr_en(use_buf9), .wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q9));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf10 (.wr_clk(cam_pclk),.wr_en(use_buf10),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q10));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf11 (.wr_clk(cam_pclk),.wr_en(use_buf11),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q11));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf12 (.wr_clk(cam_pclk),.wr_en(use_buf12),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q12));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf13 (.wr_clk(cam_pclk),.wr_en(use_buf13),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q13));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf14 (.wr_clk(cam_pclk),.wr_en(use_buf14),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q14));
    line_buffer_dc #(.H_ACTIVE(H_ACTIVE)) u_buf15 (.wr_clk(cam_pclk),.wr_en(use_buf15),.wr_addr(wr_addr),.wr_data(cam_y),.rd_clk(pix_clk),.rd_addr(rd_addr),.rd_data(line_q15));

    // ============================================================
    // PIX DOMAIN: descriptor FIFO + bob + watermark correction
    // + FIX: underflow deadlock breaker (release current if starving CAM)
    // ============================================================
    localparam integer DESC_BUF_LSB    = 0;
    localparam integer DESC_BUF_MSB    = BUF_BITS-1;
    localparam integer DESC_Y_LSB      = BUF_BITS;
    localparam integer DESC_Y_MSB      = DESC_Y_LSB + CAM_DESC_Y_BITS - 1;
    localparam integer DESC_FRAME_LSB  = DESC_Y_LSB + CAM_DESC_Y_BITS;
    localparam integer DESC_FRAME_MSB  = DESC_FRAME_LSB + CAM_DESC_FRAME_BITS - 1;
    localparam integer DESC_MARKER_BIT = CAM_DESC_DATA_BITS-1;

    reg [CAM_DESC_DATA_BITS-1:0] desc_fifo [0:DESC_DEPTH-1];
    reg [DESC_BITS-1:0]          desc_wr_ptr;
    reg [DESC_BITS-1:0]          desc_rd_ptr;
    reg [5:0]                    desc_count;

    reg [DESC_BITS-1:0]  desc_wr_ptr_n;
    reg [DESC_BITS-1:0]  desc_rd_ptr_n;
    reg [5:0]            desc_count_n;

    reg                  have_any_line;
    reg                  have_any_line_n;

    reg de_d, vsync_d;
    reg repeat_phase, repeat_phase_n;

    reg [BUF_BITS-1:0] cur_buf_idx_r;
    reg [BUF_BITS-1:0] cur_buf_idx_r_n;
    reg                cur_buf_valid;
    reg                cur_buf_valid_n;

    reg [15:0] rel_accum;
    reg [15:0] rel_accum_n;
    reg        rel_pend_n;
    reg [15:0] rel_pend_data_n;

    // 16 bufferhez skálázott WM-ek (hysteresissel)
    localparam integer HIGH_WM    = (NUM_BUFS - 2);  // 14: drop threshold
    localparam integer LOW_WM     = (NUM_BUFS - 8);  // 8 : duplicate threshold
    localparam integer SAFE_START = V_ACTIVE - 128;

    wire safe_for_correction = (v_cnt >= SAFE_START);

    reg do_drop, do_drop_n;

    reg [7:0] dup_budget, dup_budget_n;

    reg [9:0] underflow_cnt, overflow_cnt;
    reg [2:0] drop_used, dup_used;
    reg       resync_used;
    reg [5:0] desc_min, desc_max;

    reg [15:0] dbg_last_drop_v,    dbg_last_drop_v_n;
    reg [15:0] dbg_last_dup_v,     dbg_last_dup_v_n;
    reg [15:0] dbg_last_resync_v,  dbg_last_resync_v_n;

    reg [15:0] dbg_last_drop_h,    dbg_last_drop_h_n;
    reg [15:0] dbg_last_dup_h,     dbg_last_dup_h_n;
    reg [15:0] dbg_last_resync_h,  dbg_last_resync_h_n;

    reg [9:0] underflow_cnt_n, overflow_cnt_n;
    reg [2:0] drop_used_n, dup_used_n;
    reg       resync_used_n;
    reg [5:0] desc_min_n, desc_max_n;

    // NEW: underflow streak (deadlock breaker)
    reg [3:0] uf_streak, uf_streak_n;

    reg       marker_found_pix;
    reg [4:0] marker_off_pix;   // 0..31
    integer mi;
    reg [DESC_BITS-1:0] midx;

    always @* begin
        marker_found_pix = 1'b0;
        marker_off_pix   = 5'd0;
        midx             = desc_rd_ptr;

        for (mi = 0; mi < DESC_DEPTH; mi = mi + 1) begin
            if (!marker_found_pix && (mi < desc_count)) begin
                midx = (desc_rd_ptr + mi[DESC_BITS-1:0]) & DESC_MASK;
                if (desc_fifo[midx][DESC_MARKER_BIT]) begin
                    marker_found_pix = 1'b1;
                    marker_off_pix   = mi[4:0];
                end
            end
        end
    end

    reg       seek_armed,  seek_active;
    reg [4:0] seek_rem;

    reg       seek_armed_n, seek_active_n;
    reg [4:0] seek_rem_n;

    localparam integer ST_RX_DUPBUF      = 0;
    localparam integer ST_REL_NOT_OWNED  = 1;
    localparam integer ST_REL_HITS_CUR   = 2;
    localparam integer ST_DESC_OVERFLOW  = 3;
    localparam integer ST_SEEK_EMPTY     = 4;

    reg [15:0] pix_own_map, pix_own_map_n;
    reg [15:0] pix_fault_sticky, pix_fault_sticky_n;

    reg [15:0] pix_rx_dupbuf_cnt,     pix_rx_dupbuf_cnt_n;
    reg [15:0] pix_rel_not_owned_cnt, pix_rel_not_owned_cnt_n;
    reg [7:0]  pix_overflow_rel_lo8,  pix_overflow_rel_lo8_n;

    reg [BUF_BITS-1:0] rx_idx;
    reg                rx_was_owned;

    reg [CAM_DESC_FRAME_BITS-1:0] out_frame_id_expected, out_frame_id_expected_n;
    reg                           need_frame_resync,     need_frame_resync_n;
    reg                           freeze_frame,          freeze_frame_n;

    localparam integer DBG_BUS_W = 224;

    reg [DBG_BUS_W-1:0] dbg_bus_pix = {DBG_BUS_W{1'b0}};
    reg                 dbg_tog_pix = 1'b0;

    always @(posedge pix_clk or negedge resetn) begin
        if (!resetn) begin
            desc_wr_ptr   <= {DESC_BITS{1'b0}};
            desc_rd_ptr   <= {DESC_BITS{1'b0}};
            desc_count    <= 6'd0;

            have_any_line <= 1'b0;
            de_d          <= 1'b0;
            vsync_d       <= 1'b1;
            vsync_toggle_pix <= 1'b0;
            repeat_phase  <= 1'b0;

            cur_buf_idx_r <= 4'd0;
            cur_buf_valid <= 1'b0;

            rel_accum     <= 16'h0000;
            rel_pend      <= 1'b0;
            rel_pend_data <= 16'h0000;

            do_drop       <= 1'b0;
            dup_budget    <= 8'd0;

            underflow_cnt <= 10'd0;
            overflow_cnt  <= 10'd0;
            drop_used     <= 3'd0;
            dup_used      <= 3'd0;
            resync_used   <= 1'b0;
            desc_min      <= 6'd0;
            desc_max      <= 6'd0;

            dbg_last_drop_v   <= 16'd0;
            dbg_last_dup_v    <= 16'd0;
            dbg_last_resync_v <= 16'd0;

            dbg_last_drop_h   <= 16'd0;
            dbg_last_dup_h    <= 16'd0;
            dbg_last_resync_h <= 16'd0;

            seek_armed    <= 1'b0;
            seek_active   <= 1'b0;
            seek_rem      <= 5'd0;

            dbg_bus_pix   <= {DBG_BUS_W{1'b0}};
            dbg_tog_pix   <= 1'b0;

            pix_own_map            <= 16'h0000;
            pix_fault_sticky       <= 16'h0000;
            pix_rx_dupbuf_cnt      <= 16'd0;
            pix_rel_not_owned_cnt  <= 16'd0;
            pix_overflow_rel_lo8   <= 8'd0;

            rx_idx       <= {BUF_BITS{1'b0}};
            rx_was_owned <= 1'b0;

            uf_streak    <= 4'd0;

            out_frame_id_expected <= {CAM_DESC_FRAME_BITS{1'b0}};
            need_frame_resync     <= 1'b0;
            freeze_frame          <= 1'b0;

        end else begin
            // defaults
            desc_wr_ptr_n    = desc_wr_ptr;
            desc_rd_ptr_n    = desc_rd_ptr;
            desc_count_n     = desc_count;

            have_any_line_n  = have_any_line;
            de_d             <= de;
            vsync_d          <= vsync;
            vsync_toggle_pix <= vsync_toggle_pix;
            repeat_phase_n   = repeat_phase;

            cur_buf_idx_r_n  = cur_buf_idx_r;
            cur_buf_valid_n  = cur_buf_valid;

            rel_accum_n      = rel_accum;
            rel_pend_n       = rel_pend;
            rel_pend_data_n  = rel_pend_data;

            do_drop_n        = do_drop;

            dup_budget_n     = dup_budget;

            underflow_cnt_n  = underflow_cnt;
            overflow_cnt_n   = overflow_cnt;
            drop_used_n      = drop_used;
            dup_used_n       = dup_used;
            resync_used_n    = resync_used;
            desc_min_n       = desc_min;
            desc_max_n       = desc_max;

            dbg_last_drop_v_n   = dbg_last_drop_v;
            dbg_last_dup_v_n    = dbg_last_dup_v;
            dbg_last_resync_v_n = dbg_last_resync_v;

            dbg_last_drop_h_n   = dbg_last_drop_h;
            dbg_last_dup_h_n    = dbg_last_dup_h;
            dbg_last_resync_h_n = dbg_last_resync_h;

            seek_armed_n     = seek_armed;
            seek_active_n    = seek_active;
            seek_rem_n       = seek_rem;

            pix_own_map_n            = pix_own_map;
            pix_fault_sticky_n       = pix_fault_sticky;
            pix_rx_dupbuf_cnt_n      = pix_rx_dupbuf_cnt;
            pix_rel_not_owned_cnt_n  = pix_rel_not_owned_cnt;
            pix_overflow_rel_lo8_n   = pix_overflow_rel_lo8;

            uf_streak_n              = uf_streak;

            out_frame_id_expected_n  = out_frame_id_expected;
            need_frame_resync_n      = need_frame_resync;
            freeze_frame_n           = freeze_frame;

            if (!vsync && vsync_d && vblank) begin
                do_drop_n      = (desc_count > HIGH_WM);
                repeat_phase_n = 1'b0;
                uf_streak_n    = 4'd0;

                vsync_toggle_pix <= ~vsync_toggle_pix;
            end

            if (desc_new_pix) begin
                have_any_line_n = 1'b1;

                rx_idx       = desc_data_pix[DESC_BUF_MSB:DESC_BUF_LSB];
                rx_was_owned = pix_own_map_n[rx_idx];

                if (rx_was_owned) begin
                    pix_fault_sticky_n[ST_RX_DUPBUF] = 1'b1;
                    pix_rx_dupbuf_cnt_n              = pix_rx_dupbuf_cnt_n + 16'd1;
                end

                pix_own_map_n = pix_own_map_n | onehot16(rx_idx);

                if (desc_count_n < DESC_DEPTH) begin
                    desc_fifo[desc_wr_ptr_n] = desc_data_pix;
                    desc_wr_ptr_n            = (desc_wr_ptr_n + 1'b1) & DESC_MASK;
                    desc_count_n             = desc_count_n + 1'b1;
                end else begin
                    overflow_cnt_n = overflow_cnt_n + 10'd1;
                    pix_fault_sticky_n[ST_DESC_OVERFLOW] = 1'b1;

                    rel_accum_n   = rel_accum_n | onehot16(rx_idx);
                    pix_own_map_n = pix_own_map_n & ~onehot16(rx_idx);

                    pix_overflow_rel_lo8_n = pix_overflow_rel_lo8_n + 8'd1;
                end
            end

            if ((h_cnt == 11'd0) && (v_cnt == V_ACTIVE)) begin
                seek_armed_n  = need_frame_resync_n;
                seek_active_n = 1'b0;
                seek_rem_n    = 5'd0;
            end

            if (frame_start) begin
                seek_armed_n  = 1'b0;
                seek_active_n = 1'b0;
                seek_rem_n    = 5'd0;
                uf_streak_n   = 4'd0;

                freeze_frame_n = 1'b0;

                if (desc_count_n != 0)
                    out_frame_id_expected_n = desc_fifo[desc_rd_ptr_n][DESC_FRAME_MSB:DESC_FRAME_LSB];
            end

            if (vblank && safe_for_correction && seek_armed_n && !seek_active_n) begin
                if (marker_found_pix) begin
                    if (marker_off_pix == 5'd0) begin
                        seek_armed_n = 1'b0;
                    end else begin
                        seek_active_n = 1'b1;
                        seek_rem_n    = marker_off_pix;
                    end
                end
            end

            if (vblank && safe_for_correction && seek_active_n) begin
                if ((seek_rem_n != 5'd0) && (desc_count_n != 0)) begin
                    rel_accum_n = rel_accum_n | onehot16(desc_fifo[desc_rd_ptr_n][DESC_BUF_MSB:DESC_BUF_LSB]);

                    if (!pix_own_map_n[desc_fifo[desc_rd_ptr_n][DESC_BUF_MSB:DESC_BUF_LSB]]) begin
                        pix_fault_sticky_n[ST_REL_NOT_OWNED] = 1'b1;
                        pix_rel_not_owned_cnt_n              = pix_rel_not_owned_cnt_n + 16'd1;
                    end
                    pix_own_map_n = pix_own_map_n & ~onehot16(desc_fifo[desc_rd_ptr_n][DESC_BUF_MSB:DESC_BUF_LSB]);

                    desc_rd_ptr_n = (desc_rd_ptr_n + 1'b1) & DESC_MASK;
                    desc_count_n  = desc_count_n - 1'b1;

                    if (dup_budget_n != 8'hFF)
                        dup_budget_n = dup_budget_n + 8'd1;

                    if (seek_rem_n == 5'd1) begin
                        seek_rem_n    = 5'd0;
                        seek_active_n = 1'b0;
                        seek_armed_n  = 1'b0;
                        resync_used_n = 1'b1;
                        dbg_last_resync_v_n = {6'd0, v_cnt};
                        dbg_last_resync_h_n = {5'd0, h_cnt};

                        need_frame_resync_n = 1'b0;
                    end else begin
                        seek_rem_n = seek_rem_n - 1'b1;
                    end
                end else begin
                    if (seek_rem_n != 5'd0) begin
                        pix_fault_sticky_n[ST_SEEK_EMPTY] = 1'b1;
                    end
                    seek_active_n = 1'b0;
                    seek_armed_n  = 1'b0;
                    seek_rem_n    = 5'd0;
                end
            end

            if (de && !de_d && (v_cnt < V_ACTIVE) && !freeze_frame_n) begin
                if (!repeat_phase_n) begin
                    if (desc_count_n != 0) begin
                        uf_streak_n = 4'd0;

                        if (desc_fifo[desc_rd_ptr_n][DESC_FRAME_MSB:DESC_FRAME_LSB] != out_frame_id_expected_n) begin
                            freeze_frame_n      = 1'b1;
                            need_frame_resync_n = 1'b1;
                        end else begin
                            if (cur_buf_valid_n) begin
                                rel_accum_n = rel_accum_n | onehot16(cur_buf_idx_r_n);

                                if (!pix_own_map_n[cur_buf_idx_r_n]) begin
                                    pix_fault_sticky_n[ST_REL_NOT_OWNED] = 1'b1;
                                    pix_rel_not_owned_cnt_n              = pix_rel_not_owned_cnt_n + 16'd1;
                                end
                                pix_own_map_n = pix_own_map_n & ~onehot16(cur_buf_idx_r_n);
                            end

                            cur_buf_idx_r_n = desc_fifo[desc_rd_ptr_n][DESC_BUF_MSB:DESC_BUF_LSB];
                            cur_buf_valid_n = 1'b1;

                            if (desc_fifo[desc_rd_ptr_n][DESC_MARKER_BIT]) begin
                                cur_buf_idx_r_n = desc_fifo[desc_rd_ptr_n][BUF_BITS-1:0];
                                cur_buf_valid_n = 1'b1;
                            end

                            if (desc_fifo[desc_rd_ptr_n][BUF_BITS])
                                repeat_phase_n = 1'b0;

                            desc_rd_ptr_n = (desc_rd_ptr_n + 1'b1) & DESC_MASK;
                            desc_count_n  = desc_count_n - 1'b1;
                        end
                    end else begin
                        // UNDERFLOW
                        underflow_cnt_n = underflow_cnt_n + 10'd1;
                        if (uf_streak_n != 4'hF) uf_streak_n = uf_streak_n + 4'd1;

                        // DEADLOCK BREAKER (16 buf-hoz skálázva)
                        if (cur_buf_valid_n &&
                           ((uf_streak_n >= 4'd2) || (popcount16(pix_own_map_n) >= 5'd14))) begin

                            rel_accum_n = rel_accum_n | onehot16(cur_buf_idx_r_n);

                            if (!pix_own_map_n[cur_buf_idx_r_n]) begin
                                pix_fault_sticky_n[ST_REL_NOT_OWNED] = 1'b1;
                                pix_rel_not_owned_cnt_n              = pix_rel_not_owned_cnt_n + 16'd1;
                            end
                            pix_own_map_n   = pix_own_map_n & ~onehot16(cur_buf_idx_r_n);
                            cur_buf_valid_n = 1'b0;
                        end
                    end
                end

                repeat_phase_n = ~repeat_phase_n;

                if (desc_count_n < desc_min_n) desc_min_n = desc_count_n;
                if (desc_count_n > desc_max_n) desc_max_n = desc_count_n;

                if (cur_buf_valid_n && (rel_accum_n & onehot16(cur_buf_idx_r_n))) begin
                    pix_fault_sticky_n[ST_REL_HITS_CUR] = 1'b1;
                end
            end

            // Drift correction: only adjust descriptor depth during HDMI VBLANK
            if (line_start_any && vblank && safe_for_correction) begin
                if (desc_count_n < LOW_WM) begin
                    if (dup_budget_n != 8'd0)
                        dup_budget_n = dup_budget_n - 8'd1;

                    if (dup_used_n != 3'd7) dup_used_n = dup_used_n + 3'd1;
                    dbg_last_dup_v_n = {6'd0, v_cnt};
                    dbg_last_dup_h_n = {5'd0, h_cnt};
                    uf_streak_n = 4'd0;
                end else if ((desc_count_n != 0) && (do_drop_n || (desc_count_n > HIGH_WM))) begin
                    rel_accum_n = rel_accum_n | onehot16(desc_fifo[desc_rd_ptr_n][DESC_BUF_MSB:DESC_BUF_LSB]);

                    if (!pix_own_map_n[desc_fifo[desc_rd_ptr_n][DESC_BUF_MSB:DESC_BUF_LSB]]) begin
                        pix_fault_sticky_n[ST_REL_NOT_OWNED] = 1'b1;
                        pix_rel_not_owned_cnt_n              = pix_rel_not_owned_cnt_n + 16'd1;
                    end
                    pix_own_map_n = pix_own_map_n & ~onehot16(desc_fifo[desc_rd_ptr_n][DESC_BUF_MSB:DESC_BUF_LSB]);
                    rel_accum_n = rel_accum_n | onehot16(desc_fifo[desc_rd_ptr_n][BUF_BITS-1:0]);

                    if (!pix_own_map_n[desc_fifo[desc_rd_ptr_n][BUF_BITS-1:0]]) begin
                        pix_fault_sticky_n[ST_REL_NOT_OWNED] = 1'b1;
                        pix_rel_not_owned_cnt_n              = pix_rel_not_owned_cnt_n + 16'd1;
                    end
                    pix_own_map_n = pix_own_map_n & ~onehot16(desc_fifo[desc_rd_ptr_n][BUF_BITS-1:0]);

                    desc_rd_ptr_n = (desc_rd_ptr_n + 1'b1) & DESC_MASK;
                    desc_count_n  = desc_count_n - 1'b1;
                    do_drop_n     = 1'b0;

                    if (dup_budget_n != 8'hFF)
                        dup_budget_n = dup_budget_n + 8'd1;

                    if (drop_used_n != 3'd7) drop_used_n = drop_used_n + 3'd1;
                    dbg_last_drop_v_n = {6'd0, v_cnt};
                    dbg_last_drop_h_n = {5'd0, h_cnt};
                    uf_streak_n = 4'd0;
                end

                if (desc_count_n < desc_min_n) desc_min_n = desc_count_n;
                if (desc_count_n > desc_max_n) desc_max_n = desc_count_n;
            end

            if (!rel_pend_n && (rel_accum_n != 16'h0000)) begin
                rel_pend_data_n = rel_accum_n;
                rel_pend_n      = 1'b1;
                rel_accum_n     = 16'h0000;
            end

            if (rel_pend && !rel_busy_pix) begin
                rel_pend_n = 1'b0;
            end

            if (frame_start) begin
                // bus formatot NEM törjük: own_map low8, marker_off low4, desc_count low5
                dbg_bus_pix <= {
                    dbg_last_drop_v_n,           // [223:208]
                    dbg_last_dup_v_n,            // [207:192]
                    dbg_last_resync_v_n,         // [191:176]
                    dbg_last_drop_h_n,           // [175:160]
                    dbg_last_dup_h_n,            // [159:144]
                    dbg_last_resync_h_n,         // [143:128]

                    pix_fault_sticky_n,          // [127:112]
                    pix_own_map_n[7:0],          // [111:104]
                    pix_rx_dupbuf_cnt_n,         // [103:88]
                    pix_rel_not_owned_cnt_n,     // [87:72]
                    pix_overflow_rel_lo8_n,      // [71:64]

                    8'd0,                        // [63:56]
                    1'b0,                        // [55]
                    8'd0,                        // [54:47]
                    marker_found_pix,            // [46]
                    marker_off_pix[3:0],         // [45:42] low4
                    desc_max_n[4:0],             // [41:37] low5
                    desc_min_n[4:0],             // [36:32] low5
                    resync_used_n,               // [31]
                    dup_used_n,                  // [30:28]
                    drop_used_n,                 // [27:25]
                    overflow_cnt_n,              // [24:15]
                    underflow_cnt_n,             // [14:5]
                    desc_count_n[4:0]            // [4:0]
                };
                dbg_tog_pix <= ~dbg_tog_pix;

                underflow_cnt_n = 10'd0;
                overflow_cnt_n  = 10'd0;
                drop_used_n     = 3'd0;
                dup_used_n      = 3'd0;
                resync_used_n   = 1'b0;

                desc_min_n      = desc_count_n;
                desc_max_n      = desc_count_n;

                uf_streak_n     = 4'd0;
            end

            desc_wr_ptr   <= desc_wr_ptr_n;
            desc_rd_ptr   <= desc_rd_ptr_n;
            desc_count    <= desc_count_n;

            have_any_line <= have_any_line_n;
            repeat_phase  <= repeat_phase_n;

            cur_buf_idx_r <= cur_buf_idx_r_n;
            cur_buf_valid <= cur_buf_valid_n;

            rel_accum     <= rel_accum_n;
            rel_pend      <= rel_pend_n;
            rel_pend_data <= rel_pend_data_n;

            do_drop       <= do_drop_n;
            dup_budget    <= dup_budget_n;

            underflow_cnt <= underflow_cnt_n;
            overflow_cnt  <= overflow_cnt_n;
            drop_used     <= drop_used_n;
            dup_used      <= dup_used_n;
            resync_used   <= resync_used_n;
            desc_min      <= desc_min_n;
            desc_max      <= desc_max_n;

            seek_armed    <= seek_armed_n;
            seek_active   <= seek_active_n;
            seek_rem      <= seek_rem_n;

            pix_own_map           <= pix_own_map_n;
            pix_fault_sticky      <= pix_fault_sticky_n;
            pix_rx_dupbuf_cnt     <= pix_rx_dupbuf_cnt_n;
            pix_rel_not_owned_cnt <= pix_rel_not_owned_cnt_n;
            pix_overflow_rel_lo8  <= pix_overflow_rel_lo8_n;

            uf_streak             <= uf_streak_n;

            out_frame_id_expected <= out_frame_id_expected_n;
            need_frame_resync     <= need_frame_resync_n;
            freeze_frame          <= freeze_frame_n;

            dbg_last_drop_v   <= dbg_last_drop_v_n;
            dbg_last_dup_v    <= dbg_last_dup_v_n;
            dbg_last_resync_v <= dbg_last_resync_v_n;

            dbg_last_drop_h   <= dbg_last_drop_h_n;
            dbg_last_dup_h    <= dbg_last_dup_h_n;
            dbg_last_resync_h <= dbg_last_resync_h_n;
        end
    end

    // ------------------------------------------------------------
    // DEBUG CDC: pix -> cam snapshot bus (128-bit)
    // ------------------------------------------------------------
    reg [2:0]   dbg_tsync = 3'b000;
    reg [DBG_BUS_W-1:0] dbg_bus_sync1 = {DBG_BUS_W{1'b0}};
    reg [DBG_BUS_W-1:0] dbg_bus_sync2 = {DBG_BUS_W{1'b0}};
    wire dbg_new = dbg_tsync[2] ^ dbg_tsync[1];

    always @(posedge cam_pclk or negedge cam_resetn) begin
        if (!cam_resetn) begin
            dbg_tsync <= 3'b000;
            dbg_bus_sync1 <= {DBG_BUS_W{1'b0}};
            dbg_bus_sync2 <= {DBG_BUS_W{1'b0}};

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

            dbg_fault_sticky_cam      <= 16'd0;
            dbg_own_map_cam           <= 8'd0;
            dbg_rx_dupbuf_cnt_cam     <= 16'd0;
            dbg_rel_not_owned_cnt_cam <= 16'd0;
            dbg_overflow_rel_lo8_cam  <= 8'd0;

            dbg_last_drop_v_cam     <= 16'd0;
            dbg_last_dup_v_cam      <= 16'd0;
            dbg_last_resync_v_cam   <= 16'd0;
            dbg_last_drop_h_cam     <= 16'd0;
            dbg_last_dup_h_cam      <= 16'd0;
            dbg_last_resync_h_cam   <= 16'd0;

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

                dbg_fault_sticky_cam      <= dbg_bus_sync2[127:112];
                dbg_own_map_cam           <= dbg_bus_sync2[111:104];
                dbg_rx_dupbuf_cnt_cam     <= dbg_bus_sync2[103:88];
                dbg_rel_not_owned_cnt_cam <= dbg_bus_sync2[87:72];
                dbg_overflow_rel_lo8_cam  <= dbg_bus_sync2[71:64];

                dbg_last_drop_v_cam     <= dbg_bus_sync2[223:208];
                dbg_last_dup_v_cam      <= dbg_bus_sync2[207:192];
                dbg_last_resync_v_cam   <= dbg_bus_sync2[191:176];
                dbg_last_drop_h_cam     <= dbg_bus_sync2[175:160];
                dbg_last_dup_h_cam      <= dbg_bus_sync2[159:144];
                dbg_last_resync_h_cam   <= dbg_bus_sync2[143:128];
            end
        end
    end

    // ------------------------------------------------------------
    // Read selected line buffer
    // ------------------------------------------------------------
    reg [7:0] cam_y_sample;
    always @* begin
        case (cur_buf_idx_r)
            4'd0:  cam_y_sample = line_q0;
            4'd1:  cam_y_sample = line_q1;
            4'd2:  cam_y_sample = line_q2;
            4'd3:  cam_y_sample = line_q3;
            4'd4:  cam_y_sample = line_q4;
            4'd5:  cam_y_sample = line_q5;
            4'd6:  cam_y_sample = line_q6;
            4'd7:  cam_y_sample = line_q7;
            4'd8:  cam_y_sample = line_q8;
            4'd9:  cam_y_sample = line_q9;
            4'd10: cam_y_sample = line_q10;
            4'd11: cam_y_sample = line_q11;
            4'd12: cam_y_sample = line_q12;
            4'd13: cam_y_sample = line_q13;
            4'd14: cam_y_sample = line_q14;
            4'd15: cam_y_sample = line_q15;
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
