// top.v - Tang Nano 9K
`timescale 1ns / 1ps
`default_nettype none

module top(
    input  wire        clk,
    input  wire        resetn,

    input  wire        cam1_pclk,
    input  wire [7:0]  cam1_d,

    inout  wire        tvp_sda,
    inout  wire        tvp_scl,

    output wire        tmds_clk_n,
    output wire        tmds_clk_p,
    output wire [2:0]  tmds_d_n,
    output wire [2:0]  tmds_d_p,

    output wire        dbg_i2c_scl,
    inout  wire        dbg_i2c_sda
);

    wire tvp_init_done;
    tvp5150_i2c_init u_tvp_i2c (
        .clk       (clk),
        .resetn    (resetn),
        .tvp_sda   (tvp_sda),
        .tvp_scl   (tvp_scl),
        .init_done (tvp_init_done)
    );

    wire clk_p5;
    wire pll_lock;

    Gowin_rPLL u_pll (
        .clkin  (clk),
        .clkout (clk_p5),
        .lock   (pll_lock)
    );

    wire pix_clk;
    Gowin_CLKDIV u_div_5 (
        .clkout (pix_clk),
        .hclkin (clk_p5),
        .resetn (pll_lock)
    );

    wire sys_resetn;
    Reset_Sync u_Reset_Sync_pix (
        .clk       (pix_clk),
        .ext_reset (resetn & pll_lock),
        .resetn    (sys_resetn)
    );

    wire cam_resetn_raw = resetn & tvp_init_done;
    wire cam_resetn;

    Reset_Sync u_Reset_Sync_cam (
        .clk       (cam1_pclk),
        .ext_reset (cam_resetn_raw),
        .resetn    (cam_resetn)
    );

    wire        cam_frame_valid;
    wire        cam_line_valid;
    wire [7:0]  cam_y;
    wire        cam_y_valid;
    wire        cam_frame_start;
    wire        cam_field_toggle;

    tvp5150_capture u_cam_cap (
        .pclk         (cam1_pclk),
        .resetn       (cam_resetn),
        .d            (cam1_d),

        .frame_valid  (cam_frame_valid),
        .line_valid   (cam_line_valid),
        .y_out        (cam_y),
        .y_valid      (cam_y_valid),

        .frame_start  (cam_frame_start),
        .field_toggle (cam_field_toggle)
    );

    wire [9:0] raw_lines_last;
    wire       raw_new;

    cam_field_line_counter u_cnt_raw (
        .cam_pclk             (cam1_pclk),
        .cam_resetn           (cam_resetn),
        .cam_line_valid       (cam_line_valid),
        .cam_field_toggle     (cam_field_toggle),
        .lines_per_field_last (raw_lines_last),
        .new_field_ready      (raw_new)
    );

    // --- existing debug ---
    wire [4:0] dbg_desc_count_cam;
    wire [9:0] dbg_underflow_cam;
    wire [9:0] dbg_overflow_cam;

    wire [2:0] dbg_drop_used_cam;
    wire [2:0] dbg_dup_used_cam;
    wire       dbg_resync_used_cam;

    wire [4:0] dbg_desc_min_cam;
    wire [4:0] dbg_desc_max_cam;

    wire [3:0] dbg_marker_off_cam;
    wire       dbg_marker_found_cam;

    wire [15:0] dbg_cam_fieldtog_cnt_cam;
    wire [15:0] dbg_cam_marker_inj_cnt_cam;
    wire [15:0] dbg_cam_desc_sent_cnt_cam;

    // --- NEW debug wires ---
    wire [15:0] dbg_fault_sticky_cam;
    wire [7:0]  dbg_own_map_cam;
    wire [15:0] dbg_rx_dupbuf_cnt_cam;
    wire [15:0] dbg_rel_not_owned_cnt_cam;
    wire [7:0]  dbg_overflow_rel_lo8_cam;
    wire [3:0]  dbg_free_cnt_cam;
    wire [3:0]  dbg_free_min_cam;
    wire [3:0]  dbg_free_max_cam;
    wire [15:0] dbg_alloc_fail_cnt_cam;
    wire [15:0] dbg_rel_doublefree_cnt_cam;

    wire [5:0] dbg_cam_descq_cnt_cam;

    hdmi_480p_core u_hdmi (
        .pix_clk          (pix_clk),
        .pix_clk_5x       (clk_p5),
        .resetn           (sys_resetn),

        .cam_pclk         (cam1_pclk),
        .cam_resetn       (cam_resetn),
        .cam_line_valid   (cam_line_valid),
        .cam_y_valid      (cam_y_valid),
        .cam_y            (cam_y),
        .cam_frame_toggle (cam_field_toggle),

        .dbg_desc_count_cam      (dbg_desc_count_cam),
        .dbg_underflow_low10_cam (dbg_underflow_cam),
        .dbg_overflow_low10_cam  (dbg_overflow_cam),
        .dbg_drop_used_cam       (dbg_drop_used_cam),
        .dbg_dup_used_cam        (dbg_dup_used_cam),
        .dbg_resync_used_cam     (dbg_resync_used_cam),
        .dbg_desc_min_cam        (dbg_desc_min_cam),
        .dbg_desc_max_cam        (dbg_desc_max_cam),
        .dbg_marker_off_cam      (dbg_marker_off_cam),
        .dbg_marker_found_cam    (dbg_marker_found_cam),

        .dbg_cam_fieldtog_cnt    (dbg_cam_fieldtog_cnt_cam),
        .dbg_cam_marker_inj_cnt  (dbg_cam_marker_inj_cnt_cam),
        .dbg_cam_desc_sent_cnt   (dbg_cam_desc_sent_cnt_cam),

        .dbg_fault_sticky_cam       (dbg_fault_sticky_cam),
        .dbg_own_map_cam            (dbg_own_map_cam),
        .dbg_rx_dupbuf_cnt_cam      (dbg_rx_dupbuf_cnt_cam),
        .dbg_rel_not_owned_cnt_cam  (dbg_rel_not_owned_cnt_cam),
        .dbg_overflow_rel_lo8_cam   (dbg_overflow_rel_lo8_cam),
        .dbg_free_cnt_cam           (dbg_free_cnt_cam),
        .dbg_free_min_cam           (dbg_free_min_cam),
        .dbg_free_max_cam           (dbg_free_max_cam),
        .dbg_alloc_fail_cnt_cam     (dbg_alloc_fail_cnt_cam),
        .dbg_rel_doublefree_cnt_cam (dbg_rel_doublefree_cnt_cam),

        .dbg_cam_descq_cnt_cam      (dbg_cam_descq_cnt_cam),

        .tmds_clk_p        (tmds_clk_p),
        .tmds_clk_n        (tmds_clk_n),
        .tmds_d_p          (tmds_d_p),
        .tmds_d_n          (tmds_d_n)
    );

    // field-toggle period measured in pix_clk cycles -> cam domain
    wire [31:0] cam_field_period_pix;
    wire        cam_field_period_new;

    toggle_period_pix_to_cam #(.CNT_W(32)) u_camfield_period (
        .pix_clk        (pix_clk),
        .pix_resetn     (sys_resetn),

        .cam_clk        (cam1_pclk),
        .cam_resetn     (cam_resetn),

        .cam_toggle     (cam_field_toggle),

        .period_pix_cam (cam_field_period_pix),
        .period_new_cam (cam_field_period_new)
    );

    wire [15:0] cam_field_period_lo = cam_field_period_pix[15:0];
    wire [15:0] cam_field_period_hi = cam_field_period_pix[31:16];

    // Pager -> I2C logger
    wire        log_new;
    wire [7:0]  log_page;
    wire [15:0] log_value;
    wire        i2c_busy;

    i2c_diag_pager #(
        .CLK_HZ  (27000000),
        .PERIODS (10),
        .PAGES   (34)  // 0..33
    ) u_pager (
        .clk    (cam1_pclk),
        .resetn (cam_resetn),

        .i2c_busy (i2c_busy),

        .raw_lines_per_field (raw_lines_last),

        .dbg_desc_count      (dbg_desc_count_cam),
        .dbg_underflow_low10 (dbg_underflow_cam),
        .dbg_overflow_low10  (dbg_overflow_cam),

        .dbg_drop_used       (dbg_drop_used_cam),
        .dbg_dup_used        (dbg_dup_used_cam),
        .dbg_resync_used     (dbg_resync_used_cam),

        .dbg_desc_min        (dbg_desc_min_cam),
        .dbg_desc_max        (dbg_desc_max_cam),

        .dbg_marker_off      (dbg_marker_off_cam),
        .dbg_marker_found    (dbg_marker_found_cam),

        .cam_field_period_lo (cam_field_period_lo),
        .cam_field_period_hi (cam_field_period_hi),

        .dbg_cam_fieldtog_cnt   (dbg_cam_fieldtog_cnt_cam),
        .dbg_cam_marker_inj_cnt (dbg_cam_marker_inj_cnt_cam),
        .dbg_cam_desc_sent_cnt  (dbg_cam_desc_sent_cnt_cam),

        .dbg_fault_sticky        (dbg_fault_sticky_cam),
        .dbg_own_map             (dbg_own_map_cam),
        .dbg_rx_dupbuf_cnt       (dbg_rx_dupbuf_cnt_cam),
        .dbg_rel_not_owned_cnt   (dbg_rel_not_owned_cnt_cam),
        .dbg_overflow_rel_lo8    (dbg_overflow_rel_lo8_cam),

        .dbg_free_cnt            (dbg_free_cnt_cam),
        .dbg_free_min            (dbg_free_min_cam),
        .dbg_free_max            (dbg_free_max_cam),
        .dbg_alloc_fail_cnt      (dbg_alloc_fail_cnt_cam),
        .dbg_rel_doublefree_cnt  (dbg_rel_doublefree_cnt_cam),

        .dbg_cam_descq_cnt_cam   (dbg_cam_descq_cnt_cam),

        .new_sample (log_new),
        .out_page   (log_page),
        .out_value  (log_value)
    );

    i2c_frame_logger_arduino #(
        .I2C_ADDR      (7'h5D),
        .CLKS_PER_HALF (2700)
    ) u_logger (
        .clk    (cam1_pclk),
        .resetn (cam_resetn),

        .new_sample (log_new),
        .page       (log_page),
        .value      (log_value),

        .sda (dbg_i2c_sda),
        .scl (dbg_i2c_scl),

        .busy (i2c_busy)
    );

endmodule

module Reset_Sync (
    input  wire clk,
    input  wire ext_reset,
    output wire resetn
);
    reg [3:0] reset_cnt = 4'b0000;

    always @(posedge clk or negedge ext_reset) begin
        if (!ext_reset)
            reset_cnt <= 4'b0000;
        else
            reset_cnt <= reset_cnt + !resetn;
    end

    assign resetn = &reset_cnt;
endmodule

`default_nettype wire
