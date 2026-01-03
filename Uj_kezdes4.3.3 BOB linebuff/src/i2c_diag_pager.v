`timescale 1ns / 1ps
`default_nettype none
/*
0 raw_lines_per_field – nyers sorok/field (cam_field_line_counter mérése);
   tipikus 0xF2/0xF3 = 242/243 sor/field minták (resync előtti/utáni hossz)
1 dbg_desc_count – PIX oldali desc FIFO pillanatnyi töltöttség (0..16)
2 dbg_underflow_low10 – underflow események száma (nincs descriptor amikor kéne sor)
3 dbg_overflow_low10 – overflow események száma (tele a PIX FIFO, eldob descriptor)
4 dbg_drop_used – drop beavatkozások száma (SAFE zónában descriptor eldobás)
5 dbg_dup_used – dup beavatkozások száma (SAFE zónában sor ismétlés)
6 dbg_resync_used – resync (marker-seek) történt-e az előző HDMI frame-ben (flag)
7 dbg_desc_min – PIX desc FIFO minimum töltöttség az előző HDMI frame alatt
8 dbg_desc_max – PIX desc FIFO maximum töltöttség az előző HDMI frame alatt
9 dbg_marker_found + dbg_marker_off – talált-e frame-start markert a FIFO-ban, és hányadik elemre van (offset)
10 cam_field_period_lo – field period mérés alsó 16 bit (pix_clk ciklusokban)
11 cam_field_period_hi – field period mérés felső 16 bit
12 dbg_cam_fieldtog_cnt – field-toggle élek száma (kamera oldal)
13 dbg_cam_marker_inj_cnt – beadott frame-start markerek száma (kamera oldal)
14 dbg_cam_desc_sent_cnt – CAM oldalon előállított/elküldött deskriptorok száma
15 lines_per_field_est – becsült sor/field = (desc_sent_delta / fieldtog_delta)
16 dbg_fault_sticky – sticky fault bitek (RX_DUPBUF / REL_NOT_OWNED / REL_HITS_CUR / DESC_OVERFLOW / SEEK_EMPTY)
17 dbg_own_map – PIX oldali buffer ownership bitmap (melyik 0..7 buf “owned”)
18 dbg_cam_desc_push_cnt – hányszor jött át CAM→PIX descriptor (CDC WR)
19 dbg_pix_desc_pop_cnt – hányszor popolt a PIX descriptor FIFO
20 dbg_overflow_rel_lo8 – overflow miatti azonnali release-ek számlálója (low8)
21 dbg_free_cnt – CAM free buffer darabszám (0..8) line_start pillanatban
22 dbg_free_min – free_cnt minimum az adott fieldben
23 dbg_free_max – free_cnt maximum az adott fieldben
24 dbg_alloc_fail_cnt – CAM oldali alloc fail-ek száma (elfogyott a free buf)
25 dbg_rel_doublefree_cnt – double-free gyanú: olyan release jött, ami már free volt
26 desc_delta_lat – előjeles delta (latched): desc töltöttség változás field-enként
27 free_delta_lat – előjeles delta (latched): free_cnt változás field-enként
28 uf_delta_lat – underflow delta (field-enkénti növekmény)
29 of_delta_lat – overflow delta (field-enkénti növekmény)
30 drop_accum – drop események összegzett száma (akkumulált)
31 dup_accum – dup események összegzett száma (akkumulált)
32 resync_accum – resync események összegzett száma (akkumulált)
33 dbg_cam_descq_cnt_cam – CAM→PIX descriptor queue töltöttség (0..31/32)
34 last_drop_v – utolsó drop esemény v_cnt értéke
35 last_dup_v – utolsó dup esemény v_cnt értéke
36 in_sof_cnt16 – input start-of-field számláló LSB16
37 out_vsync_cnt16 – HDMI vsync számláló LSB16
38 sof_delta16 = in_sof_cnt16 - out_vsync_cnt16 (mod 16 bit)
39 lock_status16 – bitmező (bit0=marker_found/locked, bit1=lock_lost_pulse_latched, bit2=resync_event_latched, bit3=frame_miss_event_latched)
40 soft_drop_lines_cnt – VBLANK-ban végrehajtott soft sor-eldobások száma
41 soft_dup_lines_cnt – VBLANK-ban végrehajtott soft sor-duplázások száma
42 hard_resync_cnt – hard resync események száma (reset utáni első lockra limitált)
43 last_soft_corr_v – utolsó soft korrekció v_cnt értéke
44 corr_skip_marker_cnt – marker miatt kihagyott soft-drop próbálkozások száma
45 dbg_align_pop_total – marker align miatt eldobott descriptorok összesítve
46 dbg_align_hit_cnt – hány frame-ben futott align korrekció
47 dbg_marker_miss_cnt – marker_found hiányzások száma frame elején
48 dbg_marker_off_snapshot – utolsó marker offset snapshot
49 desc_count_now – aktuális descriptor FIFO töltöttség (PIX domain, 16 bit)
50 desc_err_now – előjeles eltérés a targethez képest (PIX domain)
51 marker_distance – hány descriptor a következő markerig (0xFFFF = nincs a scanben)
52 last_resync_reason – 0=none,1=marker_align,2=frameid_mismatch,3=under,4=over
53 last_soft_corr_v – utolsó soft dup/drop v_cnt (ismételt exportra)
54 corr_pending_flags – bit0=dup_pending, bit1=drop_pending, bit2=align_pending, bit3=has_corrected_this_frame
55 rel_sent_cnt_pix – hány release ment át PIX→CAM CDC-n (összesítve)
56 rel_rx_cnt_cam – hány release-t fogadott CAM oldal
57 pop_lines_cnt – hány aktív sorhoz fogyasztottunk deskriptort az utolsó HDMI frame-ben
58 hold_lines_cnt – hány aktív sor készült cached/dup/hold útvonalról pop nélkül
59 hold_stuck_abort_cnt – hány alkalommal kellett hold/dup állapotot megszakítani
60 cam_marker_drop_or_defer_cnt – marker beadás halasztások/kényszerített eldobások száma
61 cam_block_idx – aktuális CAM blokk index (8 soros blokkok a fielden belül)
62 blocks_per_field_target – cél blokk darabszám egy fielden belül
63 cam_stopped_early_cnt – hányszor ért véget a field a targetnél kevesebb blokkal
64 hdmi_frame_repeat_cnt – hány HDMI frame-et kellett ismételni marker hiány miatt
65 fill_lines_cnt – hány HDMI sort töltöttünk ki field_exhausted kitartással
66 blocks_left_snapshot – actív régió végén hány blokk maradt (snapshot)
67 cam_blocks_per_field_last – legutóbbi field alatt CAM hány blokkot küldött
68 marker_at_head – frame_start pillanatban volt-e marker a FIFO elején
69 field_start_ok_cnt – hány frame tudott markerrel indulni
*/
module i2c_diag_pager #(
    parameter integer CLK_HZ  = 27000000,
    parameter integer PERIODS = 4,
    parameter integer PAGES   = 70   // 0..69
)(
    input  wire       clk,
    input  wire       resetn,

    input  wire       i2c_busy,

    input  wire [9:0] raw_lines_per_field,

    input  wire [4:0] dbg_desc_count,
    input  wire [9:0] dbg_underflow_low10,
    input  wire [9:0] dbg_overflow_low10,

    input  wire [2:0] dbg_drop_used,
    input  wire [2:0] dbg_dup_used,
    input  wire       dbg_resync_used,

    input  wire [4:0] dbg_desc_min,
    input  wire [4:0] dbg_desc_max,

    input  wire [4:0] dbg_marker_off,
    input  wire       dbg_marker_found,

    input  wire [15:0] dbg_align_pop_total,
    input  wire [15:0] dbg_align_hit_cnt,
    input  wire [15:0] dbg_marker_miss_cnt,
    input  wire [7:0]  dbg_marker_off_snapshot,

    input  wire [15:0] cam_field_period_lo,
    input  wire [15:0] cam_field_period_hi,

    input  wire [15:0] dbg_cam_fieldtog_cnt,
    input  wire [15:0] dbg_cam_marker_inj_cnt,
    input  wire [15:0] dbg_cam_desc_sent_cnt,
    input  wire [15:0] de_line_cnt,

    // pages 16..25
    input  wire [15:0] dbg_fault_sticky,       // page16
    input  wire [7:0]  dbg_own_map,            // page17 (low8)
    input  wire [15:0] dbg_cam_desc_push_cnt,  // page18
    input  wire [15:0] dbg_pix_desc_pop_cnt,   // page19
    input  wire [7:0]  dbg_overflow_rel_lo8,   // page20 (low8)

    input  wire [3:0]  dbg_free_cnt,           // page21 (low4)
    input  wire [3:0]  dbg_free_min,           // page22 (low4)
    input  wire [3:0]  dbg_free_max,           // page23 (low4)
    input  wire [15:0] dbg_alloc_fail_cnt,     // page24
    input  wire [15:0] dbg_rel_doublefree_cnt, // page25
    input  wire [15:0] rel_sent_cnt_pix,       // page55
    input  wire [15:0] rel_rx_cnt_cam,         // page56

    input  wire [5:0]  dbg_cam_descq_cnt_cam,  // page33
    input  wire [5:0]  dbg_cam_block_idx_cam,  // page59
    input  wire [5:0]  dbg_blocks_per_field_target_cam, // page60
    input  wire [15:0] dbg_cam_stopped_early_cnt_cam,   // page61

    input  wire [15:0] dbg_last_drop_v,        // page34
    input  wire [15:0] dbg_last_dup_v,         // page35
    input  wire [15:0] in_sof_cnt16,           // page36
    input  wire [15:0] out_vsync_cnt16,        // page37
    input  wire [15:0] sof_delta16,            // page38
    input  wire [15:0] lock_status16,          // page39

    input  wire [15:0] dbg_soft_drop_lines_cnt,  // page40
    input  wire [15:0] dbg_soft_dup_lines_cnt,   // page41
    input  wire [15:0] dbg_hard_resync_cnt,      // page42
    input  wire [15:0] dbg_last_soft_corr_v,     // page43
    input  wire [15:0] dbg_corr_skip_marker_cnt, // page44

    input  wire [15:0] dbg_desc_count_now,       // page49
    input  wire [15:0] dbg_desc_err_now,         // page50
    input  wire [15:0] dbg_marker_distance,      // page51
    input  wire [15:0] dbg_last_resync_reason,   // page52
    input  wire [15:0] dbg_corr_pending_flags,   // page54

    input  wire [15:0] dbg_pop_lines_cnt,        // page55
    input  wire [15:0] dbg_hold_lines_cnt,       // page56
    input  wire [15:0] dbg_hold_stuck_abort_cnt, // page57
    input  wire [15:0] dbg_cam_marker_drop_or_defer_cnt, // page58

    input  wire [15:0] hdmi_frame_repeat_cnt,    // page64
    input  wire [15:0] fill_lines_cnt,           // page65
    input  wire [15:0] blocks_left_snapshot,     // page66
    input  wire [15:0] cam_blocks_per_field_last,// page67
    input  wire [15:0] marker_at_head,           // page68
    input  wire [15:0] field_start_ok_cnt,       // page69

    output reg         new_sample = 1'b0,
    output reg [7:0]   out_page   = 8'd0,
    output reg [15:0]  out_value  = 16'd0
);

    localparam DIAG_LITE = 1;
    localparam integer PAGES_EFF = DIAG_LITE ? 40 : PAGES;

    localparam integer PERIOD = (CLK_HZ / PERIODS);
    localparam [9:0]   LPF_INVALID = 10'h3FF; // 1023

    function [16:0] delta17;
        input [15:0] cur;
        input [15:0] prev;
        begin
            if (cur >= prev) delta17 = {1'b0,cur} - {1'b0,prev};
            else             delta17 = {1'b0,cur} + 17'd65536 - {1'b0,prev};
        end
    endfunction

    function [9:0] delta10_reset_ok;
        input [9:0] cur;
        input [9:0] prev;
        begin
            if (cur >= prev) delta10_reset_ok = cur - prev;
            else             delta10_reset_ok = cur;
        end
    endfunction

    reg [24:0] cnt  = 25'd0;
    reg [7:0]  page = 8'd0;
    reg [15:0] value_mux;

    reg [15:0] last_desc_sent = 16'd0;
    reg [15:0] last_fieldtog  = 16'd0;

    reg [16:0] div_rem   = 17'd0;
    reg [16:0] div_denom = 17'd1;
    reg [9:0]  div_quot  = 10'd0;
    reg        div_busy  = 1'b0;

    reg [9:0]  lines_per_field_est = LPF_INVALID;

    reg  signed [7:0] desc_delta_lat = 8'sd0;
    reg  signed [7:0] free_delta_lat = 8'sd0;
    reg  [9:0]        uf_delta_lat   = 10'd0;
    reg  [9:0]        of_delta_lat   = 10'd0;

    reg [15:0] drop_accum_work   = 16'd0;
    reg [15:0] dup_accum_work    = 16'd0;
    reg [15:0] resync_accum_work = 16'd0;

    reg [15:0] drop_accum   = 16'd0;
    reg [15:0] dup_accum    = 16'd0;
    reg [15:0] resync_accum = 16'd0;

    reg [15:0] prev_fieldtog_cnt = 16'd0;
    reg [4:0]  prev_desc_count   = 5'd0;
    reg [3:0]  prev_free_cnt     = 4'd0;
    reg [9:0]  prev_uf           = 10'd0;
    reg [9:0]  prev_of           = 10'd0;

    wire field_event = (dbg_cam_fieldtog_cnt != prev_fieldtog_cnt);

    generate
        if (DIAG_LITE) begin : g_diag_lite
            always @* begin
                value_mux = 16'h0000;
                case (page)
                    8'd0:  value_mux = {6'd0, raw_lines_per_field};
                    8'd1:  value_mux = {11'd0, dbg_desc_count};
                    8'd2:  value_mux = {6'd0, dbg_underflow_low10};
                    8'd3:  value_mux = {6'd0, dbg_overflow_low10};
                    8'd4:  value_mux = {13'd0, dbg_drop_used};
                    8'd5:  value_mux = {13'd0, dbg_dup_used};
                    8'd6:  value_mux = {15'd0, dbg_resync_used};
                    8'd7:  value_mux = {11'd0, dbg_desc_min};
                    8'd8:  value_mux = {11'd0, dbg_desc_max};
                    8'd9:  value_mux = {10'd0, dbg_marker_found, dbg_marker_off};

                    8'd10: value_mux = cam_field_period_lo;
                    8'd11: value_mux = cam_field_period_hi;

                    8'd12: value_mux = dbg_cam_fieldtog_cnt;
                    8'd13: value_mux = dbg_cam_marker_inj_cnt;
                    8'd14: value_mux = dbg_cam_desc_sent_cnt;

                    8'd15: value_mux = {6'd0, lines_per_field_est};

                    8'd16: value_mux = dbg_fault_sticky;
                    8'd17: value_mux = {8'd0, dbg_own_map};
                    8'd18: value_mux = dbg_cam_desc_push_cnt;
                    8'd19: value_mux = dbg_pix_desc_pop_cnt;
                    8'd20: value_mux = {8'd0, dbg_overflow_rel_lo8};

                    8'd21: value_mux = {12'd0, dbg_free_cnt};
                    8'd22: value_mux = {12'd0, dbg_free_min};
                    8'd23: value_mux = {12'd0, dbg_free_max};
                    8'd24: value_mux = dbg_alloc_fail_cnt;
                    8'd25: value_mux = dbg_rel_doublefree_cnt;

                    8'd26: value_mux = {{8{desc_delta_lat[7]}}, desc_delta_lat};
                    8'd27: value_mux = {{8{free_delta_lat[7]}}, free_delta_lat};
                    8'd28: value_mux = {6'd0, uf_delta_lat};
                    8'd29: value_mux = {6'd0, of_delta_lat};
                    8'd30: value_mux = drop_accum;
                    8'd31: value_mux = dup_accum;
                    8'd32: value_mux = resync_accum;

                    8'd33: value_mux = de_line_cnt;
                    8'd34: value_mux = dbg_last_drop_v;
                    8'd35: value_mux = dbg_last_dup_v;
                    8'd36: value_mux = in_sof_cnt16;
                    8'd37: value_mux = out_vsync_cnt16;
                    8'd38: value_mux = sof_delta16;
                    8'd39: value_mux = lock_status16;

                    default: value_mux = 16'h0000;
                endcase
            end
        end else begin : g_diag_full
            always @* begin
                value_mux = 16'h0000;
                case (page)
                    8'd0:  value_mux = {6'd0, raw_lines_per_field};
                    8'd1:  value_mux = {11'd0, dbg_desc_count};
                    8'd2:  value_mux = {6'd0, dbg_underflow_low10};
                    8'd3:  value_mux = {6'd0, dbg_overflow_low10};
                    8'd4:  value_mux = {13'd0, dbg_drop_used};
                    8'd5:  value_mux = {13'd0, dbg_dup_used};
                    8'd6:  value_mux = {15'd0, dbg_resync_used};
                    8'd7:  value_mux = {11'd0, dbg_desc_min};
                    8'd8:  value_mux = {11'd0, dbg_desc_max};
                    8'd9:  value_mux = {10'd0, dbg_marker_found, dbg_marker_off};

                    8'd10: value_mux = cam_field_period_lo;
                    8'd11: value_mux = cam_field_period_hi;

                    8'd12: value_mux = dbg_cam_fieldtog_cnt;
                    8'd13: value_mux = dbg_cam_marker_inj_cnt;
                    8'd14: value_mux = dbg_cam_desc_sent_cnt;

                    8'd15: value_mux = {6'd0, lines_per_field_est};

                    8'd16: value_mux = dbg_fault_sticky;
                    8'd17: value_mux = {8'd0, dbg_own_map};
                    8'd18: value_mux = dbg_cam_desc_push_cnt;
                    8'd19: value_mux = dbg_pix_desc_pop_cnt;
                    8'd20: value_mux = {8'd0, dbg_overflow_rel_lo8};

                    8'd21: value_mux = {12'd0, dbg_free_cnt};
                    8'd22: value_mux = {12'd0, dbg_free_min};
                    8'd23: value_mux = {12'd0, dbg_free_max};
                    8'd24: value_mux = dbg_alloc_fail_cnt;
                    8'd25: value_mux = dbg_rel_doublefree_cnt;

                    8'd26: value_mux = {{8{desc_delta_lat[7]}}, desc_delta_lat};
                    8'd27: value_mux = {{8{free_delta_lat[7]}}, free_delta_lat};
                    8'd28: value_mux = {6'd0, uf_delta_lat};
                    8'd29: value_mux = {6'd0, of_delta_lat};
                    8'd30: value_mux = drop_accum;
                    8'd31: value_mux = dup_accum;
                    8'd32: value_mux = resync_accum;

                    8'd33: value_mux = {10'd0, dbg_cam_descq_cnt_cam};

                    8'd34: value_mux = dbg_last_drop_v;
                    8'd35: value_mux = dbg_last_dup_v;
                    8'd36: value_mux = in_sof_cnt16;
                    8'd37: value_mux = out_vsync_cnt16;
                    8'd38: value_mux = sof_delta16;
                    8'd39: value_mux = lock_status16;

                    8'd40: value_mux = dbg_soft_drop_lines_cnt;
                    8'd41: value_mux = dbg_soft_dup_lines_cnt;
                    8'd42: value_mux = dbg_hard_resync_cnt;
                    8'd43: value_mux = dbg_last_soft_corr_v;
                    8'd44: value_mux = dbg_corr_skip_marker_cnt;

                    8'd45: value_mux = dbg_align_pop_total;
                    8'd46: value_mux = dbg_align_hit_cnt;
                    8'd47: value_mux = dbg_marker_miss_cnt;
                    8'd48: value_mux = {8'd0, dbg_marker_off_snapshot};

                    8'd49: value_mux = dbg_desc_count_now;
                    8'd50: value_mux = dbg_desc_err_now;
                    8'd51: value_mux = dbg_marker_distance;
                    8'd52: value_mux = dbg_last_resync_reason;
                    8'd53: value_mux = dbg_last_soft_corr_v;
                    8'd54: value_mux = dbg_corr_pending_flags;

                    8'd55: value_mux = rel_sent_cnt_pix;
                    8'd56: value_mux = rel_rx_cnt_cam;
                    8'd57: value_mux = dbg_pop_lines_cnt;
                    8'd58: value_mux = dbg_hold_lines_cnt;
                    8'd59: value_mux = dbg_hold_stuck_abort_cnt;
                    8'd60: value_mux = dbg_cam_marker_drop_or_defer_cnt;

                    8'd61: value_mux = {10'd0, dbg_cam_block_idx_cam};
                    8'd62: value_mux = {10'd0, dbg_blocks_per_field_target_cam};
                    8'd63: value_mux = dbg_cam_stopped_early_cnt_cam;
                    8'd64: value_mux = hdmi_frame_repeat_cnt;
                    8'd65: value_mux = fill_lines_cnt;
                    8'd66: value_mux = blocks_left_snapshot;
                    8'd67: value_mux = cam_blocks_per_field_last;
                    8'd68: value_mux = marker_at_head;
                    8'd69: value_mux = field_start_ok_cnt;

                    default: value_mux = 16'h0000;
                endcase
            end
        end
    endgenerate

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            cnt        <= 25'd0;
            page       <= 8'd0;
            new_sample <= 1'b0;
            out_page   <= 8'd0;
            out_value  <= 16'd0;

            last_desc_sent      <= 16'd0;
            last_fieldtog       <= 16'd0;
            div_rem             <= 17'd0;
            div_denom           <= 17'd1;
            div_quot            <= 10'd0;
            div_busy            <= 1'b0;
            lines_per_field_est <= LPF_INVALID;

            desc_delta_lat      <= 8'sd0;
            free_delta_lat      <= 8'sd0;
            uf_delta_lat        <= 10'd0;
            of_delta_lat        <= 10'd0;

            drop_accum_work     <= 16'd0;
            dup_accum_work      <= 16'd0;
            resync_accum_work   <= 16'd0;

            drop_accum          <= 16'd0;
            dup_accum           <= 16'd0;
            resync_accum        <= 16'd0;

            prev_fieldtog_cnt   <= 16'd0;
            prev_desc_count     <= 5'd0;
            prev_free_cnt       <= 4'd0;
            prev_uf             <= 10'd0;
            prev_of             <= 10'd0;

        end else begin
            new_sample <= 1'b0;

            if (field_event) begin
                desc_delta_lat <= $signed({1'b0, dbg_desc_count}) - $signed({1'b0, prev_desc_count});
                free_delta_lat <= $signed({1'b0, dbg_free_cnt})   - $signed({1'b0, prev_free_cnt});

                uf_delta_lat   <= delta10_reset_ok(dbg_underflow_low10, prev_uf);
                of_delta_lat   <= delta10_reset_ok(dbg_overflow_low10,  prev_of);

                if (drop_accum_work > (16'hFFFF - {13'd0, dbg_drop_used}))
                    drop_accum_work <= 16'hFFFF;
                else
                    drop_accum_work <= drop_accum_work + {13'd0, dbg_drop_used};

                if (dup_accum_work > (16'hFFFF - {13'd0, dbg_dup_used}))
                    dup_accum_work <= 16'hFFFF;
                else
                    dup_accum_work <= dup_accum_work + {13'd0, dbg_dup_used};

                if (dbg_resync_used) begin
                    if (resync_accum_work != 16'hFFFF)
                        resync_accum_work <= resync_accum_work + 16'd1;
                end

                prev_fieldtog_cnt <= dbg_cam_fieldtog_cnt;
                prev_desc_count   <= dbg_desc_count;
                prev_free_cnt     <= dbg_free_cnt;
                prev_uf           <= dbg_underflow_low10;
                prev_of           <= dbg_overflow_low10;
            end

            if (div_busy) begin
                if (div_denom == 17'd0) begin
                    div_busy            <= 1'b0;
                    lines_per_field_est <= LPF_INVALID;
                end else if (div_rem >= div_denom) begin
                    div_rem <= div_rem - div_denom;

                    if (div_quot != 10'h3FF)
                        div_quot <= div_quot + 10'd1;
                    else begin
                        div_busy            <= 1'b0;
                        lines_per_field_est <= 10'h3FF;
                    end
                end else begin
                    div_busy            <= 1'b0;
                    lines_per_field_est <= div_quot;
                end
            end

            if (cnt == PERIOD-1) begin
                if (!i2c_busy) begin
                    cnt        <= 25'd0;

                    out_page   <= page;
                    out_value  <= value_mux;
                    new_sample <= 1'b1;

                    if (page == 8'd14) begin
                        div_rem   <= delta17(dbg_cam_desc_sent_cnt, last_desc_sent);
                        div_denom <= delta17(dbg_cam_fieldtog_cnt,  last_fieldtog);

                        last_desc_sent <= dbg_cam_desc_sent_cnt;
                        last_fieldtog  <= dbg_cam_fieldtog_cnt;

                        div_quot <= 10'd0;

                        if (delta17(dbg_cam_fieldtog_cnt, last_fieldtog) == 17'd0) begin
                            div_busy            <= 1'b0;
                            lines_per_field_est <= LPF_INVALID;
                        end else begin
                            div_busy <= 1'b1;
                        end
                    end

                    if (page == (PAGES_EFF-1)) begin
                        drop_accum         <= drop_accum_work;
                        dup_accum          <= dup_accum_work;
                        resync_accum       <= resync_accum_work;

                        drop_accum_work    <= 16'd0;
                        dup_accum_work     <= 16'd0;
                        resync_accum_work  <= 16'd0;

                        page <= 8'd0;
                    end else begin
                        page <= page + 8'd1;
                    end
                end
            end else begin
                cnt <= cnt + 25'd1;
            end
        end
    end

endmodule

`default_nettype wire
