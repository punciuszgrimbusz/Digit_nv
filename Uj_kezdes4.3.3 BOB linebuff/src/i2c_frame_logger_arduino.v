`timescale 1ns / 1ps
`default_nettype none

module i2c_frame_logger_arduino #(
    parameter I2C_ADDR      = 7'h5D,   // Arduino Wire.begin(0x5D)
    parameter CLKS_PER_HALF = 2700     // 27MHz / (2*2700) ~ 5 kHz SCL
)(
    input  wire       clk,          // cam1_pclk
    input  wire       resetn,       // aktív alacsony

    input  wire       new_sample,   // 1 clk ciklus strobe
    input  wire [7:0] page,         // DATA0
    input  wire [15:0] value,       // DATA1/DATA2

    inout  wire       sda,          // open-drain
    inout  wire       scl,          // open-drain

    output reg        busy
);

    // -------------------------------------------------
    // I2C timing
    // -------------------------------------------------
    reg [15:0] clk_cnt;
    wire       half_tick = (clk_cnt == CLKS_PER_HALF - 1);

    reg scl_reg;
    reg sda_drive;     // 1 = drive SDA low, 0 = release

    assign scl = (scl_reg == 1'b0) ? 1'b0 : 1'bz;
    assign sda =  sda_drive        ? 1'b0 : 1'bz;

    // -------------------------------------------------
    // Transaction: START, ADDR_WR, DATA0, DATA1, DATA2, STOP
    // -------------------------------------------------
    localparam [7:0] ADDR_WR = {I2C_ADDR, 1'b0};

    reg [7:0]  page_shadow;
    reg [15:0] value_shadow;

    reg        pending;      // NEW: van elküldendő minta

    reg [7:0] shift_reg;
    reg [3:0] bit_idx;

    // 0=ADDR, 1=DATA0(page), 2=DATA1(lo), 3=DATA2(hi)
    reg [1:0] byte_idx;

    reg [3:0]  state;

    localparam [3:0]
        ST_IDLE     = 4'd0,
        ST_START_A  = 4'd1,
        ST_START_B  = 4'd2,
        ST_BIT_LOW  = 4'd3,
        ST_BIT_HIGH = 4'd4,
        ST_ACK_LOW  = 4'd5,
        ST_ACK_HIGH = 4'd6,
        ST_STOP_A   = 4'd7,
        ST_STOP_B   = 4'd8;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            clk_cnt      <= 16'd0;
            scl_reg      <= 1'b1;
            sda_drive    <= 1'b0;
            state        <= ST_IDLE;

            byte_idx     <= 2'd0;
            shift_reg    <= 8'h00;
            bit_idx      <= 4'd0;

            page_shadow  <= 8'h00;
            value_shadow <= 16'h0000;

            pending      <= 1'b0;
            busy         <= 1'b0;
        end else begin
            // clock divider
            if (half_tick) clk_cnt <= 16'd0;
            else           clk_cnt <= clk_cnt + 16'd1;

            // latch newest sample anytime, és jelöld pending-nek
            if (new_sample) begin
                page_shadow  <= page;
                value_shadow <= value;
                pending      <= 1'b1;
            end

            case (state)
                ST_IDLE: begin
                    scl_reg   <= 1'b1;
                    sda_drive <= 1'b0;
                    busy      <= 1'b0;

                    // csak akkor indítunk, ha van pending minta
                    if (pending && half_tick) begin
                        pending   <= 1'b0;

                        byte_idx  <= 2'd0;
                        bit_idx   <= 4'd7;
                        shift_reg <= ADDR_WR;

                        busy      <= 1'b1;
                        state     <= ST_START_A;
                    end
                end

                // START: SDA 1->0 while SCL=1
                ST_START_A: begin
                    busy      <= 1'b1;
                    scl_reg   <= 1'b1;
                    sda_drive <= 1'b0;
                    if (half_tick) begin
                        sda_drive <= 1'b1;
                        state     <= ST_START_B;
                    end
                end

                ST_START_B: begin
                    busy <= 1'b1;
                    if (half_tick) begin
                        scl_reg <= 1'b0;
                        state   <= ST_BIT_LOW;
                    end
                end

                // bit low: set SDA
                ST_BIT_LOW: begin
                    busy    <= 1'b1;
                    scl_reg <= 1'b0;

                    if (shift_reg[bit_idx]) sda_drive <= 1'b0; // '1' -> release
                    else                    sda_drive <= 1'b1; // '0' -> drive low

                    if (half_tick) begin
                        scl_reg <= 1'b1;
                        state   <= ST_BIT_HIGH;
                    end
                end

                // bit high: clock
                ST_BIT_HIGH: begin
                    busy <= 1'b1;
                    if (half_tick) begin
                        scl_reg <= 1'b0;
                        if (bit_idx != 0) begin
                            bit_idx <= bit_idx - 1'b1;
                            state   <= ST_BIT_LOW;
                        end else begin
                            state <= ST_ACK_LOW;
                        end
                    end
                end

                // ACK (nem olvassuk, csak leadjuk a clockot)
                ST_ACK_LOW: begin
                    busy      <= 1'b1;
                    sda_drive <= 1'b0; // release
                    scl_reg   <= 1'b0;
                    if (half_tick) begin
                        scl_reg <= 1'b1;
                        state   <= ST_ACK_HIGH;
                    end
                end

                ST_ACK_HIGH: begin
                    busy <= 1'b1;
                    if (half_tick) begin
                        scl_reg <= 1'b0;

                        if (byte_idx < 2'd3) begin
                            byte_idx <= byte_idx + 2'd1;
                            bit_idx  <= 4'd7;

                            case (byte_idx)
                                2'd0: shift_reg <= page_shadow;           // DATA0
                                2'd1: shift_reg <= value_shadow[7:0];     // DATA1
                                2'd2: shift_reg <= value_shadow[15:8];    // DATA2
                                default: shift_reg <= 8'h00;
                            endcase

                            state <= ST_BIT_LOW;
                        end else begin
                            state <= ST_STOP_A;
                        end
                    end
                end

                // STOP
                ST_STOP_A: begin
                    busy      <= 1'b1;
                    scl_reg   <= 1'b0;
                    sda_drive <= 1'b1;
                    if (half_tick) begin
                        scl_reg <= 1'b1;
                        state   <= ST_STOP_B;
                    end
                end

                ST_STOP_B: begin
                    busy <= 1'b1;
                    if (half_tick) begin
                        sda_drive <= 1'b0; // release
                        busy      <= 1'b0;
                        state     <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
