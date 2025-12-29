// i2c_debug_fixed.v
// Nagyon egyszerű I2C master teszt:
// Folyamatosan ismétli a következő tranzakciót kb. 5 kHz SCL-lel:
//
//   START
//   0xBA       ; (0x5D << 1 | 0) -> Arduino slave cím 0x5D write
//   0x12       ; tetszőleges adat byte 0
//   0x34       ; tetszőleges adat byte 1
//   STOP
//
// SDA és SCL ugyanúgy open-drain, mint a tvp5150_i2c_init-ben.
// -> FONTOS: kell PULL-UP a sda/scl vonalakra! (külső ellenállás vagy CST).

`timescale 1ns / 1ps
`default_nettype none

module i2c_debug_fixed (
    input  wire clk,        // 27 MHz (cam1_pclk-hez kösd)
    input  wire resetn,     // aktív alacsony reset

    inout  wire sda,        // I2C SDA (open-drain)
    inout  wire scl         // I2C SCL (open-drain)
);

    // -------------------------------------------------
    // I2C timing: ~5 kHz SCL from 27 MHz clock
    // -------------------------------------------------
    localparam integer CLKS_PER_HALF = 2700;

    reg [15:0] clk_cnt;
    wire       half_tick;

    assign half_tick = (clk_cnt == CLKS_PER_HALF - 1);

    reg scl_reg;       // internal SCL level (0/1)
    reg sda_drive;     // 1 = drive SDA low, 0 = release (pulled up)

    // Open-drain outputs: drive low or release to 'Z'
    assign scl = (scl_reg == 1'b0) ? 1'b0 : 1'bz;
    assign sda =  sda_drive        ? 1'b0 : 1'bz;

    // -------------------------------------------------
    // Egyszerű tranzakció:
    //   START, ADDR_WR, DATA0, DATA1, STOP
    // -------------------------------------------------
    localparam [7:0] ADDR_WR = 8'hBA;   // 0x5D << 1 | 0
    localparam [7:0] DATA0   = 8'h12;
    localparam [7:0] DATA1   = 8'h34;

    reg [1:0] byte_idx;    // 0=ADDR, 1=DATA0, 2=DATA1
    reg [7:0] shift_reg;
    reg [3:0] bit_idx;

    reg [3:0]  state;
    reg [15:0] pause_cnt;

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

    // -------------------------------------------------
    // Main I2C bit-banging FSM
    // -------------------------------------------------
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            clk_cnt   <= 16'd0;
            scl_reg   <= 1'b1;   // bus idle: SCL=1
            sda_drive <= 1'b0;   // SDA released → high via pull-up
            state     <= ST_IDLE;
            pause_cnt <= 16'd0;

            byte_idx  <= 2'd0;
            shift_reg <= 8'h00;
            bit_idx   <= 4'd0;
        end else begin
            // clock divider for SCL timing
            if (half_tick)
                clk_cnt <= 16'd0;
            else
                clk_cnt <= clk_cnt + 16'd1;

            case (state)
                //-----------------------------------------
                // IDLE: bus high, kis szünet két csomag között
                //-----------------------------------------
                ST_IDLE: begin
                    scl_reg   <= 1'b1;
                    sda_drive <= 1'b0;

                    if (half_tick) begin
                        pause_cnt <= pause_cnt + 16'd1;
                        if (pause_cnt == 16'd2000) begin
                            pause_cnt <= 16'd0;
                            state     <= ST_START_A;
                        end
                    end
                end

                //-----------------------------------------
                // START condition: SDA 1→0 while SCL=1
                //-----------------------------------------
                ST_START_A: begin
                    scl_reg   <= 1'b1;
                    sda_drive <= 1'b0;   // SDA high (released)
                    if (half_tick) begin
                        sda_drive <= 1'b1;  // SDA low → START
                        state     <= ST_START_B;
                    end
                end

                ST_START_B: begin
                    if (half_tick) begin
                        scl_reg  <= 1'b0;   // pull SCL low to start data
                        byte_idx <= 2'd0;   // első byte: ADDR_WR
                        bit_idx  <= 4'd7;
                        shift_reg<= ADDR_WR;
                        state    <= ST_BIT_LOW;
                    end
                end

                //-----------------------------------------
                // Send 8 bits: SCL low phase → set SDA
                //-----------------------------------------
                ST_BIT_LOW: begin
                    scl_reg <= 1'b0;

                    // bit=1 → release SDA (high), bit=0 → drive low
                    if (shift_reg[bit_idx])
                        sda_drive <= 1'b0;
                    else
                        sda_drive <= 1'b1;

                    if (half_tick) begin
                        scl_reg <= 1'b1;      // SCL high to clock this bit
                        state   <= ST_BIT_HIGH;
                    end
                end

                //-----------------------------------------
                // Send 8 bits: SCL high phase
                //-----------------------------------------
                ST_BIT_HIGH: begin
                    if (half_tick) begin
                        scl_reg <= 1'b0;      // back to low for next bit
                        if (bit_idx != 0) begin
                            bit_idx <= bit_idx - 1'b1;
                            state   <= ST_BIT_LOW;
                        end else begin
                            // finished 8 bits, go to ACK
                            state <= ST_ACK_LOW;
                        end
                    end
                end

                //-----------------------------------------
                // ACK: release SDA while SCL goes high
                //-----------------------------------------
                ST_ACK_LOW: begin
                    sda_drive <= 1'b0;   // release SDA
                    scl_reg   <= 1'b0;
                    if (half_tick) begin
                        scl_reg <= 1'b1; // SCL high, slave should drive ACK
                        state   <= ST_ACK_HIGH;
                    end
                end

                ST_ACK_HIGH: begin
                    if (half_tick) begin
                        scl_reg <= 1'b0;
                        // ACK-et nem nézzük, csak időzítünk

                        if (byte_idx < 2) begin
                            // További byte-ok: ADDR_WR -> DATA0 -> DATA1
                            byte_idx <= byte_idx + 1'b1;
                            bit_idx  <= 4'd7;

                            case (byte_idx)
                                2'd0: shift_reg <= DATA0;
                                2'd1: shift_reg <= DATA1;
                                default: shift_reg <= 8'h00;
                            endcase

                            state <= ST_BIT_LOW;
                        end else begin
                            // 3 byte kész (ADDR, DATA0, DATA1) -> STOP
                            state <= ST_STOP_A;
                        end
                    end
                end

                //-----------------------------------------
                // STOP: SDA 0→1 while SCL=1
                //-----------------------------------------
                ST_STOP_A: begin
                    scl_reg   <= 1'b0;
                    sda_drive <= 1'b1;   // SDA low
                    if (half_tick) begin
                        scl_reg <= 1'b1; // raise SCL
                        state   <= ST_STOP_B;
                    end
                end

                ST_STOP_B: begin
                    if (half_tick) begin
                        sda_drive <= 1'b0;  // release SDA → goes high (STOP)
                        state     <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
