// tvp5150_i2c_init.v - Bit-banged I2C init for TVP5150 on Tang Nano 9K
// Sends THREE separate transactions ONCE:
//
//   1) START, 0xBA, 0x00, 0x00, STOP
//   2) START, 0xBA, 0x02, 0x00, STOP
//   3) START, 0xBA, 0x03, 0x0D, STOP
//
// After that, it goes idle and raises init_done = 1.
//
// I2C speed ~5 kHz from 27 MHz clk.

`timescale 1ns / 1ps

module tvp5150_i2c_init (
    input  wire clk,        // 27 MHz
    input  wire resetn,     // active-low

    inout  wire tvp_sda,    // I2C SDA
    inout  wire tvp_scl,    // I2C SCL

    output reg  init_done   // 1 when all 3 writes are done
);


    // -------------------------------------------------
    // I2C timing: ~5 kHz SCL from 27 MHz clock
    // -------------------------------------------------
    localparam integer CLKS_PER_HALF = 2700;

    reg [15:0] clk_cnt;
    reg        scl_reg;       // internal SCL level (0/1)
    reg        sda_drive;     // 1 = drive SDA low, 0 = release (pulled up)

    // Open-drain outputs: drive low or release to 'Z'
    assign tvp_scl = (scl_reg == 1'b0) ? 1'b0 : 1'bz;
    assign tvp_sda =  sda_drive        ? 1'b0 : 1'bz;

    // -------------------------------------------------
    // TVP5150 init as three separate writes:
    //   (reg, data) = (0x00,0x00), (0x02,0x00), (0x03,0x0D)
    //
    // Each transaction: START, ADDR_WR, reg, data, STOP
    // -------------------------------------------------
    localparam [7:0] ADDR_WR = 8'hBA;   // if board uses 0xB8, change to 8'hB8

    // Which pair we are sending: 0,1,2
    reg [1:0] pair_idx;      // 0..2
    reg [1:0] byte_in_tr;    // 0..2: 0=ADDR, 1=REG, 2=DATA

    reg [7:0] cur_reg;
    reg [7:0] cur_data;
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

    wire half_tick = (clk_cnt == CLKS_PER_HALF - 1);

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

            pair_idx   <= 2'd0;
            byte_in_tr <= 2'd0;
            cur_reg    <= 8'h00; // first pair: (0x00,0x00)
            cur_data   <= 8'h00;
            shift_reg  <= 8'h00;
            bit_idx    <= 4'd0;
            init_done  <= 1'b0;
        end else begin
            // clock divider for SCL timing
            if (half_tick)
                clk_cnt <= 16'd0;
            else
                clk_cnt <= clk_cnt + 16'd1;

            case (state)
                //-----------------------------------------
                // IDLE: bus high, pause before first sequence
                //-----------------------------------------
                ST_IDLE: begin
                    scl_reg   <= 1'b1;
                    sda_drive <= 1'b0;

                    // Only start if we haven't finished yet
                    if (!init_done && half_tick) begin
                        pause_cnt <= pause_cnt + 16'd1;
                        if (pause_cnt == 16'd2000) begin
                            pause_cnt <= 16'd0;
                            // start with current pair (pair_idx should be 0 here)
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
                        scl_reg    <= 1'b0;   // pull SCL low to start data
                        byte_in_tr <= 2'd0;   // first byte in transaction = ADDR_WR
                        bit_idx    <= 4'd7;
                        shift_reg  <= ADDR_WR;
                        state      <= ST_BIT_LOW;
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
                        // We ignore ACK value for now, just timing

                        if (byte_in_tr < 2) begin
                            // More bytes to send in THIS transaction:
                            //  byte_in_tr = 0: next is REG
                            //  byte_in_tr = 1: next is DATA
                            byte_in_tr <= byte_in_tr + 1'b1;
                            bit_idx    <= 4'd7;

                            case (byte_in_tr)
                                2'd0: shift_reg <= cur_reg;   // after ADDR → REG
                                2'd1: shift_reg <= cur_data;  // after REG  → DATA
                                default: shift_reg <= 8'h00;
                            endcase

                            state <= ST_BIT_LOW;
                        end else begin
                            // Sent ADDR, REG, DATA → transaction complete → STOP
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

                        // Move to next pair, or finish after all three
                        if (pair_idx < 2'd2) begin
                            // Advance index
                            pair_idx <= pair_idx + 1'b1;

                            // Load NEXT pair's reg/data based on OLD pair_idx
                            case (pair_idx)
                                2'd0: begin
                                    // next is pair 1: (0x02, 0x00)
                                    cur_reg  <= 8'h02;
                                    cur_data <= 8'h00;
                                end
                                2'd1: begin
                                    // next is pair 2: (0x03, 0x0D)
                                    cur_reg  <= 8'h03;
                                    cur_data <= 8'h0D;
                                end
                                default: begin
                                    cur_reg  <= 8'h00;
                                    cur_data <= 8'h00;
                                end
                            endcase

                            // start next transaction almost immediately
                            state    <= ST_START_A;
                        end else begin
                            // all pairs done: mark init_done and stay idle
                            init_done <= 1'b1;
                            state     <= ST_IDLE;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule