
`default_nettype none

module tt_um_pin_core (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    localparam DEPTH_LOG2 = 6;
    localparam DEPTH      = 1 << DEPTH_LOG2;

    // Instruction opcodes: instr[15:12]
    localparam [3:0]
        OP_NOP   = 4'h0,
        OP_SET   = 4'h1,
        OP_OUT   = 4'h2,
        OP_IN    = 4'h3,
        OP_PULL  = 4'h4,
        OP_PUSH  = 4'h5,
        OP_DELAY = 4'h6,
        OP_LDX   = 4'h7,
        OP_LDY   = 4'h8,
        OP_WAIT  = 4'h9,
        OP_JMP   = 4'hA;

    // ------------------------------------------------------------
    // Writable instruction memory
    // 64 instructions x 16 bits = 128 bytes.
    //
    // There is deliberately no $readmemh here.
    // Firmware is written through the runtime loader.
    // ------------------------------------------------------------
    reg [15:0] imem [0:DEPTH-1];

    // ------------------------------------------------------------
    // Loader interface
    //
    // ui_in[7:0] = byte being loaded
    // uio_in[7]  = LOAD_MODE
    // uio_in[6]  = LOAD_STROBE
    //
    // LOAD_MODE=1: accept firmware bytes; CPU is held.
    // LOAD_MODE=0: execute from instruction address zero.
    //
    // Each rising edge of synchronized LOAD_STROBE accepts one
    // byte. Two bytes are combined into one instruction.
    // ------------------------------------------------------------
    reg load_mode_meta;
    reg load_mode_sync;
    reg load_mode_prev;

    reg load_strobe_meta;
    reg load_strobe_sync;
    reg load_strobe_prev;

    reg [6:0] load_byte_addr;  // Byte address 0..127
    reg [7:0] load_low_byte;
    reg       load_full;
    reg       loader_armed;

    wire load_mode_pin   = uio_in[7];
    wire load_strobe_pin = uio_in[6];

    // ------------------------------------------------------------
    // CPU state
    // ------------------------------------------------------------
    reg [DEPTH_LOG2-1:0] pc;
    reg [7:0] sr;
    reg [7:0] x, y;
    reg [7:0] out_reg;

    reg       dly_active;
    reg [7:0] dly_cnt;

    wire [15:0] pins  = {uio_in, ui_in};
    wire [15:0] instr = imem[pc];
    wire [3:0]  op    = instr[15:12];

    // ------------------------------------------------------------
    // Loader + CPU
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            // Synchronizers
            load_mode_meta   <= 1'b0;
            load_mode_sync   <= 1'b0;
            load_mode_prev   <= 1'b0;

            load_strobe_meta <= 1'b0;
            load_strobe_sync <= 1'b0;
            load_strobe_prev <= 1'b0;

            // Loader state
            load_byte_addr <= 7'd0;
            load_low_byte  <= 8'd0;
            load_full      <= 1'b0;
            loader_armed   <= 1'b0;

            // CPU state
            pc         <= 0;
            sr         <= 0;
            x          <= 0;
            y          <= 0;
            out_reg    <= 8'hFF;
            dly_active <= 0;
            dly_cnt    <= 0;

            // imem is intentionally not reset. The loader fills it.
        end else begin
            // Synchronize the two control inputs to clk.
            load_mode_meta   <= load_mode_pin;
            load_mode_sync   <= load_mode_meta;
            load_mode_prev   <= load_mode_sync;

            load_strobe_meta <= load_strobe_pin;
            load_strobe_sync <= load_strobe_meta;
            load_strobe_prev <= load_strobe_sync;

            // ----------------------------------------------------
            // LOAD MODE
            // ----------------------------------------------------
            if (load_mode_sync) begin
                loader_armed <= 1'b1;

                // On entry into LOAD MODE, start at byte address 0.
                if (!load_mode_prev) begin
                    load_byte_addr <= 7'd0;
                    load_low_byte  <= 8'd0;
                    load_full      <= 1'b0;
                end

                // Keep the CPU stopped while firmware is loading.
                pc         <= 0;
                sr         <= 0;
                x          <= 0;
                y          <= 0;
                out_reg    <= 8'hFF;
                dly_active <= 0;
                dly_cnt    <= 0;

                // Accept exactly one byte per synchronized
                // rising edge of LOAD_STROBE.
                if (load_strobe_sync &&
                    !load_strobe_prev &&
                    !load_full) begin

                    if (!load_byte_addr[0]) begin
                        // Even byte address: save low byte.
                        load_low_byte <= ui_in;
                    end else begin
                        // Odd byte address: save high byte and
                        // commit the complete instruction.
                        imem[load_byte_addr[6:1]]
                            <= {ui_in, load_low_byte};

                        if (load_byte_addr == 7'd127)
                            load_full <= 1'b1;
                    end

                    load_byte_addr <= load_byte_addr + 1'b1;
                end

            // ----------------------------------------------------
            // EXECUTION MODE
            // ----------------------------------------------------
            end else if (loader_armed) begin
                pc <= pc + 1'b1;

                case (op)
                    OP_NOP: begin
                        // No operation.
                    end

                    OP_SET: begin
                        out_reg[instr[2:0]] <= instr[3];
                    end

                    OP_OUT: begin
                        // instr[3]: 0 = LSB first, 1 = MSB first
                        out_reg[instr[2:0]]
                            <= instr[3] ? sr[7] : sr[0];

                        sr <= instr[3]
                            ? {sr[6:0], 1'b0}
                            : {1'b0, sr[7:1]};
                    end

                    OP_IN: begin
                        sr <= instr[4]
                            ? {sr[6:0], pins[instr[3:0]]}
                            : {pins[instr[3:0]], sr[7:1]};
                    end

                    OP_PULL: begin
                        sr <= ui_in;
                    end

                    OP_PUSH: begin
                        out_reg <= sr;
                    end

                    OP_DELAY: begin
                        if (!dly_active) begin
                            if (instr[7:0] != 0) begin
                                dly_active <= 1'b1;
                                dly_cnt    <= instr[7:0];
                                pc         <= pc;
                            end
                        end else if (dly_cnt == 1) begin
                            dly_active <= 1'b0;
                        end else begin
                            dly_cnt <= dly_cnt - 1'b1;
                            pc      <= pc;
                        end
                    end

                    OP_LDX: begin
                        x <= instr[7:0];
                    end

                    OP_LDY: begin
                        y <= instr[7:0];
                    end

                    OP_WAIT: begin
                        if (pins[instr[3:0]] != instr[4])
                            pc <= pc;
                    end

                    OP_JMP: begin
                        case (instr[11:10])
                            2'd0: begin
                                pc <= instr[5:0];
                            end

                            2'd1: begin
                                if (x != 0)
                                    pc <= instr[5:0];
                                x <= x - 1'b1;
                            end

                            2'd2: begin
                                if (y != 0)
                                    pc <= instr[5:0];
                                y <= y - 1'b1;
                            end

                            2'd3: begin
                                if (pins[instr[9:6]])
                                    pc <= instr[5:0];
                            end
                        endcase
                    end

                    default: begin
                        // Unknown opcodes behave like NOP.
                    end
                endcase
            end
            // Before the first load, keep the CPU stopped.
        end
    end

    assign uo_out  = out_reg;
    assign uio_out = 8'd0;
    assign uio_oe  = 8'd0;

    wire _unused = &{ena, 1'b0};

endmodule

`default_nettype wire
