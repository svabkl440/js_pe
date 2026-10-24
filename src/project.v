
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
    // Instruction memory: IHP SG13G2 1024x8 single-port SRAM macro.
    //
    // Only the first 128 bytes are used (64 instructions x 16 bits).
    // Instruction i lives at byte addresses {2i, 2i+1}:
    //     byte 2i   = instr[7:0]   (low byte)
    //     byte 2i+1 = instr[15:8]  (high byte)
    //
    // The macro is 8 bits wide and has a SYNCHRONOUS read: the address
    // is sampled on a clock edge and A_DOUT is valid after that edge.
    // A 16-bit instruction therefore needs two reads, so every
    // instruction occupies exactly TWO clock cycles (one "slot"):
    //
    //   PRIME  issue read of low byte  {pc,0}   (only after load mode)
    //   LO     capture low byte;  issue read of high byte {pc,1}
    //   EXEC   A_DOUT = high byte -> instr valid; execute; issue read of
    //          low byte of {next_pc,0}.  Then -> LO.
    //
    // Steady state is LO,EXEC,LO,EXEC,... = 2 clocks per instruction,
    // with a fixed (deterministic) cost for every instruction.
    //
    // There is deliberately no preload. Firmware is written through
    // the runtime loader, one byte per SRAM write.
    // ------------------------------------------------------------
    localparam [1:0] PH_PRIME = 2'd0,
                     PH_LO    = 2'd1,
                     PH_EXEC  = 2'd2;

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
    // byte, which is written straight into the SRAM at that byte
    // address. Even byte = low half, odd byte = high half.
    // ------------------------------------------------------------
    reg load_mode_meta;
    reg load_mode_sync;
    reg load_mode_prev;

    reg load_strobe_meta;
    reg load_strobe_sync;
    reg load_strobe_prev;

    reg [6:0] load_byte_addr;  // Byte address 0..127
    reg       load_full;
    reg       loader_armed;

    wire load_mode_pin   = uio_in[7];
    wire load_strobe_pin = uio_in[6];

    // ------------------------------------------------------------
    // CPU state
    // ------------------------------------------------------------
    reg [DEPTH_LOG2-1:0] pc;
    reg [1:0] ph;
    reg [7:0] lo_reg;          // low instruction byte, captured in PH_LO
    reg [7:0] sr;
    reg [7:0] x, y;
    reg [7:0] out_reg;

    reg       dly_active;
    reg [7:0] dly_cnt;

    wire [15:0] pins  = {uio_in, ui_in};

    // ------------------------------------------------------------
    // SRAM port
    // ------------------------------------------------------------
    wire [7:0] sram_dout;

    wire run  = !load_mode_sync && loader_armed;

    // One byte written per accepted strobe edge (same condition the
    // loader state machine uses below).
    wire load_wr = load_mode_sync && load_strobe_sync &&
                   !load_strobe_prev && !load_full;

    // In PH_EXEC, A_DOUT holds the high byte, lo_reg the low byte.
    wire [15:0] instr = {sram_dout, lo_reg};
    wire [3:0]  op    = instr[15:12];

    // Next program counter, computed combinationally so the next
    // fetch can be issued in the same cycle the current one executes.
    reg [DEPTH_LOG2-1:0] next_pc;
    always @* begin
        next_pc = pc + 1'b1;
        case (op)
            OP_DELAY: begin
                if (!dly_active) begin
                    if (instr[7:0] != 0) next_pc = pc;
                end else if (dly_cnt != 8'd1) begin
                    next_pc = pc;
                end
            end
            OP_WAIT: begin
                if (pins[instr[3:0]] != instr[4]) next_pc = pc;
            end
            OP_JMP: begin
                case (instr[11:10])
                    2'd0: next_pc = instr[5:0];
                    2'd1: if (x != 0)           next_pc = instr[5:0];
                    2'd2: if (y != 0)           next_pc = instr[5:0];
                    2'd3: if (pins[instr[9:6]]) next_pc = instr[5:0];
                endcase
            end
            default: ;
        endcase
    end

    // Byte address presented to the SRAM (10-bit macro, 7 bits used).
    reg [6:0] sram_addr7;
    always @* begin
        if (load_mode_sync)  sram_addr7 = load_byte_addr;
        else case (ph)
            PH_LO:   sram_addr7 = {pc, 1'b1};
            PH_EXEC: sram_addr7 = {next_pc, 1'b0};
            default: sram_addr7 = {pc, 1'b0};      // PH_PRIME
        endcase
    end

    RM_IHPSG13_1P_1024x8_c2_bm_bist sram (
        .A_CLK      (clk),
        .A_MEN      (1'b1),
        .A_WEN      (load_wr),
        .A_REN      (run),
        .A_ADDR     ({3'b000, sram_addr7}),
        .A_DIN      (ui_in),
        .A_DLY      (1'b1),
        .A_DOUT     (sram_dout),
        .A_BM       (8'hFF),
        .A_BIST_CLK (1'b0),
        .A_BIST_EN  (1'b0),
        .A_BIST_MEN (1'b0),
        .A_BIST_WEN (1'b0),
        .A_BIST_REN (1'b0),
        .A_BIST_ADDR(10'b0),
        .A_BIST_DIN (8'b0),
        .A_BIST_BM  (8'b0)
    );

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
            load_full      <= 1'b0;
            loader_armed   <= 1'b0;

            // CPU state
            pc         <= 0;
            ph         <= PH_PRIME;
            lo_reg     <= 8'd0;
            sr         <= 0;
            x          <= 0;
            y          <= 0;
            out_reg    <= 8'hFF;
            dly_active <= 0;
            dly_cnt    <= 0;

            // SRAM is intentionally not reset. The loader fills it.
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
                            load_full      <= 1'b0;
                end

                // Keep the CPU stopped while firmware is loading.
                pc         <= 0;
                ph         <= PH_PRIME;
                sr         <= 0;
                x          <= 0;
                y          <= 0;
                out_reg    <= 8'hFF;
                dly_active <= 0;
                dly_cnt    <= 0;

                // Accept exactly one byte per synchronized
                // rising edge of LOAD_STROBE.
                if (load_wr) begin
                    if (load_byte_addr == 7'd127)
                        load_full <= 1'b1;

                    load_byte_addr <= load_byte_addr + 1'b1;
                end

            // ----------------------------------------------------
            // EXECUTION MODE
            // ----------------------------------------------------
            end else if (loader_armed) begin
                case (ph)
                    PH_PRIME: ph <= PH_LO;

                    PH_LO: begin
                        lo_reg <= sram_dout;     // low byte arrived
                        ph     <= PH_EXEC;
                    end

                    default: begin               // PH_EXEC: execute
                        ph <= PH_LO;
                        pc <= next_pc;

                        case (op)
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

                            OP_PULL: sr      <= ui_in;
                            OP_PUSH: out_reg <= sr;

                            OP_DELAY: begin
                                if (!dly_active) begin
                                    if (instr[7:0] != 0) begin
                                        dly_active <= 1'b1;
                                        dly_cnt    <= instr[7:0];
                                    end
                                end else if (dly_cnt == 1) begin
                                    dly_active <= 1'b0;
                                end else begin
                                    dly_cnt <= dly_cnt - 1'b1;
                                end
                            end

                            OP_LDX: x <= instr[7:0];
                            OP_LDY: y <= instr[7:0];

                            OP_JMP: begin
                                case (instr[11:10])
                                    2'd1: x <= x - 1'b1;
                                    2'd2: y <= y - 1'b1;
                                    default: ;
                                endcase
                            end

                            default: begin
                                // NOP, WAIT (stall handled in next_pc),
                                // unknown opcodes behave like NOP.
                            end
                        endcase
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
