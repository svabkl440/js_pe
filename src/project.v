
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
    // Loader synchronizers
    // ------------------------------------------------------------
    reg load_mode_meta;
    reg load_mode_sync;
    reg load_mode_prev;

    reg load_strobe_meta;
    reg load_strobe_sync;
    reg load_strobe_prev;

    reg [6:0] load_byte_addr;
    reg       load_full;
    reg       loader_armed;

    wire load_mode_pin   = uio_in[7];
    wire load_strobe_pin = uio_in[6];

    wire load_strobe_rise =
        load_strobe_sync && !load_strobe_prev;

    // ------------------------------------------------------------
    // CPU registers
    // ------------------------------------------------------------
    reg [DEPTH_LOG2-1:0] pc;
    reg [7:0] sr;
    reg [7:0] x, y;
    reg [7:0] out_reg;

    reg       dly_active;
    reg [7:0] dly_cnt;

    // Indicates that the synchronous memory has returned a
    // valid instruction after starting or restarting execution.
    reg instr_valid;

    wire [15:0] pins = {uio_in, ui_in};

    // ------------------------------------------------------------
    // Program memory
    // ------------------------------------------------------------
    wire        mem_re;
    wire [5:0]  mem_raddr;
    wire [15:0] mem_rdata;

    wire        mem_we;
    wire [6:0]  mem_wbyte_addr;
    wire [7:0]  mem_wdata;

    pe_store #(
        .AW(6)
    ) program_store (
        .clk(clk),

        .re(mem_re),
        .raddr(mem_raddr),
        .rdata(mem_rdata),

        .we(mem_we),
        .wbyte_addr(mem_wbyte_addr),
        .wdata(mem_wdata)
    );

    // Read the next instruction while executing the current one.
    assign mem_re =
        rst_n && loader_armed && !load_mode_sync;

    // ------------------------------------------------------------
    // Current instruction and opcode
    // ------------------------------------------------------------
    wire [15:0] instr = mem_rdata;
    wire [3:0]  op    = instr[15:12];

    // ------------------------------------------------------------
    // Next-PC calculation
    //
    // The next instruction address is calculated from the
    // instruction currently being executed. The memory reads
    // that address on the same clock edge that executes instr.
    // ------------------------------------------------------------
    reg [5:0] next_pc;

    always @* begin
        next_pc = pc + 1'b1;

        case (op)

            OP_DELAY: begin
                if (!dly_active) begin
                    if (instr[7:0] != 0)
                        next_pc = pc;
                end else if (dly_cnt != 1) begin
                    next_pc = pc;
                end
            end

            OP_WAIT: begin
                if (pins[instr[3:0]] != instr[4])
                    next_pc = pc;
            end

            OP_JMP: begin
                case (instr[11:10])

                    // Unconditional jump
                    2'd0: begin
                        next_pc = instr[5:0];
                    end

                    // Jump if X is nonzero
                    2'd1: begin
                        if (x != 0)
                            next_pc = instr[5:0];
                    end

                    // Jump if Y is nonzero
                    2'd2: begin
                        if (y != 0)
                            next_pc = instr[5:0];
                    end

                    // Jump if selected input is high
                    2'd3: begin
                        if (pins[instr[9:6]])
                            next_pc = instr[5:0];
                    end

                endcase
            end

            default: begin
                // Normal instruction: advance to the next word.
            end

        endcase
    end

    // During pipeline startup, fetch the current PC.
    // Once instr_valid is true, fetch next_pc while executing instr.
    assign mem_raddr = instr_valid ? next_pc : pc;

    // ------------------------------------------------------------
    // Firmware-loader write port
    // ------------------------------------------------------------
    assign mem_we =
        rst_n && load_mode_sync &&
        load_strobe_rise && !load_full;

    assign mem_wbyte_addr = load_byte_addr;
    assign mem_wdata      = ui_in;

    // ------------------------------------------------------------
    // Loader and CPU
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            load_mode_meta   <= 1'b0;
            load_mode_sync   <= 1'b0;
            load_mode_prev   <= 1'b0;

            load_strobe_meta <= 1'b0;
            load_strobe_sync <= 1'b0;
            load_strobe_prev <= 1'b0;

            load_byte_addr <= 7'd0;
            load_full      <= 1'b0;
            loader_armed   <= 1'b0;

            pc          <= 6'd0;
            instr_valid <= 1'b0;

            sr         <= 8'd0;
            x          <= 8'd0;
            y          <= 8'd0;
            out_reg    <= 8'hFF;

            dly_active <= 1'b0;
            dly_cnt    <= 8'd0;

        end else begin
            // Synchronize loader inputs.
            load_mode_meta   <= load_mode_pin;
            load_mode_sync   <= load_mode_meta;
            load_mode_prev   <= load_mode_sync;

            load_strobe_meta <= load_strobe_pin;
            load_strobe_sync <= load_strobe_meta;
            load_strobe_prev <= load_strobe_sync;

            // ----------------------------------------------------
            // Firmware loading
            // ----------------------------------------------------
            if (load_mode_sync) begin
                loader_armed <= 1'b1;

                if (!load_mode_prev) begin
                    load_byte_addr <= 7'd0;
                    load_full      <= 1'b0;
                end

                if (load_strobe_rise && !load_full) begin
                    if (load_byte_addr == 7'd127)
                        load_full <= 1'b1;

                    load_byte_addr <= load_byte_addr + 1'b1;
                end

                // Stop execution and invalidate any old fetch.
                pc          <= 6'd0;
                instr_valid <= 1'b0;

                sr         <= 8'd0;
                x          <= 8'd0;
                y          <= 8'd0;
                out_reg    <= 8'hFF;

                dly_active <= 1'b0;
                dly_cnt    <= 8'd0;

            // ----------------------------------------------------
            // Execution
            // ----------------------------------------------------
            end else if (loader_armed) begin

                if (!instr_valid) begin
                    // Prime the synchronous memory with instr[0].
                    // Do not execute the previous memory output.
                    pc          <= 6'd0;
                    instr_valid <= 1'b1;

                end else begin
                    // The memory simultaneously fetches next_pc.
                    // Execute the instruction already in mem_rdata.
                    pc <= next_pc;

                    case (op)

                        OP_NOP: begin
                            // No operation.
                        end

                        OP_SET: begin
                            out_reg[instr[2:0]] <= instr[3];
                        end

                        OP_OUT: begin
                            out_reg[instr[2:0]] <=
                                instr[3] ? sr[7] : sr[0];

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
                                end
                            end else if (dly_cnt == 1) begin
                                dly_active <= 1'b0;
                            end else begin
                                dly_cnt <= dly_cnt - 1'b1;
                            end
                        end

                        OP_LDX: begin
                            x <= instr[7:0];
                        end

                        OP_LDY: begin
                            y <= instr[7:0];
                        end

                        OP_WAIT: begin
                            // next_pc holds PC while the condition
                            // is not satisfied.
                        end

                        OP_JMP: begin
                            case (instr[11:10])

                                2'd0: begin
                                    // Unconditional jump.
                                end

                                2'd1: begin
                                    x <= x - 1'b1;
                                end

                                2'd2: begin
                                    y <= y - 1'b1;
                                end

                                2'd3: begin
                                    // Conditional pin jump.
                                end

                            endcase
                        end

                        default: begin
                            // Unknown opcodes behave like NOP.
                        end

                    endcase
                end
            end
        end
    end

    assign uo_out  = out_reg;
    assign uio_out = 8'd0;
    assign uio_oe  = 8'd0;

    wire _unused = &{ena, 1'b0};

endmodule

`default_nettype wire
