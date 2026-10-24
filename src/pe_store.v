
`default_nettype none

module pe_store #(
    parameter integer AW = 6
) (
    input  wire          clk,

    // CPU read port
    input  wire          re,
    input  wire [AW-1:0] raddr,
    output reg  [15:0]   rdata,

    // Firmware-loader byte-write port
    input  wire          we,
    input  wire [AW:0]   wbyte_addr,
    input  wire [7:0]    wdata
);

    localparam integer DEPTH = (1 << AW);

    reg [15:0] mem [0:DEPTH-1];

    // Synchronous instruction read
    always @(posedge clk) begin
        if (re)
            rdata <= mem[raddr];
    end

    // Write one byte without changing the other byte
    always @(posedge clk) begin
        if (we) begin
            if (wbyte_addr[0] == 1'b0)
                mem[wbyte_addr[AW:1]][7:0] <= wdata;
            else
                mem[wbyte_addr[AW:1]][15:8] <= wdata;
        end
    end

endmodule

`default_nettype wire
