`timescale 1ns / 1ps

module bram #(parameter SIZE=512*424, LOGSIZE=18, BITS=1)
    (input wire [LOGSIZE-1:0] addr,
    input wire clk,
    input wire [BITS-1:0] din,
    output reg [BITS-1:0] dout,
    input wire we);
    // let the tools infer the right number of BRAMs
    (* ram_style = "block" *)
    reg [BITS-1:0] mem[SIZE-1:0];
    always @(posedge clk) begin
        if (we) mem[addr] <= din;
        dout <= mem[addr];
    end
endmodule

module frame_bram #(parameter SIZE=512*424, LOGSIZE=18, BITS=1)
    (input wire [LOGSIZE-1:0] wr_addr,
    input wire [LOGSIZE-1:0] rd_addr,
    input wire wr_clk,
    input wire rd_clk,
    input wire [BITS-1:0] din,
    output reg [BITS-1:0] dout,
    input wire we,
    input wire rd);
    // let the tools infer the right number of BRAMs
    (* ram_style = "block" *)
    reg [BITS-1:0] mem[SIZE-1:0];
    always @(posedge wr_clk)
        if (we) mem[wr_addr] <= din;
    always @(posedge rd_clk)
        if (rd) dout <= mem[rd_addr];
endmodule