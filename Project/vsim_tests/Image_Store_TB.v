`timescale 1ns / 1ps

module image_store_tb;

    reg clock, clock_60_mhz;
    reg reset, btn;
    reg RXF;
    reg [7:0] JC;
    wire WR, RD, OE, ready;
    wire bin_frame;

    parameter OP_SIZE = 64;
    parameter DATA_SIZE = 256;

    reg [OP_SIZE-1:0] opening = 64'h00000000_DD_CC_BB_AA;

    reg [DATA_SIZE-1:0] input_data, input_data2;

    reg [(OP_SIZE+DATA_SIZE)-1:0] data;
    integer i;      // required for "for" loop

    reg [3:0] vga_index = 0;

    FT2232_Data PC_Data(.clock_60_mhz(clock_60_mhz), .RXF(RXF), .WR(WR), .RD(RD), .OE(OE));
    ram_control #(.WIDTH(4), .HEIGHT(4), .LOGSIZE(4))
            stream_handle(.reset(reset), .clk(clock), .clock_60_mhz(clock_60_mhz), .write_btn(btn), .read_index(vga_index),
                              .OE(~OE), .RD(~RD), .data(JC[7:0]), .ready(ready), .bin_out(bin_frame));

    // note: sync ready to pixel vga

    always begin   // system clock
        #5 clock = !clock;
    end

    always begin   // system clock
        #8 clock_60_mhz = !clock_60_mhz;
    end

    initial begin
        input_data[255:192] = 64'h03E8_07D0_0BB8_0FA0;        // 1000, 2000, 3000, 4000
        input_data[191:128] = 64'h115C_0D05_08AE_0457;        // 4444, 3333, 2222, 1111
        input_data[127:64]  = 64'h0351_0352_0353_0354;        //  849,  850,  851,  852
        input_data[63:0]    = 64'h0001_0002_0003_0000;        //    1,    2,    3,    0

        input_data2[255:192] = 64'h03E3_07CD_0BA9_0F93;        //  995, 1997, 2985, 4000    1111
        input_data2[191:128] = 64'h115b_0D06_0BB8_03F2;        // 4443, 3334, 3000, 1010    0000
        input_data2[127:64]  = 64'h0334_0334_0334_0334;        //  820,  820,  820,  820    1100
        input_data2[63:0]    = 64'h0000_0000_0000_0000;        //    0,    0,    0,    0    000 0

        clock = 0;
        clock_60_mhz = 0;
        // data[319:256] = opening;
        // data[255:0] = input_data;
        data = {opening, input_data};

        reset = 1;
        btn = 1;
        RXF = 1;
        #40 reset = 0; RXF=0;
        wait (RD);
        for (i=0; i<50; i=i+1)
            begin
            JC = data[(OP_SIZE+DATA_SIZE)-1:(OP_SIZE+DATA_SIZE)-8];
            // at each clock, left shift the data
            // note syntax for test bench "for" loop - no "always"
            // note the blocking assignment (immediate)
            @(posedge clock_60_mhz) data <= {data[(OP_SIZE+DATA_SIZE)-9:0],8'd0};
        end
        #100 RXF=1; btn = 0;
        #100 RXF=0;

        data = {opening, input_data2};

        wait (RD);
        for (i=0; i<50; i=i+1)
            begin
            JC = data[(OP_SIZE+DATA_SIZE)-1:(OP_SIZE+DATA_SIZE)-8];
            // at each clock, left shift the data
            // note syntax for test bench "for" loop - no "always"
            // note the blocking assignment (immediate)
            @(posedge clock_60_mhz) data <= {data[(OP_SIZE+DATA_SIZE)-9:0],8'd0};
        end
    end

    always@(posedge clock) begin
        vga_index <= vga_index + 1;
    end

endmodule;
