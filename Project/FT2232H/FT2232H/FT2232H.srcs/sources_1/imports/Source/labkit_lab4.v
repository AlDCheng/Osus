`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Updated 9/29/2017 V2.0
// Create Date: 10/1/2015 V1.0
// Design Name: 
// Module Name: labkit
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module labkit(
   input CLK100MHZ,
   input[15:0] SW, 
   input BTNC, BTNU, BTNL, BTNR, BTND,
   output[3:0] VGA_R, 
   output[3:0] VGA_B, 
   output[3:0] VGA_G,
   output[7:0] JA,
//   output[7:0] JB,
   output VGA_HS, 
   output VGA_VS, 
   output LED16_B, LED16_G, LED16_R,
   output LED17_B, LED17_G, LED17_R,
   output[15:0] LED,
   output[7:0] SEG,  // segments A-G (0-6), DP (7)
   output[7:0] AN,    // Display 0-7
   output[7:0] JD,
   input[7:0] JC,
   input[1:0] JB_IN,
   output[2:0] JB_OUT
   );
   

// create 25mhz system clock
    wire clock_25mhz;
    clock_quarter_divider clockgen(.clk100_mhz(CLK100MHZ), .clock_25mhz(clock_25mhz));

//  instantiate 7-segment display;  
    wire [31:0] data;
    wire [6:0] segments;
    display_8hex display(.clk(clock_25mhz),.data(32'd0), .seg(segments), .strobe(AN));
//    display_8hex display(.clk(clock_25mhz),.data({24'd0,JC}), .seg(segments), .strobe(AN));    
    assign SEG[6:0] = segments;
    assign SEG[7] = 1'b1;
    
//////////////////////////////////////////////////////////////////////////////////
    //assign JB[1] = JD_IN[0];
    
//    wire[1:0] mem_state;
//    assign JA[2:1] = mem_state;

//////////////////////////////////////////////////////////////////////////////////
//
    assign LED[15:2] = SW[15:2];     
//    assign JA[7:0] = 8'b0;
    assign data = {28'h0123456, SW[3:0]};   // display 0123456 + SW
    assign LED16_R = BTNL;                  // left button -> red led
    assign LED16_G = BTNC;                  // center button -> green led
    assign LED16_B = BTNR;                  // right button -> blue led
    assign LED17_R = BTNL;
    assign LED17_G = BTNC;
    assign LED17_B = BTNR; 
    
    wire BTNC_clean, BTNU_clean, BTND_clean, BTNL_clean, BTNR_clean;
    debounce debounce_btnc(.reset(global_reset), .clock(clock_25mhz), .noisy(BTNC), .clean(BTNC_clean));
    debounce debounce_btnu(.reset(global_reset), .clock(clock_25mhz), .noisy(BTNU), .clean(BTNU_clean));
    debounce debounce_btnd(.reset(global_reset), .clock(clock_25mhz), .noisy(BTND), .clean(BTND_clean));
    debounce debounce_btnl(.reset(global_reset), .clock(clock_25mhz), .noisy(BTNL), .clean(BTNL_clean));
    debounce debounce_btnr(.reset(global_reset), .clock(clock_25mhz), .noisy(BTNR), .clean(BTNR_clean));
    
    parameter WIDTH = 512;                  // Width of image frame
    parameter HEIGHT = 424;                 // Depth of image frame
    parameter LOGSIZE = 18;                 // smallet LOGSIZE s.t. 2^LOGSIZE > WIDTH*HEIGHT

    wire WR, RD, OE;                        // Write/Read/Output_Enable from FT2232H
    wire img_ready;                         // Signals when binarized frame is ready in RAM

    assign JB_OUT[0] = WR;
    assign JB_OUT[1] = RD;
    assign JB_OUT[2] = OE;
    
    wire [7:0] ft_data;
    
    FT2232_Data PC_Data(.clock_60_mhz(JB_IN[0]), .RXF(JB_IN[1]), .WR(WR), .RD(RD), .OE(OE));

    wire pixel;                             // pixel value ([0,1]) at vga_index, or internal index while updating
    wire [LOGSIZE-1:0] vga_index;           // index to read binary vals from RAM.
    wire [LOGSIZE-1:0] index_out, index_bin;
    wire [LOGSIZE-1:0] index_img_proc;

    assign vga_index = (vcount << 9)+hcount; // propagated from VGA module
    ram_control #(.WIDTH(WIDTH), .HEIGHT(HEIGHT), .LOGSIZE(LOGSIZE))
                stream_handle(.reset(BTNU_clean), .clk(CLOCK100MHZ), .clock_60_mhz(JB_IN[0]), .write_btn(BTND_clean),
                              .RXF(~JB_IN[1]), .OE(~OE), .RD(~RD), .data(JC), .ready(img_ready), .read_index(vga_index)//, .bin_out(pixel)
                              ,.wr_bin_mem(wr_bin_mem), .data_bin(wr_bin_val)
//                              ,.state_out(JD[2:1]), .start(JD[0]), .data_out(JA), .debug(JD[3]), .debug_write(JD[4]), .debug_bin(JD[6:5]));
//                              ,.combined({JA,JD}));
                              ,.index_out(index_out), .inner_write(JD[7]),
                                .low_bound(SW[4:0]), .high_bound(SW[9:5]));
                              
//    assign {JA,JD} = index_out[15:0];
//    assign {JA[6:0],JD} = index_out[14:0];
    assign JA = index_out[7:0];
    assign JD[0] = img_ready;
    assign LED[0] = img_ready;
    assign LED[1] = BTND_clean;
    
    wire wr_bin_mem, wr_bin_val;
    frame_bram #(.SIZE(WIDTH*HEIGHT), .LOGSIZE(LOGSIZE), .BITS(1'b1))
                binary_bram(.wr_addr(index_out),.rd_addr(vga_index),.wr_clk(JB_IN[0]),.rd_clk(clock_30mhz),
                            .we(wr_bin_mem),.rd(1'b1),.din(wr_bin_val),.dout(pixel));
    
    wire clock_30mhz;
    clock_half_divider stupid_clock(JB_IN[0], clock_30mhz);
    
    wire [9:0] hcount;
    wire [9:0] vcount;
    wire hsync, vsync, at_display_area;
    vga #(.HSIZE(WIDTH), .VSIZE(HEIGHT))
        vga1(.vga_clock(clock_30mhz),.hcount(hcount),.vcount(vcount),
          .hsync(hsync),.vsync(vsync),.at_display_area(at_display_area));

    wire[7:0] pixel_bin;
    assign pixel_bin = (at_display_area && pixel)? 255 : 0;     // Map [1,0] to [255,0]
    
    assign VGA_R = pixel_bin;
    assign VGA_G = pixel_bin;
    assign VGA_B = pixel_bin;
    assign VGA_HS = ~hsync;
    assign VGA_VS = ~vsync;
endmodule

module clock_quarter_divider(input clk100_mhz, output reg clock_25mhz = 0);
    reg counter = 0;

    // VERY BAD VERILOG
    // VERY BAD VERILOG
    // VERY BAD VERILOG
    // But it's a quick and dirty way to create a 25Mhz clock
    // Please use the IP Clock Wizard under FPGA Features/Clocking
    //
    // For 1 Hz pulse, it's okay to use a counter to create the pulse as in
    // assign onehz = (counter == 100_000_000); 
    // be sure to have the right number of bits.

    always @(posedge clk100_mhz) begin
        counter <= counter + 1;
        if (counter == 0) begin
            clock_25mhz <= ~clock_25mhz;
        end
    end
endmodule

module clock_half_divider(input clk60_mhz, output reg clock_30mhz = 0);

    // VERY BAD VERILOG
    // VERY BAD VERILOG
    // VERY BAD VERILOG
    // But it's a quick and dirty way to create a 25Mhz clock
    // Please use the IP Clock Wizard under FPGA Features/Clocking
    //
    // For 1 Hz pulse, it's okay to use a counter to create the pulse as in
    // assign onehz = (counter == 100_000_000); 
    // be sure to have the right number of bits.

    always @(posedge clk60_mhz) 
            clock_30mhz <= ~clock_30mhz;
    
endmodule

module vga #(parameter HSIZE=512,
             parameter VSIZE=424)
            (input vga_clock,
            output reg [9:0] hcount = 0,    // pixel number on current line
            output reg [9:0] vcount = 0,    // line number
            output reg vsync, hsync,
            output at_display_area);

    // Comments applies to XVGA 1024x768, left in for reference
    // horizontal: 1344 pixels total
    // display 1024 pixels per line
    reg hblank,vblank;
    wire hsyncon,hsyncoff,hreset,hblankon;
    assign hblankon = (hcount == 639);    // active H  1023
    assign hsyncon = (hcount == 655);     // active H + FP 1047
    assign hsyncoff = (hcount == 751);    // active H + fp + sync  1183
    assign hreset = (hcount == 799);      // active H + fp + sync + bp 1343

    // vertical: 806 lines total
    // display 768 lines
    wire vsyncon,vsyncoff,vreset,vblankon;
    assign vblankon = hreset & (vcount == 479);    // active V   767
    assign vsyncon = hreset & (vcount ==490 );     // active V + fp   776
    assign vsyncoff = hreset & (vcount == 492);    // active V + fp + sync  783
    assign vreset = hreset & (vcount == 523);      // active V + fp + sync + bp 805

    // sync and blanking
    wire next_hblank,next_vblank;
    assign next_hblank = hreset ? 0 : hblankon ? 1 : hblank;
    assign next_vblank = vreset ? 0 : vblankon ? 1 : vblank;
    always @(posedge vga_clock) begin
        hcount <= hreset ? 0 : hcount + 1;
        hblank <= next_hblank;
        hsync <= hsyncon ? 0 : hsyncoff ? 1 : hsync;  // active low

        vcount <= hreset ? (vreset ? 0 : vcount + 1) : vcount;
        vblank <= next_vblank;
        vsync <= vsyncon ? 0 : vsyncoff ? 1 : vsync;  // active low

    end

    assign at_display_area = ((hcount >= 0) && (hcount < 512) && (vcount >= 0) && (vcount < 424));

endmodule

//module gate_vga #(parameter WIDTH=512, HEIGHT=424)
//    (input clk,
//    input hcount,
//    input vcount,
    
