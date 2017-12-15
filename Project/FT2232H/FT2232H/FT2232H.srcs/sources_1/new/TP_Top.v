module TP_Top(
   input CLK100MHZ,
   input[15:0] SW, 
   input BTNC, BTNU, BTNL, BTNR, BTND,
   output[3:0] VGA_R, 
   output[3:0] VGA_B, 
   output[3:0] VGA_G,
   output[7:0] JA,
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
    assign data = (SW[15]) ? {22'd0, SW[9:0]} : {11'd0, SW[9:5], 11'd0, SW[4:0]};
    display_8hex display(.clk(clock_25mhz),.data(data), .seg(segments), .strobe(AN));
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
//    assign data = {28'h0123456, SW[3:0]};   // display 0123456 + SW
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
    
    wire [4:0] threshold_high_bound, threshold_low_bound;
    wire [4:0] blob_min_size, blob_max_size;
    wire [9:0] edge_to_ignore_up_h;
    wire [9:0] edge_to_ignore_up_v;
    wire [9:0] edge_to_ignore_bot_h;
    wire [9:0] edge_to_ignore_bot_v;
    
    switch_control switch_parameters(.clk(clock_25mhz), .SW(SW), .BTNC_clean(BTNC_clean), .BTNL_clean(BTNL_clean), .BTNR_clean(BTNR_clean),
                                     .threshold_high_bound(threshold_high_bound),
                                     .threshold_low_bound(threshold_low_bound),
                                     .blob_min_size(blob_min_size),
                                     .blob_max_size(blob_max_size),
                                     .edge_to_ignore_up_h(edge_to_ignore_up_h),
                                     .edge_to_ignore_up_v(edge_to_ignore_up_v),
                                     .edge_to_ignore_bot_h(edge_to_ignore_bot_h),
                                     .edge_to_ignore_bot_v(edge_to_ignore_bot_v));
                                     
    parameter WIDTH = 512;                  // Width of image frame
    parameter HEIGHT = 424;                 // Depth of image frame
    parameter LOGSIZE = 18;                 // smallet LOGSIZE s.t. 2^LOGSIZE > WIDTH*HEIGHT

    wire WR, RD, OE;                        // Write/Read/Output_Enable from FT2232H
    wire img_ready;                         // Signals when binarized frame is ready in RAM

    assign JB_OUT[0] = WR;
    assign JB_OUT[1] = RD;
    assign JB_OUT[2] = OE;
    
    wire [7:0] ft_data;
    
//    assign display_raw = ~SW[13];
//    assign blank_frame = SW[14];
    assign write_top_left = SW[12];         
    assign write_bottom_right = SW[11];
    
    FT2232_Data PC_Data(.clock_60_mhz(JB_IN[0]), .RXF(JB_IN[1]), .WR(WR), .RD(RD), .OE(OE));

    wire pixel;                             // pixel value ([0,1]) at vga_index, or internal index while updating
//    wire [LOGSIZE-1:0] vga_index;           // index to read binary vals from RAM.
    wire [LOGSIZE-1:0] index_out, index_bin;
    wire [LOGSIZE-1:0] index_img_proc;

//    assign vga_index = (vcount << 9)+hcount; // propagated from VGA module
    ram_control #(.WIDTH(WIDTH), .HEIGHT(HEIGHT), .LOGSIZE(LOGSIZE))
                stream_handle(.reset(BTNU_clean), .clk(CLOCK100MHZ), .clock_60_mhz(JB_IN[0]), .write_btn(BTND_clean),
                              .RXF(~JB_IN[1]), .OE(~OE), .RD(~RD), .data(JC), .ready(img_ready), .read_index(vga_index)//, .bin_out(pixel)
                              ,.wr_bin_mem(wr_bin_mem), .data_bin(wr_bin_val)
//                              ,.state_out(JD[2:1]), .start(JD[0]), .data_out(JA), .debug(JD[3]), .debug_write(JD[4]), .debug_bin(JD[6:5]));
//                              ,.combined({JA,JD}));
                              ,.index_out(index_out), .inner_write(JD[7]),
                                .low_bound(threshold_high_bound), .high_bound(threshold_low_bound));
                              
    assign JD[0] = img_ready;
    assign LED[0] = img_ready;
    assign LED[1] = BTND_clean;
    
    wire wr_bin_mem, wr_bin_val;

    frame_bram #(.SIZE(WIDTH*HEIGHT), .LOGSIZE(LOGSIZE), .BITS(1'b1))
                binary_bram(.wr_addr(index_out),.rd_addr(index_img_proc),.wr_clk(JB_IN[0]),.rd_clk(CLK100MHZ),
                            .we(wr_bin_mem),.rd(1'b1),.din(wr_bin_val),.dout(pixel));
                            
    wire [17:0] index_img_input;
    wire pixel_input;
    assign index_img_input = (vcount_in << 9)+hcount_in;
    frame_bram #(.SIZE(WIDTH*HEIGHT), .LOGSIZE(LOGSIZE), .BITS(1'b1))
                binary_input_bram(.wr_addr(index_out),.rd_addr(index_img_input),.wr_clk(JB_IN[0]),.rd_clk(clock_25mhz),
                                  .we(wr_bin_mem),.rd(1'b1),.din(wr_bin_val),.dout(pixel_input));
    wire[7:0] pixel_bin_input;
    assign pixel_bin_input = (at_display_area_in && pixel_input)? 255 : 0;     // Map [1,0] to [255,0]
    wire [9:0] hcount_in, vcount_in;
    wire at_display_area_in;
    vga #(.HSIZE(WIDTH), .VSIZE(HEIGHT)) vga_input_display(.vga_clock(clock_25mhz),.hcount(hcount_in),.vcount(vcount_in), .at_display_area(at_display_area_in));
                            
    assign JD[6] = pixel;
    
    wire [7:0] pixel_out;
    wire [9:0] touch_h; // y coordinate of user touch
    wire [9:0] touch_v; // x coordinate of user touch
    wire touch; // whether there is a valid touch
    wire [9:0] top_left_x, top_left_y, bottom_right_x, bottom_right_y;
    wire touch_ready;
    wire write_top_left, write_bottom_right;
    wire in_blob_box, in_frame_box; // whether the current pixel is on the edge of the box around the blob
    image_process #(.HSIZE(WIDTH),.VSIZE(HEIGHT)) image_process_1
                   (.clock(CLK100MHZ), .clock_30mhz(clock_30_mhz), .frame_available(img_ready),.display_raw(display_raw), .blank_frame(blank_frame),
                    .bin_index(index_img_proc),.pixel(pixel_out), .pixel_val(pixel),
                    .blob_min_size(blob_min_size), .blob_max_size(blob_max_size),
                    .edge_to_ignore_up_h(edge_to_ignore_up_h), .edge_to_ignore_up_v(edge_to_ignore_up_v),
                    .edge_to_ignore_bot_h(edge_to_ignore_bot_h), .edge_to_ignore_bot_v(edge_to_ignore_bot_v),
                    .hsync(hsync),.vsync(vsync),.in_blob_box(in_blob_box),.in_frame_box(in_frame_box),
                    .write_top_left(write_top_left), .write_bottom_right(write_bottom_right),
                    .touch_h(touch_h),.touch_v(touch_v),.touch(touch),.touch_ready(touch_ready),
                    .top_left_h(top_left_x), .top_left_v(top_left_y), .bottom_right_h(bottom_right_x), .bottom_right_v(bottom_right_y));
//                    .state_out(JD[3:1]), .erosion_debug(JD[4]), .dilation_debug(JD[5]));
                    
    wire [7:0] pixel_vga;
    assign pixel_vga = SW[10] ? pixel_bin_input : pixel_out;
    assign VGA_R = in_blob_box? 255 : pixel_vga;
    assign VGA_G = in_frame_box? 255 : in_blob_box? 0 : pixel_vga;
    assign VGA_B = in_frame_box? 0 : pixel_vga;
    assign VGA_HS = ~hsync;
    assign VGA_VS = ~vsync;
    
    wire clock_30mhz;
    clock_half_divider stupid_clock(JB_IN[0], clock_30mhz);
    
    wire is_press, is_release, is_move, is_scroll;
    wire send_mouse_enable;
    // Sends the touch location to the Teensy over serial:
    send_mouse send_mouse1(.x(touch_h), .y(touch_v), .is_press(is_press), .is_release(is_release), .is_move(is_move),
                           .top_left_x(top_left_x), .top_left_y(top_left_y), .bottom_right_x(bottom_right_x), .bottom_right_y(bottom_right_y),
                           .touch(touch), .reset(BTNU_clean), .clock(CLK100MHZ), .send_enable(send_mouse_enable),
                           .pmod(JA));
    
    gesture_detect gesture_1(.clock(CLK100MHZ), .touch_h(touch_h), .touch_v(touch_v), .touch(touch), .touch_ready(touch_ready),
                              .is_press(is_press), .is_release(is_release), .is_move(is_move), .is_scroll(is_scroll),
                              .send_mouse_enable(send_mouse_enable));
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

module switch_control(
    input clk,
    input [15:0] SW,
    input BTNC_clean,
    input BTNL_clean,
    input BTNR_clean,
    output reg [4:0] threshold_high_bound,
    output reg [4:0] threshold_low_bound,
    output reg [4:0] blob_min_size,
    output reg [4:0] blob_max_size,
    output reg [9:0] edge_to_ignore_up_h,
    output reg [9:0] edge_to_ignore_up_v,
    output reg [9:0] edge_to_ignore_bot_h,
    output reg [9:0] edge_to_ignore_bot_v);
    
    always @(posedge clk)  begin
        if (BTNC_clean) begin
            threshold_high_bound <= SW[4:0];
            threshold_low_bound <= SW[9:5];
        end
        if (BTNL_clean) begin
            blob_min_size <= SW[4:0];
            blob_max_size <= SW[9:5];
        end
        if (BTNR_clean) begin
            case(SW[14:13])
                2'b00: edge_to_ignore_up_h <= SW[9:0];
                2'b01: edge_to_ignore_up_v <= SW[9:0];
                2'b10: edge_to_ignore_bot_h <= SW[9:0];
                2'b11: edge_to_ignore_bot_v <= SW[9:0];
                default: edge_to_ignore_up_h <= SW[9:0];
            endcase
        end
    end
endmodule
