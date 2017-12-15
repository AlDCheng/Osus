module image_process #(parameter HSIZE = 512, parameter VSIZE=424)
        (input clock,
        input reset,
        input clock_30mhz,
        input frame_available, // Indicates that new data is available in frame, should be a pulse
        input display_raw,
        input blank_frame,
        input write_top_left, write_bottom_right,
        input pixel_val,  
        output [17:0] bin_index,
        output [7:0] pixel, // current pixel (according to hcount_disp, vcount_disp)
        output hsync, vsync,
        output in_blob_box, in_frame_box, // whether the current pixel is on the edge of the box around the blob
        output [9:0] touch_h, touch_v, // coordinates of user touch
        output reg [9:0] top_left_h, top_left_v, bottom_right_h, bottom_right_v,
        output reg touch, touch_ready // whether there is a valid touch
        );
      
       
    // Create 25mhz clock - used for VGA display (the monitor complains if you use a faster clock)
    wire clock_25mhz;
    clock_quarter_divider clockgen(.clk100_mhz(clock), .clock_25mhz(clock_25mhz));
   
    // These are the possible states in image processing
    parameter IDLE = 0;
    parameter GET_IMAGE = 1;
    parameter ERODE = 2;
    parameter DILATE = 3;
    parameter BLOB = 4;
    parameter PREP_DILATE = 5;
    parameter WRITE_FINAL = 6;
    reg [3:0] state;
   
    // These parameters can be changed to control how the image processing works
    parameter BLOB_DIST = 2;
    parameter BLOB_MIN_SIZE = 8; // minimun size (in either direction) a blob must be to be considered a finger touch
    parameter BLOB_MAX_SIZE = 12; // maximum size (in either direction) a blob must be to be considered a finger touch
    parameter MAX_BLOB_COUNT = 1; // the maximum number of blobs that we will look at in an attempt to find a finger touch (takes 1 cycle per pixel per blob)
   
    wire [9:0] hcount, vcount;
    wire [9:0] hcount_disp, vcount_disp; // these are the hcount and vcount for display on the monitor, as opposed to for our processing which is faster
    wire at_display_area;
    wire pixel_on; // whether the current pixel should be on
    reg [9:0] start_h, end_h, start_v, end_v; // the bounds of the box around current blob
    wire raw; // whether to display raw data
    reg frame [HSIZE-1:0][2:0];
    reg eroded [HSIZE-1:0] [2:0]; // the eroded version of the image
    reg processed [HSIZE-1:0] [VSIZE-1:0]; // the image after it has been eroded then dilated
    reg next_row [HSIZE-1:0];
    reg begin_processing; // is triggered when a new frame comes in
    reg started_blob; // whether we have found a blob yet
    reg [5:0] blob_count; // the number of blobs perviously looked at (that didn't meet criteria)
//    wire reset;
//    reg [9:0] image_addr;
    wire mem_out;
    reg last_frame_available;
    reg new_frame;
   
    // Make an hcount and vcount for displaying and also a faster one for cycling through pixels in image processing
    vga #(.HSIZE(HSIZE), .VSIZE(VSIZE)) vga_display(.vga_clock(clock_25mhz),.hcount(hcount_disp),.vcount(vcount_disp),.hsync(hsync),.vsync(vsync),.at_display_area(at_display_area));
    //vga #(.HSIZE(HSIZE), .VSIZE(VSIZE)) vga_fast(.vga_clock(clock),.hcount(hcount),.vcount(vcount),.hsync(),.vsync(),.at_display_area());
   
    reg wr_erosion;
    wire erosion_pixel;
    wire erosion_out;
    bram #(.SIZE(HSIZE*VSIZE), .LOGSIZE(18), .BITS(1))
            erosion_bram(.addr(bin_index),.clk(clock),.we(wr_erosion),.din(erosion_pixel),.dout(erosion_out));
   
    reg wr_dilation;
    wire dilation_pixel;
    wire process_out;
   
    reg reset_count, hard_reset;
   
    reg wr_blob;
    reg [9:0] h_process, v_process;
    wire process_pixel;
    wire [17:0] process_bram_index;
    wire [17:0] process_index;
    wire [17:0] disp_index;
   
    wire write_process;
    assign write_process = wr_dilation || wr_blob;
   
    assign process_index = (v_process * HSIZE)+h_process;
    assign process_bram_index = (wr_blob) ? process_index : bin_index;
    assign process_pixel = (write_process) ? dilation_pixel : 0;
   
    frame_bram #(.SIZE(HSIZE*VSIZE), .LOGSIZE(18), .BITS(1'b1))
                process_bram(.wr_addr(process_bram_index),.rd_addr(bin_index),.wr_clk(clock),.rd_clk(clock),
                            .we(write_process),.rd(1'b1),.din(process_pixel),.dout(process_out));
   
    wire write_final;
    assign write_final = (state == WRITE_FINAL);
    frame_bram #(.SIZE(HSIZE*VSIZE), .LOGSIZE(18), .BITS(1'b1))
                final_bram(.wr_addr(bin_index),.rd_addr(disp_index),.wr_clk(clock),.rd_clk(clock_25mhz),
                            .we(write_final),.rd(1'b1),.din(process_out),.dout(pixel_on));
//    bram #(.SIZE(HSIZE*VSIZE), .LOGSIZE(18), .BITS(1))
//                        dilation_bram(.addr(bin_index || process_index),.clk(clock),.we(wr_dilation || wr_blob),.din(process_pixel),.dout(dilation_out));
   
   
   
//    mybram stored_image (.addr(image_addr),.clk(clock),.we(),.din(),.dout(mem_out));
    frame_index_control #(.HSIZE(HSIZE),.VSIZE(VSIZE)) frame_division(.reset(reset || hard_reset), .reset_count(reset_count), .clk(clock), .vcount(vcount), .hcount(hcount));
   
    assign bin_index = (vcount * HSIZE)+hcount;
    assign disp_index = (vcount_disp < VSIZE & hcount_disp < HSIZE)?(vcount_disp * HSIZE)+hcount_disp : 0;
   
    assign erosion_pixel = ~blank_frame &
                            frame[hcount-1][0] &
                            frame[hcount-1][1] &
                            frame[hcount-1][2] &
                            frame[hcount]  [0] &
                            frame[hcount]  [2] &
                            frame[hcount+1][0] &
                            frame[hcount+1][1] &
                            frame[hcount+1][2];
                           
    assign dilation_pixel = eroded[hcount-1][0] |
                            eroded[hcount-1][1] |
                            eroded[hcount-1][2] |
                            eroded[hcount]  [0] |
                            eroded[hcount]  [2] |
                            eroded[hcount+1][0] |
                            eroded[hcount+1][1] |
                            eroded[hcount+1][2];
    integer index;
   
    always @(posedge clock) begin
        if (reset) begin
            state <= 0;
            wr_blob <= 0;
        end
       
        last_frame_available <= frame_available;
        new_frame <= (~last_frame_available & frame_available);
   
        if (touch_ready & touch) begin
            if (write_top_left) begin
                top_left_h <= 0; // touch_h;
                top_left_v <= 0; // touch_v;
            end
            if (write_bottom_right) begin
                bottom_right_h <= 30; // touch_h;
                bottom_right_v <= 20; //touch_v;
            end
        end
       
        case (state)
            IDLE: begin
                touch_ready <= 0;
                if (new_frame) begin
                    state <= GET_IMAGE;
                    started_blob <= 0;
                    start_h <= 0;
                    end_h <= 0;
                    start_v <= 0;
                    end_v <= 0;
                    blob_count <= 0;
                    touch <= 0;
                   
                    reset_count <= 1;
                    wr_erosion <= 0;
                    hard_reset <= 1;
                end
                else begin
                    reset_count <= 0;
                    wr_erosion <= 0;
                    hard_reset <= 0;
                end
            end
            GET_IMAGE: begin
                hard_reset <= 0;
                if ((hcount == HSIZE-2) && (vcount == 2)) begin
                    state <= ERODE;
                    reset_count <= 1;
                    wr_erosion <= 1;
                end
                else begin
                    reset_count <= 0;
                    wr_erosion <= 0;
                end
                frame[hcount][vcount] <= pixel_val;
            end
            ERODE: begin
                reset_count <= 0;
               
                if ((hcount == HSIZE-2) && (vcount == VSIZE-3)) begin
                    state <= PREP_DILATE;
                    hard_reset <= 1;
                    wr_erosion <= 0;
                end
                else if (hcount == HSIZE-2) begin     
                    for (index=0; index<HSIZE-1; index=index+1) begin
                        frame[index][0] <= frame[index][1];
                        frame[index][1] <= frame[index][2];
                        frame[index][2] <= next_row[index];
                    end
                end
                next_row[hcount] <= pixel_val;
            end
            PREP_DILATE: begin
                hard_reset <= 0;
                if ((hcount == HSIZE-2) && (vcount == 2)) begin
                    state <= DILATE;
                    reset_count <= 1;
                    wr_dilation <= 1;
                end
                eroded[hcount][vcount] <= erosion_out;
            end
            DILATE: begin
                reset_count <= 0;
                           
                if ((hcount == HSIZE-2) && (vcount == VSIZE-3)) begin
                    state <= BLOB;
                    reset_count <= 1;
                    wr_dilation <= 0;
                end
                else if (hcount == HSIZE-2) begin     
                    for (index=0; index<HSIZE-1; index=index+1) begin
                        eroded[index][0] <= eroded[index][1];
                        eroded[index][1] <= eroded[index][2];
                        eroded[index][2] <= next_row[index];
                    end
                end
                next_row[hcount] <= erosion_out;
               
            end
            BLOB: begin
                reset_count <= 0;
                wr_blob <= 1;
               
                if (hcount == 1 & vcount == 1) begin
                    started_blob <= 0;
                    if (end_h - start_h >= BLOB_MIN_SIZE & end_h - start_h <= BLOB_MAX_SIZE
                      & end_v - start_v >= BLOB_MIN_SIZE & end_v - start_v <= BLOB_MAX_SIZE) begin
                      // blob is valid
                        state <= IDLE;
                        touch <= 1;
                        touch_ready <= 1;
                    end
                    else begin
                        touch <= 0;
                        if (blob_count < MAX_BLOB_COUNT) blob_count <= blob_count + 1;
                        else begin
                            touch <= 0;
                            touch_ready <= 1;
                            wr_blob <= 0;
                            state <= WRITE_FINAL;
                        end
                    end
                end
                else if (~started_blob & process_out) begin
                    start_h <= hcount;
                    end_h <= hcount;
                    start_v <= vcount;
                    end_v <= vcount;
                    started_blob <= 1;
                    h_process <= hcount;
                    v_process <= vcount;
                end
                else if (started_blob & process_out & hcount >= start_h - BLOB_DIST &
                                                            hcount <= end_h + BLOB_DIST &
                                                            vcount >= start_v - BLOB_DIST &
                                                            vcount <= end_v + BLOB_DIST) begin
                    // pixel is part of blob
                    if (hcount < start_h) start_h <= hcount;
                    if (hcount > end_h) end_h <= hcount;
                    if (vcount < start_v) start_v <= vcount;
                    if (vcount > end_v) end_v <= vcount;   
                    h_process <= hcount;
                    v_process <= vcount;
                end
                else begin
                    wr_blob <= 0;
                end
            end
            WRITE_FINAL: begin
                reset_count <= 0;
                if ((hcount == HSIZE-2) && (vcount == VSIZE-3)) state <= IDLE;
               
            end
        endcase
    end
   
    assign in_blob_box = (hcount_disp == start_h & vcount_disp >= start_v & vcount_disp <= end_v) |
                 (hcount_disp == end_h & vcount_disp >= start_v & vcount_disp <= end_v) |
                 (vcount_disp == start_v & hcount_disp >= start_h & hcount_disp <= end_h) |
                 (vcount_disp == end_v & hcount_disp >= start_h & hcount_disp <= end_h);
   
    assign in_frame_box = (hcount_disp == top_left_h & vcount_disp >= top_left_v & vcount_disp <= bottom_right_v) |
                                  (hcount_disp == bottom_right_h & vcount_disp >= top_left_v & vcount_disp <= bottom_right_v) |
                                  (vcount_disp == top_left_v & hcount_disp >= top_left_h & hcount_disp <= bottom_right_h) |
                                  (vcount_disp == bottom_right_v & hcount_disp >= top_left_h & hcount_disp <= bottom_right_h);
                
    //assign pixel_on = display_raw? (~blank_frame & frame[hcount_disp][vcount_disp]) :
    //                      (processed[hcount_disp][vcount_disp]);
               
    assign pixel = at_display_area? (pixel_on? 255 :0) : 0;
    assign touch_h = start_h + (end_h - start_h)/2;
    assign touch_v = start_v + (end_v - start_v)/2;
       
endmodule

module frame_index_control #(parameter HSIZE=512, VSIZE=424)
    (input reset,
    input reset_count,
    input clk,
    input pause_v,
    output reg [9:0] vcount,
    output reg [9:0] hcount);
   
    always@(posedge clk) begin
        if (reset) begin
             hcount <= 1;
             vcount <= 0;
         end
        else if (reset_count) begin
            hcount <= 1;
            vcount <= 1;
        end
        else begin
            if ((hcount >= HSIZE-2) && (vcount >= VSIZE-3)) begin
                hcount <= 1;
                vcount <= 1;
            end
            else if (hcount >= HSIZE-2) begin
                hcount <= 1;
                vcount <= vcount + 1;
            end
            else hcount <= hcount + 1;
        end
    end
endmodule

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

module vga #(parameter HSIZE= 512,
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
   

    assign at_display_area = ((hcount >= 0) && (hcount < HSIZE) && (vcount >= 0) && (vcount < VSIZE));

endmodule
