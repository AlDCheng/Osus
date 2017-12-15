module image_process #(parameter HSIZE = 512, parameter VSIZE=424)
        (input clock,
        input reset,
        input clock_30mhz,
        input frame_available, // Indicates that new data is available in frame, should be a pulse
        input display_raw,
        input blank_frame,
        input write_top_left, write_bottom_right,
        input pixel_val, 
        input [4:0] blob_min_size, blob_max_size,
        input [9:0] edge_to_ignore_up_h,
        input [9:0] edge_to_ignore_up_v,
        input [9:0] edge_to_ignore_bot_h,
        input [9:0] edge_to_ignore_bot_v,
        output [17:0] bin_index,
        output [7:0] pixel, // current pixel (according to hcount_disp, vcount_disp)
        output hsync, vsync,
        output in_blob_box, in_frame_box, // whether the current pixel is on the edge of the box around the blob
        output [9:0] touch_h, touch_v, // coordinates of user touch
        output /*reg*/ [9:0] top_left_h, top_left_v, bottom_right_h, bottom_right_v,
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
    parameter BLOB_DIST = 3;
    parameter MAX_BLOB_COUNT = 4; // the maximum number of blobs that we will look at in an attempt to find a finger touch (takes 1 cycle per pixel per blob)
   
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
    assign write_process = wr_dilation | wr_erosion | wr_blob;
   
    assign process_index = (v_process * HSIZE)+h_process;
    assign process_bram_index = bin_index; //(wr_blob) ? process_index : bin_index;
    assign process_pixel = (wr_blob) ? 0 : dilation_pixel;
   
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
   
    assign bin_index = (vcount * HSIZE) + hcount;
    assign disp_index = (vcount_disp * HSIZE) + hcount_disp;
   
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
                           
   
    wire look_for_blob;
    assign look_for_blob = (hcount > edge_to_ignore_up_h
                          & hcount < HSIZE - edge_to_ignore_bot_h
                          & vcount > edge_to_ignore_up_v
                          & vcount < VSIZE - edge_to_ignore_bot_v);
   
    integer index;
    assign top_left_h = edge_to_ignore_up_h;
    assign top_left_v = edge_to_ignore_up_v;
    assign bottom_right_h = (HSIZE - edge_to_ignore_bot_h);
    assign bottom_right_v = (VSIZE - edge_to_ignore_bot_v);

   
    always @(posedge clock) begin
//        if (reset) begin
//            state <= 0;
//            wr_blob <= 0;
//        end
       
        last_frame_available <= frame_available;
        new_frame <= (~last_frame_available & frame_available);
   
//        if (touch_ready & touch) begin
//            if (write_top_left) begin
//                top_left_h <= edge_to_ignore; // touch_h;
//                top_left_v <= edge_to_ignore; // touch_v;
//            end
//            if (write_bottom_right) begin
//                bottom_right_h <= HSIZE - edge_to_ignore; // touch_h;
//                bottom_right_v <= VSIZE - edge_to_ignore; //touch_v;
//            end
//        end
       
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
                    state <= WRITE_FINAL;
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
            WRITE_FINAL: begin
                reset_count <= 0;
                if ((hcount == HSIZE-2) && (vcount == VSIZE-3)) state <= BLOB;
            end
            BLOB: begin
                reset_count <= 0;
               
                if (hcount >= HSIZE-2 & vcount >= VSIZE-3) begin
                    started_blob <= 0;
                    if (end_h - start_h >= blob_min_size & end_h - start_h <= blob_max_size
                      & end_v - start_v >= blob_min_size & end_v - start_v <= blob_max_size) begin
                      // blob is valid
                        state <= IDLE;
                        touch <= 1;
                        touch_ready <= 1;
                        wr_blob <= 0;
                    end
                    else begin
                    // blob is not valid or no blob found
                        touch <= 0;
                        if (blob_count < MAX_BLOB_COUNT) blob_count <= blob_count + 1;
                        else begin
                            touch <= 0;
                            touch_ready <= 1;
                            wr_blob <= 0;
                            state <= IDLE;
                            started_blob <= 0;
                        end
                    end
                end
                else if (look_for_blob & ~started_blob & process_out) begin
                    start_h <= hcount+1;
                    end_h <= hcount+1;
                    start_v <= vcount;
                    end_v <= vcount;
                    started_blob <= 1;
                    h_process <= hcount;
                    v_process <= vcount;
                    wr_blob <= 1;
                end
                else if (look_for_blob & started_blob & process_out & hcount+1 >= start_h - BLOB_DIST &
                                                            hcount+1 <= end_h + BLOB_DIST &
                                                            vcount >= start_v - BLOB_DIST &
                                                            vcount <= end_v + BLOB_DIST) begin
                    // pixel is part of blob
                    if (hcount+1 < start_h) start_h <= hcount+1;
                    if (hcount+1 > end_h) end_h <= hcount+1;
                    if (vcount < start_v) start_v <= vcount;
                    if (vcount > end_v) end_v <= vcount;   
                    h_process <= hcount;
                    v_process <= vcount;
                    wr_blob <= 1;
                end
                else begin
                    wr_blob <= 0;
                end
            end
        endcase
    end
   
//    assign in_blob_box = (hcount_disp == start_h & vcount_disp >= start_v & vcount_disp <= end_v) |
//                 (hcount_disp == end_h & vcount_disp >= start_v & vcount_disp <= end_v) |
//                 (vcount_disp == start_v & hcount_disp >= start_h & hcount_disp <= end_h) |
//                 (vcount_disp == end_v & hcount_disp >= start_h & hcount_disp <= end_h);

    assign in_blob_box = (hcount_disp > start_h) & (vcount_disp > start_v) & (hcount_disp < end_h) & (vcount_disp < end_v);    
   
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