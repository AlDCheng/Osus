module image_process #(HSIZE = 512, VSIZE=424)
        (input clock,
        input frame_available, // Indicates that new data is available in frame, should be a pulse
        input current_val,
        input display_raw,
        input blank_frame,
        input write_top_left, write_bottom_right,
        output [17:0] addr,
        output [7:0] pixel, // current pixel (according to hcount_disp, vcount_disp)
        output hsync, vsync,
        output in_blob_box, in_frame_box, // whether the current pixel is on the edge of the box around the blob
        output [9:0] touch_h, touch_v, // coordinates of user touch
        output reg [9:0] top_left_h, top_left_v, bottom_right_h, bottom_right_v,
        output reg touch, touch_ready); // whether there is a valid touch
       
        
    // Create 25mhz clock - used for VGA display (the monitor complains if you use a faster clock)
    wire clock_25mhz;
    clock_quarter_divider clockgen(.clk100_mhz(clock), .clock_25mhz(clock_25mhz));
    
    // These are the possible states in image processing
    parameter IDLE = 0;
    parameter GET_IMAGE = 1;
    parameter ERODE = 2;
    parameter DILATE = 3;
    parameter BLOB = 4;
    reg [3:0] state;
    
    // These parameters can be changed to control how the image processing works
    parameter BLOB_DIST = 2;
    parameter BLOB_MIN_SIZE = 6; // minimun size (in either direction) a blob must be to be considered a finger touch
    parameter BLOB_MAX_SIZE = 8; // maximum size (in either direction) a blob must be to be considered a finger touch
    parameter MAX_BLOB_COUNT = 20; // the maximum number of blobs that we will look at in an attempt to find a finger touch (takes 1 cycle per pixel per blob)
    
    wire [9:0] hcount, vcount;
    wire [9:0] hcount_disp, vcount_disp; // these are the hcount and vcount for display on the monitor, as opposed to for our processing which is faster
    wire at_display_area;
    wire pixel_on; // whether the current pixel should be on
    reg [9:0] start_h, end_h, start_v, end_v; // the bounds of the box around current blob
    wire raw; // whether to display raw data
    reg frame [HSIZE-1:0][VSIZE-1:0];
    reg eroded [HSIZE-1:0] [VSIZE-1:0]; // the eroded version of the image
    reg processed [HSIZE-1:0] [VSIZE-1:0]; // the image after it has been eroded then dilated
    reg begin_processing; // is triggered when a new frame comes in
    reg started_blob; // whether we have found a blob yet
    reg [5:0] blob_count; // the number of blobs perviously looked at (that didn't meet criteria)
    wire reset;
    reg [9:0] image_addr;
    wire mem_out;
    
    // Make an hcount and vcount for displaying and also a faster one for cycling through pixels in image processing
    vga #(.HSIZE(HSIZE), .VSIZE(VSIZE)) vga_display(.vga_clock(clock_25mhz),.hcount(hcount_disp),.vcount(vcount_disp),.hsync(hsync),.vsync(vsync),.at_display_area(at_display_area));
    vga #(.HSIZE(HSIZE), .VSIZE(VSIZE)) vga_fast(.vga_clock(clock),.hcount(hcount),.vcount(vcount),.hsync(),.vsync(),.at_display_area());
    
    mybram stored_image (.addr(image_addr),.clk(clock),.we(),.din(),.dout(mem_out));
    
    always @(posedge clock) begin
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
                if (frame_available) begin_processing <= 1;
                if (begin_processing & hcount == 0 & vcount == 0) begin
                    begin_processing <= 0;
                    state <= GET_IMAGE;
                    started_blob <= 0;
                    start_h <= 0;
                    end_h <= 0;
                    start_v <= 0;
                    end_v <= 0;
                    blob_count <= 0;
                    touch <= 0;
                    image_addr <= 1;
                end
            end
            GET_IMAGE: begin
                if (hcount == 0 & vcount == 0) state <= ERODE;
//                if (at_display_area) begin
//                    frame[hcount][vcount] <= mem_out;
//                    image_addr <= image_addr + 1;
//                end
            end
            ERODE: begin
                if (hcount == 0 & vcount == 0) state <= DILATE;
                eroded[hcount][vcount] <= ~blank_frame &
                                           frame[hcount-1][vcount-1] &
                                           frame[hcount-1][vcount]   &
                                           frame[hcount]  [vcount-1] &
                                           frame[hcount]  [vcount]   &
                                           frame[hcount+1][vcount+1] &
                                           frame[hcount+1][vcount]   &
                                           frame[hcount-1][vcount+1] &
                                           frame[hcount+1][vcount-1];
            end
            DILATE: begin
                if (hcount == 0 & vcount == 0) state <= BLOB;
                processed[hcount][vcount] <= (eroded[hcount-1][vcount-1] |
                                              eroded[hcount-1][vcount]   |
                                              eroded[hcount]  [vcount-1] |
                                              eroded[hcount]  [vcount]   |
                                              eroded[hcount+1][vcount+1] |
                                              eroded[hcount+1][vcount]   |
                                              eroded[hcount-1][vcount+1] |
                                              eroded[hcount+1][vcount-1]);
            end
            BLOB: begin
                if (hcount == 0 & vcount == 0) begin
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
                            state <= IDLE;
                        end
                    end
                end
                if (~started_blob & processed[hcount][vcount]) begin
                    processed[hcount][vcount] <= 0;
                    start_h <= hcount;
                    end_h <= hcount;
                    start_v <= vcount;
                    end_v <= vcount;
                    started_blob <= 1;
                end
                if (started_blob & processed[hcount][vcount] & hcount >= start_h - BLOB_DIST &
                                                            hcount <= end_h + BLOB_DIST &
                                                            vcount >= start_v - BLOB_DIST &
                                                            vcount <= end_v + BLOB_DIST) begin
                    // pixel is part of blob
                    processed[hcount][vcount] <= 0;
                    if (hcount < start_h) start_h <= hcount;
                    if (hcount > end_h) end_h <= hcount;
                    if (vcount < start_v) start_v <= vcount;
                    if (vcount > end_v) end_v <= vcount;
                end
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
                 
    assign pixel_on = display_raw? (~blank_frame & frame[hcount_disp][vcount_disp]) : 
                          (processed[hcount_disp][vcount_disp]);
                
    assign pixel = at_display_area? (pixel_on? 255 :0) : 0;
    assign touch_h = start_h + (end_h - start_h)/2; // check for glitches here
    assign touch_v = start_v + (end_v - start_v)/2;
        
endmodule