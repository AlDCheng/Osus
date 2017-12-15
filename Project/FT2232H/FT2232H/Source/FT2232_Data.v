`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: FT2232_Data
// Description: Controls RD and OE signals to read from FT2232H properly
// 
// Dependencies: 
// 
// Additional Comments: See FT2232H FIFO Documentation for timing properties
//                      Reacts asynchronously with changes in RXF
// 
//////////////////////////////////////////////////////////////////////////////////

module FT2232_Data(
    input clock_60_mhz,     // 60 MHz FT_2232H clock
    input RXF,              // RXF from FT_2232H: 0 if there is data to be read
    output WR,              // WR to FT_2232H: 0 if data needs to be written
    output RD,              // RD to FT_2232H: 0 to read from buffer
    output OE               // OE to FT_2232H: 0 to enable output for data
                            //      to be moved from buffer. Must be low before RD
    );

    reg RD_buf, OE_buf;
    reg tmp = 1;
    
    always@(posedge clock_60_mhz) begin
        if(!RXF) begin
            RD_buf <= OE_buf;
            OE_buf <= tmp;
            tmp <= RXF;
        end
        else begin
            RD_buf <= 1'b1;
            OE_buf <= 1'b1;
            tmp <= 1'b1;
        end
    end
        
    assign WR = 1'b1;                   // We never write to PC
//    assign RD = (RXF) ? 1'b1 : RD_buf;  // Asynchronous HI reset with RXF   
//    assign OE = (RXF) ? 1'b1 : OE_buf;  // Asynchronous HI reset with RXF   

    assign RD = RD_buf;
    assign OE = OE_buf;
    
endmodule

//////////////////////////////////////////////////////////////////////////////////
// Module Name: ram_control
// Description: handles incoming image data from FT2232H and:
//              1) stores full frame in BRAM with button press
//              2) thresholds for binary image using stored frame in BRAM
// 
// Dependencies: bram, background_mem_control, detect_new_frame
// 
// Additional Comments: read_index input allows to read from bram when possible
//                      this top module focuses on state control
// 
//////////////////////////////////////////////////////////////////////////////////
module ram_control #(parameter WIDTH=512, HEIGHT=424, LOGSIZE=18,
                     DEPTH_SIZE=16, BIN_SIZE=1) 
    (input reset,                       // global reset
    input clk,                          // 100 MHz FPGA system clock
    input clock_60_mhz,                 // 60 MHz FT_2232H clock
    input write_btn,                    // Write signal: 1 stores background frame in BRAM
    input RXF,
    input OE,                           // Output_Enable from FT_2232H
    input RD,                           // RD from FT_2232H
    input [7:0] data,                   // Incoming byte of data from FT_2232H
    input [LOGSIZE-1:0] read_index,     // Input index of Binary BRAM frame to read
    input [4:0] low_bound,
    input [4:0] high_bound,
    output ready,                       // Ready signal: 1 if complete frame is stored; 0 if updating
    output bin_out,                      // Binary pixel value at read_index or index_bin if updating
    output wr_bin_mem,
    output data_bin,
    // Signals for DEBUG           
    output start,                                                                                                                                                                                                                                                 
    output [LOGSIZE-1:0] index_out,
    output [1:0] state_out,
    output [7:0] prev_out,
    output [7:0] data_out,
    output reg debug,
    output [15:0] combined,
    output debug_write,
    output [1:0] debug_bin,
    output inner_write
    );
    
    // States:
    parameter [1:0] UPDATING = 2'b10;   // Updating binary image 
    parameter [1:0] WRITING = 2'b01;    // Storing frame to background frame BRAM
    parameter [1:0] IDLE = 2'b00;       // Idle; all processes done
    reg [1:0] state;
    
    wire [LOGSIZE-1:0] index_bin, index_bin_mem;    // coresponding index to data_bin, index input to binary BRAM
//    wire data_bin;                                  // binary value for pixel at index_bin
    
    wire wr_bin, new_frame;             // calculated write timing / write input for binary BRAM (wr_bin/wr_bin_mem)
                                                    // new_frame flag 
    
    detect_start detect_new_frame(.reset(reset), .clock_60_mhz(clock_60_mhz), .RD(RD), .data_in(data), .ready(new_frame));
    
    background_mem_control #(.WIDTH(WIDTH), .HEIGHT(HEIGHT), .LOGSIZE(LOGSIZE), .DEPTH_SIZE(DEPTH_SIZE))
            stream_control(.reset(reset || new_frame), .clk(clk), .clock_60_mhz(clock_60_mhz), .new_data(RD),
                           .RXF(RXF), .write(state[0]), .data_in(data), .data_bin_wr(data_bin), .WR_bin(wr_bin),
                           .index_bin(index_bin), .ready(ready), .data_back(combined), .inner_write(inner_write),
                           .low_bound(low_bound), .high_bound(high_bound));
    
//    bram #(.SIZE(WIDTH*HEIGHT), .LOGSIZE(LOGSIZE), .BITS(BIN_SIZE))
//            binary_bram(.addr(index_bin_mem),.clk(clock_60_mhz),.we(wr_bin_mem),.din(data_bin),.dout(bin_out));

//    frame_bram #(.SIZE(WIDTH*HEIGHT), .LOGSIZE(LOGSIZE), .BITS(BIN_SIZE))
//            binary_bram(.addr(index_bin_mem),.wr_clk(clock_60_mhz),.rd_clk(clock_30_mhz),
//                        .we(wr_bin_mem),.rd(1'b1),.din(data_bin),.dout(bin_out));

    assign wr_bin_mem = (state == UPDATING) ? wr_bin : 0;               // prohibit writes to binary BRAM when not updating
    assign index_bin_mem = (state == IDLE) ? read_index : index_bin;    // when IDLE, take index from input; else use signal from mem_control
    
    // DEBUG SIGNALS
    assign index_out = index_bin;
    assign state_out = state;
    assign start = new_frame;
    assign data_out[7:0] = data[7:0];
    assign debug_write = wr_bin_mem;
    assign debug_bin[0] = data_bin;
    assign debug_bin[1] = bin_out;

    // State Machine Model
    always@(posedge clock_60_mhz) begin
        // Global reset
        if(reset)
            state <= IDLE;
        else begin
            debug <= new_frame;
            if (state == IDLE) begin
                if (write_btn && new_frame)
                    state <= WRITING;
                else if (new_frame)
                    state <= UPDATING;
            end
            else if (state == UPDATING) begin
                if (ready && !new_frame)
                    state <= IDLE;
            end
            else if (state == WRITING) begin
                if (ready && !new_frame)
                    state <= IDLE;
            end
        end
    end
    
endmodule

//////////////////////////////////////////////////////////////////////////////////
// Module Name: background_mem_control
// Description: handles incoming image data from FT2232H and:
//              1) stores full frame in BRAM with button press
//              2) thresholds for binary image using stored frame in BRAM
// 
// Dependencies: bram
// 
// Additional Comments: adding some debug code for thresholding non-reliant
//                      on background frame BRAM
//////////////////////////////////////////////////////////////////////////////////
module background_mem_control #(parameter WIDTH=512,        // Depth of image resolution (512x424)
                                HEIGHT=424,                 // Width of image resolution
                                LOGSIZE=18,                 // Smallet LOGSIZE s.t. 2^LOGSIZE > WIDTH*HEIGHT
                                DEPTH_SIZE=16)              // Size of each pixel: 2 bytes = 16 bits
    (input reset,           // global reset
    input clk,              // 100 MHz FPGA system clock
    input clock_60_mhz,     // 60 MHz FT_2232H clock
    input new_data,         // 1 indicates there is new data to process
    input write,            // active-high write signal
    input RXF,
    input [7:0] data_in,    // data read from FT2232H
    input [4:0] low_bound,
    input [4:0] high_bound,
    output reg data_bin_wr, // data for binary image
    output reg WR_bin,      // write enable for binary image ram
    output reg [LOGSIZE-1:0] index_bin,     // index corresponding to data_bin_wr
    output reg ready,        // 1 when image frame is done processing
    // Debug
    output [15:0] data_back,
    output inner_write
    );
    
    // Determines white area in binary image if within threshold margins
    wire [4:0] BACK_THRESH_MIN;     // mem - 30 = lower bound
    wire [4:0] BACK_THRESH_MAX;      // mem - 2 = upper bound
    
    assign BACK_THRESH_MIN = high_bound;
    assign BACK_THRESH_MAX = low_bound;
    
    
    parameter BACK_DEPTH_MIN = 500;
    parameter BACK_DEPTH_MAX = 2000;
    parameter NUM_PIXELS = WIDTH*HEIGHT-1;      // Max number of pixels; used to flag ready
    
    // state of inc data: 1=2nd byte, 0=1st byte
    parameter HI = 1;
    parameter LO = 0;
    reg data_state;
    
    reg prev_RXF;

    reg [7:0] prev_data, tmp;   // previous data + temp storage
    reg [LOGSIZE-1:0] index;    // index of current pixel
    
    // 2 bytes make full distance values
    wire [15:0] data_back_wr;   // {2nd byte, 1st byte}\
    assign data_back_wr = {prev_data, data_in};
    assign data_back = data_back_wr;

    wire [15:0] ram_data;       // 2 byte at index from background BRAM
    
    // BRAM to store image 
//    bram #(.SIZE(WIDTH*HEIGHT), .LOGSIZE(LOGSIZE), .BITS(DEPTH_SIZE))
//                background_bram(.addr(index),.clk(clock_60_mhz),.we(write&&new_data),.din(data_back_wr),.dout(ram_data));
    frame_bram #(.SIZE(WIDTH*HEIGHT), .LOGSIZE(LOGSIZE), .BITS(DEPTH_SIZE))
            background_bram(.wr_addr(index),.rd_addr(index),.wr_clk(clock_60_mhz),.rd_clk(clock_60_mhz),
                        .we(write&&new_data),.rd(1'b1),.din(data_back_wr),.dout(ram_data));

                
    assign inner_write = write&&new_data;
    
    wire bin1, bin2;
//    assign bin1 = ((data_back_wr > (ram_data - 18)) &&
//             (data_back_wr < (ram_data - 12)) &&
//             (data_back_wr > BACK_DEPTH_MIN) &&
//             (ram_data > BACK_DEPTH_MIN) &&
//             (data_back_wr < BACK_DEPTH_MAX) &&
//             (ram_data < BACK_DEPTH_MAX));
    assign bin2 = ((data_back_wr > (ram_data - BACK_THRESH_MIN)) &&
                      (data_back_wr < (ram_data - BACK_THRESH_MAX)) &&
                      (data_back_wr > BACK_DEPTH_MIN) &&
                      (ram_data > BACK_DEPTH_MIN) &&
                      (data_back_wr < BACK_DEPTH_MAX) &&
                      (ram_data < BACK_DEPTH_MAX));
 
 
    // DEBUG signals
    parameter FLAT_THRES = 1500;    // 16'h05DC
    
    always@(posedge clock_60_mhz) begin
        // Global reset
        if (reset) begin
            WR_bin <= 0;
            index <= 0;
            index_bin <= 0;
            data_bin_wr <= 0;
            ready <= 0;
            prev_data <= 0;
            tmp <= data_in;
            data_state <= LO;
        end
        
        // if we have new data to proces...
        else if (new_data) begin
            // we have both components of a distance value 
            if (data_state == HI) begin     
                // If we are in UPDATE
                // ...create binary image from thresholds
                if(write == 0) begin
//                    if(data_back_wr < 1000)
//                     if ((data_back_wr > (ram_data - BACK_THRESH_MIN)) &&
//                         (data_back_wr < (ram_data - BACK_THRESH_MAX)) &&
//                         (data_back_wr > BACK_DEPTH_MIN) &&
//                         (ram_data > BACK_DEPTH_MIN) &&
//                         (data_back_wr < BACK_DEPTH_MAX) &&
//                         (ram_data < BACK_DEPTH_MAX))
//                     if ((data_back_wr > (ram_data - 100)) &&
//                         (data_back_wr < (ram_data - 30)) &&
//                         (data_back_wr > BACK_DEPTH_MIN) &&
//                         (ram_data > BACK_DEPTH_MIN) &&
//                         (data_back_wr < BACK_DEPTH_MAX) &&
//                         (ram_data < BACK_DEPTH_MAX))
                    data_bin_wr <= bin2;
//                         data_bin_wr <= bin2 && ~bin1;
//                     else
//                         data_bin_wr <= 0;
//                    data_bin_wr <= (data_back_wr < FLAT_THRES) ? 1 : 0;
                    index_bin <= index;
                end
                // Else we are in WRITING
                // ...and we prohibit wrting to binary BRAM
                else
                    data_bin_wr <= 0;
                
                // Done throughout all cases:
                WR_bin <= 1;    // write to background BRAM

                // increment index
                if (index >= NUM_PIXELS)
                    ready <= 1;
                else if (~(RXF && ~prev_RXF)) begin
                    ready <= 0;
                    index <= index + 1;
                end

                data_state <= LO;   // Toggle state of input
            end
            
            // We only have the first component of a distance value
            else begin 
                WR_bin <= 0;
                data_state <= HI;
            end
            
            // Store the previous value
//            tmp <= data_in;
//            prev_data <= tmp;
            prev_data <= data_in;
            
            
//            if (RXF && ~prev_RXF)
//                index <= index;
            prev_RXF <= RXF;
        end
       
    end
    
endmodule

//////////////////////////////////////////////////////////////////////////////////
// Module Name: detect_start
// Description: Detect starting sequence for new frame {DD,CC,BB,AA}
// 
// Dependencies:
// 
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module detect_start (
    input reset,            // global reset
    input clock_60_mhz,     // 60 MHz FT_2232H clock
    input RD,               // RD to FT_2232H: 0 to read from buffer
    input [7:0] data_in,    // data read from FT2232H
    output ready,            // flags when starting sequence is detected
    output [7:0] prev_out,
    //debug
    output [3:0] eq
    );
    
    // storage containers for past states
    reg [7:0] data_prev1, data_prev2, data_prev3/*, data_cur*/;
    
    // sequence to be detected
    parameter [7:0] START0 = 8'hAA;
    parameter [7:0] START1 = 8'hBB;
    parameter [7:0] START2 = 8'hCC;
    parameter [7:0] START3 = 8'hDD;
    
    always@(posedge clock_60_mhz) begin
        if(reset) begin
            data_prev1 <= 0;
            data_prev2 <= 0;
            data_prev3 <= 0;
//            data_cur <= 0;
        end
        else if(RD) begin
//            data_cur <= data_in;
            data_prev1 <= data_in;
            data_prev2 <= data_prev1;
            data_prev3 <= data_prev2;
            
//            eq[0] <= data_in == START0;
//            eq[1] <= data_prev1 == START1;
//            eq[2] <= data_prev2 == START2;
//            eq[3] <= data_prev3 == START3;
            
//            ready <= ((data_in == START0) && (data_prev1 == START1)
//                                && (data_prev2 == START2) && (data_prev3 == START3));
        end
    end
    
    assign ready = ((data_in == START0) && (data_prev1 == START1)
                    && (data_prev2 == START2) && (data_prev3 == START3));
    
    assign prev_out = data_in;
    
    // Debug
    assign eq[0] = data_in == START0;
    assign eq[1] = data_prev1 == START1;
    assign eq[2] = data_prev2 == START2;
    assign eq[3] = data_prev3 == START3;
    
endmodule
    