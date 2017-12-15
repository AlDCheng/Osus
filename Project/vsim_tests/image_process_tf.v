`timescale 1ns / 100ps

module image_process_tf;

	// Inputs
	reg clock;
	reg reset;
	reg frame_available;
	reg display_raw;
	reg blank_frame;
	reg write_top_left;
	reg write_bottom_right;
	wire pixel_val;

	// Outputs
	wire [17:0] bin_index;
	wire [7:0] pixel;
	wire hsync;
	wire vsync;
	wire in_blob_box;
	wire in_frame_box;
	wire [9:0] touch_h;
	wire [9:0] touch_v;
	wire [9:0] top_left_h;
	wire [9:0] top_left_v;
	wire [9:0] bottom_right_h;
	wire [9:0] bottom_right_v;
	wire touch;
	wire touch_ready;

	// Instantiate the Unit Under Test (UUT)
	image_process #(.HSIZE(28), .VSIZE(22)) uut (
		.clock(clock), 
		.reset(reset), 
		.clock_30mhz(clock), 
		.frame_available(frame_available), 
		.display_raw(display_raw), 
		.blank_frame(blank_frame), 
		.write_top_left(write_top_left), 
		.write_bottom_right(write_bottom_right), 
		.pixel_val(pixel_val), 
		.bin_index(bin_index), 
		.pixel(pixel), 
		.hsync(hsync), 
		.vsync(vsync), 
		.in_blob_box(in_blob_box), 
		.in_frame_box(in_frame_box), 
		.touch_h(touch_h), 
		.touch_v(touch_v), 
		.top_left_h(top_left_h), 
		.top_left_v(top_left_v), 
		.bottom_right_h(bottom_right_h), 
		.bottom_right_v(bottom_right_v), 
		.touch(touch), 
		.touch_ready(touch_ready)
	);
   
   test_rom frame(.addr(bin_index),.clk(clock),.dout(pixel_val));

   always #5 clock = !clock;
   
	initial begin
		// Initialize Inputs
		clock = 0;
		reset = 1;
		frame_available = 0;
		display_raw = 0;
		blank_frame = 0;
		write_top_left = 0;
		write_bottom_right = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here
      reset = 0;
      #20
      frame_available = 1;
      #20
      frame_available = 0;
	end
      
endmodule

