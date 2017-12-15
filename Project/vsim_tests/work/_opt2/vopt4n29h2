library verilog;
use verilog.vl_types.all;
entity image_process is
    generic(
        HSIZE           : integer := 512;
        VSIZE           : integer := 424
    );
    port(
        clock           : in     vl_logic;
        reset           : in     vl_logic;
        clock_30mhz     : in     vl_logic;
        frame_available : in     vl_logic;
        display_raw     : in     vl_logic;
        blank_frame     : in     vl_logic;
        write_top_left  : in     vl_logic;
        write_bottom_right: in     vl_logic;
        pixel_val       : in     vl_logic;
        bin_index       : out    vl_logic_vector(17 downto 0);
        pixel           : out    vl_logic_vector(7 downto 0);
        hsync           : out    vl_logic;
        vsync           : out    vl_logic;
        in_blob_box     : out    vl_logic;
        in_frame_box    : out    vl_logic;
        touch_h         : out    vl_logic_vector(9 downto 0);
        touch_v         : out    vl_logic_vector(9 downto 0);
        top_left_h      : out    vl_logic_vector(9 downto 0);
        top_left_v      : out    vl_logic_vector(9 downto 0);
        bottom_right_h  : out    vl_logic_vector(9 downto 0);
        bottom_right_v  : out    vl_logic_vector(9 downto 0);
        touch           : out    vl_logic;
        touch_ready     : out    vl_logic
    );
end image_process;
