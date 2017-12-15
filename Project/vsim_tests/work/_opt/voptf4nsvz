library verilog;
use verilog.vl_types.all;
entity frame_index_control is
    generic(
        HSIZE           : integer := 512;
        VSIZE           : integer := 424
    );
    port(
        reset           : in     vl_logic;
        reset_count     : in     vl_logic;
        clk             : in     vl_logic;
        pause_v         : in     vl_logic;
        vcount          : out    vl_logic_vector(9 downto 0);
        hcount          : out    vl_logic_vector(9 downto 0)
    );
end frame_index_control;
