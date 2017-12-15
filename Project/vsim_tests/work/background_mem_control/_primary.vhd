library verilog;
use verilog.vl_types.all;
entity background_mem_control is
    generic(
        WIDTH           : integer := 512;
        HEIGHT          : integer := 424;
        LOGSIZE         : integer := 18;
        DEPTH_SIZE      : integer := 16
    );
    port(
        reset           : in     vl_logic;
        clk             : in     vl_logic;
        clock_60_mhz    : in     vl_logic;
        new_data        : in     vl_logic;
        write           : in     vl_logic;
        RXF             : in     vl_logic;
        data_in         : in     vl_logic_vector(7 downto 0);
        low_bound       : in     vl_logic_vector(4 downto 0);
        high_bound      : in     vl_logic_vector(4 downto 0);
        data_bin_wr     : out    vl_logic;
        WR_bin          : out    vl_logic;
        index_bin       : out    vl_logic_vector;
        ready           : out    vl_logic;
        data_back       : out    vl_logic_vector(15 downto 0);
        inner_write     : out    vl_logic
    );
end background_mem_control;
