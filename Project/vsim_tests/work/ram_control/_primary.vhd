library verilog;
use verilog.vl_types.all;
entity ram_control is
    generic(
        WIDTH           : integer := 512;
        HEIGHT          : integer := 424;
        LOGSIZE         : integer := 18;
        DEPTH_SIZE      : integer := 16;
        BIN_SIZE        : integer := 1
    );
    port(
        reset           : in     vl_logic;
        clk             : in     vl_logic;
        clock_60_mhz    : in     vl_logic;
        write_btn       : in     vl_logic;
        RXF             : in     vl_logic;
        OE              : in     vl_logic;
        RD              : in     vl_logic;
        data            : in     vl_logic_vector(7 downto 0);
        read_index      : in     vl_logic_vector;
        low_bound       : in     vl_logic_vector(4 downto 0);
        high_bound      : in     vl_logic_vector(4 downto 0);
        ready           : out    vl_logic;
        bin_out         : out    vl_logic;
        wr_bin_mem      : out    vl_logic;
        data_bin        : out    vl_logic;
        start           : out    vl_logic;
        index_out       : out    vl_logic_vector;
        state_out       : out    vl_logic_vector(1 downto 0);
        prev_out        : out    vl_logic_vector(7 downto 0);
        data_out        : out    vl_logic_vector(7 downto 0);
        debug           : out    vl_logic;
        combined        : out    vl_logic_vector(15 downto 0);
        debug_write     : out    vl_logic;
        debug_bin       : out    vl_logic_vector(1 downto 0);
        inner_write     : out    vl_logic
    );
end ram_control;
