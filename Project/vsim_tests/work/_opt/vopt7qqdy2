library verilog;
use verilog.vl_types.all;
entity frame_bram is
    generic(
        SIZE            : integer := 217088;
        LOGSIZE         : integer := 18;
        BITS            : integer := 1
    );
    port(
        wr_addr         : in     vl_logic_vector;
        rd_addr         : in     vl_logic_vector;
        wr_clk          : in     vl_logic;
        rd_clk          : in     vl_logic;
        din             : in     vl_logic_vector;
        dout            : out    vl_logic_vector;
        we              : in     vl_logic;
        rd              : in     vl_logic
    );
end frame_bram;
