library verilog;
use verilog.vl_types.all;
entity bram is
    generic(
        SIZE            : integer := 217088;
        LOGSIZE         : integer := 18;
        BITS            : integer := 1
    );
    port(
        addr            : in     vl_logic_vector;
        clk             : in     vl_logic;
        din             : in     vl_logic_vector;
        dout            : out    vl_logic_vector;
        we              : in     vl_logic
    );
end bram;
