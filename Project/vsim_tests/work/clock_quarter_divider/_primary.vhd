library verilog;
use verilog.vl_types.all;
entity clock_quarter_divider is
    port(
        clk100_mhz      : in     vl_logic;
        clock_25mhz     : out    vl_logic
    );
end clock_quarter_divider;
