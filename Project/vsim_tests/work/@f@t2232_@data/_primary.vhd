library verilog;
use verilog.vl_types.all;
entity FT2232_Data is
    port(
        clock_60_mhz    : in     vl_logic;
        RXF             : in     vl_logic;
        WR              : out    vl_logic;
        RD              : out    vl_logic;
        OE              : out    vl_logic
    );
end FT2232_Data;
