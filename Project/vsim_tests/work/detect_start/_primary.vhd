library verilog;
use verilog.vl_types.all;
entity detect_start is
    generic(
        START0          : integer := 170;
        START1          : integer := 187;
        START2          : integer := 204;
        START3          : integer := 221
    );
    port(
        reset           : in     vl_logic;
        clock_60_mhz    : in     vl_logic;
        RD              : in     vl_logic;
        data_in         : in     vl_logic_vector(7 downto 0);
        ready           : out    vl_logic;
        prev_out        : out    vl_logic_vector(7 downto 0);
        eq              : out    vl_logic_vector(3 downto 0)
    );
end detect_start;
