library verilog;
use verilog.vl_types.all;
entity test_rom is
    port(
        addr            : in     vl_logic_vector(17 downto 0);
        clk             : in     vl_logic;
        dout            : out    vl_logic
    );
end test_rom;
