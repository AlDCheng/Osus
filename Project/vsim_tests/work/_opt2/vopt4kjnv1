library verilog;
use verilog.vl_types.all;
entity vga is
    generic(
        HSIZE           : integer := 512;
        VSIZE           : integer := 424
    );
    port(
        vga_clock       : in     vl_logic;
        hcount          : out    vl_logic_vector(9 downto 0);
        vcount          : out    vl_logic_vector(9 downto 0);
        vsync           : out    vl_logic;
        hsync           : out    vl_logic;
        at_display_area : out    vl_logic
    );
end vga;
