library verilog;
use verilog.vl_types.all;
entity image_store_tb is
    generic(
        OP_SIZE         : integer := 64;
        DATA_SIZE       : integer := 256
    );
end image_store_tb;
