
module serial_send(
     input clk,
     input [7:0] data,       // 8 bit ascii data
     input start_send,       // one clock pulse width, enter pressed, data available
     output reg xmit_data,   // serial data sent to RS232 output driver 
     output reg xmit_clk,     // baud rate; sent to logic analyzer for debuggin
     output reg xmit_done
     );

     // this section sets up the clk;
    parameter DIVISOR =  868; // create 115,200 baud rate clock, not exact, but should work.
    parameter MARK = 1'b1;
    parameter STOP_BIT = 1'b1;
    parameter START_BIT =1'b0;
    
    parameter WAIT = 0;
    parameter SEND = 1;
    
    reg [31:0] count; // FIX NUMBER OF BITS - 11 for DIVISOR=868
    reg [9:0] data_buffer;
    reg state;
    reg [3:0] sent_bits;
    
    always @(posedge clk)
    begin
        count <= xmit_clk ? 0 : count+1;
        xmit_clk <= count == DIVISOR-1;
        
        case (state)
            WAIT: begin
                xmit_done <= 0;
                if (start_send) begin
                    data_buffer <= {STOP_BIT, data, START_BIT};
                    sent_bits <= 0;
                    state <= SEND;
                end
            end
            SEND: begin
                if (sent_bits >= 10) begin
                    state <= WAIT;
                    xmit_done <= 1;
                end
                else if (xmit_clk) begin
                    sent_bits <= sent_bits + 1;
                    data_buffer <= {STOP_BIT, data_buffer[9:1]};
                    xmit_data <= data_buffer[0];
                    xmit_done <= 0;
                end
                
            end
        endcase
        
    end
           
endmodule
