//////////////////////////////////////////////////////////////////////////////////
// Module Name: send_mouse
// Description: Assigns the byte to be sent over serial in the correct order for the mouse report.
// 
// Dependencies:
// 
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module send_mouse( 
    input [15:0] x, y, // coordinates of mouse
    input [7:0] is_press, is_release, is_move, // type of mouse action
    input [15:0] top_left_x, top_left_y, bottom_right_x, bottom_right_y, // bounds of screen
    input send_enable, // whether or not to send to the mouse (should be true only after having processed new data)
    input reset, clock,
    output [7:0] pmod
    );
    
    reg tx_enable;
    reg [7:0] data_to_send;
    wire xmit_done;
    wire serial_stream;
    wire xmit_clk;
    
    reg state;
    parameter WAITING = 0;
    parameter SENDING = 1;
    parameter SPEED = 1;
    reg [3:0] parameter_to_send;
    parameter X1 = 0;
    parameter X2 = 1;
    parameter Y1 = 2;
    parameter Y2 = 3;
    parameter TOP_LEFT_X1 = 4;
    parameter TOP_LEFT_X2 = 5;
    parameter TOP_LEFT_Y1 = 6;
    parameter TOP_LEFT_Y2 = 7;
    parameter BOTTOM_RIGHT_X1 = 8;
    parameter BOTTOM_RIGHT_X2 = 9;
    parameter BOTTOM_RIGHT_Y1 = 10;
    parameter BOTTOM_RIGHT_Y2 = 11;
    parameter IS_PRESS = 12;
    parameter IS_RELEASE = 13;
    parameter IS_MOVE = 14;
    parameter NONE = 15;
    
    reg debug_tx_enable;
    reg debug_xmit_done;
    reg [2:0] count;
    reg [3:0] send_count;
    reg debug_xmit_clk;
    
    serial_send sender (.clk(clock), .data(data_to_send), .start_send(tx_enable), .xmit_data(serial_stream), .xmit_clk(xmit_clk), .xmit_done(xmit_done));
    
    assign pmod[7:1] = 7'b0;
    assign pmod[0] = serial_stream;
    
    always @(posedge clock) begin
        case (state)
            WAITING: begin
                if (send_enable) begin
                    state <= SENDING;
                    parameter_to_send <= X1;
                    data_to_send <= x[7:0];
                    tx_enable <= 1;
                end
                else begin
                    tx_enable <= 0;
                end
            end
            SENDING: begin
                if (xmit_done) begin
                    parameter_to_send <= parameter_to_send + 1;
                    case(parameter_to_send + 1)
                        X1: begin
                            data_to_send <= x[7:0];
                            tx_enable <= 1;
                        end
                        X2: begin
                            data_to_send <= x[15:8];
                            tx_enable <= 1;
                        end
                        Y1: begin
                            data_to_send <= y[7:0];
                            tx_enable <= 1;
                        end
                        Y2: begin
                            data_to_send <= y[15:8];
                            tx_enable <= 1;
                        end
                        TOP_LEFT_X1: begin
                            data_to_send <= top_left_x[7:0];
                            tx_enable <= 1;
                        end
                        TOP_LEFT_X2: begin
                            data_to_send <= top_left_x[15:8];
                            tx_enable <= 1;
                        end
                        TOP_LEFT_Y1: begin
                            data_to_send <= top_left_y[7:0];
                            tx_enable <= 1;
                        end
                        TOP_LEFT_Y2: begin
                            data_to_send <= top_left_y[15:8];
                            tx_enable <= 1;
                        end
                        BOTTOM_RIGHT_X1: begin
                            data_to_send <= bottom_right_x[7:0];
                            tx_enable <= 1;
                        end
                        BOTTOM_RIGHT_X2: begin
                            data_to_send <= bottom_right_x[15:8];
                            tx_enable <= 1;
                        end
                        BOTTOM_RIGHT_Y1: begin
                            data_to_send <= bottom_right_y[7:0];
                            tx_enable <= 1;
                        end
                        BOTTOM_RIGHT_Y2: begin
                            data_to_send <= bottom_right_y[15:8];
                            tx_enable <= 1;
                        end
                        IS_PRESS: begin
                            data_to_send <= is_press;
                            tx_enable <= 1;
                        end
                        IS_RELEASE: begin
                            data_to_send <= is_release;
                            tx_enable <= 1;
                        end
                        IS_MOVE: begin
                            data_to_send <= is_move;
                            tx_enable <= 1;
                        end
                        NONE: begin
                            tx_enable <= 0;
                            state <= WAITING;
                        end
                    endcase
                end
                else begin
                    tx_enable <= 0;
                end
            end
        endcase
    end
    
endmodule
