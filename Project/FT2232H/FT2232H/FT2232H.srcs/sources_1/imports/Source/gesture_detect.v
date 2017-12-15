//////////////////////////////////////////////////////////////////////////////////
// Module Name: gesture_detect
// Description: Calculates whether a user has pressed or released based on past inputs.
// 
// Dependencies:
// 
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module gesture_detect(
        input clock,
        input [9:0] touch_h, touch_v, // coordinates of user's fingertip
        input touch, // whether the fingertip is touching
        input touch_ready, // a pulse indicating calculations finished and the touch_h, touch_v, and touch readings are valid
        output reg is_press, is_release, is_move, is_scroll, // mouse actions
        output reg send_mouse_enable // whether to send any command to the Teensy in order to control the mouse
        );
       
    reg touch_active;
    
    parameter TIMEOUT_TOUCH = 10;
    reg [5:0] miss_count;
   
    always @(posedge clock) begin
        if (touch_ready) begin            
            if(touch & ~touch_active) begin
                is_press <= 1;
                is_release <= 0;
                is_move <= 1;
                is_scroll <= 0;
                send_mouse_enable <= 1;
                
                miss_count <= TIMEOUT_TOUCH;
                touch_active <= 1;
            end
            else if(touch) begin
                is_press <= 0;
                is_release <= 0;
                is_move <= 1;
                is_scroll <= 0;
                send_mouse_enable <= 1;
                
                miss_count <= TIMEOUT_TOUCH;
                touch_active <= 1;
            end
            else if(~touch & touch_active) begin
                if (miss_count == 0) begin
                    is_press <= 0;
                    is_release <= 1;
                    is_move <= 0;
                    is_scroll <= 0;
                    send_mouse_enable <= 1;
                    
                    touch_active <= 0;
                end
                else begin
                    is_press <= 0;
                    is_release <= 0;
                    is_move <= 1;
                    is_scroll <= 0;
                    send_mouse_enable <= 1;
                    
                    miss_count <= miss_count - 1;
                    touch_active <= 1;
                end
            end
            else if(~touch) begin
                is_press <= 0;
                is_release <= 0;
                is_move <= 0;
                is_scroll <= 0;
                send_mouse_enable <= 1;
                
                touch_active <= 0;
            end
        end
        
        else begin
            send_mouse_enable <= 0;
        end
    end
       
endmodule