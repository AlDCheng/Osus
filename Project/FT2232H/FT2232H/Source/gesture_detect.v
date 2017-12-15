module gesture_detect(
        input clock,
        input [9:0] touch_h, touch_v,
        input touch, touch_ready,
        output reg is_press, is_release, is_move, is_scroll,
        output reg send_mouse_enable);
        
    reg last_touch;
    reg [9:0] last_h, last_v;
    
    always @(posedge clock) begin
        if (touch_ready) begin
            last_touch <= touch;
            last_h <= touch_h;
            last_v <= touch_v;
        end
        
        if (touch_ready & touch & ~last_touch) begin
            is_press <= 1;
            is_release <= 0;
            is_move <= 1;
            is_scroll <= 0;
            send_mouse_enable <= 1;
        end
        else if (touch_ready & ~touch & last_touch) begin
            is_press <= 0;
            is_release <= 1;
            is_move <= 0;
            is_scroll <= 0;
            send_mouse_enable <= 1;
        end
        else if (touch_ready & touch) begin
            is_press <= 0;
            is_release <= 0;
            is_move <= 1;
            is_scroll <= 0;
            send_mouse_enable <= 1;
        end
        else if (touch_ready & ~touch) begin
            is_press <= 0;
            is_release <= 0;
            is_move <= 0;
            is_scroll <= 0;
            send_mouse_enable <= 1;
        end
        else begin
            send_mouse_enable <= 0;
        end
    end
        
endmodule