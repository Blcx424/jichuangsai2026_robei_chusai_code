module pwm_generator #(
    parameter PWM_BITS = 8,     // PWM精度 (8位 = 256级)
    parameter CLK_DIV  = 100     // 时钟分频，控制PWM频率
)(
    input wire clk,
    input wire rst_n,
    input wire [PWM_BITS-1:0] duty_cycle,  // 占空比 (0-255)
    output reg pwm_out                     // PWM输出
);
    
    reg [PWM_BITS-1:0] pwm_counter;  // PWM计数器
    reg [6:0]clk_div_counter;             // 分频计数器
    reg clk_slow;                    // 分频后时钟,T为4ms
    
    // 最大计数值 255
    localparam MAX_CNT = (1 << PWM_BITS) - 1;

    
    // 时钟分频 (降低PWM频率，通常需要几百Hz到几kHz)
    always @(posedge clk or negedge rst_n ) begin
        if (!rst_n) begin
            clk_div_counter <= 0;
            clk_slow <= 0;
        end
       else begin
            if (clk_div_counter == CLK_DIV - 1) begin
                clk_div_counter <= 0;
                clk_slow <= ~clk_slow;  // 翻转
            end
            else begin
                clk_div_counter <= clk_div_counter + 1'd1;
            end
        end
    end
    
    // PWM生成
    always @(posedge clk_slow or negedge rst_n) begin
        if (!rst_n) begin
            pwm_counter <= 0;
            pwm_out <= 1'b0;
        end
        else begin
            // 关键！！！到最大值清零
            if (pwm_counter == MAX_CNT)
                pwm_counter <= 0;
            else
                pwm_counter <= pwm_counter + 1'd1;
            
            if (pwm_counter < duty_cycle) begin
                pwm_out <= 1'b1;
            end
            else begin
                pwm_out <= 1'b0;
            end
        end
    end
endmodule