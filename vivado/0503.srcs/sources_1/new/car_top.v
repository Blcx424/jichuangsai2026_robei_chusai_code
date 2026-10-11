// ===========================================
// 模块：line_follower_top (完整集成版)
// 功能：集成蓝牙控制、寻迹、避障功能于一体
// 支持4种模式：停止、遥控、循迹+避障、纯循迹
// ===========================================
module line_follower_top(
    // 时钟与复位
    input wire clk,          // 主时钟 (如50MHz)
    input wire rst_n,        // 低电平复位
    
    // 红外寻迹传感器输入 (1:检测到黑线, 0:未检测到)
    input wire sensor_left,  // 左传感器
    input wire sensor_right, // 右传感器
 
    // 蓝牙UART输入
    input wire uart_rx,      // 蓝牙串口接收
    output        at_pin,
    // 红外避障传感器输入
    input wire ir_left,      // 左避障信号 (0:检测到障碍物, 1:未检测到障碍物)
    input wire ir_right,     // 右避障信号 (0:检测到障碍物, 1:未检测到障碍物)
        
    // 电机控制输出 
    output wire left_motor_en,   // 左电机PWM使能
    output wire left_motor_go,    // 左电机正转
    output wire left_motor_back,  // 左电机反转
    
    output wire right_motor_en,   // 右电机PWM使能
    output wire right_motor_go,   // 右电机正转
    output wire right_motor_back,  // 右电机反转
    output wire led_0,
    output wire led_1,
    output wire led_left_motor,
    output wire led_right_motor,
    output wire led_left_sensor,
    output wire led_right_sensor,
    output wire led_blue_tooth,
    output wire led_W21,
    output wire led_AB1
);
    assign led_blue_tooth = uart_rx;
    assign at_pin = 1'b1;
    // 电机动作状态编码
    localparam [1:0] 
        MOTOR_STOP  = 2'b00,
        MOTOR_FWD   = 2'b01,  // 前进
        MOTOR_BACK  = 2'b10;  // 后退
        
    // 原有工作模式
    reg [1:0] work_mode;        // 原有工作模式 00:寻迹模式, 01:避障模式
    localparam [1:0]
        MODE_LINE_FOLLOW = 2'b00,
        MODE_AVOID       = 2'b01;
    
    // 高层工作模式
    reg [1:0] top_work_mode; 
    localparam [1:0]
        TOP_MODE_STOP     = 2'b00,  // 停止模式
        TOP_MODE_REMOTE   = 2'b01,  // 遥控模式
        TOP_MODE_TRACK_AVOID = 2'b10,  // 循迹+避障模式
        TOP_MODE_TRACK    = 2'b11;  // 纯循迹模式
    
    // 脱线处理状态
    reg [31:0] lost_timer;
    reg [1:0] lost_state;
    localparam [1:0]
        LOST_IDLE      = 2'b00,  // 空闲状态
        LOST_PAUSE     = 2'b01,  // 暂停2秒
        LOST_BACKWARD  = 2'b10;  // 后退1秒
    
    // 后退逻辑常量定义
    localparam [31:0] 
        CLK_FREQ = 50_000_000,  // 50MHz时钟
        PAUSE_TIME = 2 * CLK_FREQ,  // 暂停2秒
        BACKWARD_TIME = 1 * CLK_FREQ;  // 后退1秒
    
    // 电机方向编码
    
    localparam [1:0]
        DIR_STOP  = 2'b00,
        DIR_FWD   = 2'b01,
        DIR_BACK  = 2'b10;
    
    // LED指示灯
//    assign led_0 = 1'b0;
//    assign led_1 = 1'b1;
//    assign led_left_motor = left_motor_go;
//    assign led_right_motor = right_motor_go;
//    assign led_left_sensor = ~sensor_left;
//    assign led_right_sensor = ~sensor_right;
    
    // 传感器状态组合
    wire [1:0] sensor_state;
    assign sensor_state = {sensor_left, sensor_right};
    
    wire [1:0] ir_state;
    assign ir_state = {ir_left, ir_right};
    
    // 电机控制信号
    reg [7:0] pwm_duty_left;   // 左电机PWM占空比
    reg [7:0] pwm_duty_right;  // 右电机PWM占空比
    reg [1:0] left_motor_action;
    reg [1:0] right_motor_action;
    reg left_dir_go, left_dir_back;
    reg right_dir_go, right_dir_back;
    
    // PWM信号
    wire left_pwm_out, right_pwm_out;

    // ========== 蓝牙接收相关 ==========
    integer i;  // 用于for循环
    
    // 蓝牙UART接收信号
    wire [7:0] rx_data;
    wire rx_done;
    
    // 蓝牙帧解析
    reg [7:0] rx_buffer [0:6];  // 7字节缓冲区
                 // rx_buffer[0]: 存储帧头AA
                 // rx_buffer[1]: 存储命令字（01/02/03）
                 // rx_buffer[2]: 存储左轮速度
                 // rx_buffer[3]: 存储右轮速度
                 // rx_buffer[4]: 存储左轮控制
                 // rx_buffer[5]: 存储右轮控制
                 // rx_buffer[6]: 存储帧尾55

    reg [2:0] byte_cnt;//记录当前接收到第几帧
    reg frame_ready;//完整帧就绪标志
    // 蓝牙接收状态
    reg [1:0] rx_state;
    localparam [1:0]
        RX_IDLE      = 2'b00,//空闲状态，等待帧头
        RX_RECEIVE   = 2'b01,//接收数据状态
        RX_PARSE     = 2'b10;//解析数据状态
    
    // +++ 简化：去掉连接状态和计时器 +++
    reg connected;  //蓝牙连接状态
    
    // 解析后的蓝牙控制信号
    reg [7:0] left_speed_reg;   // 左轮速度
    reg [7:0] right_speed_reg;  // 右轮速度
    reg [1:0] left_dir_reg;     // 左轮方向
    reg [1:0] right_dir_reg;    // 右轮方向
    

    // ========== 模块实例化 ==========
    // 蓝牙接收模块
    blue_tooth u_bluetooth(
        .Clk(clk),
        .uart_rx(uart_rx),
        .Reset_n(rst_n),
        .Rx_Done(rx_done),
        .Rx_Data(rx_data)
    );

    // 左电机PWM生成器
    pwm_generator #(.PWM_BITS(8)) u_pwm_left (
        .clk(clk),
        .rst_n(rst_n),
        .duty_cycle(pwm_duty_left),
        .pwm_out(left_pwm_out)
    );
    
    // 右电机PWM生成器
    pwm_generator #(.PWM_BITS(8)) u_pwm_right (
        .clk(clk),
        .rst_n(rst_n),
        .duty_cycle(pwm_duty_right),
        .pwm_out(right_pwm_out)
    );
    
    // ========== 蓝牙连接状态管理 ==========
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            connected <= 1'b0;
        end
        else begin
            if (frame_ready) begin
                connected <= 1'b1;  // 收到完整帧，标记为已连接
            end
        end
    end
    
    // ========== 加入 rx_done 边沿检测（在蓝牙接收相关区域）==========
reg rx_done_dly;
wire rx_done_pulse;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rx_done_dly <= 1'b0;
    else
        rx_done_dly <= rx_done;
end

assign rx_done_pulse = rx_done && !rx_done_dly;

// 删除原来的 led_test_aa 逻辑，换成两个测试信号
reg led_test_any;  // 收到任何字节就亮
reg led_test_aa;   // 收到 AA 才亮
reg led_test_T1;
reg led_test_L6;
reg led_test_K13;
reg led_test_A15;
reg led_test_D19;
reg led_test_R18;
reg led_test_W21;
reg led_test_AB1;


    assign led_0 = led_test_T1; 
    assign led_1 = led_test_L6 ;
    assign led_left_motor = led_test_K13;
    assign led_left_sensor = led_test_A15;
    assign led_right_motor = led_test_D19;
    assign led_right_sensor = led_test_R18;
    assign led_W21 = led_test_W21;
    assign led_AB1 = led_test_AB1;
//    assign led_0 = led_test_any;   // 替换原来的 assign
//    assign led_1 = led_test_aa;    // 替换原来的 assign led_1 = 1'b1
    // ========== 蓝牙帧解析状态机 ==========
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state <= RX_IDLE;
            byte_cnt <= 3'd0;
            frame_ready <= 1'b0;
            // 清空缓冲区
            for (i = 0; i < 7; i = i + 1) begin
                rx_buffer[i] <= 8'h0;
            end
            left_speed_reg  <= 8'd0;
            right_speed_reg <= 8'd0;
            left_dir_reg    <= DIR_STOP;
            right_dir_reg   <= DIR_STOP;
        end
        else begin
            case (rx_state)
                
                RX_IDLE: begin
                    frame_ready <= 1'b0;
                        
                    if (rx_done && rx_data == 8'hAA) begin
                        rx_buffer[0] <= rx_data;
                        byte_cnt <= 3'd1;
                        rx_state <= RX_RECEIVE;
                        led_test_aa <= 1'b1;
                    end
                end
                
                RX_RECEIVE: begin
                    if (rx_done) begin
                        
                        case (byte_cnt)
                            3'd1: rx_buffer[1] <= rx_data;  // 命令字
                            3'd2: rx_buffer[2] <= rx_data;  // 左轮速度
                            3'd3: rx_buffer[3] <= rx_data;  // 右轮速度
                            3'd4: rx_buffer[4] <= rx_data;  // 左轮控制
                            3'd5: rx_buffer[5] <= rx_data;  // 右轮控制
                            3'd6: rx_buffer[6] <= rx_data;  // 帧尾
                        endcase                   
                        
                        if (byte_cnt == 3'd6) begin
                            if (rx_data == 8'h55) begin
                                frame_ready <= 1'b1;
                                rx_state <= RX_PARSE;
                                led_test_any <= 1'b1;
                            end
//                          
                        end
                        else begin
                            byte_cnt <= byte_cnt + 3'd1;
                        end
                    end
                end
                
                RX_PARSE: begin
                        
                        // 解析控制参数
                        left_speed_reg <= rx_buffer[2];
                        right_speed_reg <= rx_buffer[3];
                        
                        // 解析左轮方向
                        case (rx_buffer[4])
                            8'h01: left_dir_reg <= DIR_FWD;
                            8'h02: left_dir_reg <= DIR_BACK;
                            default: left_dir_reg <= DIR_STOP;
                        endcase
                        
                        // 解析右轮方向
                        case (rx_buffer[5])
                            8'h01: right_dir_reg <= DIR_FWD;
                            8'h02: right_dir_reg <= DIR_BACK;
                            default: right_dir_reg <= DIR_STOP;
                        endcase
                    
                    frame_ready <= 1'b0;
                    rx_state <= RX_IDLE;
                end
                
                default: rx_state <= RX_IDLE;  
            endcase
        end
    end
    
    // ========== 加入 frame_ready 边沿检测 ==========
reg frame_ready_dly;
wire frame_ready_posedge;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        frame_ready_dly <= 1'b0;
    else
        frame_ready_dly <= frame_ready;
end

assign frame_ready_posedge = frame_ready && !frame_ready_dly;

    // ========== 高层工作模式控制 ==========
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            top_work_mode <= TOP_MODE_STOP;  // 默认停止
        end
        else begin
            if (frame_ready) begin
                // 根据命令字切换高层模式
                case (rx_buffer[1])  // 命令字
                    8'h01: top_work_mode <= TOP_MODE_REMOTE;      // 遥控
                    8'h02: top_work_mode <= TOP_MODE_TRACK_AVOID; // 循迹+避障
                    8'h03: top_work_mode <= TOP_MODE_TRACK;       // 循迹
                    default: top_work_mode <= TOP_MODE_STOP;     // 保持
                endcase
            end
        end
    end
    
    // ========== 原有工作模式切换 ==========
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            work_mode <= MODE_LINE_FOLLOW;
        end
        else begin
            // 只有在高层模式是寻迹相关时，才执行原有的工作模式切换
            if (top_work_mode == TOP_MODE_TRACK_AVOID || top_work_mode == TOP_MODE_TRACK) begin
                // 原有的工作模式切换逻辑保持不变
                if (ir_state != 2'b11) begin
                    work_mode <= MODE_AVOID;
                end
                else if (ir_state == 2'b11) begin
                    work_mode <= MODE_LINE_FOLLOW;
                end
            end
        end
    end
    
    // ========== 主控逻辑 ==========
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_duty_left  <= 8'd0;
            pwm_duty_right <= 8'd0;
            left_motor_action  <= MOTOR_STOP;
            right_motor_action <= MOTOR_STOP;
            lost_timer <= 0;
            lost_state <= LOST_IDLE;
        end
        else begin
            // 高层控制逻辑
            case (top_work_mode)
                TOP_MODE_STOP: begin
                    pwm_duty_left  <= 8'd0;
                    pwm_duty_right <= 8'd0;
                    left_motor_action  <= MOTOR_STOP;
                    right_motor_action <= MOTOR_STOP;
                    lost_timer <= 0;
                    lost_state <= LOST_IDLE;
                end
                
                TOP_MODE_REMOTE: begin
                    // 遥控模式 - 直接使用解析出的信号
                    pwm_duty_left <= (left_speed_reg == 8'h00) ? 8'd0 : left_speed_reg;
                    pwm_duty_right <= (right_speed_reg == 8'h00) ? 8'd0 : right_speed_reg;
                    
                    left_motor_action <= left_dir_reg;
                    right_motor_action <= right_dir_reg;
                    
                    lost_timer <= 0;
                    lost_state <= LOST_IDLE;
                end
                
                TOP_MODE_TRACK_AVOID: begin
                    // 循迹+避障模式
                    if (work_mode == MODE_AVOID) begin
                        pwm_duty_left  <= 8'd150;
                        pwm_duty_right <= 8'd150;
                        
                        case (ir_state)
                            2'b00: begin
                                left_motor_action  <= MOTOR_BACK;
                                right_motor_action <= MOTOR_BACK;
                            end
                            2'b01: begin
                                left_motor_action  <= MOTOR_FWD;  
                                right_motor_action <= MOTOR_STOP;   
                            end
                            2'b10: begin
                                left_motor_action  <= MOTOR_STOP;   
                                right_motor_action <= MOTOR_FWD;  
                            end
                            2'b11: begin
                                left_motor_action  <= MOTOR_FWD;
                                right_motor_action <= MOTOR_FWD;
                            end
                        endcase
                    end
                    else if (work_mode == MODE_LINE_FOLLOW) begin
                        if (lost_state == LOST_IDLE) begin
                            case (sensor_state)
                                2'b11: begin
                                    pwm_duty_left  <= 8'd110;
                                    pwm_duty_right <= 8'd110;
                                    left_motor_action  <= MOTOR_FWD;
                                    right_motor_action <= MOTOR_FWD;
                                end
                                2'b01: begin
                                    pwm_duty_left  <= 8'd150;     
                                    pwm_duty_right <= 8'd40;
                                    left_motor_action  <= MOTOR_FWD;  
                                    right_motor_action <= MOTOR_FWD;   
                                end
                                2'b10: begin
                                    pwm_duty_left  <= 8'd40;
                                    pwm_duty_right <= 8'd150;     
                                    left_motor_action <= MOTOR_FWD;   
                                    right_motor_action <= MOTOR_FWD;  
                                end
                                2'b00: begin
                                    pwm_duty_left  <= 8'd110;
                                    pwm_duty_right <= 8'd110;
                                    left_motor_action  <= MOTOR_FWD;
                                    right_motor_action <= MOTOR_FWD;
//                                    lost_timer <= 0;
//                                    lost_state <= LOST_PAUSE;
                                end
                            endcase
                        end
                        // 脱线处理
                        else if (lost_state == LOST_PAUSE) begin
                            lost_timer <= lost_timer + 1;
                            pwm_duty_left  <= 8'd0;
                            pwm_duty_right <= 8'd0;
                            left_motor_action  <= MOTOR_STOP;
                            right_motor_action <= MOTOR_STOP;
                            
                            if (lost_timer >= PAUSE_TIME) begin
                                lost_timer <= 0;
                                lost_state <= LOST_BACKWARD;
                            end
                        end
                        else if (lost_state == LOST_BACKWARD) begin
                            lost_timer <= lost_timer + 1;
                            pwm_duty_left  <= 8'd120;
                            pwm_duty_right <= 8'd120;
                            left_motor_action  <= MOTOR_BACK;
                            right_motor_action <= MOTOR_BACK;
                            
                            if (lost_timer >= BACKWARD_TIME) begin
                                lost_timer <= 0;
                                lost_state <= LOST_IDLE;
                            end
                        end
                    end
                end
                
                TOP_MODE_TRACK: begin
                    // 纯循迹模式
                    if (lost_state == LOST_IDLE) begin
                        case (sensor_state)
                            2'b11: begin
                                pwm_duty_left  <= 8'd110;
                                pwm_duty_right <= 8'd110;
                                left_motor_action  <= MOTOR_FWD;
                                right_motor_action <= MOTOR_FWD;
                            end
                            2'b01: begin
                                pwm_duty_left  <= 8'd150;     
                                pwm_duty_right <= 8'd40;
                                left_motor_action  <= MOTOR_FWD;  
                                right_motor_action <= MOTOR_FWD;   
                            end
                            2'b10: begin
                                pwm_duty_left  <= 8'd40;
                                pwm_duty_right <= 8'd150;     
                                left_motor_action <= MOTOR_FWD;   
                                right_motor_action <= MOTOR_FWD;  
                            end
                            2'b00: begin
                                pwm_duty_left  <= 8'd110;
                                pwm_duty_right <= 8'd110;
                                left_motor_action  <= MOTOR_FWD;
                                right_motor_action <= MOTOR_FWD;
//                                lost_timer <= 0;
//                                lost_state <= LOST_PAUSE;
                            end
                        endcase
                    end
                    // 脱线处理
                    else if (lost_state == LOST_PAUSE) begin
                        lost_timer <= lost_timer + 1;
                        pwm_duty_left  <= 8'd0;
                        pwm_duty_right <= 8'd0;
                        left_motor_action  <= MOTOR_STOP;
                        right_motor_action <= MOTOR_STOP;
                        
                        if (lost_timer >= PAUSE_TIME) begin
                            lost_timer <= 0;
                            lost_state <= LOST_BACKWARD;
                        end
                    end
                    else if (lost_state == LOST_BACKWARD) begin
                        lost_timer <= lost_timer + 1;
                        pwm_duty_left  <= 8'd120;
                        pwm_duty_right <= 8'd120;
                        left_motor_action  <= MOTOR_BACK;
                        right_motor_action <= MOTOR_BACK;
                        
                        if (lost_timer >= BACKWARD_TIME) begin
                            lost_timer <= 0;
                            lost_state <= LOST_IDLE;
                        end
                    end
                end
            endcase
        end
    end
    
    // ========== 方向控制信号生成 ==========
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            left_dir_go   <= 1'b0;
            left_dir_back <= 1'b0;
            right_dir_go  <= 1'b0;
            right_dir_back <= 1'b0;
        end
        else begin
            // 左电机
            case (left_motor_action)
                MOTOR_STOP: begin left_dir_go<=0; left_dir_back<=0; end
                MOTOR_FWD:  begin left_dir_go<=1; left_dir_back<=0; end
                MOTOR_BACK: begin left_dir_go<=0; left_dir_back<=1; end
                default:    begin left_dir_go<=0; left_dir_back<=0; end
            endcase

            // 右电机
            case (right_motor_action)
                MOTOR_STOP: begin right_dir_go<=0; right_dir_back<=0; end
                MOTOR_FWD:  begin right_dir_go<=1; right_dir_back<=0; end
                MOTOR_BACK: begin right_dir_go<=0; right_dir_back<=1; end
                default:    begin right_dir_go<=0; right_dir_back<=0; end
            endcase
        end
    end
    
    // ========== 输出分配 ==========
    assign left_motor_en   = left_pwm_out;
    assign right_motor_en  = right_pwm_out;
    assign left_motor_go   = left_dir_go;
    assign left_motor_back = left_dir_back;
    assign right_motor_go  = right_dir_go;
    assign right_motor_back = right_dir_back;
    
endmodule