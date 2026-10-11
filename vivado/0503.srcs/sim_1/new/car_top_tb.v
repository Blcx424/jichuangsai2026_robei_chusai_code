`timescale 1ns / 1ps
// ===========================================
// Testbench：测试 line_follower_top 帧解析完整链路
// 重点观察：rx_state / byte_cnt / frame_ready / top_work_mode
// ===========================================
module tb_line_follower_top;

    // ========== 时钟复位 ==========
    reg clk;
    reg rst_n;

    // ========== 输入 ==========
    reg  uart_rx;
    reg  sensor_left;
    reg  sensor_right;
    reg  ir_left;
    reg  ir_right;

    // ========== 输出 ==========
    wire left_motor_en;
    wire left_motor_go;
    wire left_motor_back;
    wire right_motor_en;
    wire right_motor_go;
    wire right_motor_back;
    wire led_0, led_1;
    wire led_left_motor, led_right_motor;
    wire led_left_sensor, led_right_sensor;
    wire led_blue_tooth;
    wire at_pin;
    wire [7:0] Rx_Data;

    // ========== 参数 ==========
    parameter BAUD       = 115200;
    parameter BIT_PERIOD = 1_000_000_000 / BAUD; // ~8680ns

    // ========== 实例化 ==========
    line_follower_top u_top (
        .clk              (clk),
        .rst_n            (rst_n),
        .sensor_left      (sensor_left),
        .sensor_right     (sensor_right),
        .uart_rx          (uart_rx),
        .at_pin           (at_pin),
//        .Rx_Data          (Rx_Data),
        .ir_left          (ir_left),
        .ir_right         (ir_right),
        .left_motor_en    (left_motor_en),
        .left_motor_go    (left_motor_go),
        .left_motor_back  (left_motor_back),
        .right_motor_en   (right_motor_en),
        .right_motor_go   (right_motor_go),
        .right_motor_back (right_motor_back),
        .led_0            (led_0),
        .led_1            (led_1),
        .led_left_motor   (led_left_motor),
        .led_right_motor  (led_right_motor),
        .led_left_sensor  (led_left_sensor),
        .led_right_sensor (led_right_sensor),
        .led_blue_tooth   (led_blue_tooth)
    );

    // ========== 时钟 50MHz ==========
    initial clk = 0;
    always #10 clk = ~clk;

    // ========== UART发送任务 ==========
    task uart_send_byte;
        input [7:0] data;
        integer j;
        begin
            uart_rx = 1'b0;
            #BIT_PERIOD;
            for (j = 0; j < 8; j = j + 1) begin
                uart_rx = data[j];
                #BIT_PERIOD;
            end
            uart_rx = 1'b1;
            #BIT_PERIOD;
            #BIT_PERIOD;
        end
    endtask

    task send_frame;
        input [7:0] cmd, lspd, rspd, ldir, rdir;
        begin
            $display("[%0t] === 发送帧: AA %02X %02X %02X %02X %02X 55 ===",
                     $time, cmd, lspd, rspd, ldir, rdir);
            uart_send_byte(8'hAA);
            uart_send_byte(cmd);
            uart_send_byte(lspd);
            uart_send_byte(rspd);
            uart_send_byte(ldir);
            uart_send_byte(rdir);
            uart_send_byte(8'h55);
            $display("[%0t] === 帧发送完毕 ===", $time);
        end
    endtask

    // ========== 主测试 ==========
    initial begin
        // 初始化所有输入
        uart_rx      = 1'b1;
        sensor_left  = 1'b1;
        sensor_right = 1'b1;
        ir_left      = 1'b1;
        ir_right     = 1'b1;
        rst_n        = 1'b0;
        #200;
        rst_n = 1'b1;
        #1000;

        // --- 测试1：正确帧，命令01（遥控模式）---
        $display("");
        $display("========================================");
        $display("测试1：AA 01 64 64 01 01 55（遥控模式）");
        $display("预期：frame_ready=1, top_work_mode=01");
        $display("========================================");
        send_frame(8'h01, 8'h64, 8'h64, 8'h01, 8'h01);
        #100000;

        // --- 测试2：正确帧，命令02（循迹+避障）---
        $display("");
        $display("========================================");
        $display("测试2：AA 02 80 80 01 01 55（循迹+避障）");
        $display("预期：frame_ready=1, top_work_mode=10");
        $display("========================================");
        send_frame(8'h02, 8'h80, 8'h80, 8'h01, 8'h01);
        #100000;

        // --- 测试3：帧尾错误（手动发送，不发固定的55）---
        $display("");
        $display("========================================");
        $display("测试3：AA 01 64 64 01 01 FF（帧尾错误）");
        $display("预期：frame_ready不置1，回到IDLE");
        $display("========================================");
        // 手动发7个字节，帧尾用 FF
        uart_send_byte(8'hAA);
        uart_send_byte(8'h01);
        uart_send_byte(8'h64);
        uart_send_byte(8'h64);
        uart_send_byte(8'h01);
        uart_send_byte(8'h01);
        uart_send_byte(8'hFF);   // ← 故意发错误帧尾
        $display("[%0t] === 错误帧发送完毕 ===", $time);
        #100000;

        // --- 测试4：错误帧后恢复 ---
        $display("");
        $display("========================================");
        $display("测试4：AA 03 50 50 01 01 55（纯循迹）");
        $display("预期：正常恢复，frame_ready=1, top_work_mode=11");
        $display("========================================");
        send_frame(8'h03, 8'h50, 8'h50, 8'h01, 8'h01);
        #100000;

        $display("");
        $display("所有测试完成");
        $finish;
    end

endmodule