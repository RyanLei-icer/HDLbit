// `timescale  1ns / 1ps
// `include "verilog_HDLbits.v"

// module tb_top_module;

// // top_module Parameters
// parameter PERIOD = 10  ;
// parameter A  = 1'b0;

// // top_module Inputs
// reg   clk                                  = 1 ;
// reg   reset                                = 0 ;
// reg   in                                   = 0 ;

// // top_module Outputs
// wire  out                                  ;

// reg next_state                             = A;

// initial
// begin
//     $dumpfile ("HDL_bit_wave.vcd");
//     $dumpvars;
//     forever #(PERIOD/2)  clk=~clk;
// end

// initial
// begin
//     #(PERIOD*8.5) reset  =  1;
//     #(PERIOD*1.5) reset  =  0;
// end

// top_module #(
//     .A ( A ))
//  u_top_module (
//     .clk                     ( clk     ),
//     .reset                   ( reset   ),
//     .in                      ( in      ),

//     .out                     ( out     )
// );

// initial
// begin
//     in = 1;
//     # 110;
//     in = 0;
//     # 30;
//     in = 1;
//     # 200;
//     $finish;
// end

// endmodule

// //~ lemmings3 testbench
// `timescale  1ns / 1ps
// `include "verilog_HDLbits.v"

// module tb_top_module;

// // top_module Parameters
// parameter PERIOD = 10  ;
// parameter WL  = 1'b0;

// // top_module Inputs
// reg   clk                                  = 1 ;
// reg   areset                               = 0 ;
// reg   bump_left                            = 0 ;
// reg   bump_right                           = 0 ;
// reg   ground                               = 1 ;
// reg   dig                                  = 0 ;

// // top_module Outputs
// wire  walk_left                            ;
// wire  walk_right                           ;
// wire  aaah                                 ;
// wire  digging                              ;


// initial
// begin
//     forever #(PERIOD/2)  clk=~clk;
// end

// initial
// begin
    
//     $dumpfile ("HDL_bit_wave.vcd");
//     $dumpvars;
//     #(PERIOD*2) areset  =  1;
//     #(PERIOD*2) areset  =  0;
// end

// top_module #(
//     .WL ( WL ))
//  u_top_module (
//     .clk                     ( clk          ),
//     .areset                  ( areset       ),
//     .bump_left               ( bump_left    ),
//     .bump_right              ( bump_right   ),
//     .ground                  ( ground       ),
//     .dig                     ( dig          ),

//     .walk_left               ( walk_left    ),
//     .walk_right              ( walk_right   ),
//     .aaah                    ( aaah         ),
//     .digging                 ( digging      )
// );

// initial
// begin
//     #300;
//     dig = 1;
//     #5
//     dig = 0;
//     #105; //410
//     bump_left = 1;
//     bump_right = 0;
//     #5;
//     bump_left = 0;
//     bump_right = 1;
//     #5;     //420
//     bump_right = 0;
//     dig = 1;
//     #5;
//     dig = 0;
//     ground = 0;
//     #5;//430
//     bump_right = 1;
//     #5;
//     bump_left = 1;
//     bump_right = 0;
//     ground = 1;
//     #5;
//     bump_left = 0;
//     #40;
//     #200;
//     $finish;
// end

// endmodule


// //~ `New testbench
// `timescale  1ns / 1ps
// `include "verilog_HDLbits.v"

// module top_module_tb;

// // top_module Parameters
// parameter PERIOD  = 10;


// // top_module Inputs
// reg   clk                                  = 0 ;
// reg   areset                               = 0 ;

// // top_module Outputs



// initial
// begin
//     forever #(PERIOD/2)  clk=~clk;
// end

// initial
// begin
//     $dumpfile ("HDL_bit_wave.vcd");
//     $dumpvars;
//     #(PERIOD*2) areset  =  1;
//     #(PERIOD*2) areset  =  0;
// end

// top_module  u_top_module (
//     .clk                     ( clk      ),
//     .areset                  ( areset   )
// );

// initial
// begin
//     #55;

//     $finish;
// end

// endmodule

// //~ `New testbench
// `timescale  1ns / 1ps
// `include "verilog_HDLbits.v"

// module tb_top_module;

// // top_module Parameters
// parameter PERIOD      = 10;
// parameter idle_start  = 0;

// // top_module Inputs
// reg   clk                                  = 0 ;
// reg   in                                   = 0 ;
// reg   reset                                = 0 ;

// // top_module Outputs
// wire  done                                 ;


// initial
// begin
//     forever #(PERIOD/2)  clk=~clk;
// end

// initial
// begin
//     $dumpfile ("HDL_bit_wave.vcd");
//     $dumpvars;
//     #(PERIOD*2) reset  =  1;
//     #(PERIOD*2) reset  =  0;
// end

// top_module 
//  u_top_module (
//     .clk                     ( clk     ),
//     .in                      ( in      ),
//     .reset                   ( reset   ),

//     .done                    ( done    )
// );

// initial
// begin
//     in=1;
//     #45;
//     in=1;
//     #30;
//     in=0;
//     #90;
//     in=1;
//     #20;
//     in=0;
//     #30;
//     in=1;
//     #140;
//     $finish;
// end

// endmodule

// //~ `New testbench
// `timescale  1ns / 1ps
// `include "verilog_HDLbits.v"
// module tb_top_module;

// // top_module Parameters
// parameter PERIOD      = 10;
// parameter idle_start  = 0;

// // top_module Inputs
// reg   clk                                  = 0 ;
// reg   in                                   = 0 ;
// reg   reset                                = 0 ;

// // top_module Outputs
// wire  [7:0]  out_byte                      ;
// wire  done                                 ;


// initial
// begin
//     forever #(PERIOD/2)  clk=~clk;
// end

// initial
// begin
//     $dumpfile ("HDL_bit_wave.vcd");
//     $dumpvars;
//     #(PERIOD*2) reset  =  1;
//     #(PERIOD*2) reset  =  0;
// end

// top_module 
//  u_top_module (
//     .clk                     ( clk             ),
//     .in                      ( in              ),
//     .reset                   ( reset           ),

//     .out_byte                ( out_byte  [7:0] ),
//     .done                    ( done            )
// );

// initial
// begin
//     in=1;
//     #45.001;
//     in=1;
//     #30;
//     in=0;
//     #90;
//     in=1;
//     #50;
//     in=0;
//     #30;
//     in=1;
//     #140;
//     $finish;
//     $finish;
// end

// endmodule

// //~ `New testbench
// `timescale  1ns / 1ps
// `include "verilog_HDLbits.v"
// module tb_top_module;

// // top_module Parameters
// parameter PERIOD = 10;
// parameter NONE   = 0;
// parameter ONE    = 1;
// parameter TWO    = 2;
// parameter THREE  = 3;
// parameter FOUR   = 4;
// parameter FIVE   = 5;
// parameter SIX    = 6;
// parameter DISC   = 7;
// parameter FLAG   = 8;
// parameter ERROR  = 9;

// // top_module Inputs
// reg   clk                                  = 1 ;
// reg   reset                                = 0 ;
// reg   in                                   = 0 ;

// // top_module Outputs
// wire  disc                                 ;
// wire  flag                                 ;
// wire  err                                  ;


// initial
// begin
//     forever #(PERIOD/2)  clk=~clk;
// end

// initial
// begin
//     $dumpfile ("HDL_bit_wave.vcd");
//     $dumpvars;
//     #(PERIOD*2) reset  =  1;
//     #(PERIOD*2) reset  =  0;
// end

// top_module #(
//     .NONE  ( NONE  ),
//     .ONE   ( ONE   ),
//     .TWO   ( TWO   ),
//     .THREE ( THREE ),
//     .FOUR  ( FOUR  ),
//     .FIVE  ( FIVE  ),
//     .SIX   ( SIX   ),
//     .DISC  ( DISC  ),
//     .FLAG  ( FLAG  ),
//     .ERROR ( ERROR ))
//  u_top_module (
//     .clk                     ( clk     ),
//     .reset                   ( reset   ),
//     .in                      ( in      ),

//     .disc                    ( disc    ),
//     .flag                    ( flag    ),
//     .err                     ( err     )
// );

// initial
// begin
//     in=0;#50.001;//仿真时为了符合实际，需要微小延时
//     in=1;#10;
//     in=0;#10;
//     in=1;#60;
//     in=0;#10;//flag

//     in=1;#70;
//     in=0;#10;//err

//     in=1;#10;
//     in=0;#10;
//     in=1;#50;
//     in=0;#30;//disc
//     $finish;
// end

// endmodule

//~ `New testbench
// `timescale  1ns / 1ps
// // `include "verilog_HDLbits.v"
// module tb_top_module;

// // top_module Parameters
// parameter PERIOD = 10;
// parameter A  = 0;

// // top_module Inputs
// reg   clk                                  = 1 ;
// reg   reset                                = 0 ;
// reg   s                                    = 0 ;
// reg   w                                    = 0 ;

// // top_module Outputs
// wire  z                                    ;


// initial
// begin
//     forever #(PERIOD/2)  clk=~clk;
// end

// initial
// begin
//     $dumpfile ("HDL_bit_wave.vcd");
//     $dumpvars;
//     #(PERIOD*2) reset  =  1;
//     #(PERIOD*2) reset  =  0;
// end

// top_module #(
//     .A ( A ))
//  u_top_module (
//     .clk                     ( clk     ),
//     .reset                   ( reset   ),
//     .s                       ( s       ),
//     .w                       ( w       ),

//     .z                       ( z       )
// );

// initial
// begin
//     s=0;#60.001;
//     s=1;#20;
//     w=1;#20;
//     w=0;#10
//     s=1;#120;
//     $finish;
// end

// endmodule

// `timescale 1ns/1ps
// module exam_task_func();
// 	reg 		sclk;
// 	reg [7:0] 	func_and_d;
//     reg [7:0] 	task_and_d;
    
//     initial begin
//         $dumpfile ("HDL_bit_wave.vcd");
//         $dumpvars;
// 		sclk =0;#100
// 		func_and_d = func_and( 255,1);#100
// 		task_and(255,1,task_and_d);
//         #200;$finish;
//     end
    
// 	always #10 sclk = ~sclk ;
    
//     function[7:0] func_and;	//不可调用task
//         input	[7:0]	a;
// 		input	[7:0]	b;
        
//         begin
// 			func_and=a & b;	//只能组合逻辑运算、不可添加时延
//         end
// 	endfunction
    
// 	task task_and;
//         input 	[7:0] 	a;
//         input 	[7:0] 	b;
//         output	[7:0] 	c;
//         begin
//             @(posedge sclk);
//             #0.1 c=a&b;
// 		end
// 	endtask
// endmodule

// //~ `New testbench
// `timescale  1ns / 1ps
// `include "verilog_HDLbits.v"
// module tb_multi_sel;

// // multi_sel Parameters
// parameter PERIOD  = 10;


// // multi_sel Inputs
// reg   [7:0]  d                             = 0 ;
// reg   clk                                  = 0 ;
// reg   rst                                  = 0 ;

// // multi_sel Outputs
// wire  input_grant                          ;
// wire  [10:0]  out                          ;


// initial
// begin
//     $dumpfile ("HDL_bit_wave.vcd");
//     $dumpvars;
//     forever #(PERIOD/2)  clk=~clk;
// end

// initial
// begin
//     rst=0;#10;
//     #(PERIOD*2) rst  =  1;
// end

// multi_sel  u_multi_sel (
//     .d                       ( d          ),
//     .clk                     ( clk        ),
//     .rst                     ( rst        ),
//     .input_grant             ( input_grant),
//     .out                     ( out        )
// );

// initial
// begin
//     d=0;#10;
//     d=1;#100;
//     d=5;#100;
//     d=2;#70;
//     d=3;#200;
//     $finish;
// end

// endmodule

// `timescale 1ns / 1ps
// `include "verilog_HDLbits.v"
// //****************************************VSCODE PLUG-IN**********************************//
// //----------------------------------------------------------------------------------------
// // IDE :                   VSCODE plug-in 
// // VSCODE plug-in version: Verilog-Hdl-Format-2.7.20240716
// // VSCODE plug-in author : Jiang Percy
// //----------------------------------------------------------------------------------------
// //****************************************Copyright (c)***********************************//
// // Copyright(C)            Please Write Company name
// // All rights reserved     
// // File name:              pulse_detect_tb.v
// // Last modified Date:     2024/07/23 17:05:55
// // Last Version:           V1.0
// // Descriptions:           
// //----------------------------------------------------------------------------------------
// // Created by:             Please Write You Name 
// // Created date:           2024/07/23 17:05:55
// // Version:                V1.0
// // Descriptions:           
// //                         
// //----------------------------------------------------------------------------------------
// //****************************************************************************************//

// module    pulse_detect_tb();
//     reg                                        clka                       ;
//     reg                                        clkb                       ;
//     reg                                        rst_n                      ;
//     reg                                        sig_a                      ;
//     wire                                       sig_b                      ;



//     initial
//         begin
//             #2                                             
//                     rst_n = 0   ;                          
//                     clka  = 0   ;                          
//                     clkb  = 0   ;   
//             #10                                            
//                     rst_n = 1   ;                          
//         end                                                
                                                           
//     parameter   CLKa_FREQ = 100;//Mhz                       
//     parameter   CLKb_FREQ = 40; //Mhz  
//     always # ( 1000/CLKa_FREQ/2 ) clka = ~clka ;              
//     always # ( 1000/CLKb_FREQ/2 ) clkb = ~clkb ; 
                                                           
// pulse_detect u_pulse_detect(
//     .clka                               (clka                      ),
//     .clkb                               (clkb                      ),
//     .rst_n                              (rst_n                     ),
//     .sig_a                              (sig_a                     ),
//     .sig_b                              (sig_b                     )
// );

// initial
// begin
//     sig_a=0;#10;
//     sig_a=1;#100;
//     sig_a=0;#100;
//     sig_a=1;#70;
//     sig_a=0;#200;
//     $finish;
// end
// initial begin
//         $dumpfile ("HDL_bit_wave.vcd");
//         $dumpvars;
// end

// endmodule                                                  
`timescale 1ns / 1ps
//****************************************VSCODE PLUG-IN**********************************//
//----------------------------------------------------------------------------------------
// IDE :                   VSCODE plug-in 
// VSCODE plug-in version: Verilog-Hdl-Format-2.7.20240716
// VSCODE plug-in author : Jiang Percy
//----------------------------------------------------------------------------------------
//****************************************Copyright (c)***********************************//
// Copyright(C)            Please Write Company name
// All rights reserved     
// File name:              pul_sync_wrapper_tb.v
// Last modified Date:     2024/07/23 17:37:00
// Last Version:           V1.0
// Descriptions:           
//----------------------------------------------------------------------------------------
// Created by:             Please Write You Name 
// Created date:           2024/07/23 17:37:00
// Version:                V1.0
// Descriptions:           
//                         
//----------------------------------------------------------------------------------------
//****************************************************************************************//
`include "verilog_HDLbits.v"
module    pul_sync_wrapper_tb();
    reg                                        clk_src                    ;
    reg                                        clk_des                    ;
    reg                                        rst_b                      ;
    reg                                        d_in                       ;
    wire                                       d_sync_pos                 ;
    wire                                       d_sync_neg                 ;



    initial
        begin
            #2                                             
                    rst_b = 0   ;                          
                    clk_src = 0 ;
                    clk_des = 0 ;                          
            #10                                            
                    rst_b = 1   ;                          
        end                                                
                                                           
    parameter   CLKa_FREQ = 100;//Mhz                       
    parameter   CLKb_FREQ = 80; //Mhz                     
    always # ( 1000/CLKa_FREQ/2 ) clk_src = ~clk_src ;              
    always # ( 1000/CLKb_FREQ/2 ) clk_des = ~clk_des ; 
                                                           
pul_sync_wrapper u_pul_sync_wrapper(
    .clk_src                            (clk_src                   ),
    .clk_des                            (clk_des                   ),
    .rst_b                              (rst_b                     ),
    .d_in                               (d_in                      ),
    .d_sync_pos                         (d_sync_pos                ),
    .d_sync_neg                         (d_sync_neg                )
);

initial
    begin
        d_in = 0;
        #40;
        d_in = 1;
        #10;
        d_in = 0;
        #500;
        $finish;
    end
initial begin
        $dumpfile ("HDL_bit_wave.vcd");
        $dumpvars;
end



endmodule                                                  