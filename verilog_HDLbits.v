// $dumpfile ("HDL_bit_wave.vcd");
// $dumpvars;

// // 3-bits LFSR
// module top_module (
// 	input [2:0] SW,      // R
// 	input [1:0] KEY,     // L and clk
// 	output [2:0] LEDR);  // Q
// 
// wire [2:0] d;
// 
// DFF_my DFF_3 [2:0] (.clk	(KEY[0]),
// 	    .D		(d),
// 	    .Q		(LEDR));
// always@(*)begin
// if (KEY[1] == 1'b1)
// d = SW;
// else
// d = {LEDR[1] ^ LEDR[2],LEDR[0],LEDR[2]};
// end
// endmodule
// 
// module DFF_my (
// input clk,
// input D,
// output Q
// 	);
// always@(posedge clk)
// Q <= D;
// endmodule

// // shifter register
// module top_module (
// input [3:0] SW,
// input [3:0] KEY,
// output [3:0] LEDR
// );
// // Connect the R inputs to the SW switches,
// // clk to KEY[0],
// // E to KEY[1],
// // L to KEY[2], and
// // w to KEY[3].
// // Connect the outputs to the red lights LEDR[3:0].
// 
// MUXDFF DFF_4 [3:0] (.clk    (KEY[0]),
// .E      (KEY[1]),
// .R      (SW),
// .L      (KEY[2]),
// .Q_D    ({KEY[3],LEDR[3:1]}),
// .Q      (LEDR)
// );
// endmodule
// 
// module MUXDFF (
// input	clk ,
// 	input 	E   ,
// 	input 	R   ,
// 	input 	L   ,
// 	input 	Q_D ,
// 	output 	Q)  ;
// 
// wire w1,D   ;
// 
// always@(*)begin
// if (E == 1'b0)begin
// w1 = Q          ;
// end
// else begin
// w1 = Q_D        ;
// end
// 
// if (L == 1'b0)begin
// D = w1          ;
// end
// else begin
// D = R           ;
// end
// end
// 
// always@(posedge clk)begin
// Q <= D              ;
// end
// 
// endmodule


// // 3-input LUT
// module top_module (
// input clk       ,
// input enable    ,
// input S         ,
// input A, B, C   ,
// output Z )      ;
// 
// wire [7:0] q    ;
// 
// DFF_my DFF_8 [7:0] (.clk    (clk)   ,
// .enable (enable)   ,
// .D      ({S,q[7:1]})   ,
// .Q      ({q})
// )                       ;
// always@(*)begin
// case ({A,B,C})
// 3'h0:Z = q[7];
// 3'h1:Z = q[6];
// 3'h2:Z = q[5];
// 3'h3:Z = q[4];
// 3'h4:Z = q[3];
// 3'h5:Z = q[2];
// 3'h6:Z = q[1];
// 3'h7:Z = q[0];
// endcase
// end
// endmodule
// 
// module DFF_my (
// input clk       ,
// input enable    ,
// input D         ,
// output Q
// 	)               ;
// always@(posedge clk)begin
// if (enable == 1'b0)begin
// Q <= Q      ;
// end
// else begin
// Q <= D      ;
// end
// end
// endmodule
// // 3-input LUT
// module top_module (
// 	input clk,
// 	input enable,
// 	input S,
// 
// 	input A, B, C,
// 	output reg Z
// );
// 	reg [7:0] q;
// 	// The final circuit is a shift register attached to a 8-to-1 mux.
// 
// 	// Create a 8-to-1 mux that chooses one of the bits of q based on the three-bit number {A,B,C}:
// 	// There are many other ways you could write a 8-to-1 mux
// 	// (e.g., combinational always block -> case statement with 8 cases).
// 	assign Z = q[{A, B, C}];
// 
// 	// Edge-triggered always block: This is a standard shift register (named q) with enable.
// 	// When enabled, shift to the left by 1 (discarding q[7] and and shifting in S).
// 	always @(posedge clk) begin
// 		if (enable)
// 			q <= {q[6:0], S};
// 	end
// endmodule

// // Rule-90
// module top_module(
// input clk,
// input load,
// input [511:0] data,
// output [511:0] q );
// 
// always@(posedge clk)begin
// if (load == 1'b1)begin
// q <= data;
// end
// else begin
// q <= {1'b0,q[511:1]}^{q[510:0],1'b0};
// end
// end
// endmodule

// // Relu-110
// module top_module(
// input clk,
// input load,
// input [511:0] data,
// output [511:0] q );
// 
// always@(posedge clk)begin
// if (load == 1'b1)begin
// q <= data;
// end
// else begin
// q < = {q}&~{q[510:0],1'b0} | ~{1'b0,q[511:1]}&{q[510:0],1'b0} | ~{q}&{q[510:0],1'b0};// Q* = CR'+L'R+C'R
// end
// end
// endmodule


// // Conwaylife
// module top_module(
// input clk,
// input load,
// input [255:0] data,
// output [255:0] q );
// 
// reg [3:0] x,y;
// reg [4:0] i,j;
// reg [3:0] num_adjacent;
// reg [255:0] q_next;
// 
// always@(posedge clk)begin
// for(i = 0;i < = 15;i = i+5'b1)begin
// for(j = 0;j < = 15;j = j+5'b1)begin// 不改变数据大小实现verilog位扩展
// x = i[3:0];y = j[3:0];
// num_adjacent = q[{x-1,4'b0}+(y-1)] + q[{x,4'b0}+(y-1)] + q[{x+1,4'b0}+(y-1)]
// 			   +q[{x-1,4'b0}+y]            +              q[{x+1,4'b0}+y]
// +q[{x-1,4'b0}+(y+1)] + q[{x,4'b0}+(y+1)] + q[{x+1,4'b0}+(y+1)];
// case(num_adjacent)
// 4'b0010:;
// 4'b0011:q[{x,4'b0}+y] <= 1'b1;
// default:q[{x,4'b0}+y] <= 1'b0;
// endcase
// end
// end
// end
// endmodule


// // Fsm1
// module top_module(
// input clk,
// input areset,    // Asynchronous reset to state B
// input in,
// output out);// 

// parameter A = 0, B = 1;
// reg state, next_state;

// always @(*) begin    // This is a combinational always block
// if (in == 1'b1)begin
// next_state = (state == A) ? A : B;// 注意next_state要赋初值，否则默认全为1或为0.
// end// State transition logic
// else if (state == A)begin
// next_state = B;
// end
// else begin
// next_state = A;
// end
// end

// always @(posedge clk, posedge areset) begin    // This is a sequential always block
// if (areset == 1'b1)begin// State flip-flops with asynchronous reset
// state <= B;
// end
// else begin
// state <= next_state;
// end
// end

// // Output logic
// assign out = (state == B);

// endmodule

// // Fsm1a
// module top_module(
// input clk,
// input areset,    // Asynchronous reset to state B
// input in,
// output out);// 

// parameter A = 1'b0, B = 1'b1;
// reg state, next_state;

// always@(*) begin
// 	case (state)
// 		A: next_state = in ? A : B;
// 		B: next_state = in ? B : A;
// 	endcase
// end

// always @(posedge clk, posedge areset) begin    // This is a sequential always block
// if (areset == 1'b1)begin// State flip-flops with asynchronous reset
// state <= B;
// end
// else begin
// state <= next_state;
// end
// end

// // Output logic
// assign out = (state == B);

// endmodule

// // Fsm1s(三段式状态机)
// // Note the Verilog-1995 module declaration syntax here:
// module top_module(clk,
// reset,
// in,
// out);
// input clk;
// input reset;  // Synchronous reset to state B
// input in;
// output out;   // 

// // Fill in state name declarations
// parameter A = 1'b0, B = 1'b1;
// reg present_state, next_state;

// always@(*) begin
// 	case (present_state)
// 		A: next_state = in ? A : B;
// 		B: next_state = in ? B : A;
// 	endcase
// end// 状态跳转逻辑，根据输入信号以及当前状态确定状态的次态。

// always @(posedge clk) begin
// 	if (reset) present_state <= B;		// Reset to state B
// else present_state <= next_state;
// end// 状态触发器实现，在时钟边沿实现状态寄存器的跳变以及状态复位

// assign out = (present_state == B);// 输出逻辑，根据当前状态实现输出
// endmodule


// // Fsm2
// module top_module(input clk,
// input areset,  // Asynchronous reset to OFF
// input j,
// input k,
// output out);

// // Fill in state name declarations
// parameter OFF = 0, ON = 1;
// reg state, next_state;

// always@(*) begin
// case (state)
// OFF: next_state = j ? ON  : OFF;
// ON : next_state = k ? OFF : ON ;
// endcase
// end  // 状态跳转逻辑，根据输入信号以及当前状态确定状态的次态。

// always @(posedge clk, posedge areset) begin
// if (areset) state <= OFF;		  // Reset to state OFF
// else state <= next_state;
// end  // 状态触发器实现，在时钟边沿实现状态寄存器的跳变以及状态复位

// assign out = (state == ON);  // 输出逻辑，根据当前状态实现输出
// endmodule


// // Fsm2s
// module top_module(input clk,
// input reset,  // synchronous reset to OFF
// input j,
// input k,
// output out);

// // Fill in state name declarations
// parameter OFF = 0, ON = 1;
// reg state, next_state;

// always@(*) begin
// case (state)
// OFF: next_state = j ? ON  : OFF;
// ON : next_state = k ? OFF : ON ;
// endcase
// end  // 状态跳转逻辑，根据输入信号以及当前状态确定状态的次态。

// always @(posedge clk) begin
// if (reset) state <= OFF;		  // Reset to state OFF
// else state <= next_state;
// end  // 状态触发器实现，在时钟边沿实现状态寄存器的跳变以及状态复位

// assign out = (state == ON);  // 输出逻辑，根据当前状态实现输出
// endmodule


// // Fsm3comb
// module top_module(input in,
// input [1:0] state,
// output reg [1:0] next_state,// 注意此处的reg，否则always报错
// output out);

// // Fill in state name declarations
// parameter A = 2'b00, B = 2'b01, C = 2'b10, D = 2'b11;

// always@(*) begin
// case (state)
// A: next_state = in ? B : A;
// B: next_state = in ? B : C;
// C: next_state = in ? D : A;
// D: next_state = in ? B : C;
// endcase
// end  // 状态跳转逻辑，根据输入信号以及当前状态确定状态的次态。

// assign out = (state == D);  // 输出逻辑，根据当前状态实现输出
// endmodule


// // Fsm3onehot
// module top_module(input in,
// input [3:0] state,
// output [3:0] next_state,
// output out);

// parameter A = 4'b0000, B = 4'b0001, C = 4'b0010, D = 4'b0011;

// // State transition logic: Derive an equation for each state flip-flop.
// assign next_state[A] = state[A]&(~in) | state[C]&(~in);
// assign next_state[B] = state[A]&(in)  | state[B]&(in) | state[D]&(in);
// assign next_state[C] = state[B]&(~in) | state[D]&(~in);
// assign next_state[D] = state[C]&(in) ;

// // Output logic:
// assign out = (state[D] == 1'b1);
// endmodule


// // Fsm3
// module top_module(input clk,
// input in,
// input areset,
// output out);

// // Fill in state name declarations
// parameter A = 2'b00, B = 2'b01, C = 2'b10, D = 2'b11;
// reg [1:0] state,next_state;

// // State transition logic
// always @(*) begin
// case(state)
// A: next_state = in ? B : A;
// B: next_state = in ? B : C;
// C: next_state = in ? D : A;
// D: next_state = in ? B : C;
// endcase
// end

// // State flip-flops with asynchronous reset
// always @(posedge clk or posedge areset) begin
// if (areset == 1'b1)
// state      <= A;
// else state <= next_state;
// end
// // Output logic
// assign out = (state == D);
// endmodule


// // Fsm3s
// module top_module(input clk,
// input in,
// input reset,
// output out);  // 

// parameter A = 2'b00, B = 2'b01, C = 2'b10, D = 2'b11;
// reg [1:0] state,next_state;
// // State transition logic
// always @(*) begin
// case (state)
// A: next_state = in ? B : A;
// B: next_state = in ? B : C;
// C: next_state = in ? D : A;
// D: next_state = in ? B : C;
// endcase
// end
// // State flip-flops with synchronous reset
// always @(posedge clk) begin
// if (reset == 1'b1)
// state <= A;
// else
// state <= next_state;
// end
// // Output logic
// assign out = (state == D);
// endmodule


// // Exams/ece241 2013 q4
// module top_module (input clk,
// input reset,
// input [3:1] s,
// output fr3,
// output fr2,
// output fr1,
// output dfr);
// parameter BL1 = 2'b00,BT12 = 2'b01,BT23 = 2'b10,BH3 = 2'b11;

// reg [1:0] previous_state,current_state;  // 四个状态2bit
// reg level_lower;

// // State transition logic
// always @(*) begin
// case (previous_state)
// BL1 :current_state = s[1] ? BT12 : BL1;
// BT12:current_state = s[2] ? BT23 : (s[1] ? BT12 : BL1);
// BT23:current_state = s[3] ? BH3  : (s[2] ? BT23 : BT12);
// BH3 :current_state = s[3] ? BH3  : BT23;
// endcase
// end

// // State flip-flops with synchronous reset
// always @(posedge clk) begin
// if (reset)begin
// previous_state <= BL1;
// level_lower    <= 1'b0;
// end
// else begin previous_state <= current_state;
// level_lower <= (previous_state > current_state) ? 1'b1 :
// ((previous_state < current_state) ? 1'b0 : level_lower);
// end
// end

// // Output logic
// assign fr1 = (previous_state < BH3);
// assign fr2 = (previous_state < BT23);
// assign fr3 = (previous_state < BT12);
// assign dfr = (level_lower || previous_state == BL1);
// endmodule


// // lemmings1
// module top_module(input clk,
// input areset,  // Freshly brainwashed Lemmings walk left.
// input bump_left,
// input bump_right,
// output walk_left,
// output walk_right);  // 

// parameter LEFT = 1'b0, RIGHT = 1'b1;
// reg state, next_state;

// always @(*) begin
// case ({bump_left,bump_right})
// 2'b11:next_state = ~state;
// 2'b10:next_state = RIGHT;
// 2'b01:next_state = LEFT;
// 2'b00:next_state = state;
// endcase
// // State transition logic
// end

// always @(posedge clk, posedge areset) begin
// if (areset)begin
// state <= LEFT;
// end
// else begin
// state <= next_state;
// end
// // State flip-flops with asynchronous reset
// end

// // Output logic
// assign walk_left  = (state == LEFT);
// assign walk_right = (state == RIGHT);
// endmodule


// // lemmings2
// module top_module(input clk,
// input areset,  // Freshly brainwashed Lemmings walk left.
// input bump_left,
// input bump_right,
// input ground,
// output walk_left,
// output walk_right,
// output aaah);

// parameter LEFT = 1'b0, RIGHT = 1'b1;
// reg state, next_state, FALL_DOWN;

// always @(*) begin
// if (~ground || FALL_DOWN) begin
// next_state = state;
// end
// else case ({bump_left,bump_right})
// 2'b11:next_state = ~state;
// 2'b10:next_state = RIGHT;
// 2'b01:next_state = LEFT;
// 2'b00:next_state = state;
// endcase
// // State transition logic
// end

// always @(posedge clk, posedge areset) begin
// if (areset)begin
// state     <= LEFT;
// FALL_DOWN <= 1'b0;
// end
// else if (~ground) begin
// FALL_DOWN <= 1'b1;
// end
// else begin
// state     <= next_state;
// FALL_DOWN <= 1'b0;
// end
// // State flip-flops with asynchronous reset
// end

// // Output logic
// assign walk_left  = ((state == LEFT) && (~FALL_DOWN));
// assign walk_right = ((state == RIGHT) && (~FALL_DOWN));
// assign aaah       = (FALL_DOWN == 1'b1);
// endmodule


// // lemmings3(error-not state machine)
// module top_module(input clk,
// input areset,  // Freshly brainwashed Lemmings walk left.
// input bump_left,
// input bump_right,
// input ground,
// input dig,
// output walk_left,
// output walk_right,
// output aaah,
// output digging);

// parameter LEFT = 1'b0, RIGHT = 1'b1;
// reg state, next_state;
// reg FALL_DOWN,DIG;

// always @(*) begin
// if (~ground || FALL_DOWN || DIG) begin
// next_state = state;
// end
// else case ({bump_left,bump_right})
// 2'b11:next_state = ~state;
// 2'b10:next_state = RIGHT;
// 2'b01:next_state = LEFT;
// 2'b00:next_state = state;
// endcase
// // State transition logic
// end

// always @(posedge clk, posedge areset) begin
// if (areset)begin
// state     <= LEFT;
// FALL_DOWN <= 1'b0;
// DIG       <= 1'b0;
// end
// else if (~ground) begin
// FALL_DOWN <= 1'b1;
// DIG       <= 1'b0;
// end
// else begin
// state     <= next_state;
// FALL_DOWN <= 1'b0;
// DIG       <= (dig && ~FALL_DOWN) ? 1'b1 : DIG;
// end
// // State flip-flops with asynchronous reset
// end

// // Output logic
// assign walk_left  = ((state == LEFT) && (~FALL_DOWN) && (~DIG));
// assign walk_right = ((state == RIGHT) && (~FALL_DOWN) && (~DIG));
// assign aaah       = (FALL_DOWN == 1'b1);
// assign digging    = (DIG == 1'b1);
// endmodule


// // lemmings3
// module top_module(input clk,
// input areset,  // Freshly brainwashed Lemmings walk left.
// input bump_left,
// input bump_right,
// input ground,
// input dig,
// output walk_left,
// output walk_right,
// output aaah,
// output digging);

// parameter WL = 0,WR = 1,FL = 2,FR = 3,DL = 4,DR = 5;
// reg [2:0]state,next_state;

// // State transition logic
// always @(*) begin
// case (state)
// WL: next_state = ground ? (dig ? DL : (bump_left  ? WR : WL)) : FL;
// WR: next_state = ground ? (dig ? DR : (bump_right ? WL : WR)) : FR;
// FL: next_state = ground ? WL : FL;
// FR: next_state = ground ? WR : FR;
// DL: next_state = ground ? DL : FL;
// DR: next_state = ground ? DR : FR;
// endcase
// end

// // State flip-flops with asynchronous reset
// always @(posedge clk or posedge areset) begin
// if (areset)
// state <= WL;
// else
// state <= next_state;// 注意，复位逻辑要用if，不能用case
// // case (areset)
// // 1'b0: state = next_state;
// // 1'b1: state = WL;
// // endcase
// end

// // Output logic
// assign walk_left  = (state == WL);
// assign walk_right = (state == WR);
// assign aaah       = ((state == FL) || (state == FR));
// assign digging    = ((state == DL) || (state == DR));
// endmodule


// // lemmings4
// module top_module(input clk,
//                   input areset,  // Freshly brainwashed Lemmings walk left.
//                   input bump_left,
//                   input bump_right,
//                   input ground,
//                   input dig,
//                   output walk_left,
//                   output walk_right,
//                   output aaah,
//                   output digging);
    
//     parameter WL = 0,WR = 1,DL = 2,DR = 3,FL = 4,FR = 5,OVER = 6;
//     reg [3:0]out;
//     reg [7:0]cnt;
//     reg [2:0]state,next_state;
    
//     // State transition logic
//     always @(*) begin
//         case (state)
//             WL:   next_state      = ground ? (dig ? DL : (bump_left  ? WR : WL)) : FL;
//             WR:   next_state      = ground ? (dig ? DR : (bump_right ? WL : WR)) : FR;
//             FL:   next_state      = ground ? (cnt >= 21 ? OVER : WL) : FL;
//             FR:   next_state      = ground ? (cnt >= 21 ? OVER : WR) : FR;
//             DL:   next_state      = ground ? DL : FL;
//             DR:   next_state      = ground ? DR : FR;
//             OVER: next_state      = OVER;  
//             default: next_state = WL;
//         endcase
//     end
    
//     // State flip-flops with asynchronous reset
//     always@(posedge clk or posedge areset) begin
//         if (areset) begin
//             out <= 4'b1000;
//         end
//         else
//             case(next_state)
//                 WL:begin
//                     out <= 4'b1000;
//                     cnt <= 0;
//                 end
//                 WR:begin
//                     out <= 4'b0100;
//                     cnt <= 0;
//                 end
//                 DL:begin
//                     out <= 4'b0001;
//                     cnt <= 0;
//                 end
//                 DR:begin
//                     out <= 4'b0001;
//                     cnt <= 0;
//                 end
//                 FL:begin
//                     out <= 4'b0010;
//                     cnt = cnt + 1;
//                 end
//                 FR:begin
//                     out <= 4'b0010;
//                     cnt = cnt + 1;
//                 end
//                 OVER:out <= 4'b0000;
//             endcase
//     end
    
//     assign {walk_left, walk_right, aaah, digging} = out;
// endmodule

// // one-hot FSM
// module top_module(
//     input in,
//     input [9:0] state,
//     output [9:0] next_state,
//     output out1,
//     output out2);

// parameter s0 = 4'd0, s1 = 4'd1, s2 = 4'd2, s3 = 4'd3, s4 = 4'd4;
// parameter s5 = 4'd5, s6 = 4'd6, s7 = 4'd7, s8 = 4'd8, s9 = 4'd9; //第几位

// // State transition logic: Derive an equation for each state flip-flop.
// assign next_state[s0] = (|state[s4:s0])&(~in) | (|state[s9:s7])&(~in);
// assign next_state[s1] = state[s0]&(in)  | (|state[s9:s8])&(in);
// assign next_state[s2] = state[s1]&(in);
// assign next_state[s3] = state[s2]&(in);
// assign next_state[s4] = state[s3]&(in);
// assign next_state[s5] = state[s4]&(in);
// assign next_state[s6] = state[s5]&(in);
// assign next_state[s7] = (|state[s7:s6])&(in);
// assign next_state[s8] = state[s5]&(~in);
// assign next_state[s9] = state[s6]&(~in);

// // Output logic:
// assign out1 = (state[s8] || state[s9]);
// assign out2 = (state[s7] || state[s9]);

// endmodule

// //Fsm ps2
// module top_module(
//     input clk,
//     input [7:0] in,
//     input reset,    // Synchronous reset
//     output done); //

//     parameter byte1 = 0, byte2 = 1, byte3 = 2, DONE = 3;

//     reg [1:0]state,next_state;

//     // State transition logic (combinational)
//     always @(*) begin
//         case (state)
//             byte1: next_state = in[3] ? byte2 : byte1;
//             byte2: next_state = byte3;
//             byte3: next_state = DONE;
//             DONE : next_state = in[3] ? byte2 : byte1;
//         endcase
//     end

//     // State flip-flops (sequential)
//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             state <= byte1;
//         end
//         else begin
//             state <= next_state;
//         end
//     end
//     // Output logic
//     assign done = state == DONE;

// endmodule


// Fsm ps2data
// 既然您有了一个状态机，可以识别PS\/2字节流中的三字节消息，那么添加一个数据路径，每当接收到数据包时，
// 它也会输出24位（3字节）消息（out_bytes[23:16]是第一个字节，out_bytes[15:8]是第二个字节，等等）。
// 无论何时断言done信号，out_bytes都需要是有效的。您可以在其他时间输出任何内容（即，不关心）。

 // FSM from fsm_ps2
// New: Datapath to store incoming bytes.

// module top_module(
//     input clk,
//     input [7:0] in,
//     input reset,    // Synchronous reset
//     output [23:0] out_bytes,
//     output done); //

//     parameter byte1 = 0, byte2 = 1, byte3 = 2, DONE = 3;

//     reg [1:0]state,next_state;
//     reg [23:0] outs;

//     // State transition logic (combinational)
//     always @(*) begin
//         case (state)
//             byte1: begin
//                 next_state = in[3] ? byte2 : byte1;
//             end
//             byte2: begin
//                 next_state = byte3;
//             end
//             byte3: begin
//                 next_state = DONE;
//             end
//             DONE : begin
//                 next_state = in[3] ? byte2 : byte1;
//             end
//         endcase
//     end

//     // State flip-flops (sequential)
//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             state <= byte1;
//             outs  <= 0;
//         end
//         else begin
//             state <= next_state;
//             case (state)
//             byte1: outs[23:16] = in;
//             byte2: outs[15:8] = in;
//             byte3: outs[7:0] = in;
//             DONE : outs[23:16] = in;
            
//         endcase
//         end
//     end
//     // Output logic
//     assign done = (state == DONE);
//     assign out_bytes = done ? outs : 0;
// endmodule

// Fsm serial
// 在许多(较老的)串行通信协议中，每个数据字节与一个开始位和一个停止位一起发送，以帮助接收方从位流中划分字节。
// 一种常见的方案是使用一个起始位(0)、8个数据位和1个停止位(1)。当没有传输任何内容(空闲)时，该行也处于逻辑1。
// 设计一个有限状态机，当给定一个位流时，该状态机将识别何时正确接收了字节。它需要识别开始位，等待所有8个数据位，
// 然后验证停止位是正确的。如果停止位没有在预期的时间出现，FSM 必须等待，直到找到一个停止位，然后再尝试接收下一个字节。
// module top_module(
//     input clk,
//     input in,
//     input reset,    // Synchronous reset
//     output reg done
// ); 

// parameter idle_start = 0, data = 1, stop = 2, error = 3;

// reg [2:0] state,next_state;
// reg [3:0] cnt;

// // State transition logic (combinational)
// always @(*) begin
//     case (state)
//         idle_start : next_state = (~in    ) ? data        : idle_start;
//         data       : next_state = (cnt==8 ) ? stop        : data;
//         stop       : next_state = (in     ) ? idle_start  : error;
//         error      : next_state = (in     ) ? idle_start  : error;
//     endcase
// end

// // State flip-flops (sequential)
// always @(posedge clk) begin
//     if(reset) begin
//         state <= idle_start;
//         cnt   <= 0;
//     end
//     else if(state ==error || cnt == 9)begin
//         cnt     <= 0;
//         state   <= next_state;
//     end
//     else if(next_state != idle_start) begin
//         cnt     <= cnt + 1'b1;
//         state   <= next_state;
//     end
// end

// // Output logic
// always @(posedge clk or posedge reset) begin
//     if(reset) begin
//         done  <= 0;
//     end
//     else 
//         done  <= (state == stop) && (next_state == idle_start);
// end
// endmodule

// Fsm serialdata
// 现在您已经有了一个能够识别在串行位流中何时正确接收字节的有限状态机，添加一个数据路径
// 来输出正确接收的数据字节。Out _ byte 在完成时需要是有效的，并且不关心其他情况。
// 请注意，串行协议首先发送最低有效位。
// Use FSM from Fsm_serial
// New: Datapath to latch input bits.
// module top_module(
//     input clk,
//     input in,
//     input reset,    // Synchronous reset
//     output [7:0] out_byte,
//     output reg done
// ); 

// parameter idle_start = 0, data = 1, stop = 2, error = 3;

// reg [2:0] state,next_state;
// reg [3:0] cnt;
// reg [7:0] out_r;

// // State transition logic (combinational)
// always @(*) begin
//     case (state)
//         idle_start : next_state = (~in    ) ? data        : idle_start;
//         data       : next_state = (cnt==8 ) ? stop        : data;
//         stop       : next_state = (in     ) ? idle_start  : error;
//         error      : next_state = (in     ) ? idle_start  : error;
//     endcase
// end

// // State flip-flops (sequential)
// always @(posedge clk) begin
//     if(reset) begin
//         state <= idle_start;
//         cnt   <= 0;
//     end
//     else if(state ==error || cnt == 9)begin
//         cnt     <= 0;
//         state   <= next_state;
//     end
//     else if(next_state != idle_start) begin
//         cnt     <= cnt + 1'b1;
//         state   <= next_state;
//         out_r[cnt-1]  <= in;
//     end
// end

// // Output logic
// always @(posedge clk or posedge reset) begin
//     if(reset) begin
//         done  <= 0;
//     end
//     else 
//         done  <= (state == stop) && (next_state == idle_start);
// end

// assign out_byte = done ? out_r : 0;
// endmodule

// Fsm serialdp
// 我们要添加奇偶校验到串行接收器。奇偶校验在每个数据字节后增加一个额外的位。我们将使用奇偶校验，
// 其中接收到的9位中的1s 数必须是奇数。例如，101001011满足奇数奇偶性(有5个1) ，但001001011不满足奇数奇偶性。

// 更改 FSM 和数据路径以执行奇偶校验。仅当正确接收到一个字节并通过奇偶校验时才断言已完成的信号。
// 与串行接收机有限状态机一样，该有限状态机需要识别开始位，等待所有9位(数据和奇偶校验) ，然后验证停止位是正确的。
// 如果停止位没有在预期的时间出现，FSM 必须等待，直到找到一个停止位，然后再尝试接收下一个字节。

// 提供了以下模块，可用于计算输入流的奇偶校验(这是一个带重置的 TFF)。预期的用途是，它应该被给予输入位流，并在适当的时候重置，以便它计数每个字节1位的数目。

//出错，部分测试用例‘done’不对，懒得改了
// module parity (
//     input clk,
//     input reset,
//     input in,
//     output reg odd);

//     always @(posedge clk)
//         if (reset) odd <= 1'b0;
//         else if (in) odd <= ~odd;
// endmodule

// module top_module(
//     input clk,
//     input in,
//     input reset,    // Synchronous reset
//     output [7:0] out_byte,
//     output done
// ); 

// parameter idle_start = 0, data = 1, parity_check = 2, stop = 3, error = 4;

// reg [2:0] state,next_state;
// reg [3:0] cnt;
// reg [7:0] out_r;
// wire      odd;
// reg       check,start;

// // State transition logic (combinational)
// always @(*) begin
//     case (state)
//         idle_start  : begin next_state = (in     ) ? idle_start   : data;start=1;end
//         data        :       next_state = (cnt==8 ) ? parity_check : data;
//         parity_check:       next_state = (in     ) ? stop         : error;
//         stop        : begin next_state = (in     ) ? idle_start   : data;start=1;end
//         error       :       next_state = (in     ) ? idle_start   : error;
//     endcase
// end

// // State flip-flops (sequential)

// //cnt
// always @(posedge clk) begin
//     if(reset) begin
//         cnt   <= 1'b0;
//     end
//     else if(state == data) begin
//         cnt         <= cnt + 1;
//     end
//     else cnt <= 0;
// end

// always @(posedge clk) begin
//     if (reset)
//         out_r <= 0;
//     else if(next_state==data) begin
//         out_r[cnt]  <= in;
//     end
// end
        
// //State transition logic (sequential)
// always @(posedge clk) begin
//     if(reset) begin
//         state <= idle_start;
//         check <= 0;
//     end
//     else begin
//         state <= next_state;
//         check <= odd;
//     end
// end

// assign done=check&(state==stop);
// assign out_byte = out_r;

// parity u_parity (
//     .clk(clk),
//     .in(in),
//     .reset(reset |start),
//     .odd(odd)
// );
// endmodule

// // Fsm serialdp 比较好的代码思路，有点像序列机
// module top_module(
//     input clk,
//     input in,
//     input reset,    // Synchronous reset
//     output reg [7:0] out_byte,
//     output done
// ); //
    
//     // Modify FSM and datapath from Fsm_serialdata
//     parameter START = 0, CHECK = 9, STOP = 10, IDEL = 11, ERROR = 12;
//     reg [4:0] state, next_state;
//     reg valid, start;
//     wire odd;
//     always @(*) begin
//         start = 0;
//         case (state)
//             IDEL: begin next_state = in ? IDEL : START; start = 1; end
//             CHECK:      next_state = in ? STOP : ERROR;
//             STOP: begin next_state = in ? IDEL : START; start = 1; end
//             ERROR:      next_state = in ? IDEL : ERROR;
//             default:    next_state = state + 1;
//         endcase  
//     end
    
//     always @(posedge clk)
//         if (reset)
//             state <= IDEL;
//         else begin
//             state <= next_state;
//             valid <= odd;
//         end
    
//     always @(posedge clk)
//         if ((0 <= state) & (state < 8))
//             out_byte[state] <= in;
    
    
//     parity check(clk, reset | start, in, odd);
    
//     assign done = (valid & (state == STOP));
//     // New: Add parity checking.
    
// endmodule

// Fsm hdlc
// 同步HDLC帧涉及从连续的比特流中解码寻找某一帧(即数据包)的开始和结束位置的位模式。
// (对位模式不太理解的可以参见https://zhuanlan.zhihu.com/p/46317118)。如果接收到连续的6个1(即01111110)，即是帧边界的“标志”。
// 同时为了避免输入的数据流中意外包含这个帧边界“标志”，数据的发送方必须在数据中连续的5个1之后插入一个0，
// 而数据的接收方必须将这个多余的0检测出来并丢弃掉。同时，如果输入检测到了了连续7个或更多的1时，接收方还需要发出错误信号。
// module top_module(
//     input clk,
//     input reset,    // Synchronous reset
//     input in,
//     output disc,
//     output flag,
//     output err);

//     parameter NONE  = 0;
//     parameter ONE   = 1;
//     parameter TWO   = 2;
//     parameter THREE = 3;
//     parameter FOUR  = 4;
//     parameter FIVE  = 5;
//     parameter SIX   = 6;
//     parameter DISC  = 7;
//     parameter FLAG  = 8;
//     parameter ERROR = 9;

//     reg [3:0] state, next_state;

//     always @(*) begin
//         case (state)
//             NONE    : next_state = in ? ONE     : NONE;
//             FIVE    : next_state = in ? SIX     : DISC;
//             SIX     : next_state = in ? ERROR   : FLAG;
//             DISC    : next_state = in ? ONE     : NONE;
//             FLAG    : next_state = in ? ONE     : NONE;
//             ERROR   : next_state = in ? ERROR   : NONE;
//             default : next_state = in ? state+1 : NONE;
//         endcase
//     end

//     always @(posedge clk) begin
//         if(reset) begin
//             state <= NONE;
//         end
//         else 
//             state <= next_state;
//     end

//     assign disc = state==DISC;
//     assign flag = state==FLAG;
//     assign err  = state==ERROR;
// endmodule

// Exams/ece241 2013 q8
// 实现一个 Mealy 类型的有限状态机，识别一个名为 x 的输入信号上的序列“101”。你的 FSM 应该有一个输出信号 z，
// 当“101”序列被检测到时，该输出信号断言为逻辑1。您的 FSM 还应该有一个低有效异步复位。
// 您的状态机中可能只有3个状态。您的 FSM 应该识别重叠序列。
// module top_module (
//     input clk,
//     input aresetn,    // Asynchronous active-low reset
//     input x,
//     output z ); 

//     parameter S0  = 0;
//     parameter S1  = 1;
//     parameter S2  = 2;//检测到“10”序列时的状态

//     reg [1:0] state,next_state;

//     always @(*) begin
//         case (state)
//             S0 : next_state = x ? S1  : S0;
//             S1 : next_state = x ? S1  : S2;
//             S2 : next_state = x ? S1  : S0;
//             default : next_state = S0;//安全行为
//         endcase
//     end

//     always @(posedge clk or negedge aresetn) begin
//         if (!aresetn) begin
//             state <= S0;
//         end
//         else begin
//             state <= next_state;
//         end
//     end

//     assign z = (state==S2)&&(x==1);

// endmodule


// Exams/ece241 2014 q5a
// 你是设计一个一输入一输出串行2进制补充Moore状态机。输入(x)是一系列位(每个时钟周期一位) ，
// 以数字的最低有效位开始，输出(Z)是输入的2进制补码。机器将接受任意长度的输入数字。
// 电路需要异步复位。转换在释放 Reset 时开始，在断言 Reset 时停止。
// module top_module (
//     input clk,
//     input areset,
//     input x,
//     output z
// ); 

//     parameter S = 0, S0 = 1, S1 = 2;

//     reg [1:0] state, next_state;
//     reg out;

//     always @(*) begin
//         case (state)
//             S  : next_state = x ? S1 : S;
//             S0 : next_state = x ? S0 : S1;
//             S1 : next_state = x ? S0 : S1;
//         endcase
//     end

//     always @(posedge clk or posedge areset) begin
//         if(areset) begin
//             state <= S;
//         end
//         else 
//             state <= next_state;
//     end

//     assign z = state==S1;

// endmodule //Moore型状态机

// // Exams/ece241 2014 q5b
// module top_module (
//     input clk,
//     input areset,
//     input x,
//     output z
// ); 

//     parameter S0 = 0, S1 = 1;

//     reg state, next_state;
//     reg out;

//     always @(*) begin
//         case (state)
//             S0 : next_state =  x ? S1 : S0;
//             S1 : next_state = S1;
//         endcase
//     end

//     always @(posedge clk or posedge areset) begin
//         if(areset) begin
//             state <= S0;
//         end
//         else 
//             state <= next_state;
//     end

//     assign z = state==S0 ? x : ~x;

// endmodule //mealy型状态机

// Exams/2014 q3fsm
// 考虑具有输入s和w的有限状态机。假设FSM以称为A的重置状态开始，如下所示。FSM保持状态为a = 0，
// 并且在s = 1时移至状态b。在状态B中，FSM会在接下来的三个时钟周期中检查输入w的值。如果在这些
// 时钟周期中的两个恰好中的w = 1，则FSM必须在以下时钟周期中将输出z设置为1。否则Z必须为0。
// FSM继续检查W W的下三个时钟周期，依此类推。下的定时图说明了W的不同值W的所需值。

// 使用尽可能少的状态。请注意，S输入仅在状态A中使用，因此您需要仅考虑W输入。

//双层状态机 分层状态机
// module top_module (
//     input clk,
//     input reset,   // Synchronous reset
//     input s,
//     input w,
//     output z
// );

//     parameter A = 0, B = 1;

//     reg state,next_state;
//     reg [2:0] shift;
//     reg [1:0] cnt;

//     always @(*) begin
//         case (state)
//             A : next_state = s ? B : A;
//             B : next_state = B;
//         endcase
//     end

//     always @(posedge clk) begin
//         if(reset) begin
//             state <= A;
//         end
//         else begin
//             state <= next_state;            
//         end
//     end

//     always @(posedge clk) begin
//         if(reset) begin
//             cnt <= 0;
//         end
//         else if(next_state==A) begin
//             cnt <= 0;
//         end
//         else if(cnt == 3) begin
//             cnt <= 1;
//         end
//         else begin
//             cnt <= cnt + 1;
//         end
//     end

//     always @(posedge clk) begin
//         if(reset || state==A) begin
//             shift <= 0;
//         end
//         else begin
//             shift <= {shift[1:0],w};
//         end
//     end

//     assign z = (cnt==1)&&((shift==3'b101)||(shift==3'b110)||(shift==3'b011));
// endmodule


// // Exams/2014 q3bfsm
// module top_module (
//     input clk,
//     input reset,   // Synchronous reset
//     input x,
//     output z
// );

//     reg [2:0] y,Y;

//     always @(*) begin
//         case (y)
//             3'b000 : Y = x? 3'b001 : 3'b000 ;
//             3'b001 : Y = x? 3'b100 : 3'b001 ;
//             3'b010 : Y = x? 3'b001 : 3'b010 ;
//             3'b011 : Y = x? 3'b010 : 3'b001 ;
//             3'b100 : Y = x? 3'b100 : 3'b011 ;
//             default : Y = 3'b000;
//         endcase
//     end

//     always @(posedge clk) begin
//         if(reset) begin
//             y <= 3'b000;
//         end
//         else begin
//             y <= Y;
//         end
//     end

//     assign z = (y==3'b011) || (y==3'b100);
// endmodule

// // Exams/2014 q3c
// module top_module (
//     input clk,
//     input [2:0] y,
//     input x,
//     output Y0,
//     output z
// );

//     reg [2:0] Y;

//     always @(*) begin
//         case (y)
//             3'b000 : Y = x? 3'b001 : 3'b000 ;
//             3'b001 : Y = x? 3'b100 : 3'b001 ;
//             3'b010 : Y = x? 3'b001 : 3'b010 ;
//             3'b011 : Y = x? 3'b010 : 3'b001 ;
//             3'b100 : Y = x? 3'b100 : 3'b011 ;
//             default : Y = 3'b000;
//         endcase
//     end

//     assign z = (y==3'b011) || (y==3'b100);
//     assign Y0 = Y[0];
// endmodule

// // Exams/m2014 q6b
// module top_module (
//     input [3:1] y,
//     input w,
//     output Y2);

//     parameter A = 3'b000;
//     parameter B = 3'b001;
//     parameter C = 3'b010;
//     parameter D = 3'b011;
//     parameter E = 3'b100;
//     parameter F = 3'b101;

//     reg [3:1] Y;

//     always @(*) begin
//         case (y)
//             A : Y = w? A : B ;
//             B : Y = w? D : C ;
//             C : Y = w? D : E ;
//             D : Y = w? A : F ;
//             E : Y = w? D : E ;
//             F : Y = w? D : C ;
//             default : Y = A;
//         endcase
//     end

//     assign Y2 = Y[2];
// endmodule

// // Exams/m2014 q6c
// module top_module (
//     input [6:1] y,
//     input w,
//     output Y2,
//     output Y4);

//     assign Y2 = y[1] && ~w;
//     assign Y4 = (y[3] && w) || (y[2] && w) || (y[5] && w) || (y[6] && w);
//     // assign Y4 = (y[2] | y[3] | y[5] | y[6]) & w;
// endmodule

// // Exams/m2014 q6
// module top_module (
//     input clk,
//     input reset,     // synchronous reset
//     input w,
//     output z);

//     parameter A = 3'b000;
//     parameter B = 3'b001;
//     parameter C = 3'b010;
//     parameter D = 3'b011;
//     parameter E = 3'b100;
//     parameter F = 3'b101;

//     reg [2:0] state, next_state;

//     always @(*) begin
//         case (state)
//             A : next_state = w? A : B ;
//             B : next_state = w? D : C ;
//             C : next_state = w? D : E ;
//             D : next_state = w? A : F ;
//             E : next_state = w? D : E ;
//             F : next_state = w? D : C ;
//         endcase
//     end

//     always @(posedge clk) begin
//         if(reset) begin
//             state <= A;
//         end
//         else begin
//             state <= next_state;
//         end
//     end

//     assign z = (state==E) || (state==F);

// endmodule

// // Exams/2012 q2fsm
// module top_module (
//     input clk,
//     input reset,     // synchronous reset
//     input w,
//     output z);

//     parameter A = 3'b000;
//     parameter B = 3'b001;
//     parameter C = 3'b010;
//     parameter D = 3'b011;
//     parameter E = 3'b100;
//     parameter F = 3'b101;

//     reg [2:0] state, next_state;

//     always @(*) begin
//         case (state)
//             A : next_state = ~w? A : B ;
//             B : next_state = ~w? D : C ;
//             C : next_state = ~w? D : E ;
//             D : next_state = ~w? A : F ;
//             E : next_state = ~w? D : E ;
//             F : next_state = ~w? D : C ;
//         endcase
//     end

//     always @(posedge clk) begin
//         if(reset) begin
//             state <= A;
//         end
//         else begin
//             state <= next_state;
//         end
//     end

//     assign z = (state==E) || (state==F);

// endmodule

// // Exams/2012 q2b
// module top_module (
//     input [5:0] y,
//     input w,
//     output Y1,
//     output Y3);

//     assign Y1 = y[0] && w;
//     assign Y3 = (y[1] | y[2] | y[4] | y[5]) & ~w;
// endmodule

// // Exams/2013 q2afsm
// module top_module (
//     input clk,
//     input resetn,    // active-low synchronous reset
//     input [3:1] r,   // request
//     output [3:1] g   // grant
// ); 

//     parameter A = 3'b000;
//     parameter B = 3'b001;
//     parameter C = 3'b010;
//     parameter D = 3'b100;

//     reg [2:0] state,next_state;

//     always @(*) begin
//         case (state)
//             A : next_state = r[1]? (B) : (r[2]? C : (r[3]? D :A));
//             B : next_state = r[1]? B : A;
//             C : next_state = r[2]? C : A;
//             D : next_state = r[3]? D : A;
//         endcase
//     end

//     always @(posedge clk) begin
//         if (!resetn) begin
//             state <= A;
//         end 
//         else begin
//             state <= next_state;
//         end
//     end

//     assign g = state;

// endmodule

// Exams/2013 q2bfsm
// 考虑一个用于控制某种类型电机的有限状态机。FSM具有来自电机的输入x和y，
// 并产生控制电机的输出f和g。还有一个称为clk的时钟输入端和一个名为resetn的复位输入端。

// FSM必须按以下方式工作。只要复位输入被断言，FSM就保持在称为状态A的开始状态。当复位信号被解除断言时，
// 则在下一个时钟沿之后，FSM必须将输出f设置为1达一个时钟周期。然后，FSM必须监视x输入。
// 当x在三个连续的时钟周期中产生了值1、0、1时，则g应在下一个时钟周期设置为1。在保持g＝1的同时，FSM必须监视y输入。
// 如果y在最多两个时钟周期内具有值1，则FSM应永久保持g＝1（即，直到重置）。但是，如果y在两个时钟周期内没有变为1，则FSM应永久设置g＝0（直到重置）。

//（原始考试问题仅要求一个状态图。但是在这里，实现FSM。）
// //序列检测机(序列机)实现比较方便
// module top_module (
//     input clk,
//     input resetn,    // active-low synchronous reset
//     input x,
//     input y,
//     output f,
//     output g
// ); 
//     localparam FSM_W  = 10;
//     localparam FSM_W1 = FSM_W - 1'b1;

//     reg [FSM_W1:0]   state;
//     reg [FSM_W1:0]   nxt_state;

//     localparam  IDLE      = 0;
//     localparam  AFT_RST   = 1;
//     localparam  STRT_X_MNT= 2;
//     localparam  X_1       = 3;
//     localparam  X_0       = 4;
//     localparam  X_10      = 5;
//     localparam  X_101     = 6;
//     localparam  Y_S0      = 7;
//     localparam  G_O0      = 8;
//     localparam  G_O1      = 9;

//     // State transition logic (combinational)
//     always @(*) begin
//         nxt_state[IDLE   ]          =   1'b0; // never reach for nxt_state
//         nxt_state[AFT_RST]          =   (state[IDLE   ]);
//         nxt_state[STRT_X_MNT]       =   (state[AFT_RST]);
//         nxt_state[X_1    ]          =   (state[STRT_X_MNT] &&  x) || (state[X_1    ] &&  x) || (state[X_0    ] &&  x);
//         nxt_state[X_0    ]          =   (state[STRT_X_MNT] && ~x) || (state[X_10   ] && ~x) || (state[X_0    ] && ~x);
//         nxt_state[X_10   ]          =   (state[X_1    ] && ~x);
//         nxt_state[X_101  ]          =   (state[X_10   ] &&  x);
//         nxt_state[Y_S0   ]          =   (state[X_101  ] && ~y);
//         nxt_state[G_O0   ]          =   (state[Y_S0   ] && ~y) || state[G_O0   ];
//         nxt_state[G_O1   ]          =   (state[Y_S0   ] &&  y) || (state[X_101  ] &&  y) || state[G_O1   ];
//     end

//     // State flip-flops (sequential)
//     always @(posedge clk) begin
//         if(~resetn)
//             state   <=  'b1; //IDLE
//         else begin
//             state   <=  nxt_state;
//         end  
//     end

//     //output logic
//     assign  f    =   state[AFT_RST];
//     assign  g    =   (state[X_101] || state[G_O1] || state[Y_S0]) ? 1'b1 : 1'b0;
// endmodule

// Exams/review2015 count1k
// module top_module (
//     input clk,
//     input reset,
//     output [9:0] q);

//     reg  [9:0]   Q;
//     always @(posedge clk) begin
//         if(reset) begin
//             Q <= 0;
//         end
//         else if(Q==999) begin
//             Q <= 0;
//         end
//         else
//             Q <= Q + 1;
//     end
//     assign q=Q;
// endmodule

// Exams/review2015 shiftcount
// module top_module (
//     input clk,
//     input shift_ena,
//     input count_ena,
//     input data,
//     output [3:0] q);

//     reg [3:0] Q;

//     always @(posedge clk) begin
//         if (shift_ena) begin
//             Q <= {Q[2:0],data};
//         end
//         else if(count_ena) begin
//             Q <= Q - 1;
//         end
//     end

//     assign  q = Q;
// endmodule

// Exams/review2015 fsmseq
// module top_module (
//     input clk,
//     input reset,      // Synchronous reset
//     input data,
//     output start_shifting);

//     parameter D_0=0, D_1=1,D_11=2,D_110=3,D_1101=4;
//     reg [2:0] state;
//     reg [2:0] next_state;

//     always @(*) begin
//         case (state)
//             D_0     : next_state = data ? D_1   : D_0   ;
//             D_1     : next_state = data ? D_11  : D_0   ;
//             D_11    : next_state = data ? D_11  : D_110 ;
//             D_110   : next_state = data ? D_1101: D_0   ;
//             D_1101  : next_state = D_1101               ;
//         endcase
//     end

//     always @(posedge clk) begin
//         if(reset) begin
//             state <= D_0;
//         end
//         else begin
//             state <= next_state;
//         end
//     end

//     assign start_shifting = state==D_1101; 
// endmodule

// Exams/review2015 fsmshift
// module top_module (
//     input clk,
//     input reset,      // Synchronous reset
//     output shift_ena);

//     parameter D_0=0,D_1=1,D_11=2,D_111=3,D_1111=4;

//     reg [2:0]   state,next_state;

//     always @(*) begin
//         case (state)
//             D_0     : next_state = reset ? D_1 : D_0;
//             D_1     : next_state = reset ? D_1 : D_11;
//             D_11    : next_state = reset ? D_1 : D_111;
//             D_111   : next_state = reset ? D_1 : D_1111;
//             D_1111  : next_state = reset ? D_1 : D_0;
//         endcase
//     end

//     always @(posedge clk) begin
//         if (reset) begin
//             state <= D_1;
//         end
//         else begin
//             state <= next_state;
//         end
//     end

//     assign shift_ena = (|state) || (reset);
// endmodule

// Exams/review2015 fsm
// 复杂计数器需要如下这些功能特性：

// 在数据流中检测到特定序列后启动计数器，该序列为： 1101
// 将 4bits 数据输入移位寄存器，作为计数器的初值
// 等待计数器结束计数
// 告知上层应用计数完成，并等待用户通过 ack 信号确认
// 在本题练习中，只需要实现控制状态机，不需要实现数据通路，比如计数器本身以及数据比较器等。
// 数据流从模块的 data 信号输入，当检测到 1101 序列后，状态机需要置高输出信号 shft_ena 并保持 4 个周期（用于将接下来 4bit 数据输入移位寄存器）。
// 之后，状态机置高 counting 信号，表示其正在等待计数器完成计数，当计数器完成计数输出 done_counting 信号后，counting 信号置低。
// 再此后，状态机置高 done 信号通知上层应用计数器计数完成，等待 ack 信号置高后，状态机清除 done 信号，返回空闲状态等待捕获下一个 1101 序列。
// 本题给出了一个期望输入输出的例子。图中的斜线代表当前信号为 'X', 表示状态机不关心该信号当前的值。
// 比如图例中，一旦 FSM 检测到 1101 序列后，在此次计数器事件完成前，对于当前的数据流不再关心。
// 独热码状态机思路比较清晰
// module top_module (
//     input   clk,
//     input   reset,      // Synchronous reset
//     input   data,
//     output  shift_ena,
//     output  counting,
//     input   done_counting,
//     output  done,
//     input   ack );

//     parameter IDLE  =   10'b00_0000_0000;
//     parameter S0    =   10'b00_0000_0001;
//     parameter S1    =   10'b00_0000_0010;
//     parameter S11   =   10'b00_0000_0100;
//     parameter S110  =   10'b00_0000_1000;
//     parameter SF1   =   10'b00_0001_0000;
//     parameter SF2   =   10'b00_0010_0000;
//     parameter SF3   =   10'b00_0100_0000;
//     parameter SF4   =   10'b00_1000_0000;
//     parameter COUNT =   10'b01_0000_0000;
//     parameter DONE  =   10'b10_0000_0000;

//     reg [9:0] state,next_state;

//     always @(*) begin
//         case (state)
//         IDLE  : next_state = reset  ? IDLE  : S0;
//         S0    : next_state = reset  ? IDLE  : (data   ? S1    : S0);
//         S1    : next_state = reset  ? IDLE  : (data   ? S11   : S0);
//         S11   : next_state = reset  ? IDLE  : (data   ? S11   : S110);
//         S110  : next_state = reset  ? IDLE  : (data   ? SF1 : S0);
//         SF1   : next_state = reset  ? IDLE  : (SF2);
//         SF2   : next_state = reset  ? IDLE  : (SF3);
//         SF3   : next_state = reset  ? IDLE  : (SF4);
//         SF4   : next_state = reset  ? IDLE  : (COUNT);
//         COUNT : next_state = reset  ? IDLE  : (done_counting    ? DONE  : COUNT);
//         DONE  : next_state = reset  ? IDLE  : (ack              ? S0    : DONE);
//         default : next_state = IDLE;
//         endcase
//     end

//     always @(posedge clk) begin
//         if(reset) begin
//             state <= S0;
//         end
//         else begin
//             state <= next_state;
//         end
//     end

//     assign shift_ena    = |state[7:4];
//     assign counting     =  state[8];
//     assign done         =  state[9];
// endmodule


// 在数据流中检测到序列 1101 后，电路需要将接下来的 4bit 数据移入移位寄存器。4bit 数据决定了计数器的计数周期，称为 delay[3:0]。首先到达的比特作为数据的高位。
// 之后，状态机置高 counting 信号，表示其正在等待计数器完成计数。在 FSM 中增加计数器状态，计数周期为 （delay[3:0] + 1 ）* 1000 个时钟周期。
// 比如 delay = 0 时，计数值为 1000 个周期。delay = 5 代表 6000 个周期。同时输出 count 当前剩余的计数周期，
// 输出当前剩余计数周期的千位（比如，还剩1000个周期输出 1，还剩 999 个周期时输出 0）。当计数停止后，count 的输出可以为任意数。
// 当计数完成后，电路置高 done 信号通知上层应用计数器计数完成，等待 ack 信号置高后，状态机清除 done 信号，返回空闲状态等待捕获下一个 1101 序列。

// 本题给出了一个期望输入输出的例子。图中的斜线代表当前信号为 'X', 表示状态机不关心该信号当前的值。
// 比如图例中，一旦 FSM 检测到 1101 序列并读取 delay[3:0] 后，在此次计数器事件完成前，对于当前的数据流不再关心。
// 在图例中，电路计数周期为 2000 ，因为 delay[3:0] 数值为 4'b0001 。在后续的第二个计数周期中，因为 delay[3:0] = 4'b1110，所以计数周期为 15000。
// Exams/review2015 fancytimer
// module top_module (
//     input clk,
//     input reset,      // Synchronous reset
//     input data,
//     output [3:0] count,
//     output counting,
//     output done,
//     input ack );

//     parameter IDLE  =   10'b00_0000_0000;
//     parameter S0    =   10'b00_0000_0001;
//     parameter S1    =   10'b00_0000_0010;
//     parameter S11   =   10'b00_0000_0100;
//     parameter S110  =   10'b00_0000_1000;
//     parameter SF1   =   10'b00_0001_0000;
//     parameter SF2   =   10'b00_0010_0000;
//     parameter SF3   =   10'b00_0100_0000;
//     parameter SF4   =   10'b00_1000_0000;
//     parameter COUNT =   10'b01_0000_0000;
//     parameter DONE  =   10'b10_0000_0000;

//     reg [9:0]   state,next_state;
//     reg [3:0]   shift_reg;
//     reg [9:0]   dly_cnt;//1000 ->10 bit

//     wire        done_counting;
//     wire        shift_ena;  

//     // State transition logic (combinational)
//     always @(*) begin
//         case (state)
//         IDLE  : next_state = reset  ? IDLE  : (S0);
//         S0    : next_state = reset  ? IDLE  : (data   ? S1    : S0);
//         S1    : next_state = reset  ? IDLE  : (data   ? S11   : S0);
//         S11   : next_state = reset  ? IDLE  : (data   ? S11   : S110);
//         S110  : next_state = reset  ? IDLE  : (data   ? SF1   : S0);
//         SF1   : next_state = reset  ? IDLE  : (SF2);
//         SF2   : next_state = reset  ? IDLE  : (SF3);
//         SF3   : next_state = reset  ? IDLE  : (SF4);
//         SF4   : next_state = reset  ? IDLE  : (COUNT);
//         COUNT : next_state = reset  ? IDLE  : (done_counting    ? DONE  : COUNT);
//         DONE  : next_state = reset  ? IDLE  : (ack              ? S0    : DONE);
//         default : next_state = IDLE;
//         endcase
//     end

//     // State flip-flops (sequential)
//     always @(posedge clk) begin
//         if(reset) begin
//             state <= S0;
//         end
//         else begin
//             state <= next_state;
//         end
//     end

//     // data shift, get the delay number
//     always @(posedge clk) begin
//         if(reset) begin
//             shift_reg <= 0;
//         end
//         else if (shift_ena) begin
//             shift_reg <= {shift_reg[2:0],data};
//         end
//         if (dly_cnt==999) begin
//             shift_reg <= shift_reg - 1;// dly_num
//         end
//     end

//     // 1000 clk delay count
//     always @(posedge clk) begin
//         if (!counting) begin
//             dly_cnt <= 0;
//         end
//         else if(dly_cnt!=999) begin
//             dly_cnt <= dly_cnt + 1;
//         end
//         else begin
//             dly_cnt <= 0;
//         end
//     end

//     assign done_counting = ((dly_cnt==999)&&(shift_reg==0));

//     assign shift_ena    = |state[7:4];
//     assign counting     =  state[8];
//     assign done         =  state[9];
//     assign count        =  shift_reg;

// endmodule

// Exams/review2015 fsmonehot
// module top_module(
//     input d,
//     input done_counting,
//     input ack,
//     input [9:0] state,    // 10-bit one-hot current state
//     output B3_next,
//     output S_next,
//     output S1_next,
//     output Count_next,
//     output Wait_next,
//     output done,
//     output counting,
//     output shift_ena
// ); 

//     localparam FSM_W  = 10;
//     localparam FSM_W1 = FSM_W - 1'b1;

//     reg [FSM_W1:0]   nxt_state;

//     localparam  S_0         = 0;
//     localparam  S_1         = 1;
//     localparam  S_11        = 2;
//     localparam  S_110       = 3;
//     localparam  B0          = 4;
//     localparam  B1          = 5;
//     localparam  B2          = 6;
//     localparam  B3          = 7;
//     localparam  WAT_CNT_FIN = 8;
//     localparam  WAT_ACK     = 9;

//     always @(*) begin
//         nxt_state[S_0    ]          =   (state[S_1    ] && ~d) || (state[S_0    ] && ~d) 
//                             || (state[S_110   ] && ~d) || (state[WAT_ACK       ] && ack);
//         nxt_state[S_1    ]          =   (state[S_0    ] &&  d);
//         nxt_state[S_11   ]          =   (state[S_1    ] &&  d) || (state[S_11   ] &&  d);
//         nxt_state[S_110  ]          =   (state[S_11   ] && ~d);
//         nxt_state[B0     ]          =   (state[S_110  ] &&  d);
//         nxt_state[B1     ]          =   (state[B0     ]);
//         nxt_state[B2     ]          =   (state[B1     ]);
//         nxt_state[B3     ]          =   (state[B2     ]);

//         nxt_state[WAT_CNT_FIN   ]   =   state[B3]
//                                     || (state[WAT_CNT_FIN   ] && ~done_counting);

//         nxt_state[WAT_ACK       ]   =   (state[WAT_CNT_FIN   ] && done_counting) 
//                                     || (state[WAT_ACK       ] && ~ack);
//     end

//     assign          B3_next         =   nxt_state[B3];
//     assign          S_next          =   nxt_state[S_0];
//     assign          S1_next         =   nxt_state[S_1];
//     assign          Count_next      =   nxt_state[WAT_CNT_FIN];
//     assign          Wait_next       =   nxt_state[WAT_ACK];
//     assign          done            =   state[WAT_ACK] ;
//     assign          counting        =   state[WAT_CNT_FIN];
//     assign          shift_ena       =   state[B0] 
//                                     ||  state[B1]
//                                     ||  state[B2]
//                                     ||  state[B3];

// endmodule

// Bugs mux2
// 原输出为高阻态
// module top_module (
//     input sel,
//     input [7:0] a,
//     input [7:0] b,
//     output out  );//注意，这里的out位宽=1，无法输出8位的a与b，加个[7:0]
//     // 再把输出改为assign out = sel ? a : b;

//     assign out = (~sel & a) | (sel & b);

// endmodule

// Bugs nand3
// // module andgate ( output out, input a, input b, input c, input d, input e );
// module top_module (input a, input b, input c, output out);//

// andgate inst1 ( a, b, c, out );//位置

// endmodule
// module top_module (input a, input b, input c, output out);//

//     wire R;
//     andgate inst1 ( R, a, b, c, 1'b1, 1'b1 );//未使用到的引脚默认为0,连接顺序

//     assign out = !R;//取反逻辑要在外面,因为!out 是一个逻辑非运算，不能被用作输出端口。
//                     // 输出端口只能是被定义为 output 的信号。
// endmodule

// Bugs mux4
// module mux2 (
//     input sel,
//     input [7:0] a,
//     input [7:0] b,
//     output [7:0] out
// );
// endmodule
// module top_module (
//     input [1:0] sel,
//     input [7:0] a,
//     input [7:0] b,
//     input [7:0] c,
//     input [7:0] d,
//     output [7:0] out  ); //

//     wire mux0, mux1;
//     mux2 mux0_inst ( sel[0],    a,    b, mux0 );
//     mux2 mux1_inst ( sel[0],    c,    d, mux1 );
//     mux2 mux2_inst ( sel[1], mux0, mux1,  out );

// endmodule

// Bugs addsubz
// synthesis verilog_input_version verilog_2001
// module top_module ( 
//     input do_sub,
//     input [7:0] a,
//     input [7:0] b,
//     output reg [7:0] out,
//     output reg result_is_zero
// );//

//     always @(*) begin
//         case (do_sub)
//           0: out = a+b;
//           1: out = a-b;
//         endcase

//         if (!out)//逻辑非为！
//             result_is_zero = 1;
//         else
//             result_is_zero = 0;//if条件未写全，导致信号锁定到固定电平无法改变
//     end

// endmodule

// Bugs case
// module top_module (
//     input [7:0] code,
//     output reg [3:0] out,
//     output reg valid);//

//      always @(*)
//         case (code)
//             8'h45: begin out = 0;valid=1;end
//             8'h16: begin out = 1;valid=1;end
//             8'h1e: begin out = 2;valid=1;end
//             8'h26: begin out = 3;valid=1;end
//             8'h25: begin out = 4;valid=1;end
//             8'h2e: begin out = 5;valid=1;end
//             8'h36: begin out = 6;valid=1;end
//             8'h3d: begin out = 7;valid=1;end
//             8'h3e: begin out = 8;valid=1;end
//             8'h46: begin out = 9;valid=1;end
//             default: begin out=0;valid=0;end
//         endcase

// endmodule

// Sim/circuit1
// module top_module (
//     input a,
//     input b,
//     output q );//

//     assign q = a&&b; // Fix me

// endmodule

// Sim/circuit2
// module top_module (
//     input a,
//     input b,
//     input c,
//     input d,
//     output q );//

//     assign q = ^~{a,b,c,d}; // Fix me
//     //同或使用异或非^~
// endmodule

// Sim/circuit3
// module top_module (
//     input a,
//     input b,
//     input c,
//     input d,
//     output q );//

//     assign q = !((!c&&!d)||(!a&&!b)); // Fix me

// endmodule

// Sim/circuit4
// module top_module (
//     input a,
//     input b,
//     input c,
//     input d,
//     output q );//

//     assign q = !(!b&&!c); // Fix me

// endmodule

// Sim/circuit5
// module top_module (
//     input [3:0] a,
//     input [3:0] b,
//     input [3:0] c,
//     input [3:0] d,
//     input [3:0] e,
//     output reg [3:0] q );

//     always @(*) begin
//         case (c)
//             0 : q = b ;
//             1 : q = e ;
//             2 : q = a ;
//             3 : q = d ;
//             default :q = 4'hf ;
//         endcase
//     end
// endmodule

// Sim/circuit6
// module top_module (
//     input [2:0] a,
//     output reg [15:0] q ); 

//     always @(*) begin
//         case (a)
//             0 : q = 16'h1232;
//             1 : q = 16'haee0;
//             2 : q = 16'h27d4;
//             3 : q = 16'h5a0e;
//             4 : q = 16'h2066;
//             5 : q = 16'h64ce;
//             6 : q = 16'hc526;
//             7 : q = 16'h2f19;
//         endcase
//     end

// endmodule

// Sim/circuit7
// module top_module (
//     input clk,
//     input a,
//     output reg q );

//     always @(posedge clk) begin
//         q <= !a;
//     end
// endmodule

// Sim/circuit8
// module top_module (
//     input clock,
//     input a,
//     output reg p,
//     output q );

//     reg q_out;
//     always @(*) begin
//         if(clock) begin
//             p <= a;
//         end
//         else begin
//             p <= p;
//         end
//     end

//     always @(negedge clock) begin
//         q_out <= a;
//     end

//     assign q = q_out;
// endmodule

// Sim/circuit9
// module top_module (
//     input clk,
//     input a,
//     output reg [3:0] q );

//     always @(posedge clk ) begin
//         if(a) begin
//             q <= 4;
//         end
//         else if(q==6) begin
//             q <= 0;
//         end
//         else begin
//             q <= q + 1;
//         end
//     end
// endmodule

// Sim/circuit10
// 这题要画出状态转移图
// module top_module (
//     input clk,
//     input a,
//     input b,
//     output reg q,
//     output reg state  );

//     always @(*) begin
//         if (!state) begin
//             q = a^b;
//         end
//         else begin
//             q = a^~b;
//         end
//     end

//     always @(posedge clk) begin
//         if ({a,b,q}==3'b110) begin
//             state <= 1;
//         end
//         else if({a,b,q}==3'b001) begin
//             state <= 0;
//         end
//         else begin
//             state <= state;
//         end
//     end
//    endmodule

// Tb/clock
// `timescale 1ps/1ps
// module top_module ( );

//     reg clk = 0;
//     always #5 clk = ~clk;

//     dut dutinst(clk);
// endmodule

// Tb/tb1
// `timescale 1ps/1ps
// module top_module ( output reg A, output reg B );//

// // generate input patterns here
// initial begin
//     A=0;#10;
//     A=1;#10;
//     A=0;
// end

// initial begin
//     B=0;#15;
//     B=1;#25;
//     B=0;
// end
// endmodule


// Tb/and
// `timescale 1ps/1ps
// module top_module();
//     reg [1:0] in;

//     initial begin
//         in=0;#10;
//         in=1;#10;
//         in=2;#10;
//         in=3;
//     end
//     andgate inst(.in(in),.out());
// endmodule

// Tb/tb2
// `timescale 1ps/1ps
// module top_module();

//     reg clk=0;
//     reg in;
//     reg [2:0] s;
//     reg out;

//     always #5 clk=~clk;

//     initial begin
//         in=0;#20;
//         in=1;#10;
//         in=0;#10;
//         in=1;#30;
//         in=0;
//     end

//     initial begin
//         s=2;#10;
//         s=6;#10;
//         s=2;#10;
//         s=7;#10;
//         s=0;
//     end

//     q7 inst(clk,in,s,out);
// endmodule

// Tb/tff
// `timescale 1ps/1ps
// module top_module ();

//     reg clk=0;
//     reg reset;
//     reg t;  //input

//     wire q; // output

//     always #5 clk=~clk;

//     initial begin
//         reset=1;#15;
//         reset=0;
//         t=0;#15;
//         t=1;#15;
//         t=0;
//     end
//     tff tff_inst (clk,reset,t,q);
// endmodule

// Cs450/timer
// module top_module(
// 	input clk, 
// 	input load, 
// 	input [9:0] data, 
// 	output tc
// );

//     reg [9:0] counter;

//     always @(posedge clk) begin
//         if(load) begin
//             counter = data;
//         end
//         else if(counter!=0) begin
//             counter <= counter - 1;
//         end
//         else begin
//             counter <= 0;
//         end
//     end

//     assign tc = counter==0;
// endmodule

// Cs450/counter 2bc
// Branch Predictor（分支预测器）是计算机体系结构中的一种技术，用于预测程序中的分支指令（如条件语句和循环语句）的执行路径。
// 分支指令的执行路径取决于程序运行时的条件，比如条件语句中的条件判断结果、循环语句中的循环计数器等。
// 由于分支指令的执行路径不确定，因此它们会影响程序的性能和效率。

// 分支预测器是通过分析程序执行历史来预测分支指令的执行路径的。它会记录程序中分支指令的执行情况，并根据历史记录来预测分支指令
// 的执行路径。根据预测结果，处理器可以提前执行分支指令的下一条指令，从而提高程序的性能和效率。
// module top_module(
//     input clk,
//     input areset,
//     input train_valid,
//     input train_taken,
//     output reg [1:0] state
// );

//     localparam SNT=0;
//     localparam WNT=1;
//     localparam WT =2;
//     localparam ST =3;

//     reg [1:0] next_state;

//     always @(*) begin
//         case (state)
//             SNT : next_state = train_valid ? (train_taken ? WNT : SNT) : (SNT);
//             WNT : next_state = train_valid ? (train_taken ? WT  : SNT) : (WNT);
//             WT  : next_state = train_valid ? (train_taken ? ST  : WNT) : (WT );
//             ST  : next_state = train_valid ? (train_taken ? ST  : WT ) : (ST );
//         endcase
//     end

//     always @(posedge clk or posedge areset) begin
//         if (areset) begin
//             state <= WNT;
//         end
//         else begin
//             state <= next_state;
//         end
//     end
// endmodule

// Cs450/history shift
// 分支方向预测器通常结构为由程序计数器和分支历史索引的计数器表。分支历史是来自最近分支的“已获取”或“未获取”结果的序列。
// 在硬件上，分支历史寄存器可以实现为n位移位寄存器。在每个条件分支方向被预测后，其预测的方向被移位到移位寄存器中。
// 移位寄存器保存最近的N个分支结果。

// 由于分支预测是推测性的，因此管道刷新会增加额外的复杂性。当发生分支预测错误时，需要将处理器状态回滚到预测错误的
// 分支之后的状态。这包括回滚全局历史寄存器，其中可能包含由比错误预测的分支更年轻的分支移入的预测分支结果，但现在需要丢弃。

// 在这里，我们假设在分支预测器之外存在硬件，这些硬件会记住用于预测每个分支的分支历史寄存器的状态，这些寄存器将被保存下来，
// 以便以后用于分支预测器训练和管道刷新。当发生分支预测错误时，该硬件通知分支预测器分支预测错误，分支应该采取的方向，
// 以及与被错误预测的分支之前的程序中对应的分支历史寄存器的状态。

// 当然，由于处理器重新启动到错误预测的分支之后的点，因此在管道刷新之后的分支历史寄存器需要附加错误预测的分支的实际方向。

// 描述：
// 构建一个32位全局历史移位寄存器，包括支持回滚状态以响应由分支错误预测引起的管道刷新。当进行分支预测时(predict_valid = 1)，
// 从LSB端移入predict__valid以更新预测分支的分支历史。(predict_history[0]是最年轻分支的方向。)当发生分支错误预测时(train_mispredict = 1)，
// 在错误预测的分支完成后用历史记录加载分支历史寄存器。这是错误预测的分支(train_history)与分支(train_taken)的实际结果相连接之前的历史记录。
// 如果预测和错误预测同时发生，则错误预测优先，因为管道刷新也将刷新当前正在进行预测的分支。Predict_history是分支历史寄存器的值。
// Areset是一个异步重置，它将历史计数器重置为零。
// module top_module(
//     input clk,
//     input areset,

//     input predict_valid,
//     input predict_taken,
//     output reg [31:0] predict_history,

//     input train_mispredicted,
//     input train_taken,
//     input [31:0] train_history
// );

//     always @(posedge clk or posedge areset) begin
//         if (areset) begin
//             predict_history <= 0;
//         end
//         else if(train_mispredicted) begin
//                 predict_history <= {train_history[30:0],train_taken};
//         end
//         else if(predict_valid) begin
//             predict_history <= {predict_history[30:0],predict_taken};
//         end
//     end

// endmodule

// Cs450/gshare
// Branch direction predictor：
// 分支预测器生成条件分支指令方向的选择/不选择预测。它位于处理器流水线的前端，负责将指令导向正确的程序执行路径。
// 分支预测器通常与分支目标缓冲区(BTB)一起使用，其中 BTB 预测目标地址，而分支预测器选择的是分支到目标还是继续沿直通路径获取。

// 在流水线的后期（通常在分支执行或停用时），执行的分支指令的结果将被发送回分支预测器，以训练它通过观察过去的分支行为
// 来更准确地预测未来的分支。当存在错误预测的分支时，也可能存在流水线刷新。

// 在本练习中，假定分支方向预测器位于右边的图中所示的假设的处理器流水线的提取阶段。本练习只构建由图中的蓝色虚线矩形显示的分支预测器。
// 分支方向预测是一个组合路径：pc寄存器用于计算已选择/不选择预测，影响next-pc多路复用器确定下一个周期pc的值。
// 相反，对模式历史表（PHT）和分支历史寄存器的更新将在下一个时钟上升沿生效，正如存储在触发器中的状态所预期的那样。

// Gshare 预测变量
// 分支预测器通常构建为由程序计数器和分支历史记录索引的计数器表。表索引是分支地址和历史记录的哈希表，并尝试为每个分支和历史记录组合
// 提供自己的计数器索引（至少减少冲突次数）。每个计数器索引都包含一个两位饱和计数器，用于执行与过去相同的分支和历史模式时记住分支方向。
// 这种预测变量样式的一个例子是 gshare 预测变量。在 gshare 算法中，分支地址 （pc） 和历史位“共享”表索引位。基本 gshare 算法
// 通过将 N 个分支地址位和 N 个全局分支历史位一起计算 N 位 PHT 表索引。
// 然后使用 N 位索引访问两位饱和计数器的 2^N 项表中的一项。此计数器的值提供预测（ 0 或 1 = 未取，2 或 3 = 已取）。
// 训练以类似的方式索引表。训练 pc 和历史用于计算表索引。然后，该索引处的两位计数器根据分支的实际结果递增或递减。

// 描述
// 使用 7 位 pc 和 7 位全局历史构建 gshare 分支预测器，全局历史记录经过哈希处理（使用异或）为 7 位索引。此索引访问包含
// 两位饱和计数器的 128 个计数器索引（类似于  cs450/counter_2bc ）。分支预测器应包含一个 7 位全局分支历史寄存器（类似于 cs450/history_shift ）。

// 分支预测器有两组接口：一组用于预测，另一组用于训练。预测接口用于处理器的获取阶段，要求分支预测器对所获取的指令
// 进行分支方向预测。一旦这些分支沿着流水线前进并被执行，分支的真实结果就变得已知。使用实际的分支方向结果对分支预测器进行训练。

// 当一个给定的 pc 分支预测被请求（predict_valid = 1）时，分支预测器用于做出预测的分支历史记录器的预测的
// 分支方向和状态。然后（在下一个上升沿）为预测的分支更新分支历史寄存器。

// 当要求对分支进行训练 （ train_valid = 1 ）时，分支预测器会被告知正在训练的分支的 pc 和分支历史记录值，
// 实际的分支结果以及分支是否为错误预测（需要流水线刷新）。更新模式历史表 （PHT） 以训练分支预测器以便下次更准确的预测此分支。
// 此外如果正在训练的分支被错误预测，也会在错误预测的分支完成执行后立即将分支历史寄存器复位。

// 如果在同一周期中发生错误预测和预测（针对不同的、较新的指令）的训练，则两个操作都需要修改分支历史记录寄存器。发生这种情况时，
// 训练优先。因为无论如何都会丢弃被预测的分支。如果同一 PHT 条目的训练和预测同时发生，则预测会在训练之前得到 PHT 状态，
// 因为训练仅在下一个时钟上升沿修改 PHT 。下面的时序图显示了训练时的时序和同时预测 PHT 条目0的时序。第4周期的训练请求改变了第5周期
// 的 PHT 进入状态，但是第4周期的预测请求输出了第4周期的 PHT 状态，而没有考虑第4周期训练请求的影响。
// module top_module(
//     input clk,
//     input areset,

//     input  predict_valid,
//     input  [6:0] predict_pc,
//     output predict_taken,
//     output reg [6:0] predict_history,

//     input train_valid,
//     input train_taken,
//     input train_mispredicted,
//     input [6:0] train_history,
//     input [6:0] train_pc
// );
//     reg [1:0] PHT[127:0]; //待更新模式历史表,128个2位饱和计数器索引
//     integer i;
//     always @(posedge clk, posedge areset) begin
//         if (areset) begin
//             predict_history <= 0;
//             for (i=0; i<128; i=i+1) PHT[i] <= 2'b01; //memory赋初值
//         end
//         else begin
//             if (train_valid && train_mispredicted)
//                 predict_history <= {train_history[5:0], train_taken};
//             else if (predict_valid)
//                 predict_history <= {predict_history[5:0], predict_taken};

//             if (train_valid) begin
//                 if (train_taken)
//                     PHT[train_history ^ train_pc] <= (PHT[train_history ^ train_pc] == 2'b11) ? 2'b11 : (PHT[train_history ^ train_pc] + 1);
//             else
//                     PHT[train_history ^ train_pc] <= (PHT[train_history ^ train_pc] == 2'b00) ? 2'b00 : (PHT[train_history ^ train_pc] - 1);
//             end
//         end
//     end
//     assign predict_taken = PHT[predict_history ^ predict_pc][1];
// endmodule


// 牛客
// // VL4 移位运算与乘法
// // 题目描述：已知d为一个8位数，请在每个时钟周期分别输出该数乘1/3/7/8,并输出一个信号通知此时刻输入的d有效（d给出的信号的上升沿表示写入有效）
// `timescale 1ns/1ns
// module multi_sel(
// input [7:0]d ,
// input clk,
// input rst,
// output reg input_grant,
// output reg [10:0]out
// );
// //*************code***********//
// // 序列机
// reg [1:0] state,next_state;
// reg [10:0] out_temp=0;
// always@(*)begin
//     case(state)
//         default:next_state=state+1;
//     endcase
// end

// always@(posedge clk or negedge rst)begin
//     if(!rst)begin
//         input_grant <= 1'b0;
//         out         <= 11'b0;
//         state       <= 0;
//         // next_state  <= 0;//加入此语句会导致next_state阻塞为0；
//     end
//     else begin
//         state <= next_state;
//         out_temp <= state==0?d:out_temp;//注意此处复位后马上赋值
//     end
// end

// always@(*)begin
//     input_grant = (state==1);
//     case(state)
//         1:out=out_temp;//*1
//         2:out=(out_temp<<2)-out_temp;//*3
//         3:out=(out_temp<<3)-out_temp;//*7
//         0:out=(out_temp<<3);//*8
//         default:out=0;
//     endcase
// end
// //*************code***********//
// endmodule

// // VL5 位拆分与运算
// // 题目描述：           
// // 现在输入了一个压缩的16位数据，其实际上包含了四个数据[3:0][7:4][11:8][15:12],
// // 现在请按照sel选择输出四个数据的相加结果,并输出valid_out信号（在不输出时候拉低）
// // 0:   不输出且只有此时的输入有效 
// // 1：输出[3:0]+[7:4]
// // 2：输出[3:0]+[11:8]
// // 3：输出[3:0]+[15:12]
// `timescale 1ns/1ns

// module data_cal(
// input clk,
// input rst,
// input [15:0]d,
// input [1:0]sel,

// output [4:0]out,
// output validout
// );
// //*************code***********//
// reg [15:0]  data;
// reg         valid_r;
// reg [4:0]   out_r;
// always@(posedge clk or negedge rst)begin
//     if(!rst)begin
//         out_r <= 0;
//         valid_r <= 0;
//     end
//     else begin
//         case (sel)
//             0:begin out_r<=0;data<=d;valid_r<=0;end//题目中指出“在不输出时拉低”
//             1:begin out_r<=data[3:0]+data[7:4];valid_r<=1;end
//             2:begin out_r<=data[3:0]+data[11:8];valid_r<=1;end
//             3:begin out_r<=data[3:0]+data[15:12];valid_r<=1;end
//         endcase
//     end
// end

// assign validout=valid_r;
// assign out=out_r;
// //*************code***********//
// endmodule

// // VL6 多功能数据处理器
// // 描述
// // 根据指示信号select的不同，对输入信号a,b实现不同的运算。输入信号a,b为8bit有符号数，
// // 当select信号为0，输出a；当select信号为1，输出b；当select信号为2，输出a+b；当select信号为3，输出a-b.
// `timescale 1ns/1ns
// module data_select(
// 	input clk,
// 	input rst_n,
// 	input signed[7:0]a,
// 	input signed[7:0]b,
// 	input [1:0]select,
// 	output reg signed [8:0]c
// );

// always@(posedge clk or negedge rst_n)begin
// 	if(!rst_n)begin
// 		c<=0;
// 	end
// 	else begin
// 		case(select)
// 			0:c<=a;
// 			1:c<=b;
// 			2:c<=a+b;
// 			3:c<=a-b;
// 		endcase
// 	end
// end
// endmodule

// // VL7 求两个数的差值
// // 描述
// // 根据输入信号a,b的大小关系，求解两个数的差值：输入信号a,b为8bit位宽的无符号数。如果a>b，则输出a-b，如果a≤b，则输出b-a。
// `timescale 1ns/1ns
// module data_minus(
// 	input clk,
// 	input rst_n,
// 	input [7:0]a,
// 	input [7:0]b,

// 	output  reg [8:0]c
// );

// always@(posedge clk or negedge rst_n)begin
// 	if(!rst_n)begin
// 		c<=0;
// 	end
// 	else begin
// 		c=(a>b)?(a-b):(b-a);
// 	end
// end
// endmodule

// // VL8 使用generate…for语句简化代码
// // 描述
// // 在某个module中包含了很多相似的连续赋值语句，请使用generata…for语句编写代码，替代该语句，要求不能改变原module的功能。
// // 使用Verilog HDL实现以上功能并编写testbench验证。

// // module template_module( 
// //     input [7:0] data_in,
// //     output [7:0] data_out
// // );
// //     assign data_out [0] = data_in [7];
// //     assign data_out [1] = data_in [6];
// //     assign data_out [2] = data_in [5];
// //     assign data_out [3] = data_in [4];
// //     assign data_out [4] = data_in [3];
// //     assign data_out [5] = data_in [2];
// //     assign data_out [6] = data_in [1];
// //     assign data_out [7] = data_in [0];
    
// // endmodule
// `timescale 1ns/1ns
// module gen_for_module( 
//     input [7:0] data_in,
//     output [7:0] data_out
// );

// genvar i;
// generate for(i=0;i<=7;i=i+1)
//     assign data_out[i] = data_in[7-i];
// endgenerate
// endmodule

// // VL9 使用子模块实现三输入数的大小比较
// // 描述
// // 在数字芯片设计中，通常把完成特定功能且相对独立的代码编写成子模块，
// // 在需要的时候再在主模块中例化使用，以提高代码的可复用性和设计的层次性，方便后续的修改。
// // 请编写一个子模块，将输入两个8bit位宽的变量data_a,data_b，并输出data_a,
// // data_b之中较小的数。并在主模块中例化，实现输出三个8bit输入信号的最小值的功能。

// `timescale 1ns/1ns
// module main_mod(
// 	input clk,
// 	input rst_n,
// 	input [7:0]a,
// 	input [7:0]b,
// 	input [7:0]c,
	
// 	output [7:0]d
// );

// wire	[7:0]	t1,t2;
// sub_mod u1(
// 	.clk(clk),
// 	.rst_n(rst_n),
// 	.a(a),
// 	.b(b),
// 	.c(t1)
// );

// sub_mod u2(
// 	.clk(clk),
// 	.rst_n(rst_n),
// 	.a(b),
// 	.b(c),
// 	.c(t2)
// );

// sub_mod u3(
// 	.clk(clk),
// 	.rst_n(rst_n),
// 	.a(t1),
// 	.b(t2),
// 	.c(d)
// );  //注意，此处需要三个比较器才能同时输出比较结果，只用2个会发生比较错位

// endmodule

// module sub_mod(
// 	input clk,
// 	input rst_n,
// 	input [7:0]a,
// 	input [7:0]b,

// 	output reg [7:0]c
// );

// always@(posedge clk or negedge rst_n)begin
// 	if(!rst_n)begin
// 		c <= 0;
// 	end
// 	else begin
// 		c <= (a<b)?a:b;
// 	end
// end

// endmodule

// `timescale 1ns/1ns
// module main_mod(
// 	input clk,
// 	input rst_n,
// 	input [7:0]a,
// 	input [7:0]b,
// 	input [7:0]c,
	
// 	output [7:0]d
// );

// wire	[7:0]	t1;
// reg		[7:0]	t2;
// sub_mod u1(
// 	.clk(clk),
// 	.rst_n(rst_n),
// 	.a(a),
// 	.b(b),
// 	.c(t1)
// );

// sub_mod u2(
// 	.clk(clk),
// 	.rst_n(rst_n),
// 	.a(t1),
// 	.b(t2),
// 	.c(d)
// );

// always@(posedge clk or negedge rst_n)begin
// 	if(!rst_n)begin
// 		t2 <= 0;
// 	end
// 	else begin
// 		t2 <= c;
// 	end
// end     //或者此处将c打一拍也可以

// endmodule

// module sub_mod(
// 	input clk,
// 	input rst_n,
// 	input [7:0]a,
// 	input [7:0]b,

// 	output reg [7:0]c
// );

// always@(posedge clk or negedge rst_n)begin
// 	if(!rst_n)begin
// 		c <= 0;
// 	end
// 	else begin
// 		c <= (a<b)?a:b;
// 	end
// end

// endmodule

// // 知识补充function用法
// module tryfact;
// // 函数的定义-------
//     function [31:0] factorial;
//         input   [3:0]   operand;
//         reg     [3:0]   index;
//         begin
//             factorial = 1; //0的阶乘为 1，1的阶乘也为 1
//             for(index=2; index<=operand; index=index+1)
//             factorial = index * factorial;
//         end
//     endfunction
// //函数的测试--------
// reg [31:0]  result;
// reg [3:0]   n;
// initial
// begin
//     result=1;
//     for(n=2;n<=9;n=n+1)
//     begin
//         result = factorial(n);
//         $display("Partial result n= %d result= %d",n,result);
//     end
//     $display("Finalresult= %d",result);
// end
// endmodule//模块结束
// // Partial result n=  2 result=          2
// // Partial result n=  3 result=          6
// // Partial result n=  4 result=         24
// // Partial result n=  5 result=        120
// // Partial result n=  6 result=        720
// // Partial result n=  7 result=       5040
// // Partial result n=  8 result=      40320
// // Partial result n=  9 result=     362880
// // Finalresult=     362880

// // VL10 使用函数实现数据大小端转换
// // 描述
// // 在数字芯片设计中，经常把实现特定功能的模块编写成函数，在需要的时候再在主模块中调用，
// // 以提高代码的复用性和提高设计的层次，分别后续的修改。
// // 请用函数实现一个4bit数据大小端转换的功能。实现对两个不同的输入分别转换并输出。
// `timescale 1ns/1ns
// module function_mod(
// 	input [3:0]a,
// 	input [3:0]b,
	
// 	output [3:0]c,
// 	output [3:0]d
// );

// assign {c,d} = {endian(a),endian(b)};

// function 	[3:0] endian;
// 	input 	[3:0] In;
// 	reg		[2:0] i;	//注意循环变量不要溢出，否则死循环

// 	for(i=0;i<=3;i=i+1)
// 		endian[i] = In[3-i];
// endfunction

// // 函数测试-----
// reg [3:0] x,y;
// initial begin
//     x = 5;
//     y = endian(x);
//     $display("y= %d",y);
// end
// endmodule

// // VL11 4位数值比较器电路
// `timescale 1ns/1ns

// module comparator_4(
// 	input		[3:0]       A   	,
// 	input	   [3:0]		B   	,

//  	output	 wire		Y2    , //A>B
// 	output   wire        Y1    , //A=B
//     output   wire        Y0      //A<B
// );

// assign {Y2,Y1,Y0} = {(A>B),(A==B),(A<B)};	//最高层次抽象

// // 门级电路
// assign Y2 = (A[3] & ~B[3]) | ((A[3] ~^ B[3]) & ((A[2] & ~B[2]) | ((A[2] ~^ B[2]) & ((A[1] & ~B[1]) | ((A[1] ~^ B[1]) & (A[0] & ~B[0]))))));
// assign Y1 = (A ^ B) == 4'b0000;
// assign Y0 = (~A[3] & B[3]) | ((A[3] ~^ B[3]) & ((~A[2] & B[2]) | ((A[2] ~^ B[2]) & ((~A[1] & B[1]) | ((A[1] ~^ B[1]) & (~A[0] & B[0]))))));
// endmodule

// // VL12 4bit超前进位加法器电路
// `timescale 1ns/1ns

// module lca_4(
// 	input		[3:0]       A_in  ,
// 	input	    [3:0]		B_in  ,
//     input                   C_1   ,

// 	output	 wire			CO    ,
// 	output   wire [3:0]	    S
// );
    
// 	wire [4:0]	C;

// 	assign C[0] = C_1;

// 	genvar i;
// 	generate begin
// 		for(i=0;i<=3;i=i+1)begin
// 			assign C[i+1] = (A_in[i]&B_in[i])|((A_in[i]^B_in[i])&C[i]);
// 			assign S[i] = A_in[i]^B_in[i]^C[i];
// 		end
// 	end
// 	endgenerate

// 	assign CO = C[4];
// endmodule

// // VL13 优先编码器电路1
// `timescale 1ns/1ns

// module encoder_0(
//    input      [8:0]         I_n   ,

//    output reg [3:0]         Y_n   
// );

// always@(*)begin
//     casex(I_n)
//         9'b1_1111_1111:Y_n = 4'b1111;//1
//         9'b0_xxxx_xxxx:Y_n = 4'b0110;//2
//         9'b1_0xxx_xxxx:Y_n = 4'b0111;//3
//         9'b1_10xx_xxxx:Y_n = 4'b1000;//4
//         9'b1_110x_xxxx:Y_n = 4'b1001;//5
//         9'b1_1110_xxxx:Y_n = 4'b1010;//6
//         9'b1_1111_0xxx:Y_n = 4'b1011;//7
//         9'b1_1111_10xx:Y_n = 4'b1100;//8
//         9'b1_1111_110x:Y_n = 4'b1101;//9
//         9'b1_1111_1110:Y_n = 4'b1110;//10
//     endcase
// end

// endmodule

// // VL14 用优先编码器1实现键盘编码电路
// // 描述
// // 请使用优先编码器1实现键盘编码电路，可添加并例化题目中已给出的优先编码器代码。
// // 10个按键分别对应十进制数0-9，按键9的优先级别最高；按键悬空时，按键输出高电平，
// // 按键按下时，按键输出低电平；键盘编码电路的输出是8421BCD码。
// // 要求：键盘编码电路要有工作状态标志，以区分没有按键按下和按键0按下两种情况。
// `timescale 1ns/1ns
// module encoder_0(
//     input      [8:0]         I_n   ,
    
//     output reg [3:0]         Y_n   
// );

// always @(*)begin
//     casex(I_n)
//         9'b111111111 : Y_n = 4'b1111;
//         9'b0xxxxxxxx : Y_n = 4'b0110;
//         9'b10xxxxxxx : Y_n = 4'b0111;
//         9'b110xxxxxx : Y_n = 4'b1000;
//         9'b1110xxxxx : Y_n = 4'b1001;
//         9'b11110xxxx : Y_n = 4'b1010;
//         9'b111110xxx : Y_n = 4'b1011;
//         9'b1111110xx : Y_n = 4'b1100;
//         9'b11111110x : Y_n = 4'b1101;
//         9'b111111110 : Y_n = 4'b1110;
//         default      : Y_n = 4'b1111;
//     endcase    
// end 

// endmodule

// module key_encoder(
//     input      [9:0]         S_n   ,         
    
//     output wire[3:0]         L     ,
//     output wire              GS
// );

// wire    [3:0]   T;

// encoder_0 u1(
//             .I_n(S_n[9:1]),
//             .Y_n(T)
// );

// assign L  = ~T;
// assign GS = (T==4'b1111) ? (S_n[0] ? 0 : 1) : 1;

// endmodule

// // VL15 优先编码器Ⅰ
// // 下表是8线-3线优先编码器Ⅰ的功能表。请根据该功能表，用Verilog实现该优先编码器Ⅰ。
// `timescale 1ns/1ns

// module encoder_83(
//     input      [7:0]       I   ,
//     input                  EI  ,
    
//     output wire [2:0]      Y   ,
//     output wire            GS  ,
//     output wire            EO    
// );

// reg [4:0]   T; //T={Y,GS,EO}

// always@(*)begin
//     if(!EI)begin
//         T = 5'b0;
//     end
//     else begin
//         casex(I)
//             8'b0000_0000: T = 5'b0_0001;
//             8'b1xxx_xxxx: T = 5'b1_1110;
//             8'b01xx_xxxx: T = 5'b1_1010;
//             8'b001x_xxxx: T = 5'b1_0110;
//             8'b0001_xxxx: T = 5'b1_0010;
//             8'b0000_1xxx: T = 5'b0_1110;
//             8'b0000_01xx: T = 5'b0_1010;
//             8'b0000_001x: T = 5'b0_0110;
//             8'b0000_0001: T = 5'b0_0010;
//         endcase
//     end
// end

// assign {Y,GS,EO} = T;

// endmodule

// // VL16 使用8线-3线优先编码器Ⅰ实现16线-4线优先编码器
// // 请使用2片该优先编码器Ⅰ及必要的逻辑电路实现16线-4线优先编码器。优先编码器Ⅰ的真值表和代码已给出。
// // 可将优先编码器Ⅰ的代码添加到本题答案中，并例化。
// `timescale 1ns/1ns
// module encoder_83(
//     input      [7:0]       I   ,
//     input                  EI  ,
    
//     output wire [2:0]      Y   ,
//     output wire            GS  ,
//     output wire            EO    
// );
// assign Y[2] = EI & (I[7] | I[6] | I[5] | I[4]);
// assign Y[1] = EI & (I[7] | I[6] | ~I[5]&~I[4]&I[3] | ~I[5]&~I[4]&I[2]);
// assign Y[0] = EI & (I[7] | ~I[6]&I[5] | ~I[6]&~I[4]&I[3] | ~I[6]&~I[4]&~I[2]&I[1]);

// assign EO = EI&~I[7]&~I[6]&~I[5]&~I[4]&~I[3]&~I[2]&~I[1]&~I[0];

// assign GS = EI&(I[7] | I[6] | I[5] | I[4] | I[3] | I[2] | I[1] | I[0]);
// //assign GS = EI&(| I);

// endmodule

// module encoder_164(
//     input      [15:0]      A   ,
//     input                  EI  ,
    
//     output wire [3:0]      L   ,
//     output wire            GS  ,
//     output wire            EO    
// );

// wire GSH,GSL,EOH;
// wire [2:0]  Y_H,Y_L;

// encoder_83 U_H(
//     .I  (A[15:8]),
//     .EI (EI),
//     .Y  (Y_H),
//     .GS (GSH),
//     .EO (EOH)
// );

// encoder_83 U_L(
//     .I  (A[7:0]),
//     .EI (EOH),
//     .Y  (Y_L),
//     .GS (GSL),
//     .EO (EO)
// );

// assign L = EI ? (EOH ? {1'b0,Y_L} :{1'b1,Y_H}) : 0;
// assign GS= GSH | GSL;

// endmodule

// // VL17 用3-8译码器实现全减器
// // 描述
// // 请使用3-8译码器和必要的逻辑门实现全减器，全减器接口图如下，A是被减数，
// // B是减数，Ci是来自低位的借位，D是差，Co是向高位的借位。
// `timescale 1ns/1ns

// module decoder_38(
//     input             E      ,
//     input             A0     ,
//     input             A1     ,
//     input             A2     ,
    
//     output reg       Y0n    ,  
//     output reg       Y1n    , 
//     output reg       Y2n    , 
//     output reg       Y3n    , 
//     output reg       Y4n    , 
//     output reg       Y5n    , 
//     output reg       Y6n    , 
//     output reg       Y7n    
// );

// always @(*)begin
//     if(!E)begin
//         Y0n = 1'b1;
//         Y1n = 1'b1;
//         Y2n = 1'b1;
//         Y3n = 1'b1;
//         Y4n = 1'b1;
//         Y5n = 1'b1;
//         Y6n = 1'b1;
//         Y7n = 1'b1;
//     end  
//     else begin
//         case({A2,A1,A0})
//             3'b000 : begin
//                         Y0n = 1'b0; Y1n = 1'b1; Y2n = 1'b1; Y3n = 1'b1; 
//                         Y4n = 1'b1; Y5n = 1'b1; Y6n = 1'b1; Y7n = 1'b1;
//                     end 
//             3'b001 : begin
//                         Y0n = 1'b1; Y1n = 1'b0; Y2n = 1'b1; Y3n = 1'b1; 
//                         Y4n = 1'b1; Y5n = 1'b1; Y6n = 1'b1; Y7n = 1'b1;
//                     end 
//             3'b010 : begin
//                         Y0n = 1'b1; Y1n = 1'b1; Y2n = 1'b0; Y3n = 1'b1; 
//                         Y4n = 1'b1; Y5n = 1'b1; Y6n = 1'b1; Y7n = 1'b1;
//                     end 
//             3'b011 : begin
//                         Y0n = 1'b1; Y1n = 1'b1; Y2n = 1'b1; Y3n = 1'b0; 
//                         Y4n = 1'b1; Y5n = 1'b1; Y6n = 1'b1; Y7n = 1'b1;
//                     end 
//             3'b100 : begin
//                         Y0n = 1'b1; Y1n = 1'b1; Y2n = 1'b1; Y3n = 1'b1; 
//                         Y4n = 1'b0; Y5n = 1'b1; Y6n = 1'b1; Y7n = 1'b1;
//                     end 
//             3'b101 : begin
//                         Y0n = 1'b1; Y1n = 1'b1; Y2n = 1'b1; Y3n = 1'b1; 
//                         Y4n = 1'b1; Y5n = 1'b0; Y6n = 1'b1; Y7n = 1'b1;
//                     end 
//             3'b110 : begin
//                         Y0n = 1'b1; Y1n = 1'b1; Y2n = 1'b1; Y3n = 1'b1; 
//                         Y4n = 1'b1; Y5n = 1'b1; Y6n = 1'b0; Y7n = 1'b1;
//                     end 
//             3'b111 : begin
//                         Y0n = 1'b1; Y1n = 1'b1; Y2n = 1'b1; Y3n = 1'b1; 
//                         Y4n = 1'b1; Y5n = 1'b1; Y6n = 1'b1; Y7n = 1'b0;
//                     end 
//             default: begin
//                         Y0n = 1'b1; Y1n = 1'b1; Y2n = 1'b1; Y3n = 1'b1; 
//                         Y4n = 1'b1; Y5n = 1'b1; Y6n = 1'b1; Y7n = 1'b1;
//                     end
//         endcase  
//     end 
// end    

// endmodule

// module decoder1(
//     input             A     ,
//     input             B     ,
//     input             Ci    ,
    
//     output wire       D     ,
//     output wire       Co         
// );

// wire [7:0] t;

// decoder_38 u(
//     .E  (1'b1)  ,
//     .A0 (Ci)    ,
//     .A1 (B)     ,
//     .A2 (A)     ,

//     .Y0n(t[0])    ,  
//     .Y1n(t[1])    , 
//     .Y2n(t[2])    , 
//     .Y3n(t[3])    , 
//     .Y4n(t[4])    , 
//     .Y5n(t[5])    , 
//     .Y6n(t[6])    , 
//     .Y7n(t[7])    
// );

// assign D = ~& {t[1],t[2],t[4],t[7]}; //1 2 4 7
// assign Co= ~& {t[1],t[2],t[3],t[7]}; //1 2 3 7
// endmodule

// // VL18 实现3-8译码器
// `timescale 1ns/1ns

// module decoder_38(
//     input             E1_n   ,
//     input             E2_n   ,
//     input             E3     ,
//     input             A0     ,
//     input             A1     ,
//     input             A2     ,
    
//     output wire       Y0_n   ,  
//     output wire       Y1_n   , 
//     output wire       Y2_n   , 
//     output wire       Y3_n   , 
//     output wire       Y4_n   , 
//     output wire       Y5_n   , 
//     output wire       Y6_n   , 
//     output wire       Y7_n   
// );
//     assign Y0_n = !((&{E3,!E2_n,!E1_n}) && (&{!A2,!A1,!A0}));
//     assign Y1_n = !((&{E3,!E2_n,!E1_n}) && (&{!A2,!A1,A0}));
//     assign Y2_n = !((&{E3,!E2_n,!E1_n}) && (&{!A2,A1,!A0}));
//     assign Y3_n = !((&{E3,!E2_n,!E1_n}) && (&{!A2,A1,A0}));
//     assign Y4_n = !((&{E3,!E2_n,!E1_n}) && (&{A2,!A1,!A0}));
//     assign Y5_n = !((&{E3,!E2_n,!E1_n}) && (&{A2,!A1,A0}));
//     assign Y6_n = !((&{E3,!E2_n,!E1_n}) && (&{A2,A1,!A0}));
//     assign Y7_n = !((&{E3,!E2_n,!E1_n}) && (&{A2,A1,A0}));

// endmodule

// VL19 使用3-8译码器①实现逻辑函数
// ②请使用3-8译码器①和必要的逻辑门实现函数L=(~A)·C+A·B
// `timescale 1ns/1ns

// module decoder_38(
//     input             E1_n   ,
//     input             E2_n   ,
//     input             E3     ,
//     input             A0     ,
//     input             A1     ,
//     input             A2     ,
    
//     output wire       Y0_n   ,  
//     output wire       Y1_n   , 
//     output wire       Y2_n   , 
//     output wire       Y3_n   , 
//     output wire       Y4_n   , 
//     output wire       Y5_n   , 
//     output wire       Y6_n   , 
//     output wire       Y7_n   
// );
// wire E ;
// assign E = E3 & ~E2_n & ~E1_n;
// assign  Y0_n = ~(E & ~A2 & ~A1 & ~A0);
// assign  Y1_n = ~(E & ~A2 & ~A1 &  A0);
// assign  Y2_n = ~(E & ~A2 &  A1 & ~A0);
// assign  Y3_n = ~(E & ~A2 &  A1 &  A0);
// assign  Y4_n = ~(E &  A2 & ~A1 & ~A0);
// assign  Y5_n = ~(E &  A2 & ~A1 &  A0);
// assign  Y6_n = ~(E &  A2 &  A1 & ~A0);
// assign  Y7_n = ~(E &  A2 &  A1 &  A0);

// endmodule

// module decoder0(
//     input             A     ,
//     input             B     ,
//     input             C     ,
    
//     output wire       L
// );
// wire [7:0] t;

// decoder_38 inst1(
//     .E1_n (1'b0),
//     .E2_n (1'b0),
//     .E3   (1'b1),
//     .A0   (C),
//     .A1   (B),
//     .A2   (A),
//     .Y0_n (t[0]),
//     .Y1_n (t[1]),
//     .Y2_n (t[2]),
//     .Y3_n (t[3]),
//     .Y4_n (t[4]),
//     .Y5_n (t[5]),
//     .Y6_n (t[6]),
//     .Y7_n (t[7])   
// );
// assign L = !(&{t[1],t[3],t[6],t[7]});
// endmodule`timescale 1ns/1ns

// VL20 数据选择器实现逻辑电路
// 描述
// 请使用此4选1数据选择器和必要的逻辑门实现下列表达式。
// L=A∙B+A∙~C+B∙C
// `timescale 1ns/1ns

// module data_sel(
//     input             S0     ,
//     input             S1     ,
//     input             D0     ,
//     input             D1     ,
//     input             D2     ,
//     input             D3     ,
    
//     output wire        Y    
// );

// assign Y = ~S1 & (~S0&D0 | S0&D1) | S1&(~S0&D2 | S0&D3);

// endmodule

// module sel_exp(
//     input             A     ,
//     input             B     ,
//     input             C     ,
    
//     output wire       L            
// );

// data_sel inst1(
// .S0 (C),
// .S1 (1'b0),
// .D0 (A),
// .D1 (B),
// .D2 (),
// .D3 (),
// .Y  (L)
// );
// endmodule

// VL21 根据状态转移表实现时序电路
// `timescale 1ns/1ns

// module seq_circuit(
//     input                A   ,
//     input                clk ,
//     input                rst_n,
    
//     output   wire        Y   
// );

// reg [1:0] state,next_state;

// always@(*)begin
//     case(state)
//         2'b00:next_state=A?2'b11:2'b01;
//         2'b01:next_state=A?2'b00:2'b10;
//         2'b10:next_state=A?2'b01:2'b11;
//         2'b11:next_state=A?2'b10:2'b00;
//     endcase
// end

// always@(posedge clk,negedge rst_n)begin
//     if(!rst_n) state <= 2'b00;
//     else begin
//         state <= next_state;
//     end
// end

// assign Y = (state==2'b11);

// endmodule


// VL22 根据状态转移图实现时序电路
// `timescale 1ns/1ns

// module seq_circuit(
//     input                C   ,
//     input                clk ,
//     input                rst_n,
    
//     output   wire        Y   
// );

// reg [1:0] state,next_state;

// always@(*)begin
//     case(state)
//         2'b00:next_state=C?2'b01:2'b00;
//         2'b01:next_state=C?2'b01:2'b11;
//         2'b10:next_state=C?2'b10:2'b00;
//         2'b11:next_state=C?2'b10:2'b11;
//     endcase
// end

// always@(posedge clk,negedge rst_n)begin
//     if(!rst_n) state <= 2'b00;
//     else begin
//         state <= next_state;
//     end
// end

// assign Y = (state==2'b11)||((state==2'b10)&C);

// endmodule

// VL23 ROM的简单实现
// 描述
// 实现一个深度为8，位宽为4bit的ROM，数据初始化为0，2，4，6，8，10，12，14。可以通过输入地址addr，输出相应的数据data。
// `timescale 1ns/1ns
// module rom(
// 	input clk,
// 	input rst_n,
// 	input [7:0]addr,
	
// 	output [3:0]data
// );

// reg [3:0] r [7:0];

// // always@(posedge clk,negedge rst_n)begin
// // 	if(!rst_n)begin
// // 		{r[0],r[1],r[2],r[3],r[4],r[5],r[6],r[7]} <= {4'd0,4'd2,4'd4,4'd6,4'd8,4'd10,4'd12,4'd14};
// // 	end
// // end
// integer i; 
// always@(posedge clk or negedge rst_n)begin 
//     if(!rst_n)begin 
//         for(i = 0; i < 7; i = i + 1)begin 
//             r[i] <= i * 2; 
//             end 
//         end 
//     end

// assign data = r[addr];
// endmodule

// VL24 边沿检测
// 描述
// 有一个缓慢变化的1bit信号a，编写一个程序检测a信号的上升沿给出指示信号rise，当a信号出现下降沿时给出指示信号down。
// 注：rise,down应为单脉冲信号，在相应边沿出现时的下一个时钟为高，之后恢复到0，一直到再一次出现相应的边沿。
// `timescale 1ns/1ns
// module edge_detect(
// 	input clk,
// 	input rst_n,
// 	input a,
	
// 	output reg rise,
// 	output reg down
// );
	
// reg pedge;

// always@(posedge clk,negedge rst_n)begin
// 	if(!rst_n)begin
// 		pedge <= 1'b0;
// 		rise  <= 0;
// 		down  <= 0;
// 	end
// 	else begin
// 		pedge <= a;
// 		rise  <= ((!pedge&a)===1)?1:0;
// 		down  <= ((pedge&!a)===1)?1:0;//全等消除不定态
// 	end
// end

// endmodule

// //~ `New testbench
// `timescale  1ns / 1ps

// module tb_edge_detect;

// // edge_detect Parameters
// parameter PERIOD  = 10;

// // edge_detect Inputs
// reg   clk                                  = 0 ;
// reg   rst_n                                = 0 ;
// reg   a                                    = 0 ;

// // edge_detect Outputs
// wire  rise                                 ;
// wire  down                                 ;

// initial
// begin
//     $dumpfile ("HDL_bit_wave.vcd");
//     $dumpvars;
//     forever #(PERIOD/2)  clk=~clk;
// end

// initial
// begin
//     #(PERIOD*2) rst_n  =  1;
// end

// edge_detect  u_edge_detect (
//     .clk                     ( clk     ),
//     .rst_n                   ( rst_n   ),
//     .a                       ( a       ),

//     .rise                    ( rise    ),
//     .down                    ( down    )
// );

// initial
// begin
//     #0.001;
//     #(PERIOD*2) a  =  1;
//     #(PERIOD*2) a  =  0;
//     #(PERIOD*2) a  =  1;
//     #(PERIOD*2) a  =  0;

//     $finish;
// end

// endmodule

// 牛客进阶挑战
