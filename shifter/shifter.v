`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define LEFT 2'b00
`define ALGO_RIGHT 2'b11
`define LOGI_RIGHT 2'b10
module shifter (
	input  [`DATA_WIDTH - 1:0] A,
	input  [              4:0] B,
	input  [              1:0] Shiftop,
	output [`DATA_WIDTH - 1:0] Result
);
	// TODO: Please add your logic code here
	wire [`DATA_WIDTH - 1:0] left;
	wire [`DATA_WIDTH - 1:0] algo_right;
	wire [`DATA_WIDTH - 1:0] logi_right;
	wire signed [`DATA_WIDTH - 1:0] signed_A;
	assign signed_A=A;
	assign left=A<<B;
	assign algo_right=signed_A>>>B;
	assign logi_right=A>>B;
	assign Result=({32{Shiftop==`LEFT}}&left)|
			({32{Shiftop==`ALGO_RIGHT}}&algo_right)|
			({32{Shiftop==`LOGI_RIGHT}}&logi_right);
	
endmodule
