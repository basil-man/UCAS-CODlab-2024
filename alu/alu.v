`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ALUop_AND 3'b000
`define ALUop_OR 3'b001
`define ALUop_ADD 3'b010
`define ALUop_SUB 3'b110
`define ALUop_SLT 3'b111
`define ALUop_XOR 3'b100
`define ALUop_NOR 3'b101
`define ALUop_SLTU 3'b011

module alu(
	input  [`DATA_WIDTH - 1:0]  A,
	input  [`DATA_WIDTH - 1:0]  B,
	input  [              2:0]  ALUop,
	output                      Overflow,
	output                      CarryOut,
	output                      Zero,
	output [`DATA_WIDTH - 1:0]  Result
);
	//wires
	wire [`DATA_WIDTH-1:0]out_and;
	wire [`DATA_WIDTH-1:0]out_or;
	wire [`DATA_WIDTH-1:0]out_xor;
	wire [`DATA_WIDTH-1:0]out_add_sub_slt;
	wire [`DATA_WIDTH-1:0]out_slt;
	wire [`DATA_WIDTH-1:0]out_sltu;
	wire [`DATA_WIDTH-1:0]neg_B;
	wire cin;

	//32-bit and
	assign out_and=A&B;

	//32-bit or
	assign out_or=A|B;

	//32_bit xor
	assign out_xor=A^B;

	//32-bit ADD/SUB/SLT
	wire Bsig;
	wire CO;
	assign Bsig=ALUop==`ALUop_ADD;
	assign neg_B=({`DATA_WIDTH{Bsig}}&B)|({`DATA_WIDTH{(~Bsig)}}&(~B));
	assign cin=~Bsig;
	assign {CO,out_add_sub_slt}=A+neg_B+{31'b0,cin};
	assign CarryOut=CO^(ALUop==`ALUop_SUB);
	assign Overflow=(~A[(`DATA_WIDTH-1)]&~neg_B[(`DATA_WIDTH-1)]&out_add_sub_slt[(`DATA_WIDTH-1)])|(A[(`DATA_WIDTH-1)]&neg_B[(`DATA_WIDTH-1)]&~out_add_sub_slt[(`DATA_WIDTH-1)]);
	assign out_slt={31'b0,out_add_sub_slt[(`DATA_WIDTH-1)]^Overflow};
	
	assign out_sltu={31'b0,((A[31]^B[31])&~out_slt[0])|(~(A[31]^B[31])&out_slt[0])};


	//result
	assign Result=({`DATA_WIDTH{ALUop==`ALUop_AND}}&out_and)|
		({`DATA_WIDTH{ALUop==`ALUop_OR}}&out_or)|
		({`DATA_WIDTH{ALUop==`ALUop_ADD}}&out_add_sub_slt)|
		({`DATA_WIDTH{ALUop==`ALUop_SUB}}&out_add_sub_slt)|
		({`DATA_WIDTH{ALUop==`ALUop_SLT}}&out_slt)|
		({`DATA_WIDTH{ALUop==`ALUop_XOR}}&out_xor)|
		({`DATA_WIDTH{ALUop==`ALUop_NOR}}&(~out_or))|
		({`DATA_WIDTH{ALUop==`ALUop_SLTU}}&out_sltu);
	assign Zero=Result==`DATA_WIDTH'b0;



endmodule
