`timescale 10ns / 1ns

`define ADDR_WIDTH 32
`define ALUop_AND 3'b000
`define ALUop_OR 3'b001
`define ALUop_ADD 3'b010
`define ALUop_SUB 3'b110
`define ALUop_SLT 3'b111
`define ALUop_XOR 3'b100
`define ALUop_NOR 3'b101
`define ALUop_SLTU 3'b011

`define OPCODE_R 6'b000000
`define OPCODE_REGIMM 6'b000001
`define OPCODE_J 5'b00001
`define OPCODE_I_branch 4'b0001
`define OPCODE_I_acc 3'b001
`define WADDR_31 5'b11111

`define ALUopMUX_ADD 3'b000
`define ALUopMUX_SUB 3'b001
`define ALUopMUX_Racc 3'b010
`define ALUopMUX_SLT 3'b011
`define ALUopMUX_Iacc 3'b100

module simple_cpu(
	input             clk,
	input             rst,

	output [31:0]     PC,
	input  [31:0]     Instruction,

	output [31:0]     Address,
	output            MemWrite,
	output [31:0]     Write_data,
	output [ 3:0]     Write_strb,

	input  [31:0]     Read_data,
	output            MemRead
);

	// THESE THREE SIGNALS ARE USED IN OUR TESTBENCH
	// PLEASE DO NOT MODIFY SIGNAL NAMES
	// AND PLEASE USE THEM TO CONNECT PORTS
	// OF YOUR INSTANTIATION OF THE REGISTER FILE MODULE
	wire			RF_wen;
	wire [4:0]		RF_waddr;
	wire [31:0]		RF_wdata;

	// TODO: PLEASE ADD YOUR CODE BELOW
	//wires
	wire [31:0] rdata1,rdata2;
	reg [31:0] reg_PC;
	wire [31:0] next_PC;
	wire [31:0] PCp4;
	wire Branch;
	wire [5:0] opcode,funct;
	wire [4:0] rs,rt,rd,shamt;
	wire [15:0] imm_and_offset;
	wire [25:0] target_addr;
	wire [31:0] extended_offset;
	wire R_cc,R_shift,R_jmp,R_mov,REGIMM,J,I_branch,I_acc,I_mem_r,I_mem_w;
	wire [2:0] ALUop,ALUin,R_accop,I_accop;
	wire [4:0]raddr1,raddr2;
	wire [31:0] shift_result;
	wire [31:0] A,B,Result;
	wire [31:0] MemRead_data;
	wire [31:0] mask,masked_data,lwlr_signal,lwl_00,lwl_01,lwl_10,lwl_11,lwr_00,lwr_01,lwr_10,lwr_11;
	wire [7:0] strb_swlr;
	wire [3:0] strb_else;
	wire Zero;
	wire [31:0] shifter_A;
	wire [4:0] shifter_B;
	wire [1:0] shifter_op;

//PC
	always @(posedge clk) begin
		if(rst==1)begin
			reg_PC<=0;
		end
		else begin
			reg_PC<=next_PC;
		end
	end

//IF SECTION
	assign PC=reg_PC;
	assign PCp4=PC+4;

//ID SECTION
	//analysis the instruction
	assign opcode=Instruction[31:26];
	assign funct=Instruction[5:0];
	assign rs=Instruction[25:21];
	assign rt=Instruction[20:16];
	assign rd=Instruction[15:11];
	assign shamt=Instruction[10:6];
	assign imm_and_offset=Instruction[15:0];
	assign target_addr=Instruction[25:0];

	//sign-extend and zero-extend
	assign extended_offset={{16{imm_and_offset[15]&(~(I_acc&(opcode[2:1]==2'b10)))}},imm_and_offset};

	//types
	assign R_acc=(opcode[5:0]==`OPCODE_R)&(funct[5]);
	assign R_shift=(opcode[5:0]==`OPCODE_R)&(~funct[3])&(~funct[5]);
	assign R_jmp=(opcode[5:0]==`OPCODE_R)&(funct[3])&(~funct[1]);
	assign R_mov=(opcode[5:0]==`OPCODE_R)&(~funct[5])&(funct[3])&(funct[1]);
	assign REGIMM=opcode[5:0]==`OPCODE_REGIMM;
	assign J=opcode[5:1]==`OPCODE_J;
	assign I_branch=opcode[5:2]==`OPCODE_I_branch;
	assign I_acc=opcode[5:3]==`OPCODE_I_acc;
	assign I_mem_r=opcode[5]&(~opcode[3]);
	assign I_mem_w=opcode[5]&opcode[3];

	//Branch
	assign Branch=(REGIMM&(rt[0]^Result))|
			(I_branch&(opcode[0]^Zero));

	//reg_file write
	assign RF_wen=R_acc|I_acc|R_shift|(R_mov&(funct[0]^Zero))|I_mem_r|(J&opcode[0])|(R_jmp&funct[0]);
	assign RF_waddr=(rd&{5{R_acc|R_shift|R_mov}})|(rt&{5{I_acc|I_mem_r}})|(`WADDR_31&{5{J|R_jmp}});

	//reg_file
	reg_file reg_file(
		.clk(clk),
		.waddr (RF_waddr),
		.raddr1 (rs),
		.raddr2 (rt),
		.wen(RF_wen),
		.wdata (RF_wdata),
		.rdata1 (rdata1),
		.rdata2 (rdata2)
	);
	
	//ALUop
	assign ALUop=({3{R_mov|I_mem_r|I_mem_w}}&`ALUopMUX_ADD)|
			({3{I_branch&~opcode[1]}}&`ALUopMUX_SUB)|
			({3{R_acc}}&`ALUopMUX_Racc)|
			({3{REGIMM|(I_branch&opcode[1])}}&`ALUopMUX_SLT)|
			({3{I_acc}}&`ALUopMUX_Iacc);//get mux signal from opcode
	assign I_accop[2]=(opcode[2]|(~opcode[0]))&opcode[1];
	assign I_accop[1]=~opcode[2];
	assign I_accop[0]=((~opcode[2])&opcode[1])|(opcode[2]&opcode[0]);


//EX SECTION
	//ALU control
	assign R_accop[2]=(~(funct[3]&funct[0]))&funct[1];
	assign R_accop[1]=~funct[2];
	assign R_accop[0]=funct[3]|(funct[2]&funct[0]);
	assign ALUin=({3{ALUop==`ALUopMUX_ADD}}&`ALUop_ADD)|
			({3{ALUop==`ALUopMUX_SUB}}&`ALUop_SUB)|
			({3{ALUop==`ALUopMUX_Racc}}&R_accop)|
			({3{ALUop==`ALUopMUX_SLT}}&`ALUop_SLT)|
			({3{ALUop==`ALUopMUX_Iacc}}&I_accop);//use mux signal to choose the ALUop
	
	//ALU
	assign A=({32{R_mov}}&32'b0)|
		({32{I_branch&opcode[1]}}&rdata2)|
		({32{J|R_jmp}}&PC)|
		({32{~(R_mov|(I_branch&opcode[1])|J|R_jmp)}}&rdata1);
	assign B=({32{R_acc|(I_branch&(~opcode[1]))|R_mov}}&rdata2)|
		({32{I_branch&opcode[1]}}&rdata1)|
		({32{J|R_jmp}}&32'd8)|
		({32{~(R_acc|(I_branch&(~opcode[1]))|R_mov|(I_branch&opcode[1])|J|R_jmp)}}&(REGIMM?0:extended_offset));
	alu alu(
		.A(A),
		.B(B),
		.ALUop(ALUin),
		.Overflow(),
		.CarryOut(),
		.Zero(Zero),
		.Result(Result)
	);

	//shift_result
	assign shifter_A=I_mem_r?Read_data:rdata2;
	assign shifter_B=({5{R_shift}}&(funct[2]?rdata1[4:0]:shamt))|
			({5{I_mem_r}}&{Result[1:0],3'b000})|
			({5{I_mem_w}}&{(opcode[1:0]==2'b10&~opcode[2])?(~Result[1:0]):Result[1:0],3'b000});//~Result[1:0] refers to 3-Result[1:0]
	assign shifter_op=({2{R_shift}}&funct[1:0])|
			({2{I_mem_r}}&2'b10)|
			({2{I_mem_w}}&((opcode[1:0]==2'b10&~opcode[2])?2'b10:2'b00));
	shifter shifter(
		.A(shifter_A),
		.B(shifter_B),
		.Shiftop(shifter_op),
		.Result(shift_result)
	);

	//next PC
	assign next_PC=({32{Branch}}&({extended_offset[29:0],2'b00}+PCp4))|
			({32{J}}&({PCp4[31:28],target_addr,2'b00}))|
			({32{R_jmp}}&rdata1)|
			({32{~(Branch|J|R_jmp)}}&PCp4);
	
//MEM SECTION
	//mem access
	assign Address={Result[31:2],2'b00};
	assign MemRead=I_mem_r;
	assign MemWrite=I_mem_w;
	assign Write_data=shift_result;
	assign {strb_swlr,strb_else}=({12{Result[1:0]==2'b00}}&{8'b00011111,{{2{opcode[1]}},opcode[0],1'b1}})|
			({12{Result[1:0]==2'b01}}&{8'b00111110,{opcode[1],opcode[0],2'b10}})|
			({12{Result[1:0]==2'b10}}&{8'b01111100,{opcode[0],3'b100}})|
			({12{Result[1:0]==2'b11}}&12'b111110001000);//generate strb_else for S instructions and strb_swlr for SWL and SWR instructions
	assign Write_strb=(({4{!(opcode[1:0]==2'b10)}})&strb_else)|
				({4{opcode[1:0]==2'b10}}&(~opcode[2]?strb_swlr[7:4]:strb_swlr[3:0]));

//WB SECTION
	//read data process
	assign mask={{16{opcode[1]}},{8{opcode[0]}},8'b11111111};
	assign masked_data=mask&shift_result;
	assign MemRead_data=({32{opcode[1:0]==2'b11}}&masked_data)|
			({32{opcode[1:0]==2'b01}}&{{16{(~opcode[2])&masked_data[15]}},masked_data[15:0]})|
			({32{opcode[1:0]==2'b00}}&{{24{(~opcode[2])&masked_data[7]}},masked_data[7:0]});//choose the masked data of LB,LW,LH
	
	//lwl and lwr
	assign lwl_00={Read_data[7:0],rdata2[23:0]};
	assign lwl_01={Read_data[15:0],rdata2[15:0]};
	assign lwl_10={Read_data[23:0],rdata2[7:0]};
	assign lwl_11=Read_data;
	assign lwr_00=Read_data;
	assign lwr_01={rdata2[31:24],Read_data[31:8]};
	assign lwr_10={rdata2[31:16],Read_data[31:16]};
	assign lwr_11={rdata2[31:8],Read_data[31:24]};
	assign lwlr_signal=({32{opcode[2]}}&(({32{Result[1:0]==2'b00}}&lwr_00)|
						({32{Result[1:0]==2'b01}}&lwr_01)|
						({32{Result[1:0]==2'b10}}&lwr_10)|
						({32{Result[1:0]==2'b11}}&lwr_11)))|
			({32{~opcode[2]}}&(({32{Result[1:0]==2'b00}}&lwl_00)|
						({32{Result[1:0]==2'b01}}&lwl_01)|
						({32{Result[1:0]==2'b10}}&lwl_10)|
						({32{Result[1:0]==2'b11}}&lwl_11)));
	
	//wdata
	assign RF_wdata=({32{R_acc|(I_acc&(opcode[2:0]!=3'b111))|(J&opcode[0])|(R_jmp&funct[0])}}&Result)|
			({32{I_acc&(opcode[2:0]==3'b111)}}&{imm_and_offset,16'b0})|
			({32{R_shift}}&shift_result)|
			({32{R_mov}}&rdata1)|
			({32{I_mem_r&(!(opcode[1:0]==2'b10))}}&(MemRead_data))|
			(({32{I_mem_r&(opcode[1:0]==2'b10)}}&lwlr_signal));



endmodule
