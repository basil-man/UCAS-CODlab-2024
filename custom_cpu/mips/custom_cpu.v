`timescale 10ns / 1ns
`timescale 10ns / 1ns

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

`define INST_NOP 32'b0

module custom_cpu(
	input         clk,
	input         rst,

	//Instruction request channel
	output [31:0] PC,
	output        Inst_Req_Valid,
	input         Inst_Req_Ready,

	//Instruction response channel
	input  [31:0] Instruction,
	input         Inst_Valid,
	output        Inst_Ready,

	//Memory request channel
	output [31:0] Address,
	output        MemWrite,
	output [31:0] Write_data,
	output [ 3:0] Write_strb,
	output        MemRead,
	input         Mem_Req_Ready,

	//Memory data response channel
	input  [31:0] Read_data,
	input         Read_data_Valid,
	output        Read_data_Ready,

	input         intr,

	output [31:0] cpu_perf_cnt_0,
	output [31:0] cpu_perf_cnt_1,
	output [31:0] cpu_perf_cnt_2,
	output [31:0] cpu_perf_cnt_3,
	output [31:0] cpu_perf_cnt_4,
	output [31:0] cpu_perf_cnt_5,
	output [31:0] cpu_perf_cnt_6,
	output [31:0] cpu_perf_cnt_7,
	output [31:0] cpu_perf_cnt_8,
	output [31:0] cpu_perf_cnt_9,
	output [31:0] cpu_perf_cnt_10,
	output [31:0] cpu_perf_cnt_11,
	output [31:0] cpu_perf_cnt_12,
	output [31:0] cpu_perf_cnt_13,
	output [31:0] cpu_perf_cnt_14,
	output [31:0] cpu_perf_cnt_15,

	output [69:0] inst_retire
);

/* The following signal is leveraged for behavioral simulation, 
* which is delivered to testbench.
*
* STUDENTS MUST CONTROL LOGICAL BEHAVIORS of THIS SIGNAL.
*
* inst_retired (70-bit): detailed information of the retired instruction,
* mainly including (in order) 
* { 
*   reg_file write-back enable  (69:69,  1-bit),
*   reg_file write-back address (68:64,  5-bit), 
*   reg_file write-back data    (63:32, 32-bit),  
*   retired PC                  (31: 0, 32-bit)
* }
*
*/


	//wire [69:0] inst_retire;
	
// TODO: Please add your custom CPU code here

	// TODO: PLEASE ADD YOUR CODE BELOW
	//FSM states
	wire			RF_wen;
	wire [4:0]		RF_waddr;
	wire [31:0]		RF_wdata;
    localparam  INIT= 10'b0000000001,
                IF  = 10'b0000000010,
                LD  = 10'b0000000100,
                EX  = 10'b0000001000,
                ID  = 10'b0000010000,
                WB  = 10'b0000100000,
                IW  = 10'b0001000000,
                ST  = 10'b0010000000,
                RDW = 10'b0100000000,
				INTR= 10'b1000000000;
	//wires
	wire [31:0] rdata1, rdata2;
	wire [31:0] next_PC;
	wire Branch;
	wire [5:0] opcode, funct;
	wire [4:0] rs, rt, rd, shamt;
	wire [15:0] imm_and_offset;
	wire [25:0] target_addr;
	wire [31:0] extended_offset;
	wire R_acc, R_shift, R_jmp, R_mov, REGIMM, J, I_branch, I_acc, I_mem_r, I_mem_w;
	wire [2:0] ALUop, ALUin, R_accop, I_accop;
	wire [4:0]raddr1, raddr2;
	wire [31:0] shift_result;
	wire [31:0] A, B, Result;
	wire [31:0] MemRead_data;
	wire [31:0] mask, masked_data, lwlr_signal, lwl_00, lwl_01, lwl_10, lwl_11, lwr_00, lwr_01, lwr_10, lwr_11;
	wire [7:0] strb_swlr;
	wire [3:0] strb_else;
	wire Zero;
	wire [31:0] shifter_A;
	wire [4:0] shifter_B;
	wire [1:0] shifter_op;

//inst_retire
	assign inst_retire[69]    = RF_wen;
	assign inst_retire[68:64] = RF_waddr;
	assign inst_retire[63:32] = RF_wdata;
	assign inst_retire[31:0]  = reg_PC;

  //FSM regs
	//state regs
	reg [9:0] current_state;
	reg [9:0] next_state;
	
	//PC regs
	reg [31:0] reg_PC;
	reg [31:0] reg_PCp4;
	reg [31:0] reg_PCpShift;

	//inside regs
	reg [31:0] reg_Instruction;
	reg [31:0] reg_Read_data;
	reg [31:0] reg_rdata1;
	reg [31:0] reg_rdata2;
	reg [31:0] reg_Result;
	
	//current states
	wire state_INTR,state_INIT, state_IF, state_LD, state_EX, state_ID, state_WB, state_IW, state_ST, state_RDW;
	assign state_INIT= current_state[0];
	assign state_IF  = current_state[1];
	assign state_LD  = current_state[2];
	assign state_EX  = current_state[3];
	assign state_ID  = current_state[4];
	assign state_WB  = current_state[5];
	assign state_IW  = current_state[6];
	assign state_ST  = current_state[7];
	assign state_RDW = current_state[8];
	assign state_INTR= current_state[9];


//INTR
	wire ERET=reg_Instruction==32'b01000010000000000000000000011000;
	reg [31:0] EPC;
	always @(posedge clk)begin
		if(state_INTR)begin
			EPC<=PC;
		end

	end	

	reg intr_mask;
	always @(posedge clk)begin
		if(rst||(state_ID&&ERET))
			intr_mask<=0;
		else if(state_INTR)
			intr_mask<=1;
		else 
			intr_mask<=intr_mask;
	end


//FSM part 1
	always @(posedge clk) begin
		if(rst)
			current_state<= INIT;
		else
			current_state<= next_state;
	end


//FSM part 2
	always @(*) begin
		case (current_state)
			INIT: begin
				next_state = IF;
			end

			IF: begin
				if(intr&&!intr_mask)
					next_state = INTR;
				else if(Inst_Req_Ready)
					next_state = IW;
				else
					next_state = IF;
			end 

			IW: begin
				if(Inst_Valid)
					next_state = ID;
				else 
					next_state = IW;
			end

			ID: begin
				if(reg_Instruction==`INST_NOP)
					next_state = IF;
				else
					next_state = EX;//not NOP
			end 

			EX: begin
				if(REGIMM|I_branch|(J&(~opcode[0]))|ERET)//REGIMM or I-type jump or J
					next_state = IF;
				else if(R_acc|R_jmp|R_mov|R_shift|I_acc|(J&opcode[0]))  //R-type or I-type operation or JAL
					next_state = WB;
				else if(I_mem_r)//Load inst
					next_state = LD;
				else if(I_mem_w)//Store inst
					next_state = ST;
				else
					next_state = EX;
			end 

			ST: begin
				if(Mem_Req_Ready)
					next_state = IF;
				else
					next_state = ST;
			end 

			LD: begin
				if(Mem_Req_Ready)
					next_state = RDW;
				else
					next_state = LD;
			end 

			RDW: begin
				if(Read_data_Valid)
					next_state = WB;
				else
					next_state = RDW;
			end 

			WB: begin
				next_state = IF;
			end 

			INTR: begin
				next_state = IF;
			end

			default: begin
				next_state = INIT;
			end
		endcase
	end
	
	//IW
	always @(posedge clk)begin
		if(rst)
			reg_Instruction<= 32'b0;
		else if(state_IW&&Inst_Valid)
			reg_Instruction<= Instruction;//get inst into reg
	end
	
	//MEM
	always @(posedge clk)begin
		if(rst)
			reg_Read_data<= 32'b0;
		else
			reg_Read_data<= Read_data;//get Read_data into reg
	end

	//get new PC
	wire PC_update;
	assign PC_update = (state_IW&Inst_Valid)|(state_EX&(Branch|J|R_jmp|ERET));
	always @(posedge clk) begin
		if(rst)
			reg_PC<= 32'b0;
		else if(state_INTR)
			reg_PC<=32'h100;
		else if(PC_update)
			reg_PC<= next_PC;
	end
	assign PC = reg_PC;

	//INSTs
	assign Inst_Req_Valid  = state_IF&~(intr&~intr_mask);
	assign Inst_Ready      = state_IW|state_INIT;
	assign Read_data_Ready = state_RDW|state_INIT;


//ID SECTION

	//analysis the instruction
	assign opcode         = reg_Instruction[31:26];
	assign funct          = reg_Instruction[5:0];
	assign rs             = reg_Instruction[25:21];
	assign rt             = reg_Instruction[20:16];
	assign rd             = reg_Instruction[15:11];
	assign shamt          = reg_Instruction[10:6];
	assign imm_and_offset = reg_Instruction[15:0];
	assign target_addr    = reg_Instruction[25:0];

	//sign-extend and zero-extend
	assign extended_offset = {{16{imm_and_offset[15]&(~(I_acc&(opcode[2:1]==2'b10)))}},imm_and_offset};

	//types
	assign R_acc   = (opcode[5:0]==`OPCODE_R)&(funct[5]);
	assign R_shift = (opcode[5:0]==`OPCODE_R)&(~funct[3])&(~funct[5]);
	assign R_jmp   = (opcode[5:0]==`OPCODE_R)&(funct[3])&(~funct[1]);
	assign R_mov   = (opcode[5:0]==`OPCODE_R)&(~funct[5])&(funct[3])&(funct[1]);
	assign REGIMM  = opcode[5:0] == `OPCODE_REGIMM;
	assign J       = opcode[5:1]==`OPCODE_J;
	assign I_branch= opcode[5:2]==`OPCODE_I_branch;
	assign I_acc   = opcode[5:3]==`OPCODE_I_acc;
	assign I_mem_r = opcode[5]&(~opcode[3]);
	assign I_mem_w = opcode[5]&opcode[3];

	//Branch
	assign Branch = (REGIMM&(rt[0]^Result[0]))|
					(I_branch&(opcode[0]^Zero));

	//reg_file write
	assign RF_wen   = (R_acc|I_acc|R_shift|(R_mov&(funct[0]^Zero))|I_mem_r|(J&opcode[0])|(R_jmp&funct[0]))&state_WB;
	assign RF_waddr = (rd&{5{R_acc|R_shift|R_mov}})|(rt&{5{I_acc|I_mem_r}})|(`WADDR_31&{5{J|R_jmp}});

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
	always @(posedge clk)begin
		if(rst)
			reg_rdata1<= 32'b0;
		else
			reg_rdata1<= rdata1;
	end

	always @(posedge clk)begin
		if(rst)
			reg_rdata2<= 32'b0;
		else
			reg_rdata2<= rdata2;
	end



	
	//ALUop
	assign ALUop = ({3{R_mov|I_mem_r|I_mem_w}}&`ALUopMUX_ADD)		|
				   ({3{I_branch&~opcode[1]}}&`ALUopMUX_SUB)			|
				   ({3{R_acc}}&`ALUopMUX_Racc)						|
				   ({3{REGIMM|(I_branch&opcode[1])}}&`ALUopMUX_SLT) |
				   ({3{I_acc}}&`ALUopMUX_Iacc);  					 //get mux signal from opcode
	assign I_accop[2] = (opcode[2]|(~opcode[0]))&opcode[1];
	assign I_accop[1] = ~opcode[2];
	assign I_accop[0] = ((~opcode[2])&opcode[1])|(opcode[2]&opcode[0]);


//EX SECTION
	//ALU control
	assign R_accop[2] = (~(funct[3]&funct[0]))&funct[1];
	assign R_accop[1] = ~funct[2];
	assign R_accop[0] = funct[3]|(funct[2]&funct[0]);
	assign ALUin      = ({3{ALUop==`ALUopMUX_ADD}}&`ALUop_ADD)|
						({3{ALUop==`ALUopMUX_SUB}}&`ALUop_SUB)|
						({3{ALUop==`ALUopMUX_Racc}}&R_accop)  |
						({3{ALUop==`ALUopMUX_SLT}}&`ALUop_SLT)|
						({3{ALUop==`ALUopMUX_Iacc}}&I_accop)  ;  //use mux signal to choose the ALUop
	
	//ALU
	assign A=((({32{R_mov}}&32'b0)																|
			   ({32{I_branch&opcode[1]}}&reg_rdata2)											|
			   ({32{J|R_jmp}}&PC)																|
			   ({32{~(R_mov|(I_branch&opcode[1])|J|R_jmp)}}&reg_rdata1))&{32{state_EX|state_WB}})
			| (32'd4&{32{state_IW}})
			| ({extended_offset[29:0],2'b00}&{32{state_ID}});
	assign B=((({32{R_acc|(I_branch&(~opcode[1]))|R_mov}}&reg_rdata2)	|
			   ({32{I_branch&opcode[1]}}&reg_rdata1)					|
			   ({32{J|R_jmp}}&32'd4)									|
		 	  ({32{~(R_acc|(I_branch&(~opcode[1]))|R_mov|(I_branch&opcode[1])|J|R_jmp)}}&(REGIMM?0:extended_offset)))&{32{state_EX|state_WB}})
			| (reg_PC&{32{state_IW}})
			| (reg_PCp4&{32{state_ID}});
	alu alu(
		.A(A),
		.B(B),
		.ALUop((state_ID|state_IW)?`ALUop_ADD:ALUin),
		.Overflow(),
		.CarryOut(),
		.Zero(Zero),
		.Result(Result)
	);
	always @(posedge clk)begin
		if(rst)
			reg_PCp4<=32'b0;
		else if(state_IW)
			reg_PCp4<=Result;
	end
	always @(posedge clk)begin
		if(rst)
			reg_PCpShift<=32'b0;
		else if(state_ID)
			reg_PCpShift<=Result;
	end
	always @(posedge clk)begin
		if(rst)
			reg_Result<=32'b0;
		else if(state_EX)
			reg_Result<=Result;
	end


	//shift_result
	assign shifter_A=I_mem_r?reg_Read_data:reg_rdata2;
	assign shifter_B=({5{R_shift}}&(funct[2]?reg_rdata1[4:0]:shamt))	|
			({5{I_mem_r}}&{reg_Result[1:0],3'b000})						|
			({5{I_mem_w}}&{(opcode[1:0]==2'b10&~opcode[2])?(~reg_Result[1:0]):reg_Result[1:0],3'b000});//~reg_Result[1:0] refers to 3-reg_Result[1:0]
	assign shifter_op=({2{R_shift}}&funct[1:0])	|
			({2{I_mem_r}}&2'b10)				|
			({2{I_mem_w}}&((opcode[1:0]==2'b10&~opcode[2])?2'b10:2'b00));
	shifter shifter(
		.A(shifter_A),
		.B(shifter_B),
		.Shiftop(shifter_op),
		.Result(shift_result)
	);

	//next PC
	wire [31:0] next_PC_EX;
	assign next_PC_EX=(({32{~ERET&Branch}}&reg_PCpShift)					|
					({32{~ERET&J}}&({reg_PCp4[31:28],target_addr,2'b00}))	|
					({32{~ERET&R_jmp}}&reg_rdata1)						|
					({32{~ERET&(~(Branch|J|R_jmp))}}&reg_PCp4))				|
					({32{ERET}}&EPC);
	assign next_PC = 	({32{state_EX}}&next_PC_EX)		|
						({32{state_IW&Inst_Valid}}&Result);
//MEM SECTION
	//mem access
	assign Address={reg_Result[31:2],2'b00};
	assign MemRead=state_LD;
	assign MemWrite=state_ST;
	assign Write_data=I_mem_w?shift_result:reg_rdata2;
	assign {strb_swlr,strb_else}=({12{reg_Result[1:0]==2'b00}}&{8'b00011111,{{2{opcode[1]}},opcode[0],1'b1}})	|
								({12{reg_Result[1:0]==2'b01}}&{8'b00111110,{opcode[1],opcode[0],2'b10}})		|
								({12{reg_Result[1:0]==2'b10}}&{8'b01111100,{opcode[0],3'b100}})					|
								({12{reg_Result[1:0]==2'b11}}&12'b111110001000);								//generate strb_else for S instructions and strb_swlr for SWL and SWR instructions
	assign Write_strb=(({4{!(opcode[1:0]==2'b10)}})&strb_else)								|
						({4{opcode[1:0]==2'b10}}&(~opcode[2]?strb_swlr[7:4]:strb_swlr[3:0]));

//WB SECTION
	//read data process
	assign mask={{16{opcode[1]}},{8{opcode[0]}},8'b11111111};
	assign masked_data=mask&shift_result;
	assign MemRead_data=({32{opcode[1:0]==2'b11}}&masked_data)									|
			({32{opcode[1:0]==2'b01}}&{{16{(~opcode[2])&masked_data[15]}},masked_data[15:0]})	|
			({32{opcode[1:0]==2'b00}}&{{24{(~opcode[2])&masked_data[7]}},masked_data[7:0]});	//choose the masked data of LB,LW,LH
	
	//lwl and lwr
	assign lwl_00 = {reg_Read_data[7:0],reg_rdata2[23:0]};
	assign lwl_01 = {reg_Read_data[15:0],reg_rdata2[15:0]};
	assign lwl_10 = {reg_Read_data[23:0],reg_rdata2[7:0]};
	assign lwl_11 = reg_Read_data;
	assign lwr_00 = reg_Read_data;
	assign lwr_01 = {reg_rdata2[31:24],reg_Read_data[31:8]};
	assign lwr_10 = {reg_rdata2[31:16],reg_Read_data[31:16]};
	assign lwr_11 = {reg_rdata2[31:8],reg_Read_data[31:24]};
	assign lwlr_signal=({32{opcode[2]}}&(({32{reg_Result[1:0]==2'b00}}&lwr_00)	|
						({32{reg_Result[1:0]==2'b01}}&lwr_01)					|
                        ({32{reg_Result[1:0]==2'b10}}&lwr_10)					|
						({32{reg_Result[1:0]==2'b11}}&lwr_11)))					|
					   ({32{~opcode[2]}}&(({32{reg_Result[1:0]==2'b00}}&lwl_00)	|
						({32{reg_Result[1:0]==2'b01}}&lwl_01)					|
						({32{reg_Result[1:0]==2'b10}}&lwl_10)					|
						({32{reg_Result[1:0]==2'b11}}&lwl_11)));
	
	//wdata
	assign RF_wdata=({32{R_acc|(I_acc&(opcode[2:0]!=3'b111))|(J&opcode[0])|(R_jmp&funct[0])}}&reg_Result)	|
			({32{I_acc&(opcode[2:0]==3'b111)}}&{imm_and_offset,16'b0})										|
			({32{R_shift}}&shift_result)																	|
			({32{R_mov}}&reg_rdata1)																		|
			({32{I_mem_r&(!(opcode[1:0]==2'b10))}}&(MemRead_data))											|
			(({32{I_mem_r&(opcode[1:0]==2'b10)}}&lwlr_signal));


	//performance counters
	reg [31:0]counter_clks;//count clks
	assign cpu_perf_cnt_0=counter_clks;
	always @(posedge clk) begin
		if(rst)
			counter_clks<=32'b0;
		else
			counter_clks<=counter_clks+32'b1;
	end

	reg [31:0]counter_Instructions;//count instructions
	assign cpu_perf_cnt_1=counter_Instructions;
	always @(posedge clk) begin
		if(rst)
			counter_Instructions<=32'b0;
		else if(state_EX)
			counter_Instructions<=counter_Instructions+32'b1;
	end

	reg [31:0]counter_Inst_operations;//count Instructions of operations
	assign cpu_perf_cnt_2=counter_Inst_operations;
	always @(posedge clk) begin
		if(rst)
			counter_Inst_operations<=32'b0;
		else if(state_EX&&(R_acc||I_acc))
			counter_Inst_operations<=counter_Inst_operations+32'b1;
	end

	reg [31:0]counter_Inst_stores;//count Instructions of stores
	assign cpu_perf_cnt_3=counter_Inst_stores;
	always @(posedge clk) begin
		if(rst)
			counter_Inst_stores<=32'b0;
		else if(state_ST&&Mem_Req_Ready)
			counter_Inst_stores<=counter_Inst_stores+32'b1;
	end

	reg [31:0]counter_Inst_loads;//count Instructions of loads
	assign cpu_perf_cnt_4=counter_Inst_loads;
	always @(posedge clk) begin
		if(rst)
			counter_Inst_loads<=32'b0;
		else if(state_RDW&&Read_data_Valid)
			counter_Inst_loads<=counter_Inst_loads+32'b1;
	end


endmodule
