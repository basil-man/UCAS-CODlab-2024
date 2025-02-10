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

`define ALUopMUX_ADD 3'b000
`define ALUopMUX_SUB 3'b001
`define ALUopMUX_Racc 3'b010
`define ALUopMUX_SLT 3'b011
`define ALUopMUX_Iacc 3'b100

`define OPCODE_R 7'b0110011
`define OPCODE_I_LOAD 7'b0000011
`define OPCODE_I_OP 7'b0010011
`define OPCODE_S 7'b0100011
`define OPCODE_U 5'b10111
`define OPCODE_B 7'b1100011
`define OPCODE_J 6'b110111
`define WADDR_31 7'b0010011

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
	
	//wires
	wire [31:0] rdata1,rdata2;
	wire [31:0] next_PC;
	wire Branch;
	wire [6:0] opcode;
	wire [4:0] rs1,rs2_shamt,rt,rd;
	wire [2:0] ALUin;
	wire [4:0]raddr1,raddr2;
	wire [31:0] shift_result;
	wire [31:0] A,B,Result,A_EX_WB,B_EX_WB;
	wire [31:0] MemRead_data;
	wire [31:0] mask,masked_data,imm;
	wire Zero;
	wire [31:0] shifter_A;
	wire [4:0] shifter_B;
	wire [1:0] shifter_op;
	wire [2:0] funct3;
	wire [6:0] funct7;
	wire B_type,R_type,I_Load,I_Op_alu,I_Op_shift,JAL_JALR_type,U_type,S_type;
	wire [2:0] ALUop_R_I_type,ALUop_B_type;
	wire [31:0] PCpShift;
//BUS regs
	reg [31:0] IW_PC;
	reg IW_NOP;// =1 when Branch: insert a bubble in pipe
	wire [31:0] ID_PC=IW_PC;
	reg [31:0] ID_Instruction;
	reg EX_U_type       ;
	reg EX_R_type       ;
	reg EX_JAL_JALR_type;
	reg EX_I_Load       ;
	reg [31:0]EX_imm          ;
	reg EX_I_Op_shift   ;
	reg EX_I_Op_alu     ;
	reg [4:0]EX_rd           ;
	reg [2:0]EX_funct3       ;
	reg EX_S_type       ;
	reg [31:0]EX_rdata2       ;
	reg [31:0]EX_PC           ;
	reg [6:0]EX_opcode       ;
	reg [31:0]EX_rdata1       ;
	reg [4:0]EX_rs2_shamt    ;
	reg [4:0]EX_rs1          ;
	reg [6:0]EX_funct7       ;
	reg EX_B_type       ;	

	reg stall_cnt;
	reg [31:0] 	MEM_Result;
	reg			MEM_S_type;
	reg [31:0] 	MEM_rdata2;
	reg [2:0] 	MEM_funct3;
	reg [31:0] 	MEM_PC;
	reg [4:0] 	MEM_rd;
	reg 		MEM_wen;
	reg [31:0] 	MEM_EX_wdata;
	reg 		MEM_I_Load;
	reg [31:0] 	RDW_PC;
	reg [4:0] 	RDW_rd;
	reg 		RDW_wen;
	reg [31:0] 	RDW_EX_wdata;
	reg 		RDW_I_Load;
	reg	[2:0]	RDW_funct3;
	reg [31:0]  RDW_Result;
    reg [31:0] 	WB_PC;
	reg [4:0] 	WB_rd;
	reg 		WB_wen;
	reg [31:0]	WB_wdata;



//FSM regs
	reg [8:0] current_state;
	reg [8:0] next_state;
	wire pipe_go=IF_readygo&IW_readygo&MEM_readygo&RDW_readygo;

	
/*pipe_IF*/
	reg [31:0] reg_PC;
	wire [31:0]PCp4 = PC+4;
	reg IF_readygo;
	wire [31:0] Branch_target;

	assign Inst_Req_Valid=(~rst)&(~IF_readygo);
	//wait till Inst_Req_Ready is 1
	always @(posedge clk)begin
		if(rst||(pipe_go&&(!stall_pipe||(stall_pipe&&stall_cnt))))begin
			IF_readygo<=0;
		end
		else if((Inst_Req_Ready&&Inst_Req_Valid)||(stall_pipe&&pipe_go&&!stall_cnt))begin
			IF_readygo<=1;
		end	
	end
	reg delay_cnt;//make sure PC won't update for 2 pipe cycles when Branch is valid
	always @(posedge clk)begin
		if(rst)
			delay_cnt<=0;
		else if(pipe_go&&stall_pipe&&!stall_cnt)
			delay_cnt<=delay_cnt;
		else if(pipe_go&&Branch)
			delay_cnt<=1;
		else if(pipe_go&&!Branch)
			delay_cnt<=0;
	end	

	always @(posedge clk) begin 
		if(rst)
			reg_PC<=0;
		else if(pipe_go&&(delay_cnt||(stall_pipe&&!stall_cnt)))
			reg_PC<=reg_PC;
		else if(pipe_go&&~Branch)
			reg_PC<=PCp4;
		else if(pipe_go&&Branch)
			reg_PC<=Branch_target;
	end

	assign PC=reg_PC;

	assign Branch_target=({32{EX_JAL_JALR_type}}&Result)|
						({32{EX_B_type}}&PCpShift);

	assign Branch = 	  (
							(
								((EX_funct3[2:1]==2'b00)&(EX_funct3[0]^Zero))|  //beq bne
								((EX_funct3[2]&(~EX_funct3[0]))&Result[0])|         //blt bltu
								((EX_funct3[2]&EX_funct3[0])&(~Result[0]))   //bge bgeu
						  	)&EX_B_type
						  )
						  |
						  EX_JAL_JALR_type;
	

	


/*pipe_IW*/
	reg IW_readygo;

	assign Inst_Ready=~IW_readygo|rst;
	//wait for insturction
	always@(posedge clk)begin
		if((pipe_go&&(!stall_pipe||(stall_pipe&&stall_cnt)))||rst)begin
			IW_readygo<=0;
		end
		else if((Inst_Valid&&Inst_Ready)||(pipe_go&&stall_pipe&&!stall_cnt))begin
			IW_readygo<=1;
		end
	end


	reg [31:0]reg_Instruction;
	always @(posedge clk) begin
		if(rst)
			reg_Instruction<=0;
		else if(pipe_go&&stall_pipe&&!stall_cnt)
			reg_Instruction<=reg_Instruction;
		else if(Inst_Valid&&Inst_Ready)
			reg_Instruction<=Instruction;
	end

/*pipe_ID*/
	reg ID_readygo;
	//take 1 clk to finish
	always@(posedge clk)begin
		if(pipe_go||rst)begin
			ID_readygo<=0;
		end
		else begin
			ID_readygo<=1;
		end
	end


	
	wire [31:0] ID_from_Bus_Instruction=ID_Instruction;
	wire [31:0] ID_from_Bus_PC=ID_PC;
	//analysis the instruction
	assign opcode=ID_from_Bus_Instruction[6:0];
	assign funct3=ID_from_Bus_Instruction[14:12];
	
	//types
	assign R_type   	= opcode == `OPCODE_R;
	assign I_Op_alu 	= (opcode == `OPCODE_I_OP)&(funct3[1:0]!=2'b01);
	assign I_Op_shift 	= (opcode == `OPCODE_I_OP)&(funct3[1:0]==2'b01);
	assign I_Load 		= opcode == `OPCODE_I_LOAD;
	assign S_type		= opcode==`OPCODE_S;
	assign U_type		= opcode[4:0]==`OPCODE_U;
	assign B_type		= opcode==`OPCODE_B;
	assign JAL_JALR_type= {opcode[6:4],opcode[2:0]}==`OPCODE_J;

	assign funct7    = ID_from_Bus_Instruction[31:25];
	assign rs1       = ID_from_Bus_Instruction[19:15]&{5{~(U_type|(JAL_JALR_type&opcode[3]))}};
	assign rs2_shamt = ID_from_Bus_Instruction[24:20]&{5{(B_type|S_type|R_type|I_Op_shift)}};//avoid wrong hazard control
	assign rd        = ID_from_Bus_Instruction[11:7];
	
	assign imm = 	({32{I_Op_alu|I_Op_shift|I_Load}} 	& {{20{ID_from_Bus_Instruction[31]}},ID_from_Bus_Instruction[31:20]})|
					({32{S_type}}					   	& {{20{ID_from_Bus_Instruction[31]}},ID_from_Bus_Instruction[31:25],ID_from_Bus_Instruction[11:7]})|
					({32{B_type}}						& {{19{ID_from_Bus_Instruction[31]}},ID_from_Bus_Instruction[31],ID_from_Bus_Instruction[7],ID_from_Bus_Instruction[30:25],ID_from_Bus_Instruction[11:8],1'b0})|
					({32{JAL_JALR_type&opcode[3]}}		& {{11{ID_from_Bus_Instruction[31]}},ID_from_Bus_Instruction[31],ID_from_Bus_Instruction[19:12],ID_from_Bus_Instruction[20],ID_from_Bus_Instruction[30:21],1'b0})|//JAL
					({32{JAL_JALR_type&(~opcode[3])}}	& {{20{ID_from_Bus_Instruction[31]}},ID_from_Bus_Instruction[31:20]})|//JALR
					({32{U_type}}						& {ID_from_Bus_Instruction[31:12],12'b0});

	//Branch
	//assign Branch = 	(funct3[2:1]==2'b00)    & (funct3[0]^Zero)|  //beq bne
	//					(funct3[2]&(~funct3[0]))& Result[0]|         //blt bltu
	//					(funct3[2]&funct3[0])	& (~Result[0]);   //bge bgeu

	//reg_file write
	//assign RF_wen = (U_type | R_type | I_Op_shift | I_Op_alu | JAL_JALR_type | I_Load) & state_WB;
	//assign RF_waddr = rd;

	//reg_file
	reg_file reg_file(
		.clk(clk),
		.waddr (RF_waddr),
		.raddr1 (rs1),
		.raddr2 (rs2_shamt),
		.wen(RF_wen),
		.wdata (RF_wdata),
		.rdata1 (rdata1),
		.rdata2 (rdata2)
	);

/*pipe_EX*/
	reg EX_readygo;
	always@(posedge clk)begin
		if(pipe_go||rst)begin
			EX_readygo<=0;
		end
		else begin
			EX_readygo<=1;
		end
	end
  	//ALUop
	assign ALUop_R_I_type = {
		(EX_funct3[2]&(~EX_funct3[1])&(~EX_funct3[0]))|((~EX_funct3[2])&EX_funct3[1]&(~EX_funct3[0]))|((~EX_funct3[1])&(~EX_funct3[0])&(EX_I_Op_alu?1'b0:EX_funct7[5])),
		((~EX_funct3[2])&EX_funct3[1])|((~EX_funct3[2])&(~EX_funct3[1])),
		(EX_funct3[1]&(~EX_funct3[0]))|((~EX_funct3[2])&EX_funct3[1])
	};
	assign ALUop_B_type = 	{~EX_funct3[1],1'b1,EX_funct3[2]};
	assign ALUin        = 	{3{EX_R_type|EX_I_Op_alu}}					&	ALUop_R_I_type	|
							{3{EX_U_type|EX_JAL_JALR_type|EX_I_Load|EX_S_type}} & `ALUop_ADD		|
							{3{EX_B_type}} 							& ALUop_B_type;

	

	//ALU
	assign A = ({32{EX_U_type|(EX_JAL_JALR_type&EX_opcode[3])}}&EX_PC)	|
					({32{~(EX_U_type|(EX_JAL_JALR_type&EX_opcode[3]))}}&data_hazard_A);
	assign B = (EX_R_type|EX_B_type)?data_hazard_B:EX_imm;
	wire [1:0] data_risk;
	reg [31:0] temp_risk_data;
	always @(posedge clk) begin
		if(rst)
			temp_risk_data<=0;
		else if(data_risk!=2'b0)begin
			temp_risk_data<=Result;
		end
	end

	/*
		00:common
		01:from MEM
		10:from RDW
		11:from WB
	*/
	reg [1:0] Forward_A;
	reg [1:0] Forward_B;
	wire [31:0] data_hazard_A =	{32{Forward_A==2'b00}}&EX_rdata1|
								{32{Forward_A==2'b01}}&MEM_EX_wdata|
								{32{Forward_A==2'b10}}&final_wdata|
								{32{Forward_A==2'b11}}&RF_wdata;
	wire [31:0] data_hazard_B =	{32{Forward_B==2'b00}}&EX_rdata2|
								{32{Forward_B==2'b01}}&MEM_EX_wdata|
								{32{Forward_B==2'b10}}&final_wdata|
								{32{Forward_B==2'b11}}&RF_wdata;

	alu alu(
		.A(A),
		.B(B),
		.ALUop(ALUin),
		.Overflow(),
		.CarryOut(),
		.Zero(Zero),
		.Result(Result)
	);
	// always @(posedge clk)begin
	// 	if(state_IW)
	// 		reg_PCp4<=Result;
	// end
	// always @(posedge clk)begin
	// 	if(state_ID)
	// 		reg_PCpShift<=Result;
	// end
	// always @(posedge clk)begin
	// 	if(state_EX)
	// 		reg_Result<=Result;
	// end


	assign PCpShift=EX_PC+EX_imm;

	//shift_result
	assign shifter_A=data_hazard_shifter_A;
	assign shifter_B=({5{(EX_R_type&(EX_funct3[1:0]==2'b01))}}&data_hazard_shifter_B)|
			({5{EX_I_Op_shift}}&EX_rs2_shamt);
	assign shifter_op=({2{(EX_R_type&(EX_funct3[1:0]==2'b01))|EX_I_Op_shift}}&{EX_funct3[2],EX_funct7[5]})|
			({2{EX_I_Load}}&2'b10)|
			({2{EX_S_type}}&2'b00);
	wire [31:0] data_hazard_shifter_A =	{32{Forward_A==2'b00}}&EX_rdata1|
										{32{Forward_A==2'b01}}&MEM_EX_wdata|
										{32{Forward_A==2'b10}}&final_wdata|
										{32{Forward_A==2'b11}}&RF_wdata;
	wire [4:0] data_hazard_shifter_B =	{5{Forward_B==2'b00}}&EX_rdata2[4:0]|
										{5{Forward_B==2'b01}}&MEM_EX_wdata[4:0]|
										{5{Forward_B==2'b10}}&final_wdata[4:0]|
										{5{Forward_B==2'b11}}&RF_wdata[4:0];
	shifter shifter(
		.A(shifter_A),
		.B(shifter_B),
		.Shiftop(shifter_op),
		.Result(shift_result)
	);
	//EX_wdata
	wire [31:0] EX_wdata;
	assign EX_wdata	=	({32{EX_U_type}}											&EX_imm)|
						({32{(EX_R_type&(EX_funct3[1:0]==2'b01))|EX_I_Op_shift}}	&shift_result)|
						({32{(EX_R_type&(EX_funct3[1:0]!=2'b01))|EX_I_Op_alu}}		&Result)|
						({32{EX_JAL_JALR_type}}										&(EX_PC+4));
	reg wen_hazard_cnt;//make wen=0 when pipeline is stall
	always @(posedge clk) begin
		if(rst)
			wen_hazard_cnt<=0;
		else if(stall_pipe&&pipe_go&&!stall_cnt)
			wen_hazard_cnt<=1;
		else if((!stall_pipe||(stall_pipe&&stall_cnt))&&pipe_go)
			wen_hazard_cnt<=0;
	end
	wire EX_wen;
	assign EX_wen = (EX_U_type | EX_R_type | EX_I_Op_shift | EX_I_Op_alu | EX_JAL_JALR_type | EX_I_Load)&~wen_hazard_cnt;




/*pipe_RDW*/
	assign Read_data_Ready=(RDW_I_Load&~RDW_readygo)|rst;
	reg RDW_readygo;
	always@(posedge clk)begin
		if(pipe_go||rst)begin
			RDW_readygo<=0;
		end
		else if(!RDW_I_Load)begin
			RDW_readygo<=1;
		end
		else if(Read_data_Valid)begin
			RDW_readygo<=1;
		end
	end

	assign mask={{16{RDW_funct3[1]}},{8{RDW_funct3[1]|RDW_funct3[0]}},8'b11111111};
	assign masked_data=mask&Read_data>>{RDW_Result[1:0],3'b000};
	assign MemRead_data=({32{RDW_funct3[1]}}&masked_data)| //LW
			({32{(~RDW_funct3[1])&RDW_funct3[0]}}&{{16{(~RDW_funct3[2])&masked_data[15]}},masked_data[15:0]})| //LH
			({32{(~RDW_funct3[1])&(~RDW_funct3[0])}}&{{24{(~RDW_funct3[2])&masked_data[7]}},masked_data[7:0]});//LB  //choose the masked data of LB,LW,LH

	wire [31:0] final_wdata;
	assign final_wdata=RDW_I_Load?reg_MemRead_data:RDW_EX_wdata;
	reg [31:0] reg_MemRead_data;
	always @(posedge clk) begin
		if(Read_data_Ready && Read_data_Valid)
			reg_MemRead_data<=MemRead_data;
	end
	

/*pipe_MEM*/
	reg MEM_readygo;
	always@(posedge clk)begin
		if(pipe_go||rst)begin
			MEM_readygo<=0;
		end
		else if((!(MEM_I_Load||MEM_S_type)))begin
			MEM_readygo<=1;
		end
		else if(Mem_Req_Ready)begin
			MEM_readygo<=1;
		end
	end



	assign Address={MEM_Result[31:2],2'b00};
	assign MemRead=MEM_I_Load&~MEM_readygo;
	assign MemWrite=MEM_S_type&~MEM_readygo;
	assign Write_data=MEM_S_type?MEM_rdata2<<{MEM_Result[1:0],3'b000}:MEM_rdata2;
	assign Write_strb=({4{MEM_Result[1:0]==2'b00}}&{{2{MEM_funct3[1]}},MEM_funct3[0]|MEM_funct3[1],1'b1})|
			({4{MEM_Result[1:0]==2'b01}}&{MEM_funct3[1],MEM_funct3[0]|MEM_funct3[1],2'b10})|
			({4{MEM_Result[1:0]==2'b10}}&{MEM_funct3[0]|MEM_funct3[1],3'b100})|
			({4{MEM_Result[1:0]==2'b11}}&4'b1000);



	
/*pipe_WB*/
    reg WB_readygo;
	always@(posedge clk)begin
		if(pipe_go||rst)begin
			WB_readygo<=0;
		end
		else begin
			WB_readygo<=1;
		end
	end

	//wdata
	assign RF_wdata	= WB_wdata;
	assign RF_wen = WB_wen;
	assign RF_waddr = WB_rd;

	//inst_retire
	assign inst_retire[69]=RF_wen&~WB_readygo;
	assign inst_retire[68:64]=RF_waddr;
	assign inst_retire[63:32]=RF_wdata;
	assign inst_retire[31:0]=WB_PC;




	//data hazard control unit
	always@(*)begin
		if(MEM_wen&&(MEM_rd!=0)&&MEM_rd==EX_rs1)
			Forward_A = 2'b01;
		else if(RDW_wen&&(RDW_rd!=0)&&RDW_rd==EX_rs1)
			Forward_A = 2'b10;
		else if(RF_wen&&(RF_waddr!=0)&&RF_waddr==EX_rs1)
			Forward_A = 2'b11;
		else 
			Forward_A = 2'b00;
	end
	always@(*)begin
		if(MEM_wen&&(MEM_rd!=0)&&MEM_rd==EX_rs2_shamt)
			Forward_B = 2'b01;
		else if(RDW_wen&&(RDW_rd!=0)&&RDW_rd==EX_rs2_shamt)
			Forward_B = 2'b10;
		else if(RF_wen&&(RF_waddr!=0)&&RF_waddr==EX_rs2_shamt)
			Forward_B = 2'b11;
		else 
			Forward_B = 2'b00;
	end
	reg stall_pipe;
	always@(*)begin
		if(rst)
			stall_pipe=0;
		if(EX_I_Load&&(EX_rd==rs1||EX_rd==rs2_shamt))
			stall_pipe=1;//stall the pipeline
		else stall_pipe=0;
	end






//BUS Timing Logic
/*IFtoIW*/
always @(posedge clk) begin
    if (rst) begin
        IW_PC <= 0;
    end
    else if (pipe_go && stall_pipe && !stall_cnt) begin
        IW_PC <= IW_PC;
    end
    else if (pipe_go && !Branch) begin
        IW_PC <= PC;
    end
    else if (pipe_go && Branch) begin
        IW_PC <= 0;
    end
end

always @(posedge clk) begin
    if (rst) begin
        IW_NOP <= 0;
    end
    else if (pipe_go && stall_pipe && !stall_cnt) begin
        IW_NOP <= IW_NOP;
    end
    else if (pipe_go && !Branch) begin
        IW_NOP <= 0;
    end
    else if (pipe_go && Branch) begin
        IW_NOP <= 1;
    end
end

/*IWtoID*/
	always @(posedge clk) begin
		if(rst||(pipe_go&&((!Branch&&IW_NOP)||Branch)))begin
			ID_Instruction<=0;
		end
		else if(pipe_go&&stall_pipe&&!stall_cnt)
			ID_Instruction<=ID_Instruction;
		else if(pipe_go&&!Branch&&!IW_NOP)begin
			ID_Instruction <=reg_Instruction;
		end
	end
	
/*IDtoEX*/
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			stall_cnt <= 1;
		end
		else if (rst || (pipe_go && Branch)) begin
			stall_cnt <= 0;
		end
		else if (pipe_go && !Branch) begin
			stall_cnt <= 0;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_U_type <= EX_U_type;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_U_type <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_U_type <= U_type;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_R_type <= EX_R_type;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_R_type <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_R_type <= R_type;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_JAL_JALR_type <= EX_JAL_JALR_type;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_JAL_JALR_type <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_JAL_JALR_type <= JAL_JALR_type;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_I_Load <= EX_I_Load;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_I_Load <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_I_Load <= I_Load;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_imm <= EX_imm;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_imm <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_imm <= imm;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_I_Op_shift <= EX_I_Op_shift;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_I_Op_shift <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_I_Op_shift <= I_Op_shift;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_I_Op_alu <= EX_I_Op_alu;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_I_Op_alu <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_I_Op_alu <= I_Op_alu;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_rd <= EX_rd;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_rd <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_rd <= rd;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_funct3 <= EX_funct3;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_funct3 <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_funct3 <= funct3;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_S_type <= EX_S_type;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_S_type <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_S_type <= S_type;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_rdata2 <= EX_rdata2;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_rdata2 <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_rdata2 <= rdata2;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_PC <= EX_PC;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_PC <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_PC <= ID_from_Bus_PC;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_opcode <= EX_opcode;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_opcode <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_opcode <= opcode;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_rdata1 <= EX_rdata1;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_rdata1 <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_rdata1 <= rdata1;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_rs2_shamt <= EX_rs2_shamt;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_rs2_shamt <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_rs2_shamt <= rs2_shamt;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_rs1 <= EX_rs1;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_rs1 <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_rs1 <= rs1;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_funct7 <= EX_funct7;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_funct7 <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_funct7 <= funct7;
		end
	end
	always @(posedge clk) begin
		if (stall_pipe && pipe_go && !stall_cnt) begin
			EX_B_type <= EX_B_type;
		end
		else if (rst || (pipe_go && Branch)) begin
			EX_B_type <= 0;
		end
		else if (pipe_go && !Branch) begin
			EX_B_type <= B_type;
		end
	end

/*EXtoMEM*/
	always@(posedge clk)begin
		if(rst)begin
			MEM_Result	<=0;
		end
		else if(pipe_go)begin
			MEM_Result	<=Result;
		end
	end
	always@(posedge clk)begin
		if(rst)begin
			MEM_Result	<=0;
		end
		else if(pipe_go)begin
			MEM_Result	<=Result;
		end
	end
	always@(posedge clk)begin
		if(rst)begin
			MEM_S_type	<=0;
		end
		else if(pipe_go)begin
			MEM_S_type	<=EX_S_type;
		end
	end
	always@(posedge clk)begin
		if(rst)begin
			MEM_rdata2	<=0;
		end
		else if(pipe_go)begin
			MEM_rdata2	<=data_hazard_B;
		end
	end
	always@(posedge clk)begin
		if(rst)begin
			MEM_funct3	<=0;
		end
		else if(pipe_go)begin
			MEM_funct3	<=EX_funct3;
		end
	end
	always@(posedge clk)begin
		if(rst)begin
			MEM_PC		<=0;
		end
		else if(pipe_go)begin
			MEM_PC		<=EX_PC;
		end
	end
	always@(posedge clk)begin
		if(rst)begin
			MEM_rd		<=0;
		end
		else if(pipe_go)begin
			MEM_rd		<=EX_rd;
		end
	end
	always@(posedge clk)begin
		if(rst)begin
			MEM_wen		<=0;
		end
		else if(pipe_go)begin
			MEM_wen		<=EX_wen;
		end
	end
	always@(posedge clk)begin
		if(rst)begin
			MEM_EX_wdata<=0;
		end
		else if(pipe_go)begin
			MEM_EX_wdata<=EX_wdata;
		end
	end
	always@(posedge clk)begin
		if(rst)begin
			MEM_I_Load	<=0;
		end
		else if(pipe_go)begin
			MEM_I_Load	<=EX_I_Load;
		end
	end
/*MEMtoRDW*/
	always @(posedge clk) begin
		if (rst) begin
			RDW_PC <= 0;
		end
		else if (pipe_go) begin
			RDW_PC <= MEM_PC;
		end
	end
	always @(posedge clk) begin
		if (rst) begin
			RDW_rd <= 0;
		end
		else if (pipe_go) begin
			RDW_rd <= MEM_rd;
		end
	end
	always @(posedge clk) begin
		if (rst) begin
			RDW_wen <= 0;
		end
		else if (pipe_go) begin
			RDW_wen <= MEM_wen;
		end
	end
	always @(posedge clk) begin
		if (rst) begin
			RDW_EX_wdata <= 0;
		end
		else if (pipe_go) begin
			RDW_EX_wdata <= MEM_EX_wdata;
		end
	end
	always @(posedge clk) begin
		if (rst) begin
			RDW_I_Load <= 0;
		end
		else if (pipe_go) begin
			RDW_I_Load <= MEM_I_Load;
		end
	end
	always @(posedge clk) begin
		if (rst) begin
			RDW_funct3 <= 0;
		end
		else if (pipe_go) begin
			RDW_funct3 <= MEM_funct3;
		end
	end
	always @(posedge clk) begin
		if (rst) begin
			RDW_Result <= 0;
		end
		else if (pipe_go) begin
			RDW_Result <= MEM_Result;
		end
	end
/*RDWtoWB*/
	always @(posedge clk) begin
		if (rst) begin
			WB_PC <= 0;
		end
		else if (pipe_go) begin
			WB_PC <= RDW_PC;
		end
	end
	always @(posedge clk) begin
		if (rst) begin
			WB_rd <= 0;
		end
		else if (pipe_go) begin
			WB_rd <= RDW_rd;
		end
	end
	always @(posedge clk) begin
		if (rst) begin
			WB_wen <= 0;
		end
		else if (pipe_go) begin
			WB_wen <= RDW_wen;
		end
	end
	always @(posedge clk) begin
		if (rst) begin
			WB_wdata <= 0;
		end
		else if (pipe_go) begin
			WB_wdata <= RDW_I_Load ? reg_MemRead_data : RDW_EX_wdata;
		end
	end

//performance counters
	reg [31:0]counter_clks;//count clks
	assign cpu_perf_cnt_0=counter_clks;
	always @(posedge clk) begin
		if(rst)
			counter_clks<=0;
		else
			counter_clks<=counter_clks+1;
	end

	reg [31:0]counter_Instructions;//count instructions
	assign cpu_perf_cnt_1=counter_Instructions;
	always @(posedge clk) begin
		if(rst)
			counter_Instructions<=0;
		else if(pipe_go&&(WB_PC!=0||(WB_PC==0&&WB_wen!=0)))
			counter_Instructions<=counter_Instructions+1;
	end

	reg [31:0]counter_Inst_operations;//count Instructions of operations
	assign cpu_perf_cnt_2=counter_Inst_operations;
	always @(posedge clk) begin
		if(rst)
			counter_Inst_operations<=0;
		else if(pipe_go&&(I_Op_alu|(R_type&(funct3[1:0]!=2'b01))))
			counter_Inst_operations<=counter_Inst_operations+1;
	end

	reg [31:0]counter_Inst_stores;//count Instructions of stores
	assign cpu_perf_cnt_3=counter_Inst_stores;
	always @(posedge clk) begin
		if(rst)
			counter_Inst_stores<=0;
		else if(Mem_Req_Ready)
			counter_Inst_stores<=counter_Inst_stores+1;
	end

	reg [31:0]counter_Inst_loads;//count Instructions of loads
	assign cpu_perf_cnt_4=counter_Inst_loads;
	always @(posedge clk) begin
		if(rst)
			counter_Inst_loads<=0;
		else if(Read_data_Valid)
			counter_Inst_loads<=counter_Inst_loads+1;
	end


endmodule