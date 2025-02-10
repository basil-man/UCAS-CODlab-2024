`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Xu Zhang (zhangxu415@mails.ucas.ac.cn)
// 
// Create Date: 06/14/2018 11:39:09 AM
// Design Name: 
// Module Name: dma_core
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module engine_core #(
	parameter integer  DATA_WIDTH       = 32
)
(
	input    clk,
	input    rst,
	
	output [31:0]       src_base,
	output [31:0]       dest_base,
	output [31:0]       tail_ptr,
	output [31:0]       head_ptr,
	output [31:0]       dma_size,
	output [31:0]       ctrl_stat,

	input  [31:0]	    reg_wr_data,
	input  [ 5:0]       reg_wr_en,
  
	output              intr,
  
	output [31:0]       rd_req_addr,
	output [ 4:0]       rd_req_len,
	output              rd_req_valid,
	
	input               rd_req_ready,
	input  [31:0]       rd_rdata,
	input               rd_last,
	input               rd_valid,
	output              rd_ready,
	
	output [31:0]       wr_req_addr,
	output [ 4:0]       wr_req_len,
	output              wr_req_valid,
	input               wr_req_ready,
	output [31:0]       wr_data,
	output              wr_valid,
	input               wr_ready,
	output              wr_last,
	
	output              fifo_rden,
	output [31:0]       fifo_wdata,
	output              fifo_wen,
	
	input  [31:0]       fifo_rdata,
	input               fifo_is_empty,
	input               fifo_is_full
);
	// TODO: Please add your logic design here
	wire EN,INTR;
	assign EN=ctrl_stat[0];
	assign intr=ctrl_stat[31];



	localparam 	IDLE 	= 3'b001,
				RD_REQ 	= 3'b010,
				RD		= 3'b100;

	localparam 	WR_IDLE = 4'b0001,
				WR_REQ  = 4'b0010,
				WR		= 4'b0100,
				RD_FIFO = 4'b1000;
				

/*RD engine FSM*/
	wire Burst_rd_done;
	wire state_RD_IDLE,state_RD_REQ,state_RD;
	assign state_RD_IDLE=RD_current_state[0];
	assign state_RD_REQ=RD_current_state[1];
	assign state_RD=RD_current_state[2];
	//state regs
	reg [2:0] RD_current_state;
	reg [2:0] RD_next_state;

	//FSM part 1
	always @(posedge clk) begin
		if(rst)
			RD_current_state <= IDLE;
		else
			RD_current_state <= RD_next_state;
	end

	//FSM part 2
	// localparam 	IDLE 	= 3'b001,
	// 			RD_REQ 	= 3'b010,
	// 			RD		= 3'b100;
	always @(*) begin
		case (RD_current_state)
			IDLE: begin
				if(EN&&(head_ptr!=tail_ptr)&&state_WR_IDLE)
					RD_next_state = RD_REQ;
				else 
					RD_next_state = IDLE;
			end

			RD_REQ: begin
				if(fifo_is_full)
					RD_next_state = IDLE;
				else if(rd_req_ready & rd_req_valid)
					RD_next_state = RD;
				else
					RD_next_state = RD_REQ;
			end
			
			RD: begin
				if(rd_valid&&rd_last&&rd_ready)
					RD_next_state = RD_REQ;
				else
					RD_next_state = RD;
			end

			
			default: begin
				RD_next_state = IDLE;
			end
		endcase
	end

/*WR engine FSM*/
	wire Burst_wr_done;
	wire state_WR_IDLE,state_WR_REQ,state_WR;
	assign state_WR_IDLE=WR_current_state[0];
	assign state_WR_REQ=WR_current_state[1];
	assign state_WR=WR_current_state[2];
	
	//state regs
	reg [3:0] WR_current_state;
	reg [3:0] WR_next_state;
	
	//FSM part 1
	always @(posedge clk) begin
		if(rst)
			WR_current_state<= WR_IDLE;
		else
			WR_current_state<= WR_next_state;
	end
	// localparam 	WR_IDLE = 4'b0001,
	// 			WR_REQ  = 4'b0010,
	// 			WR		= 4'b0100,
	// 			RD_FIFO = 4'b1000;
	//FSM part 2
	always @(*) begin
		case (WR_current_state)
			WR_IDLE: begin
				if(EN&&state_RD_IDLE&&(head_ptr!=tail_ptr))
					WR_next_state = WR_REQ;
				else 
					WR_next_state = WR_IDLE;
			end

			WR_REQ: begin
				if(Burst_wr_done||fifo_is_empty)
					WR_next_state = WR_IDLE;
				else if(wr_req_ready&&wr_req_valid)
					WR_next_state = RD_FIFO;
				else 
					WR_next_state = WR_REQ;
			end
			
			RD_FIFO: begin
				WR_next_state = WR;
			end

			WR: begin
				if((wr_ready&&wr_last)||fifo_is_empty)
					WR_next_state = WR_REQ;
				else if(wr_ready&&!fifo_is_empty)
					WR_next_state = RD_FIFO;
				else 
					WR_next_state = WR;
			end

			default: begin
				WR_next_state = WR_IDLE;
			end
		endcase
	end


/*logics*/
	// int(N/32) + (N% 32 != 0)
	wire [26:0] burst_num=dma_size[31:5]+{26'b0,(|dma_size[4:0])};
	//int((N%32)/4) + ( ((N % 32) % 4) != 0
	wire [4:0] last_burst_len={2'b00,dma_size[4:2]+{2'b0,|dma_size[1:0]}};

	wire Burst_done=state_RD_IDLE&state_WR_IDLE&Burst_rd_done&Burst_wr_done;

	assign Burst_rd_done=(burst_num==rd_burst_num);
	assign Burst_wr_done=(burst_num==wr_burst_num);
	//rd burst num
	reg [26:0] rd_burst_num;
	always @(posedge clk) begin
		if(rst||(EN&&(head_ptr!=tail_ptr)&&Burst_done))
			rd_burst_num<=0;
		else if(state_RD&&rd_valid&&rd_last)
			rd_burst_num<=rd_burst_num+1;
	end

	//rd engine signals
	assign rd_req_addr=src_base+tail_ptr+{rd_burst_num,5'b0};
	assign rd_req_len=Burst_rd_done?last_burst_len:5'd7;
	assign rd_req_valid=state_RD_REQ&!fifo_is_full&!Burst_rd_done;
	assign rd_ready=state_RD;

	//wr burst num
	reg [26:0] wr_burst_num;
	always @(posedge clk) begin
		if(rst||(EN&&(head_ptr!=tail_ptr)&&Burst_done))
			wr_burst_num<=0;
		else if(state_WR&&wr_ready&&wr_last)
			wr_burst_num<=wr_burst_num+1;
	end

	//wr burst last cnt
	reg [2:0] wr_burst_last_cnt;
	always @(posedge clk) begin
		if(rst)
			wr_burst_last_cnt<=0;
		else if(state_WR_REQ)
			wr_burst_last_cnt<=0;
		else if(state_WR&&wr_ready)
			wr_burst_last_cnt<=wr_burst_last_cnt+1;
		else
			wr_burst_last_cnt<=wr_burst_last_cnt;
	end

	//wr engine signal
	assign wr_req_addr=dest_base+tail_ptr+{wr_burst_num,5'b0};
	assign wr_req_len=Burst_wr_done?last_burst_len:5'd7;
	assign wr_req_valid=state_WR_REQ&!fifo_is_empty;
	//assign wr_data=wr_data_delay_cnt?fifo_rdata:reg_fifo_rdata;
	assign wr_data=reg_fifo_rdata;
	assign wr_valid=state_WR;
	assign wr_last=state_WR&(wr_burst_last_cnt==wr_req_len[2:0]);

	// reg wr_data_delay_cnt;
	// always @(posedge clk) begin
	// 	if(rst)
	// 		wr_data_delay_cnt<=0;
	// 	else if(WR_current_state[3])
	// 		wr_data_delay_cnt<=1;
	// 	else 
	// 		wr_data_delay_cnt<=0;
	// end

	//fifo signal
	assign fifo_rden=WR_next_state[3];//fifo state
	reg [31:0]reg_fifo_rdata;
	always @(posedge clk) begin
		if(rst)
			reg_fifo_rdata<=0;
		else if(WR_current_state[3])
			reg_fifo_rdata<=fifo_rdata;
	end
	assign fifo_wen=!fifo_is_full&rd_valid&rd_ready;
	assign fifo_wdata=rd_rdata;


/*control and state regs*/
	reg [31:0] reg_src_base;
	reg	[31:0] reg_dest_base;
	reg	[31:0] reg_tail_ptr;
	reg	[31:0] reg_head_ptr;
	reg	[31:0] reg_dma_size;
	reg	[31:0] reg_ctrl_stat;

	always @(posedge clk) begin
		if (rst)
				reg_src_base <= 32'b0;
		else if (reg_wr_en[0])
				reg_src_base <= reg_wr_data;
		else
				reg_src_base <= reg_src_base;
	end
	assign src_base=reg_src_base;

	always @(posedge clk) begin
		if (rst)
				reg_dest_base <= 32'b0;
		else if (reg_wr_en[1])
				reg_dest_base <= reg_wr_data;
		else
				reg_dest_base <= reg_dest_base;
	end
	assign dest_base=reg_dest_base;

	always @(posedge clk) begin
		if (rst)
				reg_tail_ptr <= 32'b0;
		else if (reg_wr_en[2])
				reg_tail_ptr <= reg_wr_data;
		else if(Burst_done)//update tail_ptr when finished
				reg_tail_ptr <= reg_tail_ptr+dma_size;
	end
	assign tail_ptr=reg_tail_ptr;

	always @(posedge clk) begin
		if (rst)
				reg_head_ptr <= 32'b0;
		else if (reg_wr_en[3])
				reg_head_ptr <= reg_wr_data;
		else
				reg_head_ptr <= reg_head_ptr;
	end
	assign head_ptr=reg_head_ptr;

	always @(posedge clk) begin
		if (rst)
				reg_dma_size <= 32'b0;
		else if (reg_wr_en[4])
				reg_dma_size <= reg_wr_data;
		else
				reg_dma_size <= reg_dma_size;
	end
	assign dma_size=reg_dma_size;

	always @(posedge clk) begin
		if (rst)
				reg_ctrl_stat <= 32'b1;
		else if (reg_wr_en[5])
				reg_ctrl_stat <= reg_wr_data;
		else if(state_WR_REQ&&Burst_wr_done)
				reg_ctrl_stat <= {1'b1,reg_ctrl_stat[30:0]};
		else
				reg_ctrl_stat <= reg_ctrl_stat;
	end
	assign ctrl_stat=reg_ctrl_stat;
endmodule

