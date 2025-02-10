`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module reg_file(
	input                       clk,
	input  [`ADDR_WIDTH - 1:0]  waddr,
	input  [`ADDR_WIDTH - 1:0]  raddr1,
	input  [`ADDR_WIDTH - 1:0]  raddr2,
	input                       wen,
	input  [`DATA_WIDTH - 1:0]  wdata,
	output [`DATA_WIDTH - 1:0]  rdata1,
	output [`DATA_WIDTH - 1:0]  rdata2
);

	// TODO: Please add your logic design here、
	reg [31:0] regs [`DATA_WIDTH-1:0];
	assign rdata1=regs[raddr1]&{`DATA_WIDTH{raddr1!=0}};
	assign rdata2=regs[raddr2]&{`DATA_WIDTH{raddr2!=0}};

	always @(posedge clk) begin
		if(wen&waddr!=0)begin
			regs[waddr]<=wdata;
		end
	end

	
endmodule
