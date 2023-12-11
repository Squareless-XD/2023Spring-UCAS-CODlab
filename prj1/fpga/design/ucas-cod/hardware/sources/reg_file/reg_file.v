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
	
	reg [`DATA_WIDTH - 1:0] rf [`DATA_WIDTH - 1:0];

	// change the data in the register, except when waddr == 0
	always @(posedge clk) begin
		if (wen && waddr != `ADDR_WIDTH'b0) begin
			rf[waddr] <= wdata;
		end
	end
	
	// read the data out at raddr1 & raddr2. when one is 0, out put 0 at the same time
	assign rdata1 = {`DATA_WIDTH{|raddr1}} & rf[raddr1];
	assign rdata2 = {`DATA_WIDTH{|raddr2}} & rf[raddr2];

endmodule
