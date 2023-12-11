`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module shifter (
	input  [31:0] A,
	input  [ 4:0] B,
	input  [ 1:0] Shiftop,
	output [31:0] Result
);

	// 0x: left
	// 10: right logical shift
	// 11: right arithmetic shift
	wire [31:0] signExtend;

	assign signExtend = {{31{A[31] & Shiftop[0]}}, 1'b0} << (~B[4:0]);

	assign Result = {32{~Shiftop[1]}} &  (A[31:0] <<  B[4:0])
		      | {32{ Shiftop[1]}} & ((A[31:0] >>  B[4:0]) | signExtend[31:0]);
	
endmodule
