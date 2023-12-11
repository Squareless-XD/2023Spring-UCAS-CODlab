`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module shifter (
	input  [`DATA_WIDTH - 1:0] A,
	input  [              4:0] B,
	input  [              1:0] Shiftop,
	output [`DATA_WIDTH - 1:0] Result
);

	// 0x: left
	// 10: right logical shift
	// 11: right arithmetic shift
	wire [`DATA_WIDTH - 1:0] signExtend;

	assign signExtend[`DATA_WIDTH - 1:0] = {{`DATA_WIDTH - 1{A[`DATA_WIDTH - 1] & Shiftop[0]}}, 1'b0} << (~B[4:0]);

	assign Result = {`DATA_WIDTH{~Shiftop[1]}} &  (A[`DATA_WIDTH - 1:0] <<  B[4:0])
		      | {`DATA_WIDTH{ Shiftop[1]}} & ((A[`DATA_WIDTH - 1:0] >>  B[4:0]) | signExtend[`DATA_WIDTH - 1:0]);
	
endmodule
