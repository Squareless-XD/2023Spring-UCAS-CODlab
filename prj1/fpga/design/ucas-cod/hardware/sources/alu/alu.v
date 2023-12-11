`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module alu(
	input  [`DATA_WIDTH - 1:0]  A,
	input  [`DATA_WIDTH - 1:0]  B,
	input  [              2:0]  ALUop,
	output                      Overflow,
	output                      CarryOut,
	output                      Zero,
	output [`DATA_WIDTH - 1:0]  Result
);

	wire [`DATA_WIDTH - 1:0]  AdderResult;
	wire [`DATA_WIDTH - 1:0]  BinCalcul;
	wire                      CarryHighest;
	wire                      CarrySign;
	wire                      OverflowTemp;
	wire [`DATA_WIDTH - 2:0]  SumValue;
	wire                      SumSign;

	// if we need to do subtraction, inverse wire B
	assign BinCalcul = B ^ {`DATA_WIDTH{ALUop[2]}};

	// add them up in two assign statement, getting two carry out used for overflow calculation
	assign {CarryHighest, SumValue} = A[`DATA_WIDTH - 2:0] + BinCalcul[`DATA_WIDTH - 2:0] + ALUop[2]    ;
	assign {CarrySign,    SumSign } = A[`DATA_WIDTH - 1  ] + BinCalcul[`DATA_WIDTH - 1  ] + CarryHighest;

	// get the overflow
	assign OverflowTemp = CarryHighest ^ CarrySign;

	// if we need to return a compare result, then get the sign of summation (XORed by overflow)
	assign AdderResult = {`DATA_WIDTH{!ALUop[0]}} & {SumSign, SumValue}
			   | {`DATA_WIDTH{ ALUop[0]}} & (SumSign ^ OverflowTemp);

	// get the carry out and the overflow.
	// CarryOut should be the inverse when a subtraction is needed, because of the definition
	// (we do borrow when "borrow == 0", instead do carry out when "CarryOut == 1")
	assign CarryOut = CarrySign ^ ALUop[2];
	assign Overflow = OverflowTemp;

	// get the result, in different cases. no expansion area is set here; so when nedded, just change this line
	assign Result = {`DATA_WIDTH{ALUop[1]}} & AdderResult
		      | {`DATA_WIDTH{~ALUop[1]}} & {`DATA_WIDTH{~ALUop[0]}} & (A & B)
		      | {`DATA_WIDTH{~ALUop[1]}} & {`DATA_WIDTH{ALUop[0]}} & (A | B);

	// get the zero output, when all bits of the result are 0, "zero" will be set
	assign Zero = ~(|Result);

endmodule
