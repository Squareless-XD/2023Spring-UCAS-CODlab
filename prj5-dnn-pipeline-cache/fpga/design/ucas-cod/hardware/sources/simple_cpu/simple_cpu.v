`timescale 10ns / 1ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

`define R31_ADDR 5'b11111
`define R_TYPE   6'b000000
`define R_MOVE   4'b0011
`define R_JUMP   4'b0010
`define R_SHIFT  3'b000
`define I_CALCUL 3'b001
`define I_BRANCH 4'b0001
`define I_REGIMM 6'b000001
`define J_TYPE   5'b00001
`define JAL      6'b000011
`define JR       6'b001000
`define JALR     6'b001001
`define LUI      6'b001111

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
	wire        RF_wen;
	wire [4:0]  RF_waddr;
	wire [31:0] RF_wdata;

	// TODO: PLEASE ADD YOUR CODE BELOW

	// register in simple_cpu
	reg [31:0] reg_Instruction;
	reg [31:0] reg_Memory;
	reg [31:0] reg_RF_rdata1;
	reg [31:0] reg_RF_rdata2;
	reg [31:0] reg_alu_shift_out;

	// register read
	wire [ 4:0] RF_raddr1;
	wire [ 4:0] RF_raddr2;
	wire [31:0] RF_rdata1;
	wire [31:0] RF_rdata2;

	// register write data when using I-load instructions
	wire [31:0] RF_wpartial;

	// PC store and calculation
	reg  [31:0] PC_reg;         // the register storing PC
	wire [31:0] PC_jump_addr; // branch and jump address
	wire        PC_wen;       // whether refresh PC
	wire        PC_wen_cond;  // whether refresh PC if condition is met
	wire        PC_wen_next;  // whether refresh PC when in instruction fetching state. calculate PC+4
	wire [31:0] PC_renew;

	// alu control signals
	wire [31:0] alu_A;
	wire [31:0] alu_B;
	wire [ 2:0] alu_ALUop;
	wire        alu_Overflow;
	wire        alu_CarryOut;
	wire        alu_Zero;
	wire [31:0] alu_Result;

	// slice of the alu calculation result
	wire [1:0] bytes_sel;
	wire remain_0;
	wire remain_1;
	wire remain_2;
	wire remain_3;

	// shifter port
	wire [31:0] shift_A;
	wire [ 4:0] shift_B;
	wire [ 1:0] shift_Shiftop;
	wire [31:0] shift_Result;

	// split instruction into several parts
	wire [ 5:0] opcode;    //  6 bits
	wire [ 4:0] rs;        //  5 bits
	wire [ 4:0] rt;        //  5 bits
	wire [ 4:0] rd;        //  5 bits
	wire [ 4:0] shamt;     //  5 bits
	wire [ 5:0] funct;     //  6 bits
	wire [15:0] immediate; // 16 bits
	wire [25:0] target;    // 26 bits

	// extension for immediate
	wire [31:0] sign_ext_imm; // sign-extended from "immediate"
	wire [31:0] zero_ext_imm; // zero-extended from "immediate"

	assign opcode    = reg_Instruction[31:26];
	assign rs        = reg_Instruction[25:21];
	assign rt        = reg_Instruction[20:16];
	assign rd        = reg_Instruction[15:11];
	assign shamt     = reg_Instruction[10: 6];
	assign funct     = reg_Instruction[ 5: 0];
	assign immediate = reg_Instruction[15: 0];
	assign target    = reg_Instruction[25: 0];

	assign sign_ext_imm = $signed(immediate);
	assign zero_ext_imm = {16'b0, immediate};

	// instantiate a register file
	reg_file reg_file_inst(
		.clk    (clk      ),
		.waddr  (RF_waddr ),
		.raddr1 (RF_raddr1),
		.raddr2 (RF_raddr2),
		.wen    (RF_wen   ),
		.wdata  (RF_wdata ),
		.rdata1 (RF_rdata1),
		.rdata2 (RF_rdata2)
	);

	// instantiate an alu
	alu alu_inst (
		.A        (alu_A       ),
		.B        (alu_B       ),
		.ALUop    (alu_ALUop   ),
		.Overflow (alu_Overflow),
		.CarryOut (alu_CarryOut),
		.Zero     (alu_Zero    ),
		.Result   (alu_Result  )
	);

	// instantiate a shifter
	shifter shifter_inst (
		.A       (shift_A),
		.B       (shift_B),
		.Shiftop (shift_Shiftop),
		.Result  (shift_Result)
	);

	// register file write enable.
	wire [1:0] RF_wen_control;
	parameter RF_Wen_RType = 2'b01,
		  RF_Wen_True  = 2'b11;
	assign RF_wen_control = {2{opcode[5:3] == `I_CALCUL}} & RF_Wen_True  // I-calcul
			      | {2{opcode      == `R_TYPE  }} & RF_Wen_RType // R
			      | {2{opcode      == `JAL     }} & RF_Wen_True  // jal
			      | {2{opcode[5]   & ~opcode[3]}} & RF_Wen_True  // I-load
	;
	assign RF_wen = ( RF_wen_control == RF_Wen_RType &  // get the inverse signal to simplify circuit
			  ~({funct[5:3], funct[1]} == `R_MOVE & funct[0] != reg_alu_shift_out[0])
		        | RF_wen_control == RF_Wen_True )
		      & current_state == Writing_Back
	;

	// register file write address
	wire [1:0] RF_waddr_control;
	parameter RF_Waddr_rd  = 2'b01,
		  RF_Waddr_rt  = 2'b10,
		  RF_Waddr_r31 = 2'b11;
	assign RF_waddr_control = {2{opcode      == `R_TYPE  }} & RF_Waddr_rd  // R
				| {2{opcode[5:3] == `I_CALCUL}} & RF_Waddr_rt  // I-calcul
				| {2{opcode[5]   & ~opcode[3]}} & RF_Waddr_rt  // I_load
				| {2{opcode      == `JAL     }} & RF_Waddr_r31 // jal
	;
	assign RF_waddr = {5{RF_waddr_control == RF_Waddr_rd }} & rd
			| {5{RF_waddr_control == RF_Waddr_rt }} & rt
			| {5{RF_waddr_control == RF_Waddr_r31}} & `R31_ADDR
	;

	// register file write data
	wire [2:0] RF_wdata_control;
	parameter RF_Waddr_R    = 3'b001,
		  RF_Waddr_LUI  = 3'b010,
		  RF_Waddr_ICal = 3'b011,
		  RF_Waddr_JAL  = 3'b100,
		  RF_Waddr_Load = 3'b101;
	assign RF_wdata_control = RF_Waddr_R    & {3{opcode      == `R_TYPE  }}                  // R
				| RF_Waddr_LUI  & {3{opcode      == `LUI     }}                  // LUI
				| RF_Waddr_ICal & {3{opcode[5:3] == `I_CALCUL & opcode != `LUI}} // I-calcul
				| RF_Waddr_JAL  & {3{opcode      == `JAL     }}                  // jal
				| RF_Waddr_Load & {3{opcode[5]   & ~opcode[3]}}                  // I_load
	;
	assign RF_wdata = {32{RF_wdata_control == RF_Waddr_R   }} & (reg_alu_shift_out & {32{{funct[5:3], funct[1]} != `R_MOVE  &
								                              funct[5:3]            != `R_SHIFT &
											      funct                 != `JALR      }} |
								     reg_RF_rdata1     & {32{{funct[5:3], funct[1]} == `R_MOVE    }} |
								     reg_alu_shift_out & {32{ funct[5:3]            == `R_SHIFT   }} |
								     reg_alu_shift_out & {32{ funct                 == `JALR      }}   )
			| {32{RF_wdata_control == RF_Waddr_LUI }} & {immediate, 16'b0}
			| {32{RF_wdata_control == RF_Waddr_ICal}} & reg_alu_shift_out
			| {32{RF_wdata_control == RF_Waddr_JAL }} & reg_alu_shift_out
			| {32{RF_wdata_control == RF_Waddr_Load}} & RF_wpartial
	;
	assign RF_wpartial = {32{opcode[2:0] == 3'b000}} & load_byte_sign_ext // lb
			   | {32{opcode[2:0] == 3'b100}} & load_byte_zero_ext // lbu
			   | {32{opcode[2:0] == 3'b001}} & load_half_sign_ext // lh
			   | {32{opcode[2:0] == 3'b101}} & load_half_zero_ext // lhu
			   | {32{opcode[2:0] == 3'b011}} & reg_Memory         // lw
			   | {32{opcode[2:0] == 3'b010}} & load_word_left     // lwl
			   | {32{opcode[2:0] == 3'b110}} & load_word_right    // lwr
	;
	// used for load instructions to select write_data for register file
	wire [ 7:0] load_byte;
	wire [31:0] load_byte_sign_ext;
	wire [31:0] load_byte_zero_ext;
	wire [15:0] load_half;
	wire [31:0] load_half_sign_ext;
	wire [31:0] load_half_zero_ext;
	wire [31:0] load_word_left;
	wire [31:0] load_word_right;

	// load a byte
	assign load_byte = {8{remain_0}} & reg_Memory[ 7: 0] |
			   {8{remain_1}} & reg_Memory[15: 8] |
			   {8{remain_2}} & reg_Memory[23:16] |
			   {8{remain_3}} & reg_Memory[31:24]
	;
	assign load_byte_sign_ext = $signed(load_byte);
	assign load_byte_zero_ext = {24'b0, load_byte};

	// load a half word
	assign load_half = {16{remain_0}} & reg_Memory[15: 0]  |
			   {16{remain_2}} & reg_Memory[31:16]
	;
	assign load_half_sign_ext = $signed(load_half);
	assign load_half_zero_ext = {16'b0, load_half};

	// load a word's left part or right part
	assign load_word_left  = ({reg_Memory[ 7: 0], reg_RF_rdata2[23: 0]} & {32{remain_0}} |
				  {reg_Memory[15: 0], reg_RF_rdata2[15: 0]} & {32{remain_1}} |
				  {reg_Memory[23: 0], reg_RF_rdata2[ 7: 0]} & {32{remain_2}} |
				  {reg_Memory[31: 0]                      } & {32{remain_3}}   )
	;
	assign load_word_right = ({                      reg_Memory[31: 0]} & {32{remain_0}} |
				  {reg_RF_rdata2[31:24], reg_Memory[31: 8]} & {32{remain_1}} |
				  {reg_RF_rdata2[31:16], reg_Memory[31:16]} & {32{remain_2}} |
				  {reg_RF_rdata2[31: 8], reg_Memory[31:24]} & {32{remain_3}}   )
	;

	// register file read address.
	assign RF_raddr1 = rs;
	assign RF_raddr2 = rt & {5{opcode != `I_REGIMM}};

	// control PC calculation, combinational part
	assign PC = PC_reg; // have changed
	assign PC_jump_addr = {32{opcode      == `R_TYPE  }} & reg_RF_rdata1
			    | {32{opcode[5:2] == `I_BRANCH}} & reg_alu_shift_out
			    | {32{opcode      == `I_REGIMM}} & reg_alu_shift_out
			    | {32{opcode[5:1] == `J_TYPE  }} & {PC_reg[31:28], target, 2'b00}
	;
	assign PC_wen_cond = opcode[5:2] == `I_BRANCH & (opcode[0] ^ alu_Zero)
			   | opcode      == `I_REGIMM & (~rt[0] ^ alu_Zero)
			   | opcode[5:1] == `J_TYPE
			   | opcode      == `R_TYPE   & {funct[5:3], funct[1]} == `R_JUMP
	;
	assign PC_wen_next = (current_state == Instruct_Fecthing);
	assign PC_wen = PC_wen_cond & (current_state == Executing) | PC_wen_next;
	assign PC_renew = {32{current_state == Executing        }} & PC_jump_addr
			| {32{current_state == Instruct_Fecthing}} & alu_Result
	;

	// control alu input
	wire [ 1:0] alu_A_control;
	wire [31:0] alu_A_exe;
	parameter Alu_A_Rdata1 = 2'b00,
		  Alu_A_Rdata2 = 2'b01,
		  Alu_A_PC     = 2'b10,
		  Alu_A_Zero   = 2'b11;
	assign alu_A_control = {2{opcode      == `R_TYPE  }} & Alu_A_Zero   & {2{{funct[5:3], funct[1]} == `R_MOVE}}
			     | {2{opcode      == `R_TYPE  }} & Alu_A_PC     & {2{ funct                 == `JALR  }}
			     | {2{opcode[5:2] == `I_BRANCH}} & Alu_A_Rdata2
			     | {2{opcode      == `JAL     }} & Alu_A_PC
	;
	assign alu_A_exe = {32{alu_A_control == Alu_A_Rdata1}} & reg_RF_rdata1
			 | {32{alu_A_control == Alu_A_Rdata2}} & reg_RF_rdata2
			 | {32{alu_A_control == Alu_A_PC    }} & PC_reg
			 | {32{alu_A_control == Alu_A_Zero  }} & 32'b0
	;
	assign alu_A = {32{current_state == Instruct_Fecthing}} & PC_reg
		     | {32{current_state == Instruct_Decoding}} & PC_reg
		     | {32{current_state == Executing        }} & alu_A_exe
		//      | {32{current_state == Mem_Referencing  }} & alu_A_exe
		//      | {32{current_state == Writing_Back     }} & alu_A_exe
	;

	wire [ 2:0] alu_B_control;
	wire [31:0] alu_B_exe;
	parameter Alu_B_Rdata2   = 3'b000,
		  Alu_B_Rdata1   = 3'b001,
		  Alu_B_Zero_Ext = 3'b010,
		  Alu_B_Sign_Ext = 3'b011,
		  Alu_B_4        = 3'b100;
	assign alu_B_control = {3{opcode[5]               }} & Alu_B_Sign_Ext                          // I-load and I-store
			     | {3{opcode[5:3] == `I_CALCUL}} & ({3{ opcode[2]}} & Alu_B_Zero_Ext   |   // I-calcul
		     					        {3{~opcode[2]}} & Alu_B_Sign_Ext     )
			     | {3{opcode      == `R_TYPE  }} & ({3{funct == `JALR}} & Alu_B_4      |   // JALR
		     					        {3{funct != `JALR}} & Alu_B_Rdata2   ) // R-calcul and R-move
			     | {3{opcode[5:2] == `I_BRANCH}} & Alu_B_Rdata1                            // I-branch
			     | {3{opcode      == `I_REGIMM}} & Alu_B_Rdata2                            // I-regimm
			     | {3{opcode      == `JAL     }} & Alu_B_4                                 // JAL
	;
	assign alu_B_exe = {32{alu_B_control == Alu_B_Rdata1  }} & reg_RF_rdata1
			 | {32{alu_B_control == Alu_B_Rdata2  }} & reg_RF_rdata2
			 | {32{alu_B_control == Alu_B_Zero_Ext}} & zero_ext_imm
			 | {32{alu_B_control == Alu_B_Sign_Ext}} & sign_ext_imm
			 | {32{alu_B_control == Alu_B_4       }} & 32'd4
	;
	assign alu_B = {32{current_state == Instruct_Fecthing}} & 32'd4
		     | {32{current_state == Instruct_Decoding}} & {sign_ext_imm[29:0], 2'b00}
		     | {32{current_state == Executing        }} & alu_B_exe
		//      | {32{current_state == Mem_Referencing  }} & alu_B_exe
		//      | {32{current_state == Writing_Back     }} & alu_B_exe
	;

	// an "unextendable" way to handle with the ALUops of calculation instructions
	// R-type:   [5&1&(~3|~0), ~2,  3|2&0]
	// I-calcul: [1&~0,  ~2,  ~2&1|2&0]
	wire [2:0] alu_ALUop_R_type;
	wire [2:0] alu_ALUop_I_CALCUL;
	wire [2:0] alu_ALUop_exe;
	assign alu_ALUop_R_type   = {funct[5] & funct[1] & (~funct[3] | ~funct[0]),
				     ~funct[2],
				     funct[3] | funct[2] & funct[0]}
	;
	assign alu_ALUop_I_CALCUL = {opcode[1] & ~opcode[0],
				     ~opcode[2],
				     ~opcode[2] & opcode[1] | opcode[2] & opcode[0]}
	;
	assign alu_ALUop_exe = {3{opcode      == `R_TYPE  }} & ( {3{funct != `JALR}} & alu_ALUop_R_type |   // other R-type
							         {3{funct == `JALR}} & 3'b010             ) // JALR
			     | {3{opcode[5:3] == `I_CALCUL}} & alu_ALUop_I_CALCUL
			     | {3{opcode[5:2] == `I_BRANCH}} & {2'b11, opcode[1]}
			     | {3{opcode      == `I_REGIMM}} & 3'b111
			     | {3{opcode      == `JAL     }} & 3'b010
			     | {3{opcode[5]               }} & 3'b010             // load & store
	;
	assign alu_ALUop = {3{current_state == Instruct_Fecthing}} & 3'b010
			 | {3{current_state == Instruct_Decoding}} & 3'b010
			 | {3{current_state == Executing        }} & alu_ALUop_exe
			//  | {3{current_state == Mem_Referencing  }} & alu_ALUop_exe
			//  | {3{current_state == Writing_Back     }} & alu_ALUop_exe
	    ;

	// for selecting bytes when in I-load or I-store instruction
	assign bytes_sel = reg_alu_shift_out[1:0];
	assign remain_0  = (bytes_sel == 2'b00);
	assign remain_1  = (bytes_sel == 2'b01);
	assign remain_2  = (bytes_sel == 2'b10);
	assign remain_3  = (bytes_sel == 2'b11);

	// shifter port
	assign shift_A = reg_RF_rdata2;
	assign shift_B = {5{ funct[2]}} & reg_RF_rdata1[4:0]
		       | {5{~funct[2]}} & shamt[4:0]
	;
	assign shift_Shiftop = funct[1:0];


	// whther to read from or write data in the memory
	assign MemRead  = opcode[5] & ~opcode[3] & (current_state == Mem_Referencing);
	assign MemWrite = opcode[5] &  opcode[3] & (current_state == Mem_Referencing);

	// which data should be writen in the memory, and where should it be written
	/*
	in little endian order:
	lwl loads the data from given address, set it as the least significant byte,
		and load higher bytes (right part, including M[addr]) into register's lower  bytes (left  part);
	lwr loads the data from given address, set it as the most  significant byte,
		and load lower  bytes (left  part, including M[addr]) into register's higher bytes (right part);
	*/
	assign Address    = {reg_alu_shift_out[31:2], 2'b0};

	assign Write_data = {32{opcode[1:0] == 2'b00 }} & {4{reg_RF_rdata2[ 7:0]}}
			  | {32{opcode[1:0] == 2'b01 }} & {2{reg_RF_rdata2[15:0]}}
			  | {32{opcode[1:0] == 2'b11 }} &    reg_RF_rdata2
			  | {32{opcode[2:0] == 3'b010}} & ({32{remain_0}} & {24'b0, reg_RF_rdata2[31:24]} |
							   {32{remain_1}} & {16'b0, reg_RF_rdata2[31:16]} |
							   {32{remain_2}} & { 8'b0, reg_RF_rdata2[31: 8]} |
							   {32{remain_3}} &         reg_RF_rdata2[31: 0]    )
			  | {32{opcode[2:0] == 3'b110}} & ({32{remain_0}} &  reg_RF_rdata2[31: 0]         |
							   {32{remain_1}} & {reg_RF_rdata2[23: 0],  8'b0} |
							   {32{remain_2}} & {reg_RF_rdata2[15: 0], 16'b0} |
							   {32{remain_3}} & {reg_RF_rdata2[ 7: 0], 24'b0}   )
	;

	assign Write_strb = {4{opcode[2:0] == 3'b000}} & {remain_3, remain_2, remain_1, remain_0}
		  	  | {4{opcode[2:0] == 3'b001}} & {remain_2, remain_2, remain_0, remain_0}
			  | {4{opcode[2:0] == 3'b011}} & 4'b1111
			  | {4{opcode[2:0] == 3'b010}} & {&bytes_sel[1:0], bytes_sel[1]      , |bytes_sel[1:0], 1'b1              }
			  | {4{opcode[2:0] == 3'b110}} & {1'b1           , ~(&bytes_sel[1:0]), ~bytes_sel[1]  , ~(|bytes_sel[1:0])}
	;

	reg [4:0] current_state;
	reg [4:0] next_state;

	parameter Instruct_Fecthing = 5'b00001,
		  Instruct_Decoding = 5'b00010,
		  Executing         = 5'b00100,
		  Mem_Referencing   = 5'b01000,
		  Writing_Back      = 5'b10000;

	// 1st part: state transition execution
	always @(posedge clk) begin
		if (rst)
			current_state <= Instruct_Fecthing;
		else
			current_state <= next_state;
	end

	// 2nd part: state transition calculation
	always @(*) begin
		case (current_state)
			Instruct_Fecthing: begin
				next_state = Instruct_Decoding;
			end

			Instruct_Decoding: begin
				// NOP
				if (reg_Instruction == 32'b0)
					next_state = Instruct_Fecthing;
				else
					next_state = Executing;
			end

			Executing: begin
				// load and store
				if (opcode[5])
					next_state = Mem_Referencing;
				// R, I-calculation, JAL
				else if (opcode == `R_TYPE || opcode[5:3] == `I_CALCUL || opcode == `JAL)
					next_state = Writing_Back;
				// I-REGIMM, I-branch, J   or other instructions
				else
					next_state = Instruct_Fecthing;
			end

			Mem_Referencing: begin
				// I-load (plus by other impossible situations)
				if (~opcode[3])
					next_state = Writing_Back;
				// I-store    or others
				else
					next_state = Instruct_Fecthing;
			end

			// Writing_Back is included in the "default" case, so we ommit this situation
			default:
				next_state = Instruct_Fecthing;
		endcase
	end

	// when the clock comes up
	always @(posedge clk) begin
		if (rst)
			PC_reg <= 32'b0;
		else if  (~rst & PC_wen)
			PC_reg <= PC_renew;
	end

	// dealing with reg_Instruction
	always @(posedge clk) begin
		if (current_state == Instruct_Fecthing)
			reg_Instruction <= Instruction;
	end

	// dealing with reg_Memory, when dealing with load instruction
	always @(posedge clk) begin
		reg_Memory <= Read_data;
	end

	// dealing with reg_RF_rdata1
	always @(posedge clk) begin
		reg_RF_rdata1 <= RF_rdata1;
	end

	// dealing with reg_RF_rdata2
	always @(posedge clk) begin
		reg_RF_rdata2 <= RF_rdata2;
	end

	// dealing with reg_alu_shift_out
	always @(posedge clk) begin
		if (current_state == Instruct_Decoding)
			reg_alu_shift_out <= alu_Result;
		else if (current_state == Executing) begin
			if (opcode == `R_TYPE && funct[5:3] == `R_SHIFT)
				reg_alu_shift_out <= shift_Result;
			else
				reg_alu_shift_out <= alu_Result;
		end
	end

endmodule









// module alu(
// 	input  [`DATA_WIDTH - 1:0]  A,
// 	input  [`DATA_WIDTH - 1:0]  B,
// 	input  [              2:0]  ALUop,
// 	output                      Overflow,
// 	output                      CarryOut,
// 	output                      Zero,
// 	output [`DATA_WIDTH - 1:0]  Result
// );
// endmodule

// module reg_file(
// 	input                       clk,
// 	input  [`ADDR_WIDTH - 1:0]  waddr,
// 	input  [`ADDR_WIDTH - 1:0]  raddr1,
// 	input  [`ADDR_WIDTH - 1:0]  raddr2,
// 	input                       wen,
// 	input  [`DATA_WIDTH - 1:0]  wdata,
// 	output [`DATA_WIDTH - 1:0]  rdata1,
// 	output [`DATA_WIDTH - 1:0]  rdata2
// );
// endmodule

// module shifter (
// 	input  [`DATA_WIDTH - 1:0] A,
// 	input  [              4:0] B,
// 	input  [              1:0] Shiftop,
// 	output [`DATA_WIDTH - 1:0] Result
// );
// endmodule
