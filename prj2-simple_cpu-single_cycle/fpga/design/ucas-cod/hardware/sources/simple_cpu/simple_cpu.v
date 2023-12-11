`timescale 10ns / 1ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

`define R0_ADDR  5'b00000
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
`define JALR     6'b001001
`define JR       6'b001000
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
	// register read
	wire [ 4:0] RF_raddr1;
	wire [ 4:0] RF_raddr2;
	wire [31:0] RF_rdata1;
	wire [31:0] RF_rdata2;

	// register write data when using I-load instructions
	wire [31:0] RF_wpartial;

	// PC store and calculation
	wire        PC_adder_cin;   // the register storing PC
	reg  [29:0] PC_reg;         // the register storing PC
	wire [29:0] PC_next;        // next address of instruction
	wire [29:0] PC_branch_jump; // branch and jump address
	wire [ 1:0] PC_br_jp_control; // condition of jump or not
	wire [29:0] PC_renew;

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

	// shifter port
	wire [31:0] shift_A;
	wire [ 4:0] shift_B;
	wire [ 1:0] shift_Shiftop;
	wire [31:0] shift_Result;

	// split instruction into several parts
	wire [ 5:0] opcode;      //  6 bits
	wire [ 4:0] rs;          //  5 bits
	wire [ 4:0] rt;          //  5 bits
	wire [ 4:0] rd;          //  5 bits
	wire [ 4:0] shamt;       //  5 bits
	wire [ 5:0] funct;       //  6 bits
	wire [15:0] immediate;   // 16 bits
	wire [25:0] target_addr; // 26 bits
	// extension for immediate
	wire [31:0] sign_ext_imm;     // sign-extended from "immediate"
	wire [31:0] zero_ext_imm;     // zero-extended from "immediate"

	assign opcode      = Instruction[31:26];
	assign rs          = Instruction[25:21];
	assign rt          = Instruction[20:16];
	assign rd          = Instruction[15:11];
	assign shamt       = Instruction[10: 6];
	assign funct       = Instruction[ 5: 0];
	assign immediate   = Instruction[15: 0];
	assign target_addr = Instruction[25: 0];

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
	parameter RF_Wen_RType = 2'b01;
	parameter RF_Wen_True  = 2'b11;
	assign RF_wen_control = {2{opcode[5:3] == `I_CALCUL}} & RF_Wen_True  // I-calcul
			      | {2{opcode      == `R_TYPE  }} & RF_Wen_RType // R
			      | {2{opcode      == `JAL     }} & RF_Wen_True  // jal
			      | {2{opcode[5] & ~opcode[3]  }} & RF_Wen_True  // I-load
	;
	assign RF_wen = (RF_wen_control == RF_Wen_RType) &  // get the inverse signal to simplify circuit
			~( ({funct[5:3], funct[1]} == `R_MOVE) & (funct[0] == alu_Zero) | (funct == `JR) )
		      | (RF_wen_control == RF_Wen_True)
	;

	// register file write address
	wire [1:0] RF_waddr_control;
	parameter RF_Waddr_rd  = 2'b01;
	parameter RF_Waddr_rt  = 2'b10;
	parameter RF_Waddr_r31 = 2'b11;
	assign RF_waddr_control = {2{opcode      == `R_TYPE  }} & RF_Waddr_rd  // R
				| {2{opcode[5:3] == `I_CALCUL}} & RF_Waddr_rt  // I-calcul
				| {2{opcode[5] & (~opcode[3])}} & RF_Waddr_rt  // I_load
				| {2{opcode      == `JAL     }} & RF_Waddr_r31 // jal
	;
	assign RF_waddr = {5{RF_waddr_control == RF_Waddr_rd }} & rd
			| {5{RF_waddr_control == RF_Waddr_rt }} & rt
			| {5{RF_waddr_control == RF_Waddr_r31}} & `R31_ADDR
	;

	// register file write data
	wire [2:0] RF_wdata_control;
	parameter RF_Waddr_R    = 3'b001;
	parameter RF_Waddr_LUI  = 3'b010;
	parameter RF_Waddr_ICal = 3'b011;
	parameter RF_Waddr_JAL  = 3'b100;
	parameter RF_Waddr_Load = 3'b101;
	assign RF_wdata_control = RF_Waddr_R    & {3{opcode      == `R_TYPE  }}                   // R
				| RF_Waddr_LUI  & {3{opcode      == `LUI     }}                   // LUI
				| RF_Waddr_ICal & {3{opcode[5:3] == `I_CALCUL & ~(&opcode[2:0])}} // I-calcul
				| RF_Waddr_JAL  & {3{opcode      == `JAL     }}                   // jal
				| RF_Waddr_Load & {3{opcode[5]   & ~opcode[3]}}                   // I_load
	;
	assign RF_wdata = {32{RF_wdata_control == RF_Waddr_R   }} & (alu_Result       & {32{({funct[5:3], funct[1]} != `R_MOVE) &
								                            (funct[5:3] != `R_SHIFT)            &
											    (funct != `JALR)                      }} |
								     RF_rdata1        & {32{ {funct[5:3], funct[1]} == `R_MOVE    }} |
								     shift_Result     & {32{  funct[5:3]            == `R_SHIFT   }} |
								     {PC_next, 2'b00} & {32{  funct                 == `JALR      }}   )
			| {32{RF_wdata_control == RF_Waddr_LUI }} & {immediate, 16'b0}
			| {32{RF_wdata_control == RF_Waddr_ICal}} & alu_Result
			| {32{RF_wdata_control == RF_Waddr_JAL }} & {PC_next, 2'b00}
			| {32{RF_wdata_control == RF_Waddr_Load}} & RF_wpartial
	;
	assign RF_wpartial = {32{opcode[2:0] == 3'b000}} & load_byte_sign_ext // lb
			   | {32{opcode[2:0] == 3'b100}} & load_byte_zero_ext // lbu
			   | {32{opcode[2:0] == 3'b001}} & load_half_sign_ext // lh
			   | {32{opcode[2:0] == 3'b101}} & load_half_zero_ext // lhu
			   | {32{opcode[2:0] == 3'b011}} & Read_data          // lw
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
	assign load_byte = {8{remain_0}} & Read_data[ 7: 0] |
			   {8{remain_1}} & Read_data[15: 8] |
			   {8{remain_2}} & Read_data[23:16] |
			   {8{remain_3}} & Read_data[31:24]
	;
	assign load_byte_sign_ext = $signed(load_byte);
	assign load_byte_zero_ext = {24'b0, load_byte};

	// load a half word
	assign load_half = {16{remain_0}} & Read_data[15: 0]  |
			   {16{remain_2}} & Read_data[31:16]
	;
	assign load_half_sign_ext = $signed(load_half);
	assign load_half_zero_ext = {16'b0, load_half};

	// load a word's left part or right part
	assign load_word_left  = ({Read_data[ 7: 0], RF_rdata2[23: 0]} & {32{remain_0}} |
				  {Read_data[15: 0], RF_rdata2[15: 0]} & {32{remain_1}} |
				  {Read_data[23: 0], RF_rdata2[ 7: 0]} & {32{remain_2}} |
				  {Read_data[31: 0]                  } & {32{remain_3}}   )
	;
	assign load_word_right = ({                  Read_data[31: 0]} & {32{remain_0}} |
				  {RF_rdata2[31:24], Read_data[31: 8]} & {32{remain_1}} |
				  {RF_rdata2[31:16], Read_data[31:16]} & {32{remain_2}} |
				  {RF_rdata2[31: 8], Read_data[31:24]} & {32{remain_3}}   )
	;

	// register file read address.
	assign RF_raddr1 = rs;
	assign RF_raddr2 = rt & {5{opcode != `I_REGIMM}};

	// control PC calculation, combinational part
	assign PC           = {PC_reg, 2'b00};
	assign PC_adder_cin = (opcode == `JAL) | (opcode == `R_TYPE) & (funct == `JALR);
	assign PC_next      = PC_reg + 1 + PC_adder_cin;

	parameter Next_Instr    = 2'b00;
	parameter Branch_Offset = 2'b01;
	parameter Jump_imm      = 2'b10;
	parameter Jump_reg      = 2'b11;

	assign PC_branch_jump = {30{PC_br_jp_control == Next_Instr   }} & PC_next
		              | {30{PC_br_jp_control == Branch_Offset}} & (PC_next + sign_ext_imm[29:0])
		              | {30{PC_br_jp_control == Jump_imm     }} & {PC_next[29:26], target_addr}
		              | {30{PC_br_jp_control == Jump_reg     }} & RF_rdata1[31:2]
	;
	assign PC_br_jp_control = {2{opcode[5:2] == `I_BRANCH}} & Branch_Offset & {2{opcode[0] ^ alu_Zero}}
				| {2{opcode      == `I_REGIMM}} & Branch_Offset & {2{   ~rt[0] ^ alu_Zero}}
				| {2{opcode[5:1] == `J_TYPE  }} & Jump_imm
				| {2{opcode      == `R_TYPE  }} & Jump_reg      & {2{{funct[5:3], funct[1]} == `R_JUMP}}
	;
	assign PC_renew = PC_branch_jump;

	// control alu input
	wire [1:0] alu_A_control;
	parameter Alu_A_Rdata1 = 2'b00;
	parameter Alu_A_Rdata2 = 2'b01;
	parameter Alu_A_Zero   = 2'b11;
	assign alu_A_control = {2{opcode      == `R_TYPE  }} & Alu_A_Zero   & {2{{funct[5:3], funct[1]} == `R_MOVE}}
			     | {2{opcode[5:2] == `I_BRANCH}} & Alu_A_Rdata2
	;
	assign alu_A = {32{alu_A_control == Alu_A_Rdata1}} & RF_rdata1
		     | {32{alu_A_control == Alu_A_Rdata2}} & RF_rdata2
		//      | {32{alu_A_control == Alu_A_Zero  }} & 32'b0    // this line can be emitted
	;
	wire [1:0] alu_B_control;
	parameter Alu_B_Rdata2   = 2'b00;
	parameter Alu_B_Rdata1   = 2'b01;
	parameter Alu_B_Zero_Ext = 2'b10;
	parameter Alu_B_Sign_Ext = 2'b11;
	assign alu_B_control = {2{opcode[5]               }} & Alu_B_Sign_Ext                        // I-load and I-store
			     | {2{opcode[5:3] == `I_CALCUL}} & ({2{ opcode[2]}} & Alu_B_Zero_Ext |   // I-calcul
		     					        {2{~opcode[2]}} & Alu_B_Sign_Ext   )
			     | {2{opcode[5:2] == `I_BRANCH}} & Alu_B_Rdata1                          // I-branch
			     | {2{opcode      == `R_TYPE  }} & Alu_B_Rdata2                          // R-calcul and R-move
			     | {2{opcode      == `I_REGIMM}} & Alu_B_Rdata2                          // I-regimm
	;
	assign alu_B = {32{alu_B_control == Alu_B_Rdata1  }} & RF_rdata1
		     | {32{alu_B_control == Alu_B_Rdata2  }} & RF_rdata2
		     | {32{alu_B_control == Alu_B_Zero_Ext}} & zero_ext_imm
		     | {32{alu_B_control == Alu_B_Sign_Ext}} & sign_ext_imm
	;

	// an "unextendable" way to handle with the ALUops of calculation instructions
	// R-type:   [~3&1 | 3&5&~0, ~2,  3|2&0]
	// I-calcul: [~2&~0|2&1,  ~2,  ~2&1|2&0]
	wire [2:0] alu_ALUop_R_type;
	wire [2:0] alu_ALUop_I_CALCUL;
	assign alu_ALUop_R_type   = {~funct[3] &  funct[1] | funct[5] & funct[3] & ~funct[0],
				     ~funct[2],
				      funct[3] |  funct[2] & funct[0]}
	;
	assign alu_ALUop_I_CALCUL = {~opcode[2] & ~opcode[0] | opcode[2] & opcode[1],
				     ~opcode[2],
				     ~opcode[2] &  opcode[1] | opcode[2] & opcode[0]}
	;
	assign alu_ALUop = {3{opcode      == `R_TYPE  }} & alu_ALUop_R_type
			 | {3{opcode[5:3] == `I_CALCUL}} & alu_ALUop_I_CALCUL
			 | {3{opcode[5:2] == `I_BRANCH}} & {2'b11, opcode[1]}
			 | {3{opcode      == `I_REGIMM}} & 3'b111
			 | {3{opcode[5]               }} & 3'b010             // load & store
	;

	// for selecting bytes when in I-load or I-store instruction
	assign bytes_sel = alu_Result[1:0];

	// shifter port
	assign shift_A = RF_rdata2;
	assign shift_B = {5{ funct[2]}} & RF_rdata1[4:0]
		       | {5{~funct[2]}} & shamt[4:0]
	;
	assign shift_Shiftop = funct[1:0];


	// whther to read from or write data in the memory
	assign MemRead  = opcode[5] & ~opcode[3];
	assign MemWrite = opcode[5] &  opcode[3];

	// which data should be writen in the memory, and where should it be written
	/*
	in little endian order:
	lwl loads the data from given address, set it as the least significant byte,
		and load higher bytes (right part, including M[addr]) into register's lower  bytes (left  part);
	lwr loads the data from given address, set it as the most  significant byte,
		and load lower  bytes (left  part, including M[addr]) into register's higher bytes (right part);
	*/
	assign Address    = {alu_Result[31:2], 2'b0};

	assign Write_data = {32{opcode[1:0] == 2'b00 }} & {4{RF_rdata2[ 7:0]}}
			  | {32{opcode[1:0] == 2'b01 }} & {2{RF_rdata2[15:0]}}
			  | {32{opcode[1:0] == 2'b11 }} &    RF_rdata2
			  | {32{opcode[2:0] == 3'b010}} & ({32{remain_0}} & {24'b0, RF_rdata2[31:24]} |
							   {32{remain_1}} & {16'b0, RF_rdata2[31:16]} |
							   {32{remain_2}} & { 8'b0, RF_rdata2[31: 8]} |
							   {32{remain_3}} &         RF_rdata2[31: 0]    )
			  | {32{opcode[2:0] == 3'b110}} & ({32{remain_0}} &  RF_rdata2[31: 0]         |
							   {32{remain_1}} & {RF_rdata2[23: 0],  8'b0} |
							   {32{remain_2}} & {RF_rdata2[15: 0], 16'b0} |
							   {32{remain_3}} & {RF_rdata2[ 7: 0], 24'b0}   )
	;

	wire remain_0;
	wire remain_1;
	wire remain_2;
	wire remain_3;
	assign remain_0 = (bytes_sel == 2'b00);
	assign remain_1 = (bytes_sel == 2'b01);
	assign remain_2 = (bytes_sel == 2'b10);
	assign remain_3 = (bytes_sel == 2'b11);
	assign Write_strb = {4{opcode[2:0] == 3'b000}} & {remain_3, remain_2, remain_1, remain_0}
		  	  | {4{opcode[2:0] == 3'b001}} & {remain_2, remain_2, remain_0, remain_0}
			  | {4{opcode[2:0] == 3'b011}} & 4'b1111
			  | {4{opcode[2:0] == 3'b010}} & {&bytes_sel[1:0], bytes_sel[1]      , |bytes_sel[1:0]   , 1'b1              }
			  | {4{opcode[2:0] == 3'b110}} & {1'b1           , ~(&bytes_sel[1:0]), ~bytes_sel[1]     , ~(|bytes_sel[1:0])}
	;

	// when the clock comes up
	always @(posedge clk) begin
		if (rst)  begin
			PC_reg <= 30'b0;
		end
		else begin
			PC_reg <= PC_renew;
		end
	end

endmodule
