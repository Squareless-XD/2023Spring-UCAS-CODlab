`timescale 10ns / 1ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

`define U_LUI    7'b0110111
`define U_AUIPC  7'b0010111
`define J_TYPE   7'b1101111
`define I_JALR   7'b1100111
`define B_TYPE   7'b1100011
`define I_LOAD   7'b0000011
`define S_TYPE   7'b0100011
`define I_CALCUL 7'b0010011
`define R_TYPE   7'b0110011
`define ZERO_FIL 32'b0

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
	wire [69:0] inst_retire;

// TODO: Please add your custom CPU code here

	// register write signals
	wire        RF_wen;
	wire [4:0]  RF_waddr;
	wire [31:0] RF_wdata;

	// register in custom_cpu
	reg [31:0] reg_Inst;
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
	reg  [31:0] PC_reg;       // the register storing PC
	reg  [31:0] PC_next;      // the register storing the next value of PC (PC + 4)
	wire [31:0] PC_jump_addr; // branch and jump address
	wire        PC_wen;       // whether refresh PC

	// alu control signals
	wire [31:0] alu_A;
	wire [31:0] alu_B;
	wire [ 2:0] alu_ALUop;
	// wire        alu_Overflow;
	// wire        alu_CarryOut;
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

	// split instruction into several part
	wire [31:0] immediate;   // 32 bits
	wire [ 6:0] funct7;      //  7 bits
	wire [ 4:0] shamt;       //  5 bits
	wire [ 4:0] rs2;         //  5 bits
	wire [ 4:0] rs1;         //  5 bits
	wire [ 2:0] funct3;      //  3 bits
	wire [ 4:0] rd;          //  5 bits
	wire [ 6:0] opcode;      //  7 bits

	// state declarations
	parameter Init_State        = 9'b000000001; // 001
	parameter Instruct_Fecthing = 9'b000000010; // 002
	parameter Instruct_Waiting  = 9'b000000100; // 004
	parameter Instruct_Decoding = 9'b000001000; // 008
	parameter Executing         = 9'b000010000; // 010
	parameter Loading_Memory    = 9'b000100000; // 020
	parameter Storing_Memory    = 9'b001000000; // 040
	parameter Read_Data_Waiting = 9'b010000000; // 080
	parameter Writing_Back      = 9'b100000000; // 100

	// state judges
	wire state_INIT = current_state[0];
	wire state_IF   = current_state[1];
	wire state_IW   = current_state[2];
	wire state_ID   = current_state[3];
	wire state_EX   = current_state[4];
	wire state_LD   = current_state[5];
	wire state_ST   = current_state[6];
	wire state_RDW  = current_state[7];
	wire state_WB   = current_state[8];
	
	wire [2:0] imm_ctrl;
	
	parameter I_Type = 3'b000; // for it is zero, no need to assign condition for it
	parameter B_Type = 3'b001;
	parameter J_Type = 3'b110;
	parameter U_Type = 3'b011;
	parameter S_Type = 3'b101;
	parameter R_Type = 3'b111; // we don't care R-type instructions in imm_ctrl signal
	wire op_class_I = (imm_ctrl == I_Type); // for it is zero, no need to assign condition for it
	wire op_class_B = (imm_ctrl == B_Type);
	wire op_class_J = (imm_ctrl == J_Type);
	wire op_class_U = (imm_ctrl == U_Type);
	wire op_class_S = (imm_ctrl == S_Type);
	wire op_class_R = (imm_ctrl == R_Type); // we don't care R-type instructions here

	assign imm_ctrl = {3{opcode[4] &  opcode[2]            }} & U_Type // U-type
			| {3{opcode[3]                         }} & J_Type // J-type
			| {3{opcode[6] & ~opcode[2]            }} & B_Type // B-type
			| {3{opcode[6:4] == 3'b010             }} & S_Type // S-type
			| {3{{opcode[5:4], opcode[2]} == 3'b110}} & R_Type // R-type
	; // others are I-type

	assign immediate = {
		reg_Inst[31],
		op_class_U ? reg_Inst[30:20] : {11{reg_Inst[31]}},              // U
		imm_ctrl[1] ? reg_Inst[19:12] : {8{reg_Inst[31]}},                        // U,J
		op_class_B & reg_Inst[7] | op_class_J & reg_Inst[20] | (imm_ctrl[2] == imm_ctrl[0]) & reg_Inst[31], // B / J / I,S
		{6{~op_class_U}} & reg_Inst[30:25],                                // not U
		{4{~imm_ctrl[1] & imm_ctrl[0]}} & reg_Inst[11:8] | {4{~imm_ctrl[0]}} & reg_Inst[24:21], // S,B / I,J
		op_class_I & reg_Inst[20] | op_class_S & reg_Inst[7]  // I / S
	};
	
	assign funct7 = reg_Inst[31:25];
	assign shamt  = reg_Inst[24:20];
	assign rs2    = reg_Inst[24:20];
	assign rs1    = reg_Inst[19:15];
	assign funct3 = reg_Inst[14:12];
	assign rd     = reg_Inst[11: 7];
	assign opcode = reg_Inst[ 6: 0];

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
		// .Overflow (alu_Overflow),
		// .CarryOut (alu_CarryOut),
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
	assign RF_wen = state_WB & ({opcode[5:4], opcode[2]} != 3'b100);

	// register file write address
	assign RF_waddr = rd;

	// register file write data
	wire [1:0] RF_wdata_ctrl;
	parameter RF_Wdata_Res  = 2'b00; // for it is zero, no need to assign condition for it
	parameter RF_Wdata_Next = 2'b01;
	parameter RF_Wdata_Read = 2'b10;
	assign RF_wdata_ctrl = {2{~opcode[4] &  opcode[2]}} & RF_Wdata_Next // J-type, JALR(I-jump)
			     | {2{~opcode[5] & ~opcode[4]}} & RF_Wdata_Read // I-load
	; // PLUS BY: U, I-calculate, R-type
	assign RF_wdata = {32{RF_wdata_ctrl == RF_Wdata_Res   }} & reg_alu_shift_out
			| {32{RF_wdata_ctrl == RF_Wdata_Next  }} & PC_next
			| {32{RF_wdata_ctrl == RF_Wdata_Read  }} & RF_wpartial
	;
	assign RF_wpartial = {32{funct3 == 3'b000}} & {{24{load_byte[ 7]}}, load_byte} // LB
			   | {32{funct3 == 3'b100}} & { 24'b0             , load_byte} // LBU
			   | {32{funct3 == 3'b001}} & {{16{load_half[15]}}, load_half} // LH
			   | {32{funct3 == 3'b101}} & { 16'b0             , load_half} // LHU
			   | {32{funct3 == 3'b010}} & reg_Memory                       // LW
	;
	// used for load instructions to select write_data for register file
	wire [ 7:0] load_byte;
	wire [15:0] load_half;
	// load a byte
	assign load_byte = {8{remain_0}} & reg_Memory[ 7: 0]
			 | {8{remain_1}} & reg_Memory[15: 8]
			 | {8{remain_2}} & reg_Memory[23:16]
			 | {8{remain_3}} & reg_Memory[31:24]
	;
	// load a half word
	assign load_half = {16{remain_0}} & reg_Memory[15: 0]
			 | {16{remain_2}} & reg_Memory[31:16]
	;


	// register file read address.
	assign RF_raddr1 = rs1;
	assign RF_raddr2 = rs2;

	// control PC calculation, combinational part
	assign PC = PC_reg; // have changed
	assign PC_jump_addr = {32{op_class_J            }} & alu_Result          // J-type
			    | {32{~opcode[3] & opcode[2]}} & alu_Result & -32'd2 // JALR
			    | {32{op_class_B            }} & reg_alu_shift_out   // Branch type
	;
	assign PC_wen = opcode[6] & opcode[5] & (opcode[2] | (funct3[2] ^ funct3[0] ^ alu_Zero));

	// control alu input
	wire [ 1:0] alu_A_ctrl;
	wire [31:0] alu_A_exe;
	parameter Alu_A_Rdata1 = 2'b00; // for it is zero, no need to assign condition for it
	parameter Alu_A_PC     = 2'b10;
	parameter Alu_A_Zero   = 2'b11;

	assign alu_A_ctrl = {2{~opcode[5] & opcode[2] | opcode[3]}} & Alu_A_PC   // AUIPC, JALR
			  | {2{ opcode[5] & opcode[4] & opcode[2]}} & Alu_A_Zero // LUI
	;
	assign alu_A_exe = {32{alu_A_ctrl == Alu_A_Rdata1}} & reg_RF_rdata1
			 | {32{alu_A_ctrl == Alu_A_PC    }} & PC_reg
	;
	assign alu_A = {32{state_IW}} & PC_reg
		     | {32{state_ID}} & PC_reg
		     | {32{state_EX}} & alu_A_exe
	;

	wire        alu_B_ctrl;
	wire [31:0] alu_B_exe;
	parameter Alu_B_Rdata2   = 1'b1;
	parameter Alu_B_Sign_Ext = 1'b0; // for it is zero, no need to assign condition for it
	assign alu_B_ctrl = op_class_B | op_class_R; // B-type, R-type
	assign alu_B_exe = alu_B_ctrl ? reg_RF_rdata2 : immediate;
	assign alu_B = {32{state_IW}} & 32'd4
		     | {32{state_ID}} & immediate
		     | {32{state_EX}} & alu_B_exe
	;

	// an "unextendable" way to handle with the ALUops of calculation instructions
	// R-type:   [5&1&(~3|~0), ~2,  3|2&0]
	// I-calcul: [1&~0,  ~2,  ~2&1|2&0]
	wire [2:0] alu_ALUop_R_type;
	wire [2:0] alu_ALUop_I_CAL;
	wire [1:0] alu_ALUop_ctrl;
	wire [2:0] alu_ALUop_exe;
	parameter Alu_ALUop_Plus = 2'b00; // for it is zero, no need to assign condition for it
	parameter Alu_ALUop_R    = 2'b01;
	parameter Alu_ALUop_B    = 2'b10;
	parameter Alu_ALUop_ICal = 2'b11;
	assign alu_ALUop_R_type = alu_ALUop_I_CAL | {reg_Inst[30], 2'b00};
	assign alu_ALUop_I_CAL  = {(funct3[2] ^ funct3[1]) & ~funct3[0],
				   ~funct3[2],
				   funct3[1] & ~(funct3[2] & funct3[0]) }
	;
	assign alu_ALUop_ctrl = {2{op_class_R                         }} & Alu_ALUop_R    // R-type
			      | {2{op_class_B                         }} & Alu_ALUop_B    // Branch
			      | {2{~opcode[5] & opcode[4] & ~opcode[2]}} & Alu_ALUop_ICal // I-calculation
	;
	assign alu_ALUop_exe = {3{alu_ALUop_ctrl == Alu_ALUop_Plus}} & 3'b010                        // others
			     | {3{alu_ALUop_ctrl == Alu_ALUop_R   }} & alu_ALUop_R_type              // R-type
			     | {3{alu_ALUop_ctrl == Alu_ALUop_ICal}} & alu_ALUop_I_CAL               // I-calculation
			     | {3{alu_ALUop_ctrl == Alu_ALUop_B   }} & {~funct3[1], 1'b1, funct3[2]} // Branch
	;
	assign alu_ALUop = {3{state_IW}} & 3'b010
			 | {3{state_ID}} & 3'b010
			 | {3{state_EX}} & alu_ALUop_exe
	;

	// for selecting bytes when in I-load or I-store instruction
	assign bytes_sel = reg_alu_shift_out[1:0];
	assign remain_0  = (bytes_sel == 2'b00);
	assign remain_1  = (bytes_sel == 2'b01);
	assign remain_2  = (bytes_sel == 2'b10);
	assign remain_3  = (bytes_sel == 2'b11);

	// shifter port
	assign shift_A = reg_RF_rdata1;
	assign shift_B = opcode[5] ? reg_RF_rdata2[4:0] : shamt[4:0];
	assign shift_Shiftop = {funct3[2], reg_Inst[30]};


	// whether to read from or write data in the memory, and other handshaking signals
	assign Inst_Req_Valid  = state_IF;
	assign Inst_Ready      = state_INIT | state_IW;
	assign MemWrite        = state_ST;
	assign MemRead         = state_LD;
	assign Read_data_Ready = state_INIT | state_RDW;

	// which data should be writen in the memory, and where should it be written
	// note: only consider natural aligned memory address
	assign Address    = {reg_alu_shift_out[31:2], 2'b0};

	assign Write_data = {32{funct3[1:0] == 2'b00}} & {4{reg_RF_rdata2[ 7:0]}}
			  | {32{funct3[1:0] == 2'b01}} & {2{reg_RF_rdata2[15:0]}}
			  | {32{funct3[1:0] == 2'b10}} &    reg_RF_rdata2
	;

	assign Write_strb = {4{funct3[1:0] == 2'b00}} & {remain_3, remain_2, remain_1, remain_0}
		  	  | {4{funct3[1:0] == 2'b01}} & {remain_2, remain_2, remain_0, remain_0}
			  | {4{funct3[1:0] == 2'b10}} & 4'b1111
	;

	reg [8:0] current_state;
	reg [8:0] next_state;


	// 1st part: state transition execution
	always @(posedge clk) begin
		if (rst)
			current_state <= Init_State;
		else
			current_state <= next_state;
	end

	// 2nd part: state transition calculation
	always @(*) begin
		case (current_state)
			Init_State: begin
				next_state = Instruct_Fecthing;
			end

			Instruct_Fecthing: begin
				if (Inst_Req_Ready)                     // ready for instruction reading
					next_state = Instruct_Waiting;
				else
					next_state = Instruct_Fecthing;
			end

			Instruct_Waiting: begin
				if (Inst_Valid)                         // having prepared valid instruction
					next_state = Instruct_Decoding;
				else
					next_state = Instruct_Waiting;
			end

			// no need to care about NOP, since it works as if nothing has happened
			Instruct_Decoding: begin
				next_state = Executing;
			end

			Executing: begin
				if (opcode[5:4] == 2'b00) // I-load
					next_state = Loading_Memory;
				else if (op_class_S) // Store type
					next_state = Storing_Memory;
				else if (op_class_B) // Branch type
					next_state = Instruct_Fecthing;
				else // R, I-type other, U, J
					next_state = Writing_Back;
			end

			Loading_Memory: begin
				if (Mem_Req_Ready)
					next_state = Read_Data_Waiting;
				else
					next_state = Loading_Memory;
			end

			Storing_Memory: begin
				if (Mem_Req_Ready)
					next_state = Instruct_Fecthing;
				else
					next_state = Storing_Memory;
			end

			Read_Data_Waiting: begin
				if (Read_data_Valid)
					next_state = Writing_Back;
				else
					next_state = Read_Data_Waiting;
			end

			Writing_Back: begin
				next_state = Instruct_Fecthing;
			end

			// Writing_Back is included in the "default" case, so we ommit this situation
			default:
				next_state = Init_State;
		endcase
	end

	// 3rd part: output signals

	// when the clock comes up
	always @(posedge clk) begin
		if (rst)
			PC_reg <= `ZERO_FIL;
		else if  (!rst && state_EX && PC_wen)
			PC_reg <= PC_jump_addr;
		else if  (!rst && state_EX && !PC_wen)
			PC_reg <= PC_next;
	end

	// store the next value of PC in default condition (PC + 4)
	always @(posedge clk) begin
		if (state_IW && Inst_Valid)
			PC_next <= alu_Result;
	end

	// dealing with reg_Inst
	always @(posedge clk) begin
		if (state_IW && Inst_Valid)
			reg_Inst <= Instruction;
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
		if (state_ID)
			reg_alu_shift_out <= alu_Result;
		else if (state_EX) begin
			if (opcode[4] && !opcode[2] && funct3[1:0] == 2'b01)
				reg_alu_shift_out <= shift_Result;
			else
				reg_alu_shift_out <= alu_Result;
		end
	end

	// CPU performance counters1

	// the number of cycles
	reg [31:0] cycle_cnt;
	always @(posedge clk) begin
		if (!rst)
			cycle_cnt <= cycle_cnt + 32'd1;
	end
	assign cpu_perf_cnt_0 = cycle_cnt;

	// the number of memory load requests
	reg [31:0] mem_load_time_cnt;
	always @(posedge clk) begin
		if (!rst && state_LD && Mem_Req_Ready)
			mem_load_time_cnt <= mem_load_time_cnt + 32'd1;
	end
	assign cpu_perf_cnt_1 = mem_load_time_cnt;

	// the number of memory store requests
	reg [31:0] mem_store_time_cnt;
	always @(posedge clk) begin
		if (!rst && state_ST && Mem_Req_Ready)
			mem_store_time_cnt <= mem_store_time_cnt + 32'd1;
	end
	assign cpu_perf_cnt_2 = mem_store_time_cnt;

	// the number of instructions
	reg [31:0] inst_cnt;
	always @(posedge clk) begin
		if (!rst && state_IF && Inst_Req_Ready)
			inst_cnt <= inst_cnt + 32'd1;
	end
	assign cpu_perf_cnt_3 = inst_cnt;

	// the number of memory loading cycles
	reg [31:0] mem_load_cycle_cnt;
	always @(posedge clk) begin
		if (!rst && (state_LD || state_RDW))
			mem_load_cycle_cnt <= mem_load_cycle_cnt + 32'd1;
	end
	assign cpu_perf_cnt_4 = mem_load_cycle_cnt;

	// the number of memory storing cycles
	reg [31:0] mem_store_cycle_cnt;
	always @(posedge clk) begin
		if (!rst && state_ST)
			mem_store_cycle_cnt <= mem_store_cycle_cnt + 32'd1;
	end
	assign cpu_perf_cnt_5 = mem_store_cycle_cnt;

	// the number of instruction fetching cycles
	reg [31:0] inst_req_cycle_cnt;
	always @(posedge clk) begin
		if (!rst && (state_IF || state_IW))
			inst_req_cycle_cnt <= inst_req_cycle_cnt + 32'd1;
	end
	assign cpu_perf_cnt_6 = inst_req_cycle_cnt;

	// the number of happened jumps
	reg [31:0] inst_jump_cnt;
	always @(posedge clk) begin
		if (!rst && state_EX && PC_wen)
			inst_jump_cnt <= inst_jump_cnt + 32'd1;
	end
	assign cpu_perf_cnt_7 = inst_jump_cnt;

endmodule
