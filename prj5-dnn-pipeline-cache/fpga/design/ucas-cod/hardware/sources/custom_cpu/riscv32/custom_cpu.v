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

// `define _PERF_CNT_ENABLE_1 1'b1
// `define _PERF_CNT_ENABLE_2 1'b1
`define _AUTO_HALT 1'b1
// `define _MUL_ENABLE 1'b1

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
	// wire [69:0] inst_retire;

// TODO: Please add your custom CPU code here


`ifndef _AUTO_HALT
	assign inst_retire = {RF_wen, RF_waddr, RF_wdata, PC_WB};
`else
	assign inst_retire = halt_and_catch_fire[0] ? 70'b0
			   : halt_and_catch_fire[1] ? {{6{1'b1}}, last_retire_time, {8{1'b0}}, {4{1'b1}}, retire_num}
			   : ((~rst & valid_WB) ? {RF_wen, RF_waddr, RF_wdata, PC_WB} : 70'b0);

	parameter Stop_Cycle = 32'h300000;
	wire [1:0] halt_and_catch_fire;
	reg [31:0] halt_cnt; // at most 4e9==4*10^9
	always @(posedge clk) begin
		if (rst)
			halt_cnt <= 32'b0;
		else
			halt_cnt <= halt_cnt + 1;
	end
	assign halt_and_catch_fire = {halt_cnt == (Stop_Cycle + 1), halt_cnt == Stop_Cycle};

	reg [31:0] last_retire_time;
	always @(posedge clk) begin
		last_retire_time <= halt_cnt;
	end

	reg [19:0] retire_num;
	always @(posedge clk) begin
		if (rst) begin
			retire_num <= 20'b0;
		end
		else if (RF_wen & RF_waddr != 5'd0) begin
			retire_num <= retire_num + 1;
		end
	end
`endif


	// -------------------------------------
	// -------------------------------------
	// registers with _"SECTION" means that this register will be used in the "SECTION" section
	// like "RF_wdata_ctrl_EX" measn it will be used in EX section
	// -------------------------------------
	// -------------------------------------


	// register write signals
	wire        RF_wen;
	wire [4:0]  RF_waddr;
	wire [31:0] RF_wdata;

	// register read
	wire [ 4:0] RF_raddr1;
	wire [ 4:0] RF_raddr2;
	wire [31:0] RF_rdata1;
	wire [31:0] RF_rdata2;

	// register write data when using I-load instructions
	wire [31:0] RF_wpartial;

	// multiplication
	`ifdef _MUL_ENABLE
		wire [63:0] mul_Result; // the result of multiplication
	`endif

	// alu control signals
	wire [31:0] alu_A;
	wire [31:0] alu_B;
	reg  [ 2:0] alu_ALUop;
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

	reg erase_IF1;
	reg erase_IF2;
	reg erase_IF3;
	reg erase_IF4;
	reg valid_IF2;
	reg valid_IF3;
	reg valid_IF4;
	reg valid_ID;
	reg valid_EX;
	reg valid_MEM;
	reg valid_RDW;
	reg valid_WB;

	always @(posedge clk) begin
		if (INIT)
			erase_IF1 <= 0;
		else if (moveto_IF2)
			erase_IF1 <= 0;
		else if (PC_wen | reg_PC_wen)
			erase_IF1 <= 1;
	end
	always @(posedge clk) begin
		if (INIT)
			erase_IF2 <= 0;
		else if (PC_wen | reg_PC_wen)
			erase_IF2 <= 1;
		else if (moveto_IF2)
			erase_IF2 <= erase_IF1;
	end
	always @(posedge clk) begin
		if (INIT)
			erase_IF3 <= 0;
		else if (PC_wen | reg_PC_wen)
			erase_IF3 <= 1;
		else if (moveto_IF3)
			erase_IF3 <= erase_IF2;
	end
	always @(posedge clk) begin
		if (INIT)
			erase_IF4 <= 0;
		else if (PC_wen | reg_PC_wen)
			erase_IF4 <= 1;
		else if (moveto_IF4)
			erase_IF4 <= erase_IF3;
	end

	always @(posedge clk) begin
		if (INIT)
			valid_IF2 <= 0;
		else if (moveto_IF2)
			valid_IF2 <= 1;
		else if (moveto_IF3)
			valid_IF2 <= 0;
	end
	always @(posedge clk) begin
		if (INIT)
			valid_IF3 <= 0;
		else if (moveto_IF3)
			valid_IF3 <= valid_IF2;
		else if (moveto_IF4)
			valid_IF3 <= 0;
	end
	always @(posedge clk) begin
		if (INIT)
			valid_IF4 <= 0;
		else if (moveto_IF4)
			valid_IF4 <= valid_IF3;
		else if (moveto_ID)
			valid_IF4 <= 0;
	end
	always @(posedge clk) begin
		if (INIT | PC_wen | reg_PC_wen | erase_IF4) // if PC_wen is true, then PC will not be the predicted one "PC+4".
			valid_ID <= 0;
		else if (moveto_ID)
			valid_ID <= valid_IF4;
		else if (moveto_EX)
			valid_ID <= 0; // section down, set 0.
	end
	always @(posedge clk) begin
		if (INIT)
			valid_EX <= 0;
		else if (moveto_EX)
			valid_EX <= valid_ID;
		else if (moveto_MEM)
			valid_EX <= 0;
	end
	always @(posedge clk) begin
		if (INIT)
			valid_MEM <= 0;
		else if (moveto_MEM)
			valid_MEM <= valid_EX;
		else if (moveto_RDW)
			valid_MEM <= 0;
	end
	always @(posedge clk) begin
		if (INIT)
			valid_RDW <= 0;
		else if (moveto_RDW)
			valid_RDW <= valid_MEM;
		else if (moveto_WB)
			valid_RDW <= 0;
	end
	always @(posedge clk) begin
		if (INIT)
			valid_WB <= 0;
		else if (moveto_WB)
			valid_WB <= valid_RDW;
		else
			valid_WB <= 0;
	end

	// only one type of hazard: RAW.
	// three situations: a Store inst, and any one of the 3 insts below is Read inst.
	// this could be detected at the end of the latter inst's ID section.
	// note: opcode appears to be the last 7 bit of the wire: "Instruction", so they share the same bit index.
	// and "opcode" is used in EX section

        wire hazard_ID_cond = ~(PC_wen & ~EX_wait_cycle) & valid_ID;
        wire hazard_rs1_cond = (|rs1) & ~(op_class_U | op_class_J);
        wire hazard_rs2_cond = (|rs2) & (op_class_B | op_class_S | op_class_R);

	// only cancel hazard when PC_wen is high and at EX's first cycle
	// 1st. rd_EX same as rs1_ID, with RF_wen 1 in EX and RF_rs1 is read
	wire hazard_EX_rs1  = hazard_ID_cond & (rd_EX  == rs1) & hazard_rs1_cond & RF_wen_EX  & valid_EX ;
	wire hazard_MEM_rs1 = hazard_ID_cond & (rd_MEM == rs1) & hazard_rs1_cond & RF_wen_MEM & valid_MEM;
	wire hazard_RDW_rs1 = hazard_ID_cond & (rd_RDW == rs1) & hazard_rs1_cond & RF_wen_RDW & valid_RDW;
	wire hazard_WB_rs1  = hazard_ID_cond & (rd_WB  == rs1) & hazard_rs1_cond & RF_wen     & valid_WB ;
	wire hazard_EX_rs2  = hazard_ID_cond & (rd_EX  == rs2) & hazard_rs2_cond & RF_wen_EX  & valid_EX ;
	wire hazard_MEM_rs2 = hazard_ID_cond & (rd_MEM == rs2) & hazard_rs2_cond & RF_wen_MEM & valid_MEM;
	wire hazard_RDW_rs2 = hazard_ID_cond & (rd_RDW == rs2) & hazard_rs2_cond & RF_wen_RDW & valid_RDW;
	wire hazard_WB_rs2  = hazard_ID_cond & (rd_WB  == rs2) & hazard_rs2_cond & RF_wen     & valid_WB ;
	wire hazard_EX_Load_pre  = hazard_EX_Load  & (hazard_EX_rs1  | hazard_EX_rs2 ); // hazard with the next inst
	wire hazard_MEM_Load_pre = hazard_MEM_Load & (hazard_MEM_rs1 | hazard_MEM_rs2); // hazard with the next inst
	wire hazard_RDW_Load_pre = hazard_RDW_Load & (hazard_RDW_rs1 | hazard_RDW_rs2); // hazard with the next inst
	wire hazard_EX_Load  = ~(opcode_EX[5]  | opcode_EX[4] ); // hazard with the next inst
	wire hazard_MEM_Load = ~(opcode_MEM[5] | opcode_MEM[4]); // hazard with the next inst
	wire hazard_RDW_Load = ~(opcode_RDW[5] | opcode_RDW[4]); // hazard with the next inst

	// This is the most significant signal, controlling the global movement of this sequenced pipeline.
	// If this signal is set (1), then when the clk comes to the postive edge, the whole pipeline will be moved forward.
	// And this signal should be set only if all the pipeline sections have done their calculation.

	wire moveto_IF2 = ~INIT & Inst_Req_Ready & Inst_Req_Valid;
	wire moveto_IF3 = moveto_IF4 | ~valid_IF3;
	wire moveto_IF4 = moveto_ID | ~valid_IF4;
	wire moveto_ID = Inst_Ready & Inst_Valid; // IF needs waiting

	wire moveto_EX =
		(moveto_MEM | ~valid_EX) & ~PC_wen &
		~hazard_EX_Load_pre & ~hazard_MEM_Load_pre & ~hazard_RDW_Load_pre
	; // ID is in 1 cycle

	wire moveto_MEM = moveto_RDW | ~valid_MEM; // EX is in 1 cycle.

	wire moveto_RDW = (
		(MemRead | MemWrite) & Mem_Req_Ready | ~valid_MEM |
		~((~opcode_MEM[5] & ~opcode_MEM[4]) | (~opcode_MEM[6] & opcode_MEM[5] & ~opcode_MEM[4]))
	) & (moveto_WB | ~valid_RDW); // MEM needs waiting

	wire moveto_WB = (
		Read_data_Ready & Read_data_Valid |
		~(~opcode_RDW[5] & ~opcode_RDW[4]) | ~valid_RDW
	); // RDW needs to wait, iff section is valid and is Load inst.







	// %%%%%%%%%%%%%%%%
	// Registers
	// %%%%%%%%%%%%%%%%

	// register in custom_cpu
	reg [31:0] reg_Inst_ID;
	reg [31:0] reg_Memory;
	reg [31:0] reg_RF_rdata1;
	reg [31:0] reg_RF_rdata2;
	reg [31:0] reg_alu_shift_out;
	`ifdef _MUL_ENABLE
		reg [63:0] reg_mul;
	`endif
	reg [31:0] immediate_EX;

	reg RF_wen_EX;
	reg RF_wen_MEM;
	reg RF_wen_RDW;
	reg RF_wen_WB;

	reg [1:0] RF_wdata_ctrl_EX; // this means that this register will be used in the EX section
	reg [1:0] RF_wdata_ctrl_MEM;
	reg [1:0] RF_wdata_ctrl_RDW;
	reg [1:0] RF_wdata_ctrl_WB;

	reg [31:0] RF_wdata_MEM_part; // used in MEM section
	reg [31:0] RF_wdata_RDW_part; // used in RDW section

	reg [31:0] PC_reg;       // the register storing PC
	reg [1:0] PC_jump_addr_ctrl; // used in EX section

	reg [31:0] reg_PC_true_addr; // the true PC value but not taken in prediction

	reg reg_PC_wen; // temporarily store PC_wen, when PC_wen==1 but Inst_Valid has not come yet

	reg [1:0] alu_A_ctrl; // used in EX section
	reg       alu_B_ctrl; // used in EX section

	reg [1:0] bytes_sel_RDW;
	reg [1:0] bytes_sel_WB;

	reg [31:0] PC_next_IF2;
	reg [31:0] PC_next_IF3;
	reg [31:0] PC_next_IF4;
	reg [31:0] PC_next_ID;
	reg [31:0] PC_next_EX;
	reg [31:0] PC_next_MEM;
	reg [31:0] PC_next_RDW;
	reg [31:0] PC_next_WB;

	reg [31:0] PC_IF2;
	reg [31:0] PC_IF3;
	reg [31:0] PC_IF4;
	reg [31:0] PC_ID;
	reg [31:0] PC_EX;
	reg [31:0] PC_MEM;
	reg [31:0] PC_RDW;
	reg [31:0] PC_WB;

	reg [6:0] funct7_EX;
	reg [4:0] shamt_EX;
	reg [2:0] funct3_EX;
	reg [4:0] rd_EX;
	reg [6:0] opcode_EX;

	reg [2:0] funct3_MEM;
	reg [4:0] rd_MEM;
	reg [6:0] opcode_MEM;

	reg [2:0] funct3_RDW; // almost usedless
	reg [4:0] rd_RDW;
	reg [6:0] opcode_RDW;

	reg [2:0] funct3_WB;
	reg [4:0] rd_WB;

	reg [31:0] reg_alu_shift_out_RDW;
	reg [31:0] reg_alu_shift_out_WB;

	reg PC_wen_0_arg; // used in EX section
	reg PC_wen_1_arg; // used in EX section

	reg PC_imm_commit; // used in EX section (produced in ID stage)
	reg taken_predict_IF2;
	reg taken_predict_IF3;
	reg taken_predict_IF4;
	reg taken_predict_ID;
	reg taken_predict_EX;
	reg [31:0] PC_predict_IF2;
	reg [31:0] PC_predict_IF3;
	reg [31:0] PC_predict_IF4;
	reg [31:0] PC_predict_ID;
	reg [31:0] PC_predict_EX;

	`ifdef _MUL_ENABLE
		reg [63:0] reg_mul_RDW;
		reg [63:0] reg_mul_WB;
	`endif

	reg [31:0] reg_RF_rdata2_MEM;

	reg EX_wait_cycle;



	// %%%%%%%%%%%%%%%%
	// IF1 section
	// %%%%%%%%%%%%%%%%

	// whether to read from or write data in the memory, and other handshaking signals
	reg current_state;
	parameter state_Normal = 1'b0;
	parameter state_Init   = 1'b1;

	wire INIT = current_state; //rename of current_state

	always @(posedge clk) begin
		if (rst) begin
			current_state <= state_Init;
		end
		else begin
			current_state <= state_Normal;
		end
	end

	// control the output of PC
	always @(posedge clk) begin
		if (INIT) begin
			PC_reg <= 32'b0;
		end
		else if (moveto_IF2) begin
			if  (reg_PC_wen) begin
				PC_reg <= reg_PC_true_addr;
			end
			else if (PC_wen) begin
				PC_reg <= PC_true_addr;
			end
			else begin
				PC_reg <= PC_next_into_IF;
			end
		end
	end

	// give a predicted PC value
	wire [31:0] PC_next_into_IF = taken_predict ? PC_predict : PC_next; // have changed

	// store the next value of PC in default condition (PC + 4) in IF/IW section, will be used in ID
	wire [31:0] PC_next = PC_reg + 4;

	// opcode[6:5] == 2'b11 means this instruction is a branch(jump)
	wire PC_valid_commit = opcode_EX[6] & opcode_EX[5] & valid_EX & ~EX_wait_cycle; // used in branch predictor
	wire taken_commit = PC_wen_old; // used in branch predictor
	wire taken_predict; // used in branch predictor
	wire [31:0] PC_predict; // used in branch predictor

	always @(posedge clk) begin
		if (INIT | PC_wen_over)
			reg_PC_wen <= 0;
		else if (PC_wen)
			reg_PC_wen <= 1; // set to 1 only when PC_wen is going to disappear
	end
	wire PC_wen_over = reg_PC_wen & moveto_IF2; // check when the IF/IW section is over. Among waiting, this section will be kept as normal. 
	// above are related with EX section

	// instantiate a branch predictor
	branch_predictor branch_predictor_inst (
		.clk 		 (clk),
		.rst 		 (rst),

		.PC_now 	 (PC), // PC at IF stage
		.PC_commit 	 (PC_EX), // PC at EX stage
		.PC_valid_commit (PC_valid_commit), // PC at EX stage is valid
		.taken_commit 	 (taken_commit), // branch at EX stage is taken
		.PC_imm_commit 	 (PC_imm_commit), // branch address at EX stage is immediate
		.branch_addr 	 (PC_jump_addr), // predicted next PC at EX stage

		.taken_predict 	 (taken_predict), // predict
		.PC_predict 	 (PC_predict) // predicted next PC
	);

	always @(posedge clk) begin
		if (moveto_IF2) begin
			PC_next_IF2 <= PC_next;
			PC_IF2 <= PC_reg;

			taken_predict_IF2 <= taken_predict;
			PC_predict_IF2 <= PC_predict;
		end
	end





	// %%%%%%%%%%%%%%%%
	// IF2 section
	// %%%%%%%%%%%%%%%%







	always @(posedge clk) begin
		if (moveto_IF3) begin
			PC_next_IF3 <= PC_next_IF2;
			PC_IF3 <= PC_IF2;

			taken_predict_IF3 <= taken_predict_IF2;
			PC_predict_IF3 <= PC_predict_IF2;
		end
	end




	// %%%%%%%%%%%%%%%%
	// IF3 section
	// %%%%%%%%%%%%%%%%






	always @(posedge clk) begin
		if (moveto_IF4) begin
			PC_next_IF4 <= PC_next_IF3;
			PC_IF4 <= PC_IF3;

			taken_predict_IF4 <= taken_predict_IF3;
			PC_predict_IF4 <= PC_predict_IF3;
		end
	end




	// %%%%%%%%%%%%%%%%
	// IF4 section
	// %%%%%%%%%%%%%%%%





	always @(posedge clk) begin
		if (moveto_ID) begin
			PC_next_ID <= PC_next_IF4;
			PC_ID <= PC_IF4;
			reg_Inst_ID <= Instruction;

			taken_predict_ID <= taken_predict_IF4;
			PC_predict_ID <= PC_predict_IF4;
		end
	end



	// %%%%%%%%%%%%%%%%
	// ID section
	// %%%%%%%%%%%%%%%%

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

	wire [2:0] imm_ctrl = {3{opcode[4] &  opcode[2]             }} & U_Type
			    | {3{opcode[3]                          }} & J_Type
			    | {3{opcode[6] & ~opcode[2]             }} & B_Type
			    | {3{~opcode[6] & opcode[5] & ~opcode[4]}} & S_Type
			    | {3{opcode[5] & opcode[4] & ~opcode[2] }} & R_Type
	; // others are I-type

	// used in ID section
	wire [31:0] immediate = {
		reg_Inst_ID[31],
		op_class_U ? reg_Inst_ID[30:20] : {11{reg_Inst_ID[31]}},      // U
		imm_ctrl[1] ? reg_Inst_ID[19:12] : {8{reg_Inst_ID[31]}},      // U,J
		op_class_B & reg_Inst_ID[7] | op_class_J & reg_Inst_ID[20] | (imm_ctrl[2] == imm_ctrl[0]) & reg_Inst_ID[31], // B / J / I,S
		{6{~op_class_U}} & reg_Inst_ID[30:25],                        // not U
		{4{~imm_ctrl[1] & imm_ctrl[0]}} & reg_Inst_ID[11:8] | {4{~imm_ctrl[0]}} & reg_Inst_ID[24:21], // S,B / I,J
		op_class_I & reg_Inst_ID[20] | op_class_S & reg_Inst_ID[7]    // I / S
	};

	// used in ID section
	wire [6:0] funct7 = reg_Inst_ID[31:25]; //  7 bits
	wire [4:0] shamt  = reg_Inst_ID[24:20]; //  5 bits
	wire [4:0] rs2    = reg_Inst_ID[24:20]; //  5 bits
	wire [4:0] rs1    = reg_Inst_ID[19:15]; //  5 bits
	wire [2:0] funct3 = reg_Inst_ID[14:12]; //  3 bits
	wire [4:0] rd     = reg_Inst_ID[11: 7]; //  5 bits
	wire [6:0] opcode = reg_Inst_ID[ 6: 0]; //  7 bits


	// register file write data
	parameter RF_Wdata_Res  = 2'b00; // for it is zero, no need to assign condition for it
	parameter RF_Wdata_Next = 2'b01;
	parameter RF_Wdata_Read = 2'b10;
	`ifdef _MUL_ENABLE
		parameter RF_Wdata_Mul  = 2'b11; // M-extension
	`endif

	// assign in ID part
	wire [1:0] RF_wdata_ctrl_ID = {2{~opcode[4] &  opcode[2]}} & RF_Wdata_Next // J-type, JALR(I-jump)
				    | {2{~opcode[5] & ~opcode[4]}} & RF_Wdata_Read // I-load
				`ifdef _MUL_ENABLE
				    | {2{funct7[0]  & op_class_R}} & RF_Wdata_Mul  // R-type and Multiply/divide
				`endif
	; // PLUS BY: U, I-calculate, R-type. note Multiply

	// register file read address.
	assign RF_raddr1 = rs1; // read in ID
	assign RF_raddr2 = rs2; // read in ID

	// an "unextendable" way to handle with the ALUops of calculation instructions
	// R-type:   [5&1&(~3|~0), ~2,  3|2&0]
	// I-calcul: [1&~0,  ~2,  ~2&1|2&0]
	parameter Alu_ALUop_Plus = 2'b00; // for it is zero, no need to assign condition for it
	parameter Alu_ALUop_R    = 2'b01;
	parameter Alu_ALUop_B    = 2'b10;
	parameter Alu_ALUop_ICal = 2'b11;

	// below are calculated in EX section
	wire [2:0] alu_ALUop_R_type = {
		(funct3[2] ^ funct3[1]) & ~funct3[0] | funct7[5],
		~funct3[2],
		funct3[1] & ~(funct3[2] & funct3[0])
	}; // maybe here can be modified into faster solution
	wire [2:0] alu_ALUop_I_CAL = {
		(funct3[2] ^ funct3[1]) & ~funct3[0],
		~funct3[2],
		funct3[1] & ~(funct3[2] & funct3[0])
	};
	wire [2:0] alu_ALUop_B_type = {~funct3[1], 1'b1, funct3[2]}; // maybe here can be modified into faster solution

	wire [1:0] alu_ALUop_ctrl = {2{op_class_R                         }} & Alu_ALUop_R    // R-type
				  | {2{op_class_B                         }} & Alu_ALUop_B    // Branch
				  | {2{~opcode[5] & opcode[4] & ~opcode[2]}} & Alu_ALUop_ICal // I-calculation
	;

	// control alu input
	parameter Alu_A_Rdata1 = 2'b00; // for it is zero, no need to assign condition for it
	parameter Alu_A_PC     = 2'b10;
	parameter Alu_A_Zero   = 2'b11;

	// parameter Alu_B_Rdata2   = 1'b1;
	// parameter Alu_B_Sign_Ext = 1'b0; // for it is zero, no need to assign condition for it
	// calculated in ID section

	// for PC_branch and PC_next, actually I can use multi-cycle to reuse alu, but, plz put it later.
	parameter [1:0] PC_Jump_JAL = 2'b01;
	parameter [1:0] PC_Jump_JALR = 2'b10;
	parameter [1:0] PC_Jump_B = 2'b11;


	// dealing with reg_RF_rdata. used in EX section
	wire [31:0] reg_RF_rdata1_prep =
		{`DATA_WIDTH{hazard_EX_rs1                                                        }} & RF_wdata_MEM_part_prep |
		{`DATA_WIDTH{~hazard_EX_rs1 &  hazard_MEM_rs1 & ~hazard_MEM_Load                  }} & RF_wdata_MEM_part |
		{`DATA_WIDTH{~hazard_EX_rs1 & ~hazard_MEM_rs1 &  hazard_RDW_rs1 & ~hazard_RDW_Load}} & RF_wdata_RDW_part |
		{`DATA_WIDTH{~hazard_EX_rs1 & ~hazard_MEM_rs1 & ~hazard_RDW_rs1 & hazard_WB_rs1   }} & RF_wdata |
		{`DATA_WIDTH{
			~hazard_EX_rs1 & hazard_MEM_rs1 & hazard_MEM_Load | 
			~hazard_EX_rs1 & ~hazard_MEM_rs1 & hazard_RDW_rs1 & hazard_RDW_Load |
			~hazard_EX_rs1 & ~hazard_MEM_rs1 & ~hazard_RDW_rs1 & ~hazard_WB_rs1
		}} & RF_rdata1
	;
	wire [31:0] reg_RF_rdata2_prep =
		{`DATA_WIDTH{hazard_EX_rs2                                                        }} & RF_wdata_MEM_part_prep |
		{`DATA_WIDTH{~hazard_EX_rs2 &  hazard_MEM_rs2 & ~hazard_MEM_Load                  }} & RF_wdata_MEM_part |
		{`DATA_WIDTH{~hazard_EX_rs2 & ~hazard_MEM_rs2 &  hazard_RDW_rs2 & ~hazard_RDW_Load}} & RF_wdata_RDW_part |
		{`DATA_WIDTH{~hazard_EX_rs2 & ~hazard_MEM_rs2 & ~hazard_RDW_rs2 & hazard_WB_rs2   }} & RF_wdata |
		{`DATA_WIDTH{
			~hazard_EX_rs2 & hazard_MEM_rs2 & hazard_MEM_Load | 
			~hazard_EX_rs2 & ~hazard_MEM_rs2 & hazard_RDW_rs2 & hazard_RDW_Load |
			~hazard_EX_rs2 & ~hazard_MEM_rs2 & ~hazard_RDW_rs2 & ~hazard_WB_rs2
		}} & RF_rdata2
	;

	reg hazard_EX_rs1_reg;
	reg hazard_EX_rs2_reg;

	always @(posedge clk) begin
		if (moveto_EX) begin
			hazard_EX_rs1_reg <= hazard_EX_rs1;
			hazard_EX_rs2_reg <= hazard_EX_rs2;
		end
	end

	// register file write enable. opcode is used in ID, so this register is used in EX
	always @(posedge clk) begin
		if (INIT) begin
			RF_wen_EX <= 0;

			PC_wen_0_arg <= 0;
			PC_wen_1_arg <= 0;
		end
		else if (moveto_EX) begin
			RF_wen_EX <= ~opcode[5] | opcode[4] | opcode[2]; // except Branch or Store

			PC_wen_0_arg <= opcode[6] & opcode[5] & (opcode[2] |  (funct3[2] ^ funct3[0]));
			PC_wen_1_arg <= opcode[6] & opcode[5] & (opcode[2] | ~(funct3[2] ^ funct3[0]));
		end
	end
	always @(posedge clk) begin
		if (moveto_EX) begin
			alu_A_ctrl <= {2{~opcode[5] & opcode[2] | opcode[3]}} & Alu_A_PC   // AUIPC, JALR
				    | {2{ opcode[5] & opcode[4] & opcode[2]}} & Alu_A_Zero // LUI
			;
			alu_B_ctrl <= op_class_B | op_class_R; // B-type, R-type
			alu_ALUop <= {1'b0, ~alu_ALUop_ctrl[1] & ~alu_ALUop_ctrl[0], 1'b0}           // others
				   | {3{~alu_ALUop_ctrl[1] &  alu_ALUop_ctrl[0]}} & alu_ALUop_R_type // R-type
				   | {3{ alu_ALUop_ctrl[1] & ~alu_ALUop_ctrl[0]}} & alu_ALUop_B_type // Branch
				   | {3{ alu_ALUop_ctrl[1] &  alu_ALUop_ctrl[0]}} & alu_ALUop_I_CAL  // I-calculation
			;

			RF_wdata_ctrl_EX <= RF_wdata_ctrl_ID;
			PC_next_EX <= PC_next_ID;
			PC_EX <= PC_ID;

			funct7_EX <= funct7;
			shamt_EX  <= shamt;
			funct3_EX <= funct3;
			rd_EX     <= rd;
			opcode_EX <= opcode;

			immediate_EX <= immediate;
			PC_imm_commit <= op_class_B | op_class_J;
			taken_predict_EX <= taken_predict_ID;
			PC_predict_EX <= PC_predict_ID;

			PC_jump_addr_ctrl <= {2{op_class_J            }} & PC_Jump_JAL  // J-type
					   | {2{~opcode[3] & opcode[2]}} & PC_Jump_JALR // JALR
					   | {2{op_class_B            }} & PC_Jump_B    // Branch type
			;

			reg_RF_rdata1 <= reg_RF_rdata1_prep;
                        reg_RF_rdata2 <= reg_RF_rdata2_prep;
		end
	end
	// always @(posedge clk) begin
	// 	if (moveto_EX) begin
	// 		if (~hazard_EX_rs1 & hazard_MEM_rs1 & ~hazard_MEM_Load) begin
	// 			reg_RF_rdata1 <= RF_wdata_MEM_part;
	// 		end
	// 		else if (~hazard_EX_rs1 & ~hazard_MEM_rs1 &  hazard_RDW_rs1 & ~hazard_RDW_Load) begin
	// 			reg_RF_rdata1 <= RF_wdata_RDW_part;
	// 		end
	// 		else if (~hazard_EX_rs1 & ~hazard_MEM_rs1 & ~hazard_RDW_rs1 & hazard_WB_rs1) begin
	// 			reg_RF_rdata1 <= RF_wdata;
	// 		end
	// 		else begin
	// 			reg_RF_rdata1 <= RF_rdata1;
	// 		end
	// 	end
	// end
	// always @(posedge clk) begin
	// 	if (moveto_EX) begin
	// 		if (~hazard_EX_rs2 & hazard_MEM_rs2 & ~hazard_MEM_Load) begin
	// 			reg_RF_rdata2 <= RF_wdata_MEM_part;
	// 		end
	// 		else if (~hazard_EX_rs2 & ~hazard_MEM_rs2 &  hazard_RDW_rs2 & ~hazard_RDW_Load) begin
	// 			reg_RF_rdata2 <= RF_wdata_RDW_part;
	// 		end
	// 		else if (~hazard_EX_rs2 & ~hazard_MEM_rs2 & ~hazard_RDW_rs2 & hazard_WB_rs2) begin
	// 			reg_RF_rdata2 <= RF_wdata;
	// 		end
	// 		else begin
	// 			reg_RF_rdata2 <= RF_rdata2;
	// 		end
	// 	end
	// end


	// instantiate a register file
	reg_file reg_file_inst (
		.clk    (clk      ),
		.waddr  (RF_waddr ),
		.raddr1 (RF_raddr1),
		.raddr2 (RF_raddr2),
		.wen    (RF_wen   ),
		.wdata  (RF_wdata ),
		.rdata1 (RF_rdata1),
		.rdata2 (RF_rdata2)
	);




	// %%%%%%%%%%%%%%%%
	// EX section
	// %%%%%%%%%%%%%%%%

	wire [31:0] PC_branch = PC_EX + immediate_EX; // a new add sentence, used in EX section

	wire [31:0] PC_jump_addr = {32{~PC_jump_addr_ctrl[1] &  PC_jump_addr_ctrl[0]}} & alu_Result               // J-type
				 | {32{ PC_jump_addr_ctrl[1] & ~PC_jump_addr_ctrl[0]}} & {alu_Result[31:1], 1'b0} // JALR
				 | {32{ PC_jump_addr_ctrl[1] &  PC_jump_addr_ctrl[0]}} & PC_branch                // Branch type
	;

	// the true PC value but not taken in prediction used in EX section
	wire [31:0] PC_true_addr = PC_wen_old ? PC_jump_addr : PC_next_EX;

	// alu was calculating in EX section

	// assign PC_wen = opcode_EX[6] & opcode_EX[5] & (opcode_EX[2] | (funct3_EX[2] ^ funct3_EX[0] ^ alu_Zero));
	/*
		reg PC_wen_1;
		reg PC_wen_2;
		reg PC_wen_3;
		always @(posedge clk) begin
			PC_wen_1 <= opcode[6] & opcode[5] & opcode[2];
			PC_wen_2 <= opcode[6] & opcode[5] &  (funct3[2] ^ funct3[0]);
			PC_wen_3 <= opcode[6] & opcode[5] & ~(funct3[2] ^ funct3[0]);
		end
		assign PC_wen = PC_wen_1 | PC_wen_2 & ~alu_Zero | PC_wen_3 & alu_Zero;
	*/
	// wire PC_wen_old = PC_wen_0_arg & ~alu_Zero | PC_wen_1_arg & alu_Zero; // used in EX section (without branch prediction)
	wire PC_wen_old = alu_Zero ? PC_wen_1_arg : PC_wen_0_arg; // used in EX section (without branch prediction)

	/*
		Note:
		After adding a branch predictor, we now view use "PC_wen" as a signal that indicates whether the PC should be changed.
		Before, we set "PC_wen" whenever branch actually happens.
		But now, we set "PC_wen" in two cases:
			1. branch doesn't happen, and predictor took the branch.
			2. branch happens, and the predictor didn't take the branch (choose PC+4).
		Both cases will lead to a wrong PC, so we need to reset the predictor.

		Though the ideal situation is as above, hash function in branch predictor will lead to a wrong prediction.
		Therefore, we need to check the PC's value, and if mismatch, PC_wen should be set.
	*/
	/*
		When PC_wen is set, two step will be performed:
		1. We stop the pipeline section: IF/IW and ID by moveto_XX signal, and set reg_PC_wen to 1.
		In the end of this sycle, valid_ID will be reset 0, and PC_jump_addr will be stored.
		2. Now we will check if the IF/IW can be moved to ID section. If so, we will reset reg_PC_wen,
		and reset valid_ID (actually this is the just-finished IF/IW section), controlled by PC_wen_over.
		At the same time, PC_jump_addr will be put into IF/IW section, beginning the new branch.
		Interestingly though, after the stage, we need to keep EX section must be true.
	*/
	wire PC_wen = valid_EX & (
		PC_wen_old & ~taken_predict_EX | 				// 1.
		~PC_wen_old & taken_predict_EX | 				// 2.
		PC_wen_old & taken_predict_EX & (PC_predict_EX != PC_jump_addr) // miss in cache (inside of branch predictor)
	); // used in EX section (with branch prediction)

	// multiplication
	`ifdef _MUL_ENABLE
		assign mul_Result = reg_RF_rdata1 * reg_RF_rdata2; // calculated in EX
	`endif

	assign alu_A = {32{~alu_A_ctrl[1] & ~alu_A_ctrl[0]}} & reg_RF_rdata1
		     | {32{ alu_A_ctrl[1] & ~alu_A_ctrl[0]}} & PC_EX
	; // also alu_A == 32'b0 when alu_A_ctrl == 2'b11

	assign alu_B = alu_B_ctrl ? reg_RF_rdata2 : immediate_EX;
	
	// shifter port
	assign shift_A = reg_RF_rdata1;
	assign shift_B = opcode_EX[5] ? reg_RF_rdata2[4:0] : shamt_EX[4:0];
	assign shift_Shiftop = {funct3_EX[2], funct7_EX[5]};

	wire [31:0] reg_alu_shift_prep = (opcode_EX[4] & ~opcode_EX[2] & ~funct3_EX[1] & funct3_EX[0]) ? shift_Result : alu_Result; // calculated in EX
	wire [31:0] RF_wdata_MEM_part_prep = {32{~RF_wdata_ctrl_EX[1] & ~RF_wdata_ctrl_EX[0]}} & reg_alu_shift_prep
					   | {32{~RF_wdata_ctrl_EX[1] &  RF_wdata_ctrl_EX[0]}} & PC_next_EX
					`ifdef _MUL_ENABLE
					   | {32{ RF_wdata_ctrl_EX[1] &  RF_wdata_ctrl_EX[0]}} & mul_Result[31:0]
					`endif
	; // calculated in EX
	always @(posedge clk) begin
		if (moveto_MEM) begin
			RF_wdata_MEM_part <= RF_wdata_MEM_part_prep;
		end
	end

	always @(posedge clk) begin
		if (valid_EX & ~EX_wait_cycle) begin
			reg_PC_true_addr <= PC_true_addr;
		end
	end

	// judge whether the EX section is locked (in the last cycle)
	always @(posedge clk) begin
		if (rst | moveto_MEM)
			EX_wait_cycle <= 0;
		else if (valid_EX)
			EX_wait_cycle <= 1;
	end

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


	always @(posedge clk) begin
		if (moveto_MEM) begin
			RF_wen_MEM <= RF_wen_EX;
			RF_wdata_ctrl_MEM <= RF_wdata_ctrl_EX;

			PC_next_MEM <= PC_next_EX;
			PC_MEM <= PC_EX;

			funct3_MEM <= funct3_EX;
			rd_MEM     <= rd_EX;
			opcode_MEM <= opcode_EX;

			// if (opcode_EX[4] & ~opcode_EX[2] & ~funct3_EX[1] & funct3_EX[0]) begin // shift
			// 	reg_alu_shift_out <= shift_Result;
			// end
			// else begin
			// 	reg_alu_shift_out <= alu_Result;
			// end
			reg_alu_shift_out <= reg_alu_shift_prep;

			`ifdef _MUL_ENABLE
				reg_mul <= mul_Result;
			`endif
			reg_RF_rdata2_MEM <= reg_RF_rdata2; // a wire that choose the read data signal in EX
		end
	end



	// %%%%%%%%%%%%%%%%
	// MEM section
	// %%%%%%%%%%%%%%%%



	// Below, bytes_sel is assigned in MEM section, where EX has stored its result into a register.
	// This result is the last 2 bit of the result --- which is actually the PC+offset address --- to show the cpu the specified address
	// since the memory only supports addresses that are the multiple of 4-byte (address itself is multiple of 4)

	// for selecting bytes when in I-load or I-store instruction
	assign bytes_sel = reg_alu_shift_out[1:0]; // assigned in MEM section
	assign remain_0 = ~bytes_sel[1] & ~bytes_sel[0]; // assigned in MEM section
	assign remain_1 = ~bytes_sel[1] &  bytes_sel[0]; // assigned in MEM section
	assign remain_2 =  bytes_sel[1] & ~bytes_sel[0]; // assigned in MEM section
	assign remain_3 =  bytes_sel[1] &  bytes_sel[0]; // assigned in MEM section


	always @(posedge clk) begin
		if (moveto_RDW) begin
			RF_wen_RDW <= RF_wen_MEM;
			RF_wdata_ctrl_RDW <= RF_wdata_ctrl_MEM;
			RF_wdata_RDW_part <= RF_wdata_MEM_part;

			bytes_sel_RDW <= bytes_sel;

			PC_next_RDW <= PC_next_MEM;
			PC_RDW <= PC_MEM;

			funct3_RDW <= funct3_MEM;
			rd_RDW     <= rd_MEM;
			opcode_RDW <= opcode_MEM;
			reg_alu_shift_out_RDW <= reg_alu_shift_out;
			
			`ifdef _MUL_ENABLE
				reg_mul_RDW <= reg_mul;
			`endif
		end
	end


	// %%%%%%%%%%%%%%%%
	// RDW section
	// %%%%%%%%%%%%%%%%

	always @(posedge clk) begin
		if (moveto_WB) begin
			RF_wen_WB <= RF_wen_RDW; // only when section is valid, and not in reset state.
			RF_wdata_ctrl_WB <= RF_wdata_ctrl_RDW;

			bytes_sel_WB <= bytes_sel_RDW;
			PC_next_WB <= PC_next_RDW;
			PC_WB <= PC_RDW;

			funct3_WB <= funct3_RDW;
			rd_WB     <= rd_RDW;

			reg_Memory <= Read_data;
			reg_alu_shift_out_WB <= reg_alu_shift_out_RDW;

			`ifdef _MUL_ENABLE
				reg_mul_WB <= reg_mul_RDW;
			`endif
		end
	end





	// %%%%%%%%%%%%%%%%
	// WB section
	// %%%%%%%%%%%%%%%%

	assign RF_wen = RF_wen_WB & valid_WB & ~rst;
	// register file write address
	assign RF_waddr = rd_WB;
	// assign in WB part, three clock cycles later
	assign RF_wdata = {32{~RF_wdata_ctrl_WB[1] & ~RF_wdata_ctrl_WB[0]}} & reg_alu_shift_out_WB
			| {32{~RF_wdata_ctrl_WB[1] &  RF_wdata_ctrl_WB[0]}} & PC_next_WB
			| {32{ RF_wdata_ctrl_WB[1] & ~RF_wdata_ctrl_WB[0]}} & RF_wpartial
		`ifdef _MUL_ENABLE
			| {32{ RF_wdata_ctrl_WB[1] &  RF_wdata_ctrl_WB[0]}} & reg_mul_WB[31:0]
		`endif
	;

	assign RF_wpartial = {32{~funct3_WB[2] & ~funct3_WB[1] & ~funct3_WB[0]}} & {{24{load_byte[ 7]}}, load_byte} // LB
			   | {32{ funct3_WB[2] & ~funct3_WB[1] & ~funct3_WB[0]}} & { 24'b0             , load_byte} // LBU
			   | {32{~funct3_WB[2] & ~funct3_WB[1] &  funct3_WB[0]}} & {{16{load_half[15]}}, load_half} // LH
			   | {32{ funct3_WB[2] & ~funct3_WB[1] &  funct3_WB[0]}} & { 16'b0             , load_half} // LHU
			   | {32{~funct3_WB[2] &  funct3_WB[1] & ~funct3_WB[0]}} & reg_Memory                       // LW
	;

	// used for load instructions to select write_data for register file
	wire [ 7:0] load_byte; // load a byte
	wire [15:0] load_half; // load a half word
	assign load_byte = {8{~bytes_sel_WB[1] & ~bytes_sel_WB[0]}} & reg_Memory[ 7: 0]
			 | {8{~bytes_sel_WB[1] &  bytes_sel_WB[0]}} & reg_Memory[15: 8]
			 | {8{ bytes_sel_WB[1] & ~bytes_sel_WB[0]}} & reg_Memory[23:16]
			 | {8{ bytes_sel_WB[1] &  bytes_sel_WB[0]}} & reg_Memory[31:24]
	;
	assign load_half = {16{~bytes_sel_WB[1] & ~bytes_sel_WB[0]}} & reg_Memory[15: 0]
			 | {16{ bytes_sel_WB[1] & ~bytes_sel_WB[0]}} & reg_Memory[31:16]
	;


	// %%%%%%%%%%%%%%%%
	// output signals
	// %%%%%%%%%%%%%%%%

	// assign several outputs
	// if this instruction cannot be used for the next pipeline stage, then cpu won't send the handshake request
	assign Inst_Req_Valid  = ~rst & (moveto_IF3 | ~valid_IF2);
	assign Inst_Ready      = INIT | valid_IF4 & (moveto_EX | ~valid_ID);
	assign MemWrite        = ~rst & ~opcode_MEM[6] & opcode_MEM[5] & ~opcode_MEM[4] & valid_MEM; // Store
	assign MemRead         = ~rst & ~opcode_MEM[5] & ~opcode_MEM[4] & valid_MEM;  // Load
	assign Read_data_Ready = INIT | ~opcode_RDW[5] & ~opcode_RDW[4] & valid_RDW; // Load

	// which data should be writen in the memory, and where should it be written
	// note: only consider natural aligned memory address
	assign Address    = {reg_alu_shift_out[31:2], 2'b00};

	assign Write_data = {32{~funct3_MEM[1] & ~funct3_MEM[0]}} & {4{reg_RF_rdata2_MEM[ 7:0]}}
			  | {32{~funct3_MEM[1] &  funct3_MEM[0]}} & {2{reg_RF_rdata2_MEM[15:0]}}
			  | {32{ funct3_MEM[1] & ~funct3_MEM[0]}} &    reg_RF_rdata2_MEM
	;

	assign Write_strb = {4{~funct3_MEM[1] & ~funct3_MEM[0]}} & {remain_3, remain_2, remain_1, remain_0}
		  	  | {4{~funct3_MEM[1] &  funct3_MEM[0]}} & {remain_2, remain_2, remain_0, remain_0}
			  | {4{ funct3_MEM[1] & ~funct3_MEM[0]}} // & 4'b1111
	;

	// output PC as in the register.
	assign PC = PC_reg; // have changed



`ifdef _PERF_CNT_ENABLE_1
	// CPU performance counters1

	// the number of cycles
	reg [31:0] cycle_cnt_lo;
	always @(posedge clk) begin
		if (~INIT & (cycle_cnt_lo != 999999999)) begin
			cycle_cnt_lo <= cycle_cnt_lo + 32'd1;
		end
		else if (~INIT & (cycle_cnt_lo == 999999999)) begin
			cycle_cnt_lo <= 32'd0;
		end
	end
	assign cpu_perf_cnt_0 = cycle_cnt_lo;

	// the number of memory load requests
	reg [31:0] mem_load_times_cnt;
	always @(posedge clk) begin
		if (~INIT & Mem_Req_Ready & MemRead) begin
			mem_load_times_cnt <= mem_load_times_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_1 = mem_load_times_cnt;

	// the number of memory store requests
	reg [31:0] mem_store_times_cnt;
	always @(posedge clk) begin
		if (~INIT & Mem_Req_Ready & MemWrite) begin
			mem_store_times_cnt <= mem_store_times_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_2 = mem_store_times_cnt;

	// the number of instructions
	reg [31:0] inst_cnt;
	always @(posedge clk) begin
		if (~INIT & valid_WB) begin
			inst_cnt <= inst_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_3 = inst_cnt;

	// the number of memory loading cycles
	reg [31:0] mem_load_cycle_cnt;
	always @(posedge clk) begin
		if (~INIT & valid_RDW & Read_data_Ready) begin
			mem_load_cycle_cnt <= mem_load_cycle_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_4 = mem_load_cycle_cnt;

	// the number of all instructions got loaded
	reg [31:0] inst_all_cnt;
	always @(posedge clk) begin
		if (~INIT & Inst_Ready & Inst_Valid) begin
			inst_all_cnt <= inst_all_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_5 = inst_all_cnt;

	// the number of instruction fetching cycles
	reg [31:0] inst_req_cycle_cnt;
	always @(posedge clk) begin
		if (~INIT & (Inst_Req_Ready & Inst_Req_Valid | Inst_Ready)) begin
			inst_req_cycle_cnt <= inst_req_cycle_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_6 = inst_req_cycle_cnt;

	// the number of happened jumps
	reg [31:0] inst_jump_cnt;
	always @(posedge clk) begin
		if (~INIT & PC_wen) begin
			inst_jump_cnt <= inst_jump_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_7 = inst_jump_cnt;

	// the number of happened jumps
	reg [31:0] cycle_cnt_hi;
	always @(posedge clk) begin
		if (~INIT & (cycle_cnt_lo == 999999999)) begin
			cycle_cnt_hi <= cycle_cnt_hi + 32'd1;
		end
	end
	assign cpu_perf_cnt_8 = cycle_cnt_hi;
`endif
`ifdef _PERF_CNT_ENABLE_2
	// calculate all prediction times
	reg [31:0] total_branch_pred_num;
	always @(posedge clk) begin
		if (~INIT & taken_predict) begin
			total_branch_pred_num <= total_branch_pred_num + 32'd1;
		end
	end
	assign cpu_perf_cnt_1 = total_branch_pred_num;

	// the number of PC renewed (wrongly predicted)
	reg [31:0] total_branch_pred_wrong_num;
	always @(posedge clk) begin
		if (~INIT & PC_wen) begin
			total_branch_pred_wrong_num <= total_branch_pred_wrong_num + 32'd1;
		end
	end
	assign cpu_perf_cnt_2 = total_branch_pred_wrong_num;

	// the number of PC wront prediction, guess to be branch but not branch
	reg [31:0] total_branch_pred_wrong_num_1;
	always @(posedge clk) begin
		if (~INIT & valid_EX & PC_wen_old & ~taken_predict_EX) begin
			total_branch_pred_wrong_num_1 <= total_branch_pred_wrong_num_1 + 32'd1;
		end
	end
	assign cpu_perf_cnt_3 = total_branch_pred_wrong_num_1;

	// the number of PC wront prediction, guess to be not branch but branch
	reg [31:0] total_branch_pred_wrong_num_2;
	always @(posedge clk) begin
		if (~INIT & valid_EX & ~PC_wen_old & taken_predict_EX) begin
			total_branch_pred_wrong_num_2 <= total_branch_pred_wrong_num_2 + 32'd1;
		end
	end
	assign cpu_perf_cnt_4 = total_branch_pred_wrong_num_2;

	// the number of PC wront prediction, guess to be branch, but address is wrong
	reg [31:0] total_branch_pred_wrong_num_3;
	always @(posedge clk) begin
		if (~INIT & valid_EX & PC_wen_old & taken_predict_EX & (PC_predict_EX != PC_jump_addr)) begin
			total_branch_pred_wrong_num_3 <= total_branch_pred_wrong_num_3 + 32'd1;
		end
	end
	assign cpu_perf_cnt_5 = total_branch_pred_wrong_num_3;
`endif


endmodule


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
// module shifter (
// 	input  [31:0] A,
// 	input  [ 4:0] B,
// 	input  [ 1:0] Shiftop,
// 	output [31:0] Result
// );
// endmodule
// module branch_predictor (
//         input         clk,
//         input         rst,

//         input  [31:0] PC_now, // PC at IF stage
//         input  [31:0] PC_commit, // PC at EX stage
//         input         PC_valid_commit, // PC at EX stage is valid
//         input         taken_commit, // branch at EX stage is taken
//         input         PC_imm_commit, // branch address at EX stage is immediate
//         input  [31:0] branch_addr, // predicted next PC at EX stage

//         output        taken_predict, // predict
//         output [31:0] PC_predict // predicted next PC
// );
// endmodule
