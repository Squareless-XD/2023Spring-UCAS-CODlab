`timescale 10ns / 1ns

`define DATA_WIDTH  32 // data width of one data beat
`define State_Width 2  // width of state in the FSM


// here we want to use a simple moudule to do the branch prediction task

module branch_predictor (
        input         clk,
        input         rst,

        input  [31:0] PC_now, // PC at IF stage
        input  [31:0] PC_commit, // PC at EX stage
        input         PC_valid_commit, // PC at EX stage is valid
        input         taken_commit, // branch at EX stage is taken
        input         PC_imm_commit, // branch address at EX stage is immediate
        input  [31:0] branch_addr, // predicted next PC at EX stage

        output        taken_predict, // predict
        output [31:0] PC_predict // predicted next PC
);

        // local parameters
        parameter GHR_Width = 4; // width of global history register
        parameter PC_Inst_Hash_Width = 6; // width of PC hash function
        // parameter PC_Inst_Hash_Width = 6;

        // use 2-bit saturating counter to implement the FSM. encoded as follows:
        // 00: strongly not taken. 01: weakly not taken. 11: weakly taken. 10: strongly taken.
        parameter St_NT = 2'b00;
        parameter Wk_NT = 2'b01;
        parameter Wk_T  = 2'b11;
        parameter St_T  = 2'b10;

        // parameters derived from local parameters
        parameter GHR_Num          = (1 << GHR_Width); // number of entries in the global history register
        parameter PHT_Vec_Length   = (GHR_Num << 1); // length of the pattern history table vector, 2 * GHR_Num
        parameter PC_Inst_Hash_Num = (1 << PC_Inst_Hash_Width); // number of entries in the PC hash table
        // parameter PC_Inst_Hash_Num  = (1 << PC_Inst_Hash_Width); // number of branches in the pipeline
        parameter Branch_Tag_LEN   = (32 - PC_Inst_Hash_Width - 2); // tag of the branch hash table

        parameter Deafult_State = {GHR_Num{Wk_NT}}; // default state of the FSM.



        // registers
        reg [GHR_Width        - 1 : 0] GHR; // global history register
        reg [PHT_Vec_Length   - 1 : 0] PHT [PC_Inst_Hash_Num - 1 : 0]; // pattern history table
        reg [`DATA_WIDTH      - 1 : 0] PC_cache [PC_Inst_Hash_Num - 1 : 0]; // PC cache
        reg [Branch_Tag_LEN   - 1 : 0] PC_cache_tag [PC_Inst_Hash_Num - 1 : 0]; // the tag array of PC cache
        reg [PC_Inst_Hash_Num - 1 : 0] PC_cache_valid; // the tag array of PC cache
        reg [PC_Inst_Hash_Num - 1 : 0] PC_cache_evict; // annotating that PC cache is ready to be evicted

        // wires
        wire [PC_Inst_Hash_Width - 1 : 0] PC_inst_Hash_now; // PC hash value
        wire [PC_Inst_Hash_Width - 1 : 0] PC_inst_Hash_commit; // PC hash value
        wire [PC_Inst_Hash_Width - 1 : 0] branch_hash_now; // branch hash value
        wire [PC_Inst_Hash_Width - 1 : 0] branch_hash_commit; // branch hash value
        wire [`State_Width       - 1 : 0] current_state; // current state of the FSM
        wire [`State_Width       - 1 : 0] next_state; // next state of the FSM
        // wire [31:0] PC_branch_commit; // branch address
        wire tag_hit_now    = PC_cache_tag[branch_hash_now   ] == PC_now   [`DATA_WIDTH - 1 -: Branch_Tag_LEN]; // whether the tag is hit
        wire tag_hit_commit = PC_cache_tag[branch_hash_commit] == PC_commit[`DATA_WIDTH - 1 -: Branch_Tag_LEN]; // whether the tag is hit

        always @(posedge clk) begin
                if (rst) begin
                        GHR <= 0;
                end
                else if (PC_valid_commit & PC_imm_commit) begin
                        GHR <= {GHR[GHR_Width - 2 : 0], taken_commit};
                end
        end

        integer i;
        always @(posedge clk) begin
                if (rst) begin
                        for (i = 0; i < PC_Inst_Hash_Num; i = i + 1) begin
                                PHT[i] <= Deafult_State; // weakly taken (01) in default
                        end
                end
                else if (PC_cache_tag[branch_hash_commit] != PC_commit[`DATA_WIDTH - 1 -: Branch_Tag_LEN]) begin
                        PHT[branch_hash_commit][{GHR, 1'b0} +: `State_Width] <= Deafult_State; // weakly taken (01) in default
                        if ((branch_hash_commit != PC_inst_Hash_commit) & PC_valid_commit & PC_imm_commit) begin
                                PHT[PC_inst_Hash_commit][{GHR, 1'b0} +: `State_Width] <= next_state; // weakly taken (01) in default
                        end
                end
                else if (PC_valid_commit & PC_imm_commit) begin
                        PHT[PC_inst_Hash_commit][{GHR, 1'b0} +: `State_Width] <= next_state;
                end
        end

        always @(posedge clk) begin
                if (PC_valid_commit & PC_imm_commit) begin
                        if (~PC_cache_valid[branch_hash_commit] | ~tag_hit_commit & PC_cache_evict[branch_hash_commit]) begin
                                PC_cache[branch_hash_commit] <= branch_addr;
                                PC_cache_tag[branch_hash_commit] <= PC_commit[`DATA_WIDTH - 1 -: Branch_Tag_LEN];
                        end
                end
        end

        always @(posedge clk) begin
                if (rst) begin
                        PC_cache_valid <= 0;
                end
                else if (PC_valid_commit & PC_imm_commit) begin
                        PC_cache_valid[branch_hash_commit] <= 1;
                end
        end

        always @(posedge clk) begin
                if (rst) begin
                        PC_cache_evict <= 1;
                end
                else if (PC_valid_commit & PC_imm_commit) begin
                        if (tag_hit_commit | PC_cache_evict[branch_hash_commit]) begin
                                PC_cache_evict[branch_hash_commit] <= 0;
                        end
                        else begin
                                PC_cache_evict[branch_hash_commit] <= 1;
                        end
                end
        end

        assign PC_inst_Hash_now    = PC_now[2 +: PC_Inst_Hash_Width];
        assign PC_inst_Hash_commit = PC_commit[2 +: PC_Inst_Hash_Width];

        assign branch_hash_now    = PC_now[2 +: PC_Inst_Hash_Width];
        assign branch_hash_commit = PC_commit[2 +: PC_Inst_Hash_Width];
        

        assign current_state = PHT[PC_inst_Hash_commit][{GHR, 1'b0} +: `State_Width];
        assign next_state = taken_commit
                ? {`State_Width{current_state == St_NT}} & Wk_NT |
                  {`State_Width{current_state == Wk_NT}} & Wk_T  |
                  {`State_Width{current_state == Wk_T }} & St_T  |
                  {`State_Width{current_state == St_T }} & St_T
                : {`State_Width{current_state == St_NT}} & St_NT |
                  {`State_Width{current_state == Wk_NT}} & St_NT |
                  {`State_Width{current_state == Wk_T }} & Wk_NT |
                  {`State_Width{current_state == St_T }} & Wk_T
        ;

        assign taken_predict = tag_hit_now & PHT[PC_inst_Hash_now][{GHR, 1'b1}];
        assign PC_predict = PC_cache[branch_hash_now];


endmodule