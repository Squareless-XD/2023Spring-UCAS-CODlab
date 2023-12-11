`timescale 10ns / 1ns

`define CACHE_SET         8   // number of sets in the cache
`define CACHE_WAY         4   // number of ways in the cache
`define TAG_LEN           24  // length of tag
`define LINE_LEN          256 // length of one line (32 bytes of data)
`define DARRAY_DATA_WIDTH 256 // data width of data array
`define DARRAY_ADDR_WIDTH 3   // ----- same as `SET_ADDR_WIDTH -----
`define TARRAY_DATA_WIDTH 24  // data width of tag array
`define TARRAY_ADDR_WIDTH 3   // ----- same as `SET_ADDR_WIDTH -----
`define SET_ADDR_WIDTH    3   // address width of set index
`define BLOCK_OFF_WIDTH   5   // offset width of block set, 32 bytes in data array
`define TOTAL_MEM_ADDR    32  // total memory address
`define DATA_WIDTH        32  // data width of one data beat
`define ADDR_WIDTH        5   // address width of one data beat
`define DATA_NUM_IN_BLOCK 8   // number of data stored in one block
`define STATE_NUM_ICACHE  8  // number of states in the state machine

module icache_top (
        input              clk,
        input              rst,

        //CPU interface
        /** CPU instruction fetch request to Cache: valid signal */
        input         from_cpu_inst_req_valid,
        /** CPU instruction fetch request to Cache: address (4 byte alignment) */
        input  [31:0] from_cpu_inst_req_addr,
        /** Acknowledgement from Cache: ready to receive CPU instruction fetch request */
        output        to_cpu_inst_req_ready,

        /** Cache responses to CPU: valid signal */
        output        to_cpu_cache_rsp_valid,
        /** Cache responses to CPU: 32-bit Instruction value */
        output [31:0] to_cpu_cache_rsp_data,
        /** Acknowledgement from CPU: Ready to receive Instruction */
        input         from_cpu_cache_rsp_ready,

        //Memory interface (32 byte aligned address)
        /** Cache sending memory read request: valid signal */
        output        to_mem_rd_req_valid,
        /** Cache sending memory read request: address (32 byte alignment) */
        output [31:0] to_mem_rd_req_addr,
        /** Acknowledgement from memory: ready to receive memory read request */
        input         from_mem_rd_req_ready,

        /** Memory return read data: valid signal of one data beat */
        input         from_mem_rd_rsp_valid,
        /** Memory return read data: 32-bit one data beat */
        input  [31:0] from_mem_rd_rsp_data,
        /** Memory return read data: if current data beat is the last in this burst data transmission */
        input         from_mem_rd_rsp_last,
        /** Acknowledgement from cache: ready to receive current data beat */
        output        to_mem_rd_rsp_ready
);

//TODO: Please add your I-Cache code here

        // variables
        genvar i;
        integer j;

        // use tree-PLRU replacement policy to choose the way to evict
        reg [`CACHE_SET - 1 : 0] age_layer_1 [1:0]; // age in 1st layer, 1 for recently used compared with the other path in pair
        reg [`CACHE_SET - 1 : 0] age_layer_2      ; // age in 2nd layer, 1 for recently used compared with the other path in pair

        wire read_hit;

        // reg [`STATE_NUM_ICACHE - 1 : 0] current_state;
        // reg [`STATE_NUM_ICACHE - 1 : 0] next_state;

        // parameter State_Wait     = `STATE_NUM_ICACHE'b00000001;
        // parameter State_Tag_Rd   = `STATE_NUM_ICACHE'b00000010;
        // parameter State_Cache_Rd = `STATE_NUM_ICACHE'b00000100;
        // parameter State_Resp     = `STATE_NUM_ICACHE'b00001000;
        // parameter State_Evict    = `STATE_NUM_ICACHE'b00010000;
        // parameter State_Mem_Rd   = `STATE_NUM_ICACHE'b00100000;
        // parameter State_Recv     = `STATE_NUM_ICACHE'b01000000;
        // parameter State_Refill   = `STATE_NUM_ICACHE'b10000000;

        // wire WAIT     = current_state[0];
        // wire TAG_RD   = current_state[1];
        // wire CACHE_RD = current_state[2];
        // wire RESP     = current_state[3];
        // wire EVICT    = current_state[4];
        // wire MEM_RD   = current_state[5];
        // wire RECV     = current_state[6];
        // wire REFILL   = current_state[7];

        wire miss_disp = EVICT | MEM_RD | RECV | REFILL;
        wire hit_disp  = CACHE_RD;

        reg INIT;
        // reg WAIT;
        reg TAG_RD;
        reg CACHE_RD;
        reg RESP;
        reg EVICT;
        reg MEM_RD;
        reg RECV;
        reg REFILL;

        wire moveto_TAG_RD    = (~TAG_RD   | moveto_CACHE_RD | moveto_EVICT) & from_cpu_inst_req_valid & to_cpu_inst_req_ready;
        wire moveto_CACHE_RD  = (~CACHE_RD | moveto_RESP    ) & TAG_RD   & read_hit & ~miss_disp;
        wire moveto_RESP_hit  =  ~RESP                        & CACHE_RD;
        wire moveto_RESP_miss =  ~RESP                        & REFILL;
        wire moveto_EVICT     = (~EVICT    | moveto_MEM_RD  ) & TAG_RD   & ~read_hit & ~miss_disp & ~hit_disp;
        wire moveto_MEM_RD    = (~MEM_RD   | moveto_RECV    ) & EVICT;
        wire moveto_RECV      = (~RECV     | moveto_REFILL  ) & MEM_RD & from_mem_rd_req_ready;
        wire moveto_REFILL    = (~REFILL   | moveto_RESP    ) & RECV & from_mem_rd_rsp_valid & from_mem_rd_rsp_last; // to_mem_rd_rsp_ready is justRECV

        wire moveto_RESP = moveto_RESP_hit | moveto_RESP_miss;

        always @(posedge clk) begin
                if (rst) begin
                        INIT <= 1'b1;
                end
                else begin
                        INIT <= 1'b0;
                end
        end
        always @(posedge clk) begin
                if (INIT) begin
                        // WAIT     <= 1'b1;
                        TAG_RD   <= 1'b0;
                        CACHE_RD <= 1'b0;
                        RESP     <= 1'b0;
                        EVICT    <= 1'b0;
                        MEM_RD   <= 1'b0;
                        RECV     <= 1'b0;
                        REFILL   <= 1'b0;
                end
                else begin
                        // WAIT     <= 1'b1;
                        TAG_RD   <= moveto_TAG_RD   | TAG_RD & ~moveto_CACHE_RD & ~moveto_EVICT;
                        CACHE_RD <= moveto_CACHE_RD | CACHE_RD & ~moveto_RESP_hit;
                        RESP     <= moveto_RESP     | RESP & ~(from_cpu_cache_rsp_ready & to_cpu_cache_rsp_valid);
                        EVICT    <= moveto_EVICT    | EVICT & ~moveto_MEM_RD;
                        MEM_RD   <= moveto_MEM_RD   | MEM_RD & ~moveto_RECV;
                        RECV     <= moveto_RECV     | RECV & ~moveto_REFILL;
                        REFILL   <= moveto_REFILL   | REFILL & ~moveto_RESP_miss;
                end
        end

        // // calculate next state, based on current state and input signals (and so on)
        // always @(*) begin
        //         case (current_state)
        //                 State_Wait: begin
        //                         if (from_cpu_inst_req_valid) begin
        //                                 next_state = State_Tag_Rd;
        //                         end
        //                         else begin
        //                                 next_state = State_Wait;
        //                         end
        //                 end
        //                 State_Tag_Rd: begin
        //                         if (read_hit) begin
        //                                 next_state = State_Cache_Rd;
        //                         end
        //                         else begin // read miss
        //                                 next_state = State_Evict;
        //                         end
        //                 end
        //                 State_Cache_Rd: begin
        //                         next_state = State_Resp;
        //                 end
        //                 State_Resp: begin
        //                         if (from_cpu_cache_rsp_ready) begin
        //                                 next_state = State_Wait;
        //                         end
        //                         else begin
        //                                 next_state = State_Resp;
        //                         end
        //                 end
        //                 State_Evict: begin
        //                         next_state = State_Mem_Rd;
        //                 end
        //                 State_Mem_Rd: begin
        //                         if (from_mem_rd_req_ready) begin
        //                                 next_state = State_Recv;
        //                         end
        //                         else begin
        //                                 next_state = State_Mem_Rd;
        //                         end
        //                 end
        //                 State_Recv: begin
        //                         if (from_mem_rd_rsp_valid & from_mem_rd_rsp_last) begin
        //                                 next_state = State_Refill;
        //                         end
        //                         else begin
        //                                 next_state = State_Recv;
        //                         end
        //                 end
        //                 State_Refill: begin
        //                         next_state = State_Resp;
        //                 end
        //                 default:
        //                         next_state = State_Wait;
        //         endcase
        // end

        // // change state when clk rising edge
        // always @(posedge clk) begin
        //         if (rst) begin
        //                 current_state <= State_Wait;
        //         end
        //         else begin
        //                 current_state <= next_state;
        //         end
        // end


/*
        Consider State_Wait operations
        In State_Wait, we should check if there is a valid request from CPU
        If there is a valid request, we should go to State_Tag_Rd
        If there is no valid request, we should stay in State_Wait
        We need to take down the address of the request
*/

        reg [`DATA_WIDTH - 1 : 0] cpu_inst_addr_TAG_RD;

        assign to_cpu_inst_req_ready = INIT | (~TAG_RD | moveto_CACHE_RD | moveto_EVICT);

        always @(posedge clk) begin
                if (INIT) begin
                        cpu_inst_addr_TAG_RD <= `DATA_WIDTH'b0;
                end
                else begin
                        if (moveto_TAG_RD) begin
                                cpu_inst_addr_TAG_RD <= from_cpu_inst_req_addr;
                        end
                end
        end


/*
        Consider State_Tag_Rd operations
        In State_Tag_Rd, we should check if there is a read hit or read miss
        If there is a read hit, we should go to State_Cache_Rd
        If there is a read miss, we should go to State_Evict
        It will never happen that there is both a read hit and a read miss, or neither of them
        Therefore, we should set read_hit 1 or 0, by calculating the tag of the request, compared with set (also calculated)
        Then we need to take down the tag of the request
*/

        reg [`CACHE_WAY - 1 : 0] valid_arr   [`CACHE_SET - 1 : 0]; // store 8 sets as an array, each set has 4 ways
        // reg [`TAG_LEN   - 1 : 0] tag_set_r_reg [`CACHE_WAY - 1 : 0]; // tag array read from tag array, by a given set index
        reg [`CACHE_WAY - 1 : 0] read_hit_way_CACHE_RD; // check the data is hit in which way
        reg [`BLOCK_OFF_WIDTH - 1 : 0] block_offset_CACHE_RD; // block address of the request

        reg [`DATA_WIDTH - 1 : 0] inst_addr_EVICT; // store the address of the request, in EVICT state
        reg [`SET_ADDR_WIDTH - 1 : 0] set_idx_CACHE_RD; // store the index of set, in CACHE_RD state
        reg [`SET_ADDR_WIDTH - 1 : 0] set_idx_EVICT; // store the index of set, in EVICT state

        wire [`TAG_LEN         - 1 : 0] tag_req;      // tag of the request
        wire [`SET_ADDR_WIDTH  - 1 : 0] set_idx;      // set index of the request
        wire [`BLOCK_OFF_WIDTH - 1 : 0] block_offset; // block address of the request

        wire [`TAG_LEN   - 1 : 0] tag_set_r  [`CACHE_WAY - 1 : 0]; // tag read from tag array, by a given set index
        wire [`TAG_LEN   - 1 : 0] tag_set_w;  // tag to write into tag array

        // assignment of array_wen will be done in State_Refill
        wire [`CACHE_WAY - 1 : 0] array_wen; // write enable of each way in a given set
        wire [`CACHE_WAY - 1 : 0] read_hit_way; // check the data is hit in which way

        
        wire [`SET_ADDR_WIDTH  - 1 : 0] set_idx_REFILL;      // set index of the request


        // store the address of the request into registers (might be used in pipeline)
        always @(posedge clk) begin
                if (moveto_CACHE_RD) begin
                        set_idx_CACHE_RD <= set_idx;
                        read_hit_way_CACHE_RD <= read_hit_way;
                        block_offset_CACHE_RD <= block_offset;
                end
        end
        always @(posedge clk) begin
                if (moveto_EVICT) begin
                        inst_addr_EVICT <= cpu_inst_addr_TAG_RD;
                        set_idx_EVICT <= set_idx;
                end
        end

        // split the address into tag, set index and block offset
        // assign tag_req      = cpu_inst_addr_TAG_RD[                   `TOTAL_MEM_ADDR - 1 : `SET_ADDR_WIDTH + `BLOCK_OFF_WIDTH];
        // assign set_idx      = cpu_inst_addr_TAG_RD[`SET_ADDR_WIDTH + `BLOCK_OFF_WIDTH - 1 :                   `BLOCK_OFF_WIDTH];
        // assign block_offset = cpu_inst_addr_TAG_RD[                  `BLOCK_OFF_WIDTH - 1 :                                  0];
        assign {tag_req, set_idx, block_offset} = cpu_inst_addr_TAG_RD;

        // instantiate tag_array, each has 4 ways. use generate to generate them
        generate
                for (i = 0; i < `CACHE_WAY; i = i + 1) begin : tag_array_inst_generate
                        tag_array tag_array_inst (
                                .clk(clk),
                                .waddr(set_idx_REFILL),
                                .raddr(set_idx),
                                .wen(array_wen[i]),
                                .wdata(tag_set_w),
                                .rdata(tag_set_r[i])
                        );
                end
        endgenerate

        // Check if there is a read hit 4:0100
        // Then check if there is a tag match
        for (i = 0; i < `CACHE_WAY; i = i + 1) begin : read_hit_generate
                assign read_hit_way[i] = valid_arr[set_idx][i] & (tag_set_r[i] == tag_req);
        end
        assign read_hit = |read_hit_way; // if there is a tag match, then it is a read hit
        // note: if any bit of a vector is 1, then the vector is 1 in logical comparison
        // so can be writen in the form below

        // // store the tag of the request into registers (might be used in pipeline)
        // always @(posedge clk) begin
        //         if (rst) begin
        //                 for (j = 0; j < `CACHE_WAY; j = j + 1) begin
        //                         tag_set_r_reg[j] <= `TAG_LEN'b0;
        //                 end
        //         end
        //         else if (moveto_EVICT) begin
        //                 for (j = 0; j < `CACHE_WAY; j = j + 1) begin
        //                         tag_set_r_reg[j] <= tag_set_r[j];
        //                 end
        //         end
        // end


/*
        Consider State_Cache_Rd operations
        In State_Cache_Rd, we should read the data from the cache
        Then they should be taken down in a register
*/

        reg [`DATA_WIDTH - 1 : 0] data_result_reg; // data array read from data array, by a given set index

        wire [`LINE_LEN   - 1 : 0] data_rd_result_block; // data read from data array, hit in cache
        wire [7:0] shift_num_in_block; // number of bits to shift for data index, according to block_offset

        wire [`LINE_LEN  - 1 : 0] data_set_r [`CACHE_WAY - 1 : 0]; // data read from data array, by a given set index
        wire [`LINE_LEN  - 1 : 0] data_set_w; // data to write into data array


        // note: this wire above will also be used in REFILL

        // instantiate data_array, each has 4 ways. use generate to generate them
        generate
                for (i = 0; i < `CACHE_WAY; i = i + 1) begin : data_array_inst_generate
                        data_array data_array_inst (
                                .clk(clk),
                                .waddr(set_idx_REFILL),
                                .raddr(set_idx_CACHE_RD),
                                .wen(array_wen[i]),
                                .wdata(data_set_w),
                                .rdata(data_set_r[i])
                        );
                end
        endgenerate

        // calculate the index of the way in which the data is hit
        assign data_rd_result_block = data_set_r[0] & {`LINE_LEN{read_hit_way_CACHE_RD[0]}}
                                    | data_set_r[1] & {`LINE_LEN{read_hit_way_CACHE_RD[1]}}
                                    | data_set_r[2] & {`LINE_LEN{read_hit_way_CACHE_RD[2]}}
                                    | data_set_r[3] & {`LINE_LEN{read_hit_way_CACHE_RD[3]}}
        ;

        // calculate the number of bits to shift for data index, according to block_offset
        assign shift_num_in_block = {block_offset_CACHE_RD, 3'b0}; // block_offset * 8

        // store the data of the request into registers
        // note: this register will also be used in REFILL
        always @(posedge clk) begin
                if (moveto_RESP_hit) begin
                        data_result_reg <= data_rd_result_block[shift_num_in_block +: `DATA_WIDTH];
                end
                else if (moveto_RESP_miss) begin
                        data_result_reg <= data_recv_reg[data_rd_idx_REFILL +: `DATA_WIDTH];
                end
        end

        // modify the age of the way in which the data is hit, according to the tree-PLRU replacement policy
        always @(posedge clk) begin
                if (rst) begin
                        age_layer_1[0] <= `CACHE_SET'b0;
                end
                else if (moveto_RESP_hit & (read_hit_way_CACHE_RD[0] | read_hit_way_CACHE_RD[1])) begin
                        age_layer_1[0][set_idx_CACHE_RD] <= read_hit_way_CACHE_RD[1];
                end
        end
        always @(posedge clk) begin
                if (rst) begin
                        age_layer_1[1] <= `CACHE_SET'b0;
                end
                else if (moveto_RESP_hit & (read_hit_way_CACHE_RD[2] | read_hit_way_CACHE_RD[3])) begin
                        age_layer_1[1][set_idx_CACHE_RD] <= read_hit_way_CACHE_RD[3];
                end
        end
        always @(posedge clk) begin
                if (rst) begin
                        age_layer_2 <= `CACHE_SET'b0;
                end
                else if (moveto_RESP_hit) begin
                        age_layer_2[set_idx_CACHE_RD] <= read_hit_way_CACHE_RD[2] | read_hit_way_CACHE_RD[3];
                end
        end


/*
        Consider State_Resp operations
        In State_Resp, we should send the data to CPU
        First we should check if CPU is ready to receive the data
        If CPU is ready, we should send the data to CPU
        If CPU is not ready, we should wait until CPU is ready
*/

        assign to_cpu_cache_rsp_valid = ~rst & RESP;
        assign to_cpu_cache_rsp_data  = data_result_reg;


/*
        Consider State_Evict operations
        In State_Evict, we should calculate the way to evict (use valid_arr signalal, and age of each way)
        Then we should write the data and tag into the cache (reset corresponding valid as 0)
        If one way's valid_arr signal is 0, then it is the way to evict (start from lower bits)
        If not, check whether all the regs in its path is not poingting it, if so, then it is the way to evict
*/

        // store the way to evict
        reg [`CACHE_WAY - 1 : 0] way_to_evict_MEM_RD; // way that will be evicted, in MEM_RD state
        reg [`DATA_WIDTH - 1 : 0] inst_addr_MEM_RD; // store the address of the request, in MEM_RD state

        wire [`CACHE_WAY - 1 : 0] LRU_way_temp;     // the least recently used way
        wire [`CACHE_WAY - 1 : 0] LRU_way;          // the least recently used way
        wire [`CACHE_WAY - 1 : 0] inv_way_1st_temp; // the first way that is invalid
        wire [`CACHE_WAY - 1 : 0] inv_way_1st;      // the first way that is invalid
        wire [`CACHE_WAY - 1 : 0] way_to_evict; // way that will be evicted

        // store the way to evict
        // store the address of the request into registers (might be used in pipeline)
        always @(posedge clk) begin
                if (moveto_MEM_RD) begin
                        way_to_evict_MEM_RD <= way_to_evict;
                        inst_addr_MEM_RD <= inst_addr_EVICT;
                end
        end

        // calculate the LRU and FIV way
        assign LRU_way_temp = {
                ~age_layer_2[set_idx_EVICT] & ~age_layer_1[1][set_idx_EVICT],
                ~age_layer_2[set_idx_EVICT] &  age_layer_1[1][set_idx_EVICT],
                 age_layer_2[set_idx_EVICT] & ~age_layer_1[0][set_idx_EVICT],
                 age_layer_2[set_idx_EVICT] &  age_layer_1[0][set_idx_EVICT]
        };
        // assign LRU_way = {
        //         LRU_way_temp[3] & ~LRU_way_temp[2] & ~LRU_way_temp[1] & ~LRU_way_temp[0],
        //         LRU_way_temp[2] & ~LRU_way_temp[1] & ~LRU_way_temp[0],
        //         LRU_way_temp[1] & ~LRU_way_temp[0],
        //         LRU_way_temp[0]
        // };
        prio_selector #(`CACHE_WAY) prio_selector_LRU_way (
                .data_raw(LRU_way_temp),
                .data_sel(LRU_way)
        );

        assign inv_way_1st_temp = ~valid_arr[set_idx_EVICT];
        // assign inv_way_1st = {
        //         inv_way_1st_temp[3] & ~inv_way_1st_temp[2] & ~inv_way_1st_temp[1] & ~inv_way_1st_temp[0],
        //         inv_way_1st_temp[2] & ~inv_way_1st_temp[1] & ~inv_way_1st_temp[0],
        //         inv_way_1st_temp[1] & ~inv_way_1st_temp[0],
        //         inv_way_1st_temp[0]
        // };
        prio_selector #(`CACHE_WAY) prio_selector_inv_way_1st (
                .data_raw(inv_way_1st_temp),
                .data_sel(inv_way_1st)
        );

        // if there is no invalid way, then choose the LRU way. Otherwise, choose the first invalid way
        // apparently, there's no need to evict a block with its valid '0', 
        // THAT'S BECAUSE we we will USE THIS SIGNAL in REFILL.
        // assign way_to_evict = {`CACHE_WAY{  &(valid_arr[set_idx_EVICT]) }} & LRU_way
        //                     | {`CACHE_WAY{~(&(valid_arr[set_idx_EVICT]))}} & inv_way_1st;
        assign way_to_evict = (&(valid_arr[set_idx_EVICT])) ? LRU_way : inv_way_1st;

        // evict the chosen way, by setting its valid signal as 0
        // note: this operation will be done in REFILL
        always @(posedge clk) begin
                if (rst) begin
                        for (j = 0; j < `CACHE_SET; j = j + 1) begin
                                valid_arr[j] <= `CACHE_WAY'b0;
                        end
                end
                else if (moveto_MEM_RD) begin
                        valid_arr[set_idx_EVICT] <= valid_arr[set_idx_EVICT] & ~way_to_evict;
                end
                else if (moveto_RESP_miss) begin
                        valid_arr[set_idx_REFILL] <= valid_arr[set_idx_REFILL] | way_to_evict_REFILL;
                end
        end


/*
        Consider State_Mem_Rd operations
        In State_Mem_Rd, we should send the memory read request to real memory
        Then we should wait until memory is ready to receive the request
        At the same time, the address of cache block should be given out (the lowest bit of the block in memory)
        This address should be ended with 5'b0, because the address of cache block is 32 byte aligned
*/

        // store the way to evict
        reg [`CACHE_WAY - 1 : 0] way_to_evict_RECV; // way that will be evicted, in RECV state
        reg [`DATA_WIDTH - 1 : 0] inst_addr_RECV; // store the address of the request, in RECV state

        // store the way to evict
        // store the address of the request into registers (might be used in pipeline)
        always @(posedge clk) begin
                if (moveto_RECV) begin
                        way_to_evict_RECV <= way_to_evict_MEM_RD;
                        inst_addr_RECV <= inst_addr_MEM_RD;
                end
        end

        assign to_mem_rd_req_valid = ~rst & MEM_RD;
        assign to_mem_rd_req_addr  = {inst_addr_MEM_RD[31:5], 5'b0}; // cpu_inst_addr_TAG_RD[31:5] * 32, in pipeline


/*
        Consider State_Recv operations
        In State_Recv, we should receive the data from memory
        Use a 256-bit register to store the data. Each time, we should store 32-bit data into the register
        Then we should check if the current data beat is the last one in this burst data transmission
        If so, we should go to State_Refill (already written in state machine)
*/

        reg [`LINE_LEN - 1 : 0] data_recv_reg;          // data received from memory, 256-bit
        reg [2:0] data_recv_cnt; // number of data beat received from memory, 8-bit
        // store the way to evict
        reg [`CACHE_WAY - 1 : 0] way_to_evict_REFILL; // way that will be evicted, in REFILL state
        reg [`DATA_WIDTH - 1 : 0] inst_addr_REFILL; // store the address of the request, in REFILL state

        // store the data beat into the register. no need to reset the register
        always @(posedge clk) begin
                if (RECV & from_mem_rd_rsp_valid & to_mem_rd_rsp_ready) begin
                        data_recv_reg[{data_recv_cnt, 5'b0} +: `DATA_WIDTH] <= from_mem_rd_rsp_data;
                end
        end

        // count the number of data beat received from memory
        always @(posedge clk) begin
                if (INIT | RECV & from_mem_rd_rsp_valid & to_mem_rd_rsp_ready & from_mem_rd_rsp_last) begin
                        data_recv_cnt <= 3'b0;
                end
                else if (RECV & from_mem_rd_rsp_valid & to_mem_rd_rsp_ready) begin
                        data_recv_cnt <= data_recv_cnt + 1;
                end
        end

        // store the way to evict
        // store the address of the request into registers (might be used in pipeline)
        always @(posedge clk) begin
                if (moveto_REFILL) begin
                        way_to_evict_REFILL <= way_to_evict_RECV;
                        inst_addr_REFILL <= inst_addr_RECV;
                end
        end

        // calculate the number of bits to shift for data index, according to data_recv_cnt
        // assign data_recv_shift_num = data_recv_cnt << 5; // data_recv_cnt * 32

        assign to_mem_rd_rsp_ready = INIT | RECV;


/*
        Consider State_Refill operations
        In State_Refill, we should write the data and tag into the cache (set corresponding valid as 1)
*/

        wire [`TAG_LEN         - 1 : 0] tag_req_REFILL;      // tag of the request
        // set_idx_REFILL declared in TAG_RD section
        wire [`BLOCK_OFF_WIDTH - 1 : 0] block_offset_REFILL; // block address of the request

        wire [7:0] data_rd_idx_REFILL; // index of the data read from data array, hit in cache

        // split the address into tag, set index and block offset
        // assign tag_req_REFILL      = inst_addr_REFILL[`TOTAL_MEM_ADDR  - 1 : `SET_ADDR_WIDTH + `BLOCK_OFF_WIDTH];
        // assign set_idx_REFILL      = inst_addr_REFILL[`SET_ADDR_WIDTH + `BLOCK_OFF_WIDTH - 1 :                   `BLOCK_OFF_WIDTH];
        // assign block_offset_REFILL = inst_addr_REFILL[`BLOCK_OFF_WIDTH - 1 :                                  0];
        assign {tag_req_REFILL, set_idx_REFILL, block_offset_REFILL} = inst_addr_REFILL;

        // calculate the index of the data read from data array
        assign data_rd_idx_REFILL = {block_offset_REFILL, 3'b0}; // block_offset * 8

        // store the cache block into data array. only write when in REFILL state
        assign array_wen  = {`CACHE_WAY{REFILL}} & way_to_evict_REFILL;
        assign tag_set_w  = tag_req_REFILL;
        assign data_set_w = data_recv_reg;

        // calculate the valid signal of each way in a given set
        // NOTE: THIS TASK IS WRITTEN IN EVICT SECTION

        // send the data read to a register, preparing for sending to CPU
        // NOTE: THIS TASK IS WRITTEN IN CACHE_RD SECTION

endmodule


// module data_array(
//         input                             clk,
//         input  [`DARRAY_ADDR_WIDTH - 1:0] waddr,
//         input  [`DARRAY_ADDR_WIDTH - 1:0] raddr,
//         input                             wen,
//         input  [`DARRAY_DATA_WIDTH - 1:0] wdata,
//         output [`DARRAY_DATA_WIDTH - 1:0] rdata
// );
// endmodule
// module tag_array(
//         input                             clk,
//         input  [`TARRAY_ADDR_WIDTH - 1:0] waddr,
//         input  [`TARRAY_ADDR_WIDTH - 1:0] raddr,
//         input                             wen,
//         input  [`TARRAY_DATA_WIDTH - 1:0] wdata,
//         output [`TARRAY_DATA_WIDTH - 1:0] rdata
// );
// endmodule

// module prio_selector_recur #(
//         parameter Data_Size = 3
// )(
//         input  [Data_Size - 1 : 0] data_raw,
//         input  sel_neg,
//         output [Data_Size - 1 : 0] data_sel
// );
// endmodule
// module prio_selector #(
//         parameter Data_Size = 4
// )(
//         input  [Data_Size - 1 : 0] data_raw,
//         output [Data_Size - 1 : 0] data_sel
// );
// endmodule
// module prio_selector_4_recur #(
//         parameter Data_Size = 4
// )(
//         input  [Data_Size - 1 : 0] data_raw,
//         input  sel_neg,
//         output [Data_Size - 1 : 0] data_sel
// );
// endmodule
// module prio_selector_4 #(
//         parameter Data_Size = 16
// )(
//         input  [Data_Size - 1 : 0] data_raw,
//         output [Data_Size - 1 : 0] data_sel
// );
// endmodule
