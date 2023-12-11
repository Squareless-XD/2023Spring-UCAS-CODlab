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
`define STATE_NUM_DCACHE  11  // number of states in the state machine

module dcache_top (
	input	      clk,
	input	      rst,

	//CPU interface
	/** CPU memory/IO access request to Cache: valid signal */
	input         from_cpu_mem_req_valid,
	/** CPU memory/IO access request to Cache: 0 for read; 1 for write*/
	input         from_cpu_mem_req,
	/** CPU memory/IO access request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_mem_req_addr,
	/** CPU memory/IO access request to Cache: 32-bit write data */
	input  [31:0] from_cpu_mem_req_wdata,
	/** CPU memory/IO access request to Cache: 4-bit write strobe */
	input  [ 3:0] from_cpu_mem_req_wstrb,
	/** Acknowledgement from Cache: ready to receive CPU memory access request */
	output        to_cpu_mem_req_ready,

	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit read data */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive read data */
	input         from_cpu_cache_rsp_ready,

	//Memory/IO read interface
	/** Cache sending memory/IO read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address
	  * 4 byte alignment for I/O read 
	  * 32 byte alignment for cache read miss */
	output [31:0] to_mem_rd_req_addr,
        /** Cache sending memory read request: burst length
	  * 0 for I/O read (read only one data beat)
	  * 7 for cache read miss (read eight data beats) */
	output [ 7:0] to_mem_rd_req_len,
        /** Acknowledgement from memory: ready to receive memory read request */
	input	      from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input	      from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input	      from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready,

	//Memory/IO write interface
	/** Cache sending memory/IO write request: valid signal */
	output        to_mem_wr_req_valid,
	/** Cache sending memory write request: address
	  * 4 byte alignment for I/O write 
	  * 4 byte alignment for cache write miss
          * 32 byte alignment for cache write-back */
	output [31:0] to_mem_wr_req_addr,
        /** Cache sending memory write request: burst length
          * 0 for I/O write (write only one data beat)
          * 0 for cache write miss (write only one data beat)
          * 7 for cache write-back (write eight data beats) */
	output [ 7:0] to_mem_wr_req_len,
        /** Acknowledgement from memory: ready to receive memory write request */
	input         from_mem_wr_req_ready,

	/** Cache sending memory/IO write data: valid signal for current data beat */
	output        to_mem_wr_data_valid,
	/** Cache sending memory/IO write data: current data beat */
	output [31:0] to_mem_wr_data,
	/** Cache sending memory/IO write data: write strobe
	  * 4'b1111 for cache write-back 
	  * other values for I/O write and cache write miss according to the original CPU request*/ 
	output [ 3:0] to_mem_wr_data_strb,
	/** Cache sending memory/IO write data: if current data beat is the last in this burst data transmission */
	output        to_mem_wr_data_last,
	/** Acknowledgement from memory/IO: ready to receive current data beat */
	input	      from_mem_wr_data_ready
);

//TODO: Please add your D-Cache code here

        // variables
        genvar i;
        integer j;

        // use tree-PLRU replacement policy to choose the way to evict
        reg [`CACHE_SET - 1 : 0] age_layer_1 [1:0]; // age in 1st layer, 1 for recently used compared with the other path in pair
        reg [`CACHE_SET - 1 : 0] age_layer_2      ; // age in 2nd layer, 1 for recently used compared with the other path in pair

        reg [`CACHE_WAY - 1 : 0] valid_arr [`CACHE_SET - 1 : 0]; // store 8 sets as an array, each set has 4 ways
        reg [`CACHE_WAY - 1 : 0] dirty_arr [`CACHE_SET - 1 : 0]; // store 8 sets as an array, each set has 4 ways

        wire addr_hit; // check if the data is hit in cache, no matter it is a read hit or a write hit

/*
        TAG_RD: 考虑四种情况.
                1. 读命中/写命中: 读取 cache 中的数据, 跳往 CACHE_RD
                2. 读缺失/写缺失: 跳往 EVICT
                3. 不可缓存的读: 跳往 MEM_RD, 读取内存中的数据 (长度为1), 并将其在 RESP 中送回 CPU
                4. 不可缓存的写: 跳往 MEM_WR, 将输入的数据写入内存 (长度为1)
        CACHE_RD: 考虑两种情况:
                1. 读: 跳往 RESP
                2. 写: 跳往 DATA_CHANGE
        EVICT: 考虑两种情况:
                1. 在驱逐的 block 为 dirty 时, 添加新的分支 MEM_WR, 将 block 写回内存, 并将 dirty 置为 0
                2. 在驱逐的 block 为 clean 时, 直接进入 MEM_RD, 其余不变
        RECV: 在抵达结束条件时, 考虑三种情况:
                1. 读缺失: 跳往 REFILL
                2. 写缺失: 跳往 DATA_CHANGE
                3. 不可缓存的读: 跳往 RESP

        DATA_CHANGE: 将已收到/刚从 cache 中读取的 32-byte 数据, 根据 cpu 请求修改后, 填入选中 cache block, 
                     同时更新对应的 tag 和 valid, 并根据输入地址 offset 域, 返回指令码, 设置对应块的 dirty 为 1
        MEM_WR: 向内存发送输入请求所在的 cache block 地址 (32/4位对齐地址), 拉高 to_mem_wr_req_valid, 
                设置好 to_mem_wr_req_addr 和 to_mem_wr_req_len
        SEND: 拉高 to_mem_wr_data_valid, 准备好 to_mem_wr_data 和 to_mem_wr_data_strb. 每当 from_mem_wr_data_ready 拉高时
              直至 to_mem_wr_data_last 标记的最后一个 4-byte 数据准备发送, 此时遇上 from_mem_wr_data_ready 即可结束
              在抵达结束条件时, 考虑两种情况:
                1. 读缺失/写缺失: 跳往 MEM_RD
                3. 不可缓存的写: 跳往 WAIT
              注意 to_mem_wr_data_strb 仅在请求长度为1 (即不可缓存, to_mem_wr_req_len == 0) 时有效.
*/


        reg [`STATE_NUM_DCACHE - 1 : 0] current_state;
        reg [`STATE_NUM_DCACHE - 1 : 0] next_state;

        parameter State_Wait        = `STATE_NUM_DCACHE'b00000000001;
        parameter State_Tag_Rd      = `STATE_NUM_DCACHE'b00000000010;
        parameter State_Cache_Rd    = `STATE_NUM_DCACHE'b00000000100;
        parameter State_Resp        = `STATE_NUM_DCACHE'b00000001000;
        parameter State_Evict       = `STATE_NUM_DCACHE'b00000010000;
        parameter State_Mem_Rd      = `STATE_NUM_DCACHE'b00000100000;
        parameter State_Recv        = `STATE_NUM_DCACHE'b00001000000;
        parameter State_Refill      = `STATE_NUM_DCACHE'b00010000000;
        parameter State_Data_Change = `STATE_NUM_DCACHE'b00100000000;
        parameter State_Mem_Wr      = `STATE_NUM_DCACHE'b01000000000;
        parameter State_Send        = `STATE_NUM_DCACHE'b10000000000;

        wire WAIT        = current_state[ 0];
        wire TAG_RD      = current_state[ 1];
        wire CACHE_RD    = current_state[ 2];
        wire RESP        = current_state[ 3];
        wire EVICT       = current_state[ 4];
        wire MEM_RD      = current_state[ 5];
        wire RECV        = current_state[ 6];
        wire REFILL      = current_state[ 7];
        wire DATA_CHANGE = current_state[ 8];
        wire MEM_WR      = current_state[ 9];
        wire SEND        = current_state[10];


        // calculate next state, based on current state and input signals (and so on)
        always @(*) begin
                case (current_state)
                        State_Wait: begin
                                if (from_cpu_mem_req_valid) begin
                                        next_state = State_Tag_Rd;
                                end
                                else begin
                                        next_state = State_Wait;
                                end
                        end
                        State_Tag_Rd: begin
                                if (~cacheable) begin
                                        if (write_req_reg) begin
                                                next_state = State_Mem_Wr;
                                        end
                                        else begin
                                                next_state = State_Mem_Rd;
                                        end
                                end
                                else if (addr_hit) begin
                                        next_state = State_Cache_Rd;
                                end
                                else begin // read miss
                                        next_state = State_Evict;
                                end
                        end
                        State_Cache_Rd: begin
                                if (write_req_reg) begin
                                        next_state = State_Data_Change;
                                end
                                else begin
                                        next_state = State_Resp;
                                end
                        end
                        State_Resp: begin
                                if (from_cpu_cache_rsp_ready) begin
                                        next_state = State_Wait;
                                end
                                else begin
                                        next_state = State_Resp;
                                end
                        end
                        State_Evict: begin
                                if (evict_dirty_way) begin
                                        next_state = State_Mem_Wr;
                                end
                                else begin
                                        next_state = State_Mem_Rd;
                                end
                        end
                        State_Mem_Rd: begin
                                if (from_mem_rd_req_ready) begin
                                        next_state = State_Recv;
                                end
                                else begin
                                        next_state = State_Mem_Rd;
                                end
                        end
                        State_Recv: begin
                                if (from_mem_rd_rsp_valid & from_mem_rd_rsp_last) begin
                                        if (~cacheable_reg) begin
                                                next_state = State_Resp;
                                        end
                                        else if (write_req_reg) begin
                                                next_state = State_Data_Change;
                                        end
                                        else begin
                                                next_state = State_Refill;
                                        end
                                end
                                else begin
                                        next_state = State_Recv;
                                end
                        end
                        State_Refill: begin
                                next_state = State_Resp;
                        end
                        State_Data_Change: begin
                                next_state = State_Wait;
                        end
                        State_Mem_Wr: begin
                                if (from_mem_wr_req_ready) begin
                                        next_state = State_Send;
                                end
                                else begin
                                        next_state = State_Mem_Wr;
                                end
                        end
                        State_Send: begin
                                if (from_mem_wr_data_ready & to_mem_wr_data_last) begin
                                        if (~cacheable_reg) begin
                                                next_state = State_Wait;
                                        end
                                        else begin
                                                next_state = State_Mem_Rd;
                                        end
                                end
                                else begin
                                        next_state = State_Send;
                                end
                        end
                        default: begin
                                next_state = State_Wait;
                        end
                endcase
        end

        // change state when clk rising edge
        always @(posedge clk) begin
                if (rst) begin
                        current_state <= State_Wait;
                end
                else begin
                        current_state <= next_state;
                end
        end


/*
        Consider State_Wait operations
        In State_Wait, we should check if there is a valid request from CPU
        If there is a valid request, we should go to State_Tag_Rd
        If there is no valid request, we should stay in State_Wait
        We need to take down the address of the request

        Modify: Another two signals are added: write_req_reg and write_data. They should be stored from input signals
*/

        reg                       write_req_reg; // 0 for read, 1 for write
        reg [`DATA_WIDTH - 1 : 0] mem_addr_reg;  // address of the request
        reg [`DATA_WIDTH - 1 : 0] mem_wdata_reg; // write data of the request
        reg [              3 : 0] mem_wstrb_reg; // write strobe of the request

        assign to_cpu_mem_req_ready = rst | WAIT;

        always @(posedge clk) begin
                if (WAIT & from_cpu_mem_req_valid) begin
                        write_req_reg <= from_cpu_mem_req;
                        mem_addr_reg  <= from_cpu_mem_req_addr;
                        mem_wdata_reg <= from_cpu_mem_req_wdata;
                        mem_wstrb_reg <= from_cpu_mem_req_wstrb;
                end
        end


/*
        Consider State_Tag_Rd operations
        In State_Tag_Rd, we should check if there is a read hit or read miss
        If there is a read hit, we should go to State_Cache_Rd
        If there is a read miss, we should go to State_Evict
        It will never happen that there is both a read hit and a read miss, or neither of them
        Therefore, we should set addr_hit 1 or 0, by calculating the tag of the request, compared with set (also calculated)
        Then we need to take down the tag of the request

        Modify: 
        "uncacheable" means that cacheable == 0.
                When the request's address is from 0x0000_0000 to 0x0000_001F, and beyond 0x4000_0000, it is uncacheable.
                Now an uncachabel request should follow different path in state machine,
                and should not be stored in cache.
*/

        reg cacheable_reg; // check if the request is cacheable (stored in a register)
        reg addr_hit_reg; // check if the data is hit in cache (stored in a register)

        reg [`CACHE_WAY - 1 : 0] cache_target_way; // decide which way to read, used in CACHE_RD and SEND

        wire [`TAG_LEN         - 1 : 0] tag_req;      // tag of the request
        wire [`SET_ADDR_WIDTH  - 1 : 0] set_idx;      // set index of the request
        wire [`BLOCK_OFF_WIDTH - 1 : 0] block_offset; // block address of the request

        wire [`TAG_LEN   - 1 : 0] tag_set_r  [`CACHE_WAY - 1 : 0]; // tag read from tag array, by a given set index
        wire [`TAG_LEN   - 1 : 0] tag_set_w;  // tag to write into tag array

        // assignment of array_wen will be done in State_Refill
        wire [`CACHE_WAY - 1 : 0] array_wen; // write enable of each way in a given set

        wire                      cacheable; // check if the request is cacheable
        wire [`CACHE_WAY - 1 : 0] hit_way; // check the data is hit in which way


        // split the address into tag, set index and block offset
        // assign tag_req      = mem_addr_reg[                   `TOTAL_MEM_ADDR - 1 : `SET_ADDR_WIDTH + `BLOCK_OFF_WIDTH];
        // assign set_idx      = mem_addr_reg[`SET_ADDR_WIDTH + `BLOCK_OFF_WIDTH - 1 :                   `BLOCK_OFF_WIDTH];
        // assign block_offset = mem_addr_reg[                  `BLOCK_OFF_WIDTH - 1 :                                  0];
        assign {tag_req, set_idx, block_offset} = mem_addr_reg;

        // memantiate tag_array, each has 4 ways. use generate to generate them
        generate
                for (i = 0; i < `CACHE_WAY; i = i + 1) begin : tag_array_mem_generate
                        tag_array tag_array_mem (
                                .clk(clk),
                                .waddr(set_idx),
                                .raddr(set_idx),
                                .wen(array_wen[i]),
                                .wdata(tag_set_w),
                                .rdata(tag_set_r[i])
                        );
                end
        endgenerate

        // Check if there is a read hit 4:0100
        // First check if request is cacheable. Address from 0x0000_0000 to 0x0000_001F, and beyond 0x4000_0000 is uncacheable
        // {2'b0, 25'b?, 5'b?}
        assign cacheable = (mem_addr_reg[29:5] != 25'b0) & (mem_addr_reg[31:30] == 2'b0);
        // Then check if there is a tag match
        assign hit_way[0] = valid_arr[set_idx][0] & (tag_set_r[0] == tag_req);
        assign hit_way[1] = valid_arr[set_idx][1] & (tag_set_r[1] == tag_req);
        assign hit_way[2] = valid_arr[set_idx][2] & (tag_set_r[2] == tag_req);
        assign hit_way[3] = valid_arr[set_idx][3] & (tag_set_r[3] == tag_req);

        assign addr_hit = cacheable & (|hit_way); // if cacheable and there is a tag match, then it is a read hit
        // note: if any bit of a vector is 1, then the vector is 1 in logical comparison
        // so can be writen in the form below
        // assign addr_hit = cacheable & hit_way;

        // store cacheable in a register
        always @(posedge clk) begin
                if (TAG_RD) begin
                        cacheable_reg <= cacheable;
                end
        end

        // store addr_hit in a register
        always @(posedge clk) begin
                if (TAG_RD) begin
                        addr_hit_reg <= addr_hit;
                end
        end

        // store hit_way in a register
        always @(posedge clk) begin
                if (TAG_RD) begin
                        cache_target_way <= hit_way;
                end
                else if (EVICT) begin
                        cache_target_way <= way_to_evict;
                end
        end


/*
        Consider State_Cache_Rd operations
        In State_Cache_Rd, we should read the data from the cache
        Then they should be taken down in a register
*/

        reg [`DATA_WIDTH - 1 : 0] data_result_reg; // data array read from data array, by a given set index

        wire [`LINE_LEN - 1 : 0] data_rd_result_block; // data read from data array, hit in cache
        wire [7:0] shift_num_in_block; // number of bits to shift for data index, according to block_offset

        wire [`LINE_LEN  - 1 : 0] data_set_r [`CACHE_WAY - 1 : 0]; // data read from data array, by a given set index
        wire [`LINE_LEN  - 1 : 0] data_set_w; // data to write into data array

        // note: this wire above will also be used in REFILL

        // memantiate data_array, each has 4 ways. use generate to generate them
        generate
                for (i = 0; i < `CACHE_WAY; i = i + 1) begin : data_array_mem_generate
                        data_array data_array_mem (
                                .clk(clk),
                                .waddr(set_idx),
                                .raddr(set_idx),
                                .wen(array_wen[i]),
                                .wdata(data_set_w),
                                .rdata(data_set_r[i])
                        );
                end
        endgenerate

        // calculate the index of the way in which the data is hit
        assign data_rd_result_block = data_set_r[0] & {`LINE_LEN{cache_target_way[0]}}
                                    | data_set_r[1] & {`LINE_LEN{cache_target_way[1]}}
                                    | data_set_r[2] & {`LINE_LEN{cache_target_way[2]}}
                                    | data_set_r[3] & {`LINE_LEN{cache_target_way[3]}}
        ;

        // calculate the number of bits to shift for data index, according to block_offset
        assign shift_num_in_block = {block_offset, 3'b0}; // block_offset * 8

        // store the data of the request into registers
        // note: this register will also be used in REFILL or RECV
        always @(posedge clk) begin
                if (CACHE_RD) begin // also refresh when write_req_reg is 1 (though it is not used)
                        data_result_reg <= data_rd_result_block[shift_num_in_block +: `DATA_WIDTH];
                end
                else if (REFILL) begin
                        data_result_reg <= data_recv_reg[shift_num_in_block +: `DATA_WIDTH];
                end
                else if (RECV & from_mem_rd_rsp_valid & from_mem_rd_rsp_last & ~cacheable_reg) begin
                        data_result_reg <= from_mem_rd_rsp_data;
                end
        end

        // note: When going to DATA_CHANGE, we should also store the data into data_recv_reg (to fully use the registers)
        //       This is done in the always block in RECV section

        // modify the age of the way in which the data is hit, according to the tree-PLRU replacement policy
        always @(posedge clk) begin
                if (rst) begin
                        age_layer_1[0] <= `CACHE_SET'b0;
                end
                else if (CACHE_RD & (cache_target_way[0] | cache_target_way[1])) begin
                        age_layer_1[0][set_idx] <= cache_target_way[1];
                end
        end
        always @(posedge clk) begin
                if (rst) begin
                        age_layer_1[1] <= `CACHE_SET'b0;
                end
                else if (CACHE_RD & (cache_target_way[2] | cache_target_way[3])) begin
                        age_layer_1[1][set_idx] <= cache_target_way[3];
                end
        end
        always @(posedge clk) begin
                if (rst) begin
                        age_layer_2 <= `CACHE_SET'b0;
                end
                else if (CACHE_RD) begin
                        age_layer_2[set_idx] <= cache_target_way[2] | cache_target_way[3];
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

        modify: Check if the way to evict is dirty, if so, then we should write it back to memory first.
                Move to MEM_WR to write the dirty block.
*/

        // store the way to evict
        reg [`CACHE_WAY - 1 : 0] way_to_evict_reg; // way that will be evicted, in MEM_RD state

        wire [`CACHE_WAY - 1 : 0] LRU_way_temp;     // the least recently used way
        wire [`CACHE_WAY - 1 : 0] LRU_way;          // the least recently used way
        wire [`CACHE_WAY - 1 : 0] inv_way_1st_temp; // the first way that is invalid
        wire [`CACHE_WAY - 1 : 0] inv_way_1st;      // the first way that is invalid
        wire [`CACHE_WAY - 1 : 0] way_to_evict;     // way that will be evicted

        wire evict_dirty_way; // check if the way to evict is dirty


        // store the way to evict
        always @(posedge clk) begin
                if (EVICT) begin
                        way_to_evict_reg <= way_to_evict;
                end
        end

        // calculate the LRU and FIV way
        assign LRU_way_temp = {
                ~age_layer_2[set_idx] & ~age_layer_1[1][set_idx],
                ~age_layer_2[set_idx] &  age_layer_1[1][set_idx],
                 age_layer_2[set_idx] & ~age_layer_1[0][set_idx],
                 age_layer_2[set_idx] &  age_layer_1[0][set_idx]
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

        assign inv_way_1st_temp = ~valid_arr[set_idx];
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
        assign way_to_evict = {`CACHE_WAY{ &(valid_arr[set_idx])}} & LRU_way
                            | {`CACHE_WAY{~&(valid_arr[set_idx])}} & inv_way_1st
        ;

        // check if the way to evict is dirty
        assign evict_dirty_way = |(way_to_evict & dirty_arr[set_idx]);
        // evict the chosen way, by setting its valid signal as 0
        // note: this operation will be done in REFILL
        always @(posedge clk) begin
                if (rst) begin
                        for (j = 0; j < `CACHE_SET; j = j + 1) begin
                                valid_arr[j] <= `CACHE_WAY'b0;
                        end
                end
                else if (EVICT) begin
                        valid_arr[set_idx] <= valid_arr[set_idx] & ~way_to_evict;
                end
                else if (REFILL | DATA_CHANGE) begin
                        valid_arr[set_idx] <= valid_arr[set_idx] | cache_target_way;
                end
        end


/*
        Consider State_Mem_Rd operations
        In State_Mem_Rd, we should send the memory read request to real memory
        Then we should wait until memory is ready to receive the request
        At the same time, the address of cache block should be given out (the lowest bit of the block in memory)
        This address should be ended with 5'b0, because the address of cache block is 32 byte aligned

        modify: Assign for another two output signals: to_mem_rd_req_len and to_mem_rd_req_addr
*/

        assign to_mem_rd_req_valid = ~rst & MEM_RD;
        assign to_mem_rd_req_addr = cacheable_reg
                                  ? {mem_addr_reg[`DATA_WIDTH - 1 : 5], 5'b0} // 32 byte aligned
                                  : {mem_addr_reg[`DATA_WIDTH - 1 : 2], 2'b0} //  4 byte aligned
        ;
        assign to_mem_rd_req_len = cacheable_reg
                                 ? 8'd7 // read 8 data beats
                                 : 8'd0 // only read one data beat
        ;


/*
        Consider State_Recv operations
        In State_Recv, we should receive the data from memory
        Use a 256-bit register to store the data. Each time, we should store 32-bit data into the register
        Then we should check if the current data beat is the last one in this burst data transmission
        If so, we should go to State_Refill (already written in state machine)
*/

        reg [`LINE_LEN - 1 : 0] data_recv_reg;          // data received from memory, 256-bit
        reg [2:0] data_recv_cnt; // number of data beat received from memory, 8-bit

        // store the data beat into the register. no need to reset the register
        always @(posedge clk) begin
                if (RECV & from_mem_rd_rsp_valid & cacheable_reg) begin
                        data_recv_reg[{data_recv_cnt, 5'b0} +: `DATA_WIDTH] <= from_mem_rd_rsp_data;
                end
                else if (CACHE_RD & write_req_reg) begin // newly added
                        data_recv_reg <= data_rd_result_block;
                end
        end

        // count the number of data beat received from memory
        always @(posedge clk) begin
                if (rst | RECV & from_mem_rd_rsp_valid & from_mem_rd_rsp_last | SEND & from_mem_wr_data_ready & to_mem_wr_data_last) begin
                        data_recv_cnt <= 3'b0;
                end
                else if ((RECV & from_mem_rd_rsp_valid | SEND & from_mem_wr_data_ready) & cacheable_reg) begin
                        data_recv_cnt <= data_recv_cnt + 1;
                end
        end

        // calculate the number of bits to shift for data index, according to data_recv_cnt
        // assign data_recv_shift_num = data_recv_cnt << 5; // data_recv_cnt * 32

        assign to_mem_rd_rsp_ready = rst | RECV;


/*
        Consider State_Refill operations
        In State_Refill, we should write the data and tag into the cache (set corresponding valid as 1)

        modify: Add assignment for dirty_arr in this section
                (clear dirty sign when the data read from memory is put into cache)
*/

        // set dirty and clear dirty for certain way in a given set
        always @(posedge clk) begin
                if (rst) begin
                        for (j = 0; j < `CACHE_SET; j = j + 1) begin
                                dirty_arr[j] <= `CACHE_WAY'b0;
                        end
                end
                else if (REFILL) begin
                        dirty_arr[set_idx] <= dirty_arr[set_idx] & ~cache_target_way;
                end
                else if (DATA_CHANGE) begin
                        dirty_arr[set_idx] <= dirty_arr[set_idx] | cache_target_way;
                end
        end

        // store the cache block into data array. only write when in REFILL state
        assign array_wen  = {`CACHE_WAY{REFILL | DATA_CHANGE}} & cache_target_way;
        assign tag_set_w  = tag_req;
        assign data_set_w = {`LINE_LEN{REFILL     }} & data_recv_reg
                          | {`LINE_LEN{DATA_CHANGE}} & data_change
        ;

        // calculate the valid signal of each way in a given set
        // NOTE: THIS TASK IS WRITTEN IN EVICT SECTION

        // send the data read to a register, preparing for sending to CPU
        // NOTE: THIS TASK IS WRITTEN IN CACHE_RD SECTION


/*
        Consider State_Data_Change operations
        In State_Data_Change, we should change the data in the cache (from CACHE_RD) or read from memory (from RECV)
        Choose the offset in the block by block_offset, and change the data in the cache
        Then we should write the data and tag into the cache (set corresponding valid as 1)
*/

        // modify data in register: data_recv_reg (linked to data_set_w, in REFILL section)
        wire [`DATA_WIDTH - 1 : 0] wr_data_mask; // extended from wstrb, from 4 bytes to 32 bytes
        wire [`DATA_WIDTH - 1 : 0] wr_data_valid; // the valid data to write into corresponding block by block_offset
        wire [`LINE_LEN   - 1 : 0] data_change; // data to write into data array, in DATA_CHANGE state

        assign wr_data_mask = {
                {8{mem_wstrb_reg[3]}},
                {8{mem_wstrb_reg[2]}},
                {8{mem_wstrb_reg[1]}},
                {8{mem_wstrb_reg[0]}}
        };

        // memantiate data_array, each has 4 ways. use generate to generate them
        // If offset hits certain 4-bytes data, the mask will be " wr_data_mask" for mem_wdata_reg,
        //                                                   and "~wr_data_mask" for data_recv_reg
        // For other 4-bytes data, the mask will be "32'b0" for mem_wdata_reg, and "32'b1" for data_recv_reg
        generate
                for (i = 0; i < `DATA_NUM_IN_BLOCK; i = i + 1) begin : data_change_generate
                        assign data_change[`DATA_WIDTH * i +: `DATA_WIDTH]
                              = ((block_offset[4:2] != i) ? `DATA_WIDTH'b1: ~wr_data_mask) &
                                data_recv_reg[`DATA_WIDTH * i +: `DATA_WIDTH]
                              | ((block_offset[4:2] != i) ? `DATA_WIDTH'b0:  wr_data_mask) &
                                mem_wdata_reg
                        ;
                end
        endgenerate


/*
        Consider State_Mem_Wr operations
        In State_Mem_Wr, we should send the memory write request to real memory
        Then we should wait until memory is ready to receive the request
        At the same time, the address of cache block should be given out (the lowest bit of the block in memory)
        This address should be ended with 5'b0, because the address of cache block is 32 byte aligned
*/

        wire [`TAG_LEN - 1 : 0] tag_wr; // tag to write into tag array when cacheable, read by the way to ecivt

        assign tag_wr = {`TAG_LEN{way_to_evict_reg[0]}} & tag_set_r[0]
                      | {`TAG_LEN{way_to_evict_reg[1]}} & tag_set_r[1]
                      | {`TAG_LEN{way_to_evict_reg[2]}} & tag_set_r[2]
                      | {`TAG_LEN{way_to_evict_reg[3]}} & tag_set_r[3]
        ;

        assign to_mem_wr_req_valid = ~rst & MEM_WR;
        assign to_mem_wr_req_addr = cacheable_reg
                                  ? {tag_wr, set_idx, 5'b0} // 32 byte aligned
                                  : {mem_addr_reg[`DATA_WIDTH - 1 : 2], 2'b0} //  4 byte aligned
        ;

        // assign to_mem_wr_req_len = cacheable_reg
        //                          ? 8'd7 // write 8 data beats
        //                          : 8'd0 // only write one data beat
        // ;
        assign to_mem_wr_req_len = {1'b0, {3{cacheable_reg}}};

/*
        Consider State_Send operations
        In State_Send, we should send the memory write data to real memory
        Then we should wait until memory is ready to receive the data
        At the same time, the data should be given out (the lowest bit of the block in memory)
        This data should be ended with 5'b0, because the address of cache block is 32 byte aligned
*/

        assign to_mem_wr_data_valid = ~rst & SEND;
        assign to_mem_wr_data = cacheable_reg
                              ? data_rd_result_block[{data_recv_cnt, 5'b0} +: `DATA_WIDTH]
                              : mem_wdata_reg
        ;
        // assign to_mem_wr_data_last = ~cacheable_reg | (data_recv_cnt == 3'b111);
        assign to_mem_wr_data_last = ~cacheable_reg | data_recv_cnt[2] & data_recv_cnt[1] & data_recv_cnt[0];
        assign to_mem_wr_data_strb = mem_wstrb_reg | {4{cacheable_reg}}; // ignore the write strobe when cacheable


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
