module prio_selector_recur #(
        parameter Data_Size = 3
)(
        input  [Data_Size - 1 : 0] data_raw,
        input  sel_neg,
        output [Data_Size - 1 : 0] data_sel
);

        generate
                assign data_sel[0] = data_raw[0] & ~sel_neg;
                if (Data_Size == 1) begin
                end
                else begin
                        prio_selector_recur #(Data_Size - 1) prio_selector_inst (
                                .data_raw(data_raw[Data_Size - 1 : 1]),
                                .sel_neg(sel_neg | data_raw[0]),
                                .data_sel(data_sel[Data_Size - 1 : 1])
                        );
                end
        endgenerate

endmodule

module prio_selector #(
        parameter Data_Size = 4
)(
        input  [Data_Size - 1 : 0] data_raw,
        output [Data_Size - 1 : 0] data_sel
);

        generate
                assign data_sel[0] = data_raw[0];
                if (Data_Size >= 1) begin
                        prio_selector_recur #(Data_Size - 1) prio_selector_inst (
                                .data_raw(data_raw[Data_Size - 1 : 1]),
                                .sel_neg(data_raw[0]),
                                .data_sel(data_sel[Data_Size - 1 : 1])
                        );
                end
        endgenerate

endmodule

module prio_selector_4_recur #(
        parameter Data_Size = 4
)(
        input  [Data_Size - 1 : 0] data_raw,
        input  sel_neg,
        output [Data_Size - 1 : 0] data_sel
);

        assign data_sel[0] = data_raw[0] & ~sel_neg;
        generate
                if (Data_Size >= 2) begin
                        assign data_sel[1] = data_raw[1] & ~data_raw[0] & ~sel_neg;
                end
                if (Data_Size >= 3) begin
                        assign data_sel[2] = data_raw[2] & ~data_raw[1] & ~data_raw[0] & ~sel_neg;
                end
                if (Data_Size >= 4) begin
                        assign data_sel[3] = data_raw[3] & ~data_raw[2] & ~data_raw[1] & ~data_raw[0] & ~sel_neg;
                end
                if (Data_Size > 4) begin
                        prio_selector_4_recur #(Data_Size - 4) prio_selector_inst (
                                .data_raw(data_raw[Data_Size - 1 : 4]),
                                .sel_neg(sel_neg | data_raw[0] | data_raw[1] | data_raw[2] | data_raw[3]),
                                .data_sel(data_sel[Data_Size - 1 : 4])
                        );
                end
        endgenerate

endmodule

module prio_selector_4 #(
        parameter Data_Size = 16
)(
        input  [Data_Size - 1 : 0] data_raw,
        output [Data_Size - 1 : 0] data_sel
);

        assign data_sel[0] = data_raw[0];
        generate
                if (Data_Size >= 2) begin
                        assign data_sel[1] = data_raw[1] & ~data_raw[0];
                end
                if (Data_Size >= 3) begin
                        assign data_sel[2] = data_raw[2] & ~data_raw[1] & ~data_raw[0];
                end
                if (Data_Size >= 4) begin
                        assign data_sel[3] = data_raw[3] & ~data_raw[2] & ~data_raw[1] & ~data_raw[0];
                end
                if (Data_Size > 4) begin
                        prio_selector_4_recur #(Data_Size - 4) prio_selector_inst (
                                .data_raw(data_raw[Data_Size - 1 : 4]),
                                .sel_neg(data_raw[0] | data_raw[1] | data_raw[2] | data_raw[3]),
                                .data_sel(data_sel[Data_Size - 1 : 4])
                        );
                end
        endgenerate

endmodule
