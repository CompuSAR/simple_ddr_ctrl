`timescale 1ns / 1ps

module sddr_phy_xilinx#(
        BANK_BITS = 3,
        ROW_BITS = 13,
        COL_BITS = 10,
        DATA_BITS = 16
    )
    (
        // Inside interfaces
        input in_cpu_clock_i,
        input in_ddr_clock_i,
        input in_ddr_clock_90deg_i,
        input in_ddr_reset_n_i,
        input in_phy_reset_n_i,

        // Controller's gonna control
        input                                           ctl_odt_i,
        input                                           ctl_cs_n_i,
        input                                           ctl_cke_i,
        input                                           ctl_ras_n_i,
        input                                           ctl_cas_n_i,
        input                                           ctl_we_n_i,
        input [ROW_BITS+$clog2(DATA_BITS/8)-1:0]        ctl_addr_i,
        input [BANK_BITS-1:0]                           ctl_ba_i,
        input [DATA_BITS-1:0]                           ctl_dq_i[1:0],
        output [DATA_BITS-1:0]                          ctl_dq_o[1:0],

        input                                           ctl_data_transfer_i,
        input                                           ctl_data_write_i,
        input                                           ctl_write_level_i,
        input                                           ctl_out_dqs_i,


        // Outside interfaces
        output                                          ddr3_ck_p_o,
        output                                          ddr3_ck_n_o,
        output                                          ddr3_reset_n_o,

        output logic                                    ddr3_cke_o,
        output logic                                    ddr3_ras_n_o,
        output logic                                    ddr3_cas_n_o,
        output logic                                    ddr3_we_n_o,

        output                                          ddr3_cs_n_o,

        output logic [BANK_BITS-1:0]                    ddr3_ba_o,
        output logic [ROW_BITS+$clog2(DATA_BITS/8)-1:0] ddr3_addr_o,
        output logic                                    ddr3_odt_o,
        output [DATA_BITS/8-1:0]                        ddr3_dm_o,
        inout [DATA_BITS/8-1:0]                         ddr3_dqs_p_io,
        inout [DATA_BITS/8-1:0]                         ddr3_dqs_n_io,
        inout [DATA_BITS-1:0]                           ddr3_dq_io
    );

assign ddr3_dm_o = { DATA_BITS/8{1'b0} };
assign ddr3_reset_n_o = in_ddr_reset_n_i;
assign ddr3_cs_n_o = ctl_cs_n_i;

logic phy_reset_n;
xpm_cdc_sync_rst cdc_reset(.src_rst(in_phy_reset_n_i), .dest_clk(in_ddr_clock_i), .dest_rst(phy_reset_n));

always_ff@(negedge in_ddr_clock_i) begin
    ddr3_cke_o <= ctl_cke_i;
    ddr3_ras_n_o <= ctl_ras_n_i;
    ddr3_cas_n_o <= ctl_cas_n_i;
    ddr3_we_n_o <= ctl_we_n_i;
    ddr3_odt_o <= ctl_odt_i;

    ddr3_addr_o <= ctl_addr_i;
    ddr3_ba_o <= ctl_ba_i;
end

// Clock differential output
OBUFDS clock_buffer(
    .I(in_ddr_clock_i),
    .O(ddr3_ck_p_o),
    .OB(ddr3_ck_n_o)
);

(* IODELAY_GROUP = "DQSCLOCK" *)

IDELAYCTRL dqs_clock_delay_ctrl(
    .RDY(),
    .REFCLK(in_ddr_clock_i),
    .RST(1'b0)
);

logic delay_inc;
logic dqs_out_clock, dqs_in_clock;

(* IODELAY_GROUP = "DQSCLOCK" *)
IDELAYE2#(
    .DELAY_SRC("DATAIN"),
    .HIGH_PERFORMANCE_MODE("TRUE"),
    .IDELAY_TYPE("VARIABLE"),
    .REFCLK_FREQUENCY(303.1),
    .SIGNAL_PATTERN("CLOCK")
) dqs_clock_delay(
    .C(in_ddr_clock_i),
    .CE(delay_inc),
    .CINVCTRL(1'b0),
    .DATAIN(in_ddr_clock_i),
    .CNTVALUEIN(5'b0),
    .CNTVALUEOUT(),
    .DATAOUT(dqs_out_clock),
    .INC(1'b1),
    .LD(1'b0),
    .LDPIPEEN(1'b0),
    .REGRST(1'b0)
);

(* IODELAY_GROUP = "DQSCLOCK" *)
IDELAYE2#(
    .DELAY_SRC("DATAIN"),
    .HIGH_PERFORMANCE_MODE("TRUE"),
    .IDELAY_TYPE("VARIABLE"),
    .REFCLK_FREQUENCY(303.1),
    .SIGNAL_PATTERN("CLOCK")
) in_data_delay(
    .C(in_ddr_clock_i),
    .CE(1'b0),
    .CINVCTRL(1'b0),
    .DATAIN(in_ddr_clock_i),
    .CNTVALUEIN(5'b0),
    .CNTVALUEOUT(),
    .DATAOUT(dqs_in_clock),
    .INC(1'b1),
    .LD(1'b0),
    .LDPIPEEN(1'b0),
    .REGRST(1'b0)
);

genvar i;
generate
    for(i=0; i<DATA_BITS/8; i++) begin : dqs_gen
        logic dqs_in;
        wire direction = !(ctl_data_write_i||ctl_out_dqs_i);
        IOBUFDS dqs_buffer(
            .IO(ddr3_dqs_p_io[i]), .IOB(ddr3_dqs_n_io[i]), .O(dqs_in), .I(dqs_out_clock), .T( direction ));
    end : dqs_gen

    for(i=0; i<DATA_BITS; i++) begin : data_gen
        logic in_data_bit, out_data_bit;
        IOBUF data_buf(
            .IO(ddr3_dq_io[i]),
            .I(out_data_bit),
            .O(in_data_bit),
            .T(!ctl_data_write_i || ctl_write_level_i)
        );
        ODDR#(.DDR_CLK_EDGE("SAME_EDGE")) data_out_ddr(
            .Q(out_data_bit),
            .C(in_ddr_clock_i),
            .CE(1'b1),
            .D1(ctl_dq_i[0][i]),
            .D2(ctl_dq_i[1][i]),
            .R(1'b0),
            .S(1'b0)
        );

        IDDR#(.DDR_CLK_EDGE("SAME_EDGE_PIPELINED")) data_in_ddr(
            .C(dqs_in_clock),
            .CE(1'b1),
            .D(in_data_bit_delayed),
            .Q1(ctl_dq_o[0][i]),
            .Q2(ctl_dq_o[1][i]),
            .R(1'b0),
            .S(1'b0)
        );
    end : data_gen
endgenerate

assign delay_inc = !data_gen[0].in_data_bit && (DATA_BITS>8 ? !data_gen[8].in_data_bit : 1'b1) && ctl_write_level_i;

endmodule
