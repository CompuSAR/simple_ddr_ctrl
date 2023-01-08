`timescale 1ns / 1ps

module sddr_phy_xilinx#(
        BANK_BITS = 3,
        ROW_BITS = 13,
        COL_BITS = 10,
        DATA_BITS = 16
    )
    (
        // Inside interfaces
        input in_ddr_clock_i,
        input in_ddr_reset_p_i,
        input in_phy_reset_p_i,


        // Outside interfaces
        output                                          ddr3_ck_p_o,
        output                                          ddr3_ck_n_o,
        output                                          ddr3_reset_n_o,

        output                                          ddr3_cke_o,
        output                                          ddr3_ras_n_o,
        output                                          ddr3_cas_n_o,
        output                                          ddr3_we_n_o,

        output                                          ddr3_cs_n_o,

        output [BANK_BITS-1:0]                          ddr3_ba_o,
        output [ROW_BITS+$clog2(DATA_BITS/8)-1:0]       ddr3_addr_o,
        output                                          ddr3_odt_o,
        output [$clog2(DATA_BITS/8):0]                  ddr3_dm_o,
        inout [$clog2(DATA_BITS/8):0]                   ddr3_dqs_p_io,
        inout [$clog2(DATA_BITS/8):0]                   ddr3_dqs_n_io,
        inout [DATA_BITS-1:0]                           ddr3_dq_io
    );

assign ddr3_reset_n_o = !in_ddr_reset_p_i;
assign ddr3_cs_n_o = 1'b0;      // We don't do chip select

// Clock differential output
reg naked_clock_signal;
ODDR clock_signal_generator(
    .Q(naked_clock_signal),
    .C(in_ddr_clock_i),
    .CE(!in_phy_reset_p_i),
    .D1(1'b1),
    .D2(1'b0),
    .R(!in_phy_reset_p_i),
    .S(1'b0)
);

OBUFDS clock_buffer(
    .I(naked_clock_signal),
    .O(ddr3_ck_p_o),
    .OB(ddr3_ck_n_o)
);

endmodule
