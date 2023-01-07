`timescale 1ns / 1ps

module sddr_phy_xilinx#(
    )
    (
        // Inside interfaces
        input in_ddr_clock_i,
        input in_ddr_reset_p_i,
        input in_phy_reset_p_i,


        // Outside interfaces
        output ddr3_ck_p_o,
        output ddr3_ck_n_o,
        output ddr3_reset_n_o
    );

assign ddr3_reset_n_o = !in_ddr_reset_p_i;

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
