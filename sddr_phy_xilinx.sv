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
        input in_ddr_reset_n_i,
        input in_phy_reset_n_i,

        // Controller's gonna control
        input                                           ctl_odt_i,
        input                                           ctl_cke_i,
        input                                           ctl_ras_n_i,
        input                                           ctl_cas_n_i,
        input                                           ctl_we_n_i,


        // Outside interfaces
        output                                          ddr3_ck_p_o,
        output                                          ddr3_ck_n_o,
        output                                          ddr3_reset_n_o,

        output logic                                    ddr3_cke_o,
        output logic                                    ddr3_ras_n_o,
        output logic                                    ddr3_cas_n_o,
        output logic                                    ddr3_we_n_o,

        output                                          ddr3_cs_n_o,

        output [BANK_BITS-1:0]                          ddr3_ba_o,
        output [ROW_BITS+$clog2(DATA_BITS/8)-1:0]       ddr3_addr_o,
        inout                                           ddr3_odt_o,
        output [$clog2(DATA_BITS/8):0]                  ddr3_dm_o,
        inout [$clog2(DATA_BITS/8):0]                   ddr3_dqs_p_io,
        inout [$clog2(DATA_BITS/8):0]                   ddr3_dqs_n_io,
        inout [DATA_BITS-1:0]                           ddr3_dq_io
    );

assign ddr3_reset_n_o = in_ddr_reset_n_i;
assign ddr3_cs_n_o = 1'b0;      // We don't do chip select
//IOBUF odt_buffer( .I(ctl_odt_i), .T(!ddr3_reset_n_o), .IO(ddr3_odt_o), .O() );

logic phy_reset_n, phy_reset_p;
xpm_cdc_sync_rst cdc_reset(.src_rst(in_phy_reset_n_i), .dest_clk(in_ddr_clock_i), .dest_rst(phy_reset_n));

always_ff@(negedge in_ddr_clock_i) begin
    ddr3_cke_o <= ctl_cke_i;
    ddr3_ras_n_o <= ctl_ras_n_i;
    ddr3_cas_n_o <= ctl_cas_n_i;
    ddr3_we_n_o <= ctl_we_n_i;

    phy_reset_p <= !phy_reset_n;
end

// Clock differential output
logic naked_clock_signal;
ODDR clock_signal_generator(
    .Q(naked_clock_signal),
    .C(in_ddr_clock_i),
    .CE(phy_reset_n),
    .D1(1'b1),
    .D2(1'b0),
    .R(phy_reset_p),
    .S(1'b0)
);

OBUFDS clock_buffer(
    .I(naked_clock_signal),
    .O(ddr3_ck_p_o),
    .OB(ddr3_ck_n_o)
);

endmodule
