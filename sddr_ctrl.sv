`timescale 1ns / 1ps

module sddr_ctrl#(
        BANK_BITS = 3,
        ROW_BITS = 13,
        COL_BITS = 10,
        DATA_BITS = 16,
        BURST_LENGTH = 8
    )
    (
        // Control lines
        input cpu_clock_i,
        input ddr_clock_i,
        output ddr_reset_n_o,
        output ddr_phy_reset_n_o,

        // Command interfaces
        input                                           ctrl_cmd_valid,
        input  [15:0]                                   ctrl_cmd_address,
        input  [31:0]                                   ctrl_cmd_data,
        input                                           ctrl_cmd_write,
        output                                          ctrl_cmd_ack,
        output                                          ctrl_rsp_ready,
        output [31:0]                                   ctrl_rsp_data,

        // Data interfaces
        input                                           data_cmd_valid,
        output                                          data_cmd_ack,
        output                                          data_rsp_ready,
        input [BURST_LENGTH*DATA_BITS-1:0]              data_data_i,
        input [BURST_LENGTH*DATA_BITS-1:0]              data_data_o,

        // phy interfaces
        output                                          ddr3_cke_o,
        output                                          ddr3_ras_n_o,
        output                                          ddr3_cas_n_o,
        output                                          ddr3_we_n_o,

        output                                          ddr3_cs_n_o,

        output [BANK_BITS-1:0]                          ddr3_ba_o,
        output [ROW_BITS+$clog2(DATA_BITS/8)-1:0]       ddr3_addr_o,
        output                                          ddr3_odt_o,
        output [$clog2(DATA_BITS/8):0]                  ddr3_dm_o,
        output [$clog2(DATA_BITS/8):0]                  ddr3_dqs_o,
        input [$clog2(DATA_BITS/8):0]                   ddr3_dqs_i,
        output [DATA_BITS-1:0]                          ddr3_dq_o,
        input [DATA_BITS-1:0]                           ddr3_dq_i
    );

assign ddr3_cs_n_o = 1'b0;      // We don't do chip select

assign ctrl_cmd_ack = 1'b1;

logic [31:0] reset_state;
assign ddr_reset_n_o = reset_state[0];
assign ddr_phy_reset_n_o = reset_state[1];
logic ctrl_reset = reset_state[2];
logic bypass = !reset_state[3];
assign ddr3_odt_o = reset_state[4];
assign ddr3_cke_o = reset_state[5];

always_ff@(posedge cpu_clock_i) begin
    if( ctrl_cmd_valid && ctrl_cmd_write ) begin
        case(ctrl_cmd_address)
            16'h0000: begin     // Reset state
                reset_state <= ctrl_cmd_data;
            end
        endcase
    end
end

endmodule
