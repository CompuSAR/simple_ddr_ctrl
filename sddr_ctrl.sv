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
        //input ddr_clock_i,
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
        input [BANK_BITS+ROW_BITS+COL_BITS+$clog2(DATA_BITS/8)-1:0]
                                                        data_cmd_address,
        input                                           data_cmd_write,
        output                                          data_cmd_ack,
        output                                          data_rsp_ready,
        input [BURST_LENGTH*DATA_BITS-1:0]              data_cmd_data_i,
        input [BURST_LENGTH*DATA_BITS-1:0]              data_data_o,

        // phy interfaces
        output                                          ddr3_cke_o,
        output                                          ddr3_ras_n_o,
        output                                          ddr3_cas_n_o,
        output                                          ddr3_we_n_o,

        output                                          ddr3_cs_n_o,

        output logic [BANK_BITS-1:0]                    ddr3_ba_o,
        output logic [ROW_BITS+$clog2(DATA_BITS/8)-1:0] ddr3_addr_o,
        output                                          ddr3_odt_o,
        output [DATA_BITS/8-1:0]                        ddr3_dm_o,
        output                                          ddr3_dq_enable_o,
        output [DATA_BITS*2-1:0]                        ddr3_dq_o,
        input [DATA_BITS*2-1:0]                         ddr3_dq_i,

        output                                          data_transfer_o,
        output                                          data_write_o
    );

wire ddr_clock_i = cpu_clock_i;

logic [31:0] reset_state=0; // State of the reset signals

// Bits: CS, RAS, CAS, WE.
logic [3:0] override_cmd_cpu, output_cmd;
logic override_cmd_cpu_send = 1'b0;
logic [31:0] override_addr;

assign ddr3_we_n_o = output_cmd[0];
assign ddr3_cas_n_o = output_cmd[1];
assign ddr3_ras_n_o = output_cmd[2];
assign ddr3_cs_n_o = output_cmd[3];

assign ctrl_cmd_ack = 1'b1;

assign ddr_reset_n_o            = reset_state[0];
assign ddr_phy_reset_n_o        = reset_state[1];
logic ctrl_reset                = reset_state[2];
logic bypass                    = !reset_state[3];
assign ddr3_odt_o               = reset_state[4];
assign ddr3_cke_o               = reset_state[5];

// CPU clock domain
always_ff@(posedge cpu_clock_i) begin
    override_cmd_cpu_send <= 1'b0;

    if( ctrl_cmd_valid && ctrl_cmd_ack && ctrl_cmd_write ) begin
        case(ctrl_cmd_address)
            16'h0000: begin     // Reset state
                reset_state <= ctrl_cmd_data;
            end
            16'h0004: begin     // Override command
                override_cmd_cpu <= ctrl_cmd_data;
                override_cmd_cpu_send <= 1'b1;
            end
            16'h0008: begin     // Override address
                override_addr <= ctrl_cmd_data;
            end
        endcase
    end
end

// DDR clock domain
always_ff@(posedge ddr_clock_i) begin
    if( !reset_state[3] /* override */ && override_cmd_cpu_send ) begin
        ddr3_addr_o <= override_addr;
        ddr3_ba_o <= override_addr[31:31-BANK_BITS+1];
        output_cmd <= override_cmd_cpu; // Remant from the days we had separate CPU and DDR clocks
    end else begin
        output_cmd <= 5'b0111; // NOP
        ddr3_addr_o <= 0;
        ddr3_ba_o <= 0;
    end
end

endmodule
