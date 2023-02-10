`timescale 1ns / 1ps

module sddr_ctrl#(
        BANK_BITS = 3,
        ROW_BITS = 13,
        COL_BITS = 10,
        DATA_BITS = 16,
        BURST_LENGTH = 8,

        tRCD = 0,               // ACTIVATE to READ/WRITE
        tRC = 0,                // ACTIVATE to REFRESH
        tRP = 0,                // PRECHARGE period
        tRFC = 0,               // REFRESH to ACTIVATE
        tREFI = 0,              // Maximum average periodic refresh
        casReadLatency = 5,
        casWriteLatency = 5,
        write_recovery = 5
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
        output logic                                    ctrl_cmd_ack,
        output                                          ctrl_rsp_ready,
        output [31:0]                                   ctrl_rsp_data,

        // Data interfaces
        input                                           data_cmd_valid,
        input [BANK_BITS+ROW_BITS+COL_BITS+$clog2(DATA_BITS/8)-1:0]
                                                        data_cmd_address,
        input                                           data_cmd_write,
        output                                          data_cmd_ack,
        output logic                                    data_rsp_ready,
        input [BURST_LENGTH*DATA_BITS-1:0]              data_cmd_data_i,
        output [BURST_LENGTH*DATA_BITS-1:0]             data_data_o,

        // phy interfaces
        output logic                                    ddr3_cke_o,
        output                                          ddr3_ras_n_o,
        output                                          ddr3_cas_n_o,
        output                                          ddr3_we_n_o,

        output                                          ddr3_cs_n_o,

        output logic [BANK_BITS-1:0]                    ddr3_ba_o,
        output logic [ROW_BITS+$clog2(DATA_BITS/8)-1:0] ddr3_addr_o,
        output                                          ddr3_odt_o,
        output [DATA_BITS/8-1:0]                        ddr3_dm_o,
        output                                          ddr3_dq_enable_o,
        output [DATA_BITS-1:0]                          ddr3_dq_o[1:0],
        input [DATA_BITS-1:0]                           ddr3_dq_i[1:0],

        output logic                                    data_transfer_o,
        output logic                                    data_write_o
    );

localparam ADDRESS_BITS = BANK_BITS+ROW_BITS+COL_BITS+$clog2(DATA_BITS/8);
localparam HALF_BURST_LENGTH = BURST_LENGTH/2;
localparam CMD_DATA_BITS = BURST_LENGTH*DATA_BITS;

logic [31:0] reset_state_cpu=0, reset_state_ddr; // State of the reset signals
xpm_cdc_array_single#(.SRC_INPUT_REG(0), .WIDTH(32))
reset_state_cdc(.src_clk(cpu_clock_i), .dest_clk(ddr_clock_i), .src_in(reset_state_cpu), .dest_out(reset_state_ddr));

// Bits: CS, RAS, CAS, WE.
logic [3:0] override_cmd_cpu, override_cmd_ddr, output_cmd;
logic override_cmd_cpu_send = 1'b0, override_cmd_ddr_ready, override_cmd_cpu_received;
logic [31:0] override_addr_cpu, override_addr_ddr;

assign ddr3_we_n_o = output_cmd[0];
assign ddr3_cas_n_o = output_cmd[1];
assign ddr3_ras_n_o = output_cmd[2];
assign ddr3_cs_n_o = output_cmd[3];

assign ctrl_cmd_ack = !override_cmd_cpu_received && !override_cmd_cpu_send;

assign ddr_reset_n_o            = reset_state_ddr[0];
assign ddr_phy_reset_n_o        = reset_state_ddr[1];
wire ctrl_reset                 = reset_state_cpu[2];
wire bypass                     = !reset_state_ddr[3];
assign ddr3_odt_o               = reset_state_ddr[4];
//assign ddr3_cke_o               = reset_state_ddr[5];

logic [BURST_LENGTH*DATA_BITS-1:0] latched_write_data;
logic [HALF_BURST_LENGTH*DATA_BITS-1:0] shift_value[1:0];

assign ddr3_dq_o[0] = shift_value[0][DATA_BITS-1:0];
assign ddr3_dq_o[1] = shift_value[1][DATA_BITS-1:0];

enum { BS_PRECHARGED, BS_ACTIVATE_ROW, BS_OP, BS_READ, BS_WRITE, BS_OP_END } bank_state = BS_PRECHARGED;
reg[31:0] bank_state_counter = 0, bank_refersh_counter = 1;

xpm_cdc_handshake#(
    .DEST_EXT_HSK(0),
    .WIDTH(32+4),
    .SRC_SYNC_FF(2),
    .DEST_SYNC_FF(3)
) override_cmd_cdc(
    .dest_clk(ddr_clock_i),
    .dest_ack(),
    .dest_out( {override_cmd_ddr, override_addr_ddr} ),
    .dest_req(override_cmd_ddr_ready),

    .src_clk(cpu_clock_i),
    .src_in( {override_cmd_cpu, override_addr_cpu} ),
    .src_rcv(override_cmd_cpu_received),
    .src_send(override_cmd_cpu_send)
);

logic[ADDRESS_BITS-1:0] data_cmd_address_ddr;
logic[CMD_DATA_BITS-1:0] data_cmd_data_ddr;
logic data_cmd_write_ddr;
logic data_cmd_valid_cpu=1'b0, data_cmd_valid_ddr, data_cmd_ack_ddr=1'b0;
logic data_cmd_ack_cpu;
logic data_rsp_ready_ddr = 1'b0;

assign data_cmd_ack = !data_cmd_valid_cpu && !data_cmd_ack_cpu && reset_state_cpu[3]; // Override off

xpm_cdc_handshake#(
    .DEST_EXT_HSK(1),
    .WIDTH( ADDRESS_BITS + 1 /* Write */ + CMD_DATA_BITS ),
    .SRC_SYNC_FF(2),
    .DEST_SYNC_FF(3)
) data_cmd_cdc(
    .dest_clk(ddr_clock_i),
    .dest_ack( data_cmd_ack_ddr ),
    .dest_out( {data_cmd_address_ddr, data_cmd_write_ddr, data_cmd_data_ddr} ),
    .dest_req( data_cmd_valid_ddr ),

    .src_clk(cpu_clock_i),
    .src_in( {data_cmd_address, data_cmd_write, data_cmd_data_i} ),
    .src_rcv( data_cmd_ack_cpu ),
    .src_send( data_cmd_valid_cpu || (data_cmd_valid && data_cmd_ack) )
);

xpm_cdc_single#(
    .SRC_INPUT_REG(0),
    .DEST_SYNC_FF(2)
) data_rsp_ready_cdc(
    .src_clk(ddr_clock_i),
    .dest_clk(cpu_clock_i),
    .src_in(data_rsp_ready_ddr),
    .dest_out(data_rsp_ready)
);

// CPU clock domain
always_ff@(posedge cpu_clock_i) begin
    if( override_cmd_cpu_send && !override_cmd_cpu_received )
        override_cmd_cpu_send <= 1'b1;
    else
        override_cmd_cpu_send <= 1'b0;

    if( ctrl_cmd_valid && ctrl_cmd_ack && ctrl_cmd_write ) begin
        case(ctrl_cmd_address[15:0])
            16'h0000: begin                             // Reset state
                reset_state_cpu <= ctrl_cmd_data;
            end
            16'h0004: begin                             // Override command
                override_cmd_cpu <= ctrl_cmd_data;
                override_cmd_cpu_send <= 1'b1;
            end
            16'h0008: begin                             // Override address
                override_addr_cpu <= ctrl_cmd_data;
            end
        endcase
    end

    if( data_cmd_ack && data_cmd_valid ) begin
        data_cmd_valid_cpu <= 1'b1;
    end
    if( data_cmd_valid_cpu && data_cmd_ack_cpu ) begin
        data_cmd_valid_cpu <= 1'b0;
    end
end

// DDR clock domain
always_ff@(posedge ddr_clock_i) begin
    data_rsp_ready_ddr <= 1'b0;

    if( bypass && override_cmd_ddr_ready ) begin
        ddr3_addr_o <= override_addr_ddr;
        ddr3_ba_o <= override_addr_ddr[31:31-BANK_BITS+1];
        ddr3_cke_o <= reset_state_ddr[5];
        output_cmd <= override_cmd_ddr;

        bank_refersh_counter <= tREFI;
    end else begin
        output_cmd <= 4'b0111; // NOP
        ddr3_addr_o <= 0;
        ddr3_ba_o <= 0;

        if( !data_cmd_valid_ddr ) begin
            // Reset the ACK if we're past the state where we no longer need
            // the latches in the CDC
            if( bank_state!=BS_PRECHARGED && bank_state!=BS_ACTIVATE_ROW )
                data_cmd_ack_ddr<=1'b0;
        end

        if( bank_refersh_counter!=0 )
            bank_refersh_counter <= bank_refersh_counter-1;

        if( bank_state_counter!=0 )
            bank_state_counter<=bank_state_counter-1;
        else begin
            case( bank_state )
                BS_PRECHARGED: begin
                    if( bank_refersh_counter==0 ) begin
                        bank_refersh_counter <= tREFI;
                        bank_state <= BS_PRECHARGED;
                        bank_state_counter <= tRFC;

                        output_cmd <= 5'b0001;  // Refresh
                    end else if( data_cmd_valid_ddr && !data_cmd_ack_ddr ) begin
                        bank_state <= BS_ACTIVATE_ROW;
                        latched_write_data <= data_cmd_data_ddr;
                        data_cmd_ack_ddr <= 1'b1;
                    end
                end
                BS_ACTIVATE_ROW: begin
                    output_cmd <= 4'b0011; // Activate
                    ddr3_ba_o <= data_cmd_address_ddr[ADDRESS_BITS-1:ADDRESS_BITS-BANK_BITS];
                    ddr3_addr_o <= data_cmd_address_ddr[COL_BITS+ROW_BITS-1:COL_BITS];
                    bank_state <= BS_OP;
                    bank_state_counter <= tRCD;
                end
                BS_OP: begin
                    data_transfer_o <= 1'b1;
                    if( !data_cmd_write_ddr ) begin
                        output_cmd <= 4'b0101;  // Read
                        bank_state <= BS_READ;
                        bank_state_counter <= casReadLatency;
                    end else begin
                        output_cmd <= 4'b0100;  // Write
                        bank_state <= BS_WRITE;
                        bank_state_counter <= casWriteLatency;
                        data_write_o <= 1'b1;
                    end
                    ddr3_ba_o <= data_cmd_address_ddr[ADDRESS_BITS-1:ADDRESS_BITS-BANK_BITS];
                    ddr3_addr_o <= 0;
                    ddr3_addr_o[9:0] <= data_cmd_address_ddr[$clog2(DATA_BITS/8)+COL_BITS-1:$clog2(DATA_BITS/8)];
                    if( COL_BITS>10 )
                        ddr3_addr_o[11] <= data_cmd_address_ddr[$clog2(DATA_BITS/8)+10];
                    ddr3_addr_o[10] <= 1'b1;       // Auto precharge
                end
                BS_WRITE: begin
                    bank_state_counter <= HALF_BURST_LENGTH-1;
                    bank_state <= BS_OP_END;
                end
                BS_READ: begin
                    bank_state_counter <= HALF_BURST_LENGTH-1;
                    bank_state <= BS_OP_END;
                end
                BS_OP_END: begin
                    data_transfer_o <= 1'b0;
                    data_write_o <= 1'b0;

                    bank_state <= BS_PRECHARGED;
                    bank_state_counter <= tRP + data_cmd_write_ddr ? write_recovery : 0;

                    data_rsp_ready_ddr <= 1;
                end
            endcase
        end
    end
end


genvar i;
generate

for( i=0; i<HALF_BURST_LENGTH; i++ ) begin : shift_value_gen
    always_ff@(negedge ddr_clock_i) begin
        if( bank_state==BS_WRITE && bank_state_counter==0 ) begin
            shift_value[0][(i+1)*DATA_BITS-1:i*DATA_BITS] = latched_write_data[ i*2*DATA_BITS+DATA_BITS-1:i*2*DATA_BITS ];
        end else begin
            if( i<HALF_BURST_LENGTH-1 )
                shift_value[0][(i+1)*DATA_BITS-1:i*DATA_BITS] = shift_value[0][(i+2)*DATA_BITS-1:(i+1)*DATA_BITS];
            else
                shift_value[0][HALF_BURST_LENGTH*DATA_BITS-1:(HALF_BURST_LENGTH-1)*DATA_BITS] = ddr3_dq_i[0];
        end
    end

    always_ff@(posedge ddr_clock_i) begin
        if( bank_state==BS_WRITE && bank_state_counter==0 ) begin
            shift_value[1][(i+1)*DATA_BITS-1:i*DATA_BITS] = latched_write_data[ (i+1)*2*DATA_BITS-1:i*2*DATA_BITS+DATA_BITS ];
        end else begin
            if( i<HALF_BURST_LENGTH-1 )
                shift_value[1][(i+1)*DATA_BITS-1:i*DATA_BITS] = shift_value[1][(i+2)*DATA_BITS-1:(i+1)*DATA_BITS];
            else
                shift_value[1][HALF_BURST_LENGTH*DATA_BITS-1:(HALF_BURST_LENGTH-1)*DATA_BITS] = ddr3_dq_i[1];
        end
    end

    assign data_data_o[ i*2*DATA_BITS+DATA_BITS-1:i*2*DATA_BITS ] = shift_value[0][(i+1)*DATA_BITS-1:i*DATA_BITS];
    assign data_data_o[ (i+1)*2*DATA_BITS-1:i*2*DATA_BITS+DATA_BITS ] = shift_value[1][(i+1)*DATA_BITS-1:i*DATA_BITS];
end

endgenerate

endmodule
