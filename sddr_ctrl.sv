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
        output logic                                    ctrl_cmd_ack,
        output                                          ctrl_rsp_ready,
        output [31:0]                                   ctrl_rsp_data,

        // Data interfaces
        input                                           data_cmd_valid,
        input [BANK_BITS+ROW_BITS+COL_BITS+$clog2(DATA_BITS/8)-1:0]
                                                        data_cmd_address,
        input                                           data_cmd_write,
        output logic                                    data_cmd_ack,
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

wire ddr_clock_i = cpu_clock_i;

logic [31:0] tRCD, tRC, tRP, tRFC, tREFI;
logic [15:0] casReadLatency, casWriteLatency;
logic [31:0] write_recovery;

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
//assign ddr3_cke_o               = reset_state[5];

localparam ADDRESS_BITS = BANK_BITS+ROW_BITS+COL_BITS+$clog2(DATA_BITS/8);
localparam HALF_BURST_LENGTH = BURST_LENGTH/2;

logic [BURST_LENGTH*DATA_BITS-1:0] latched_write_value;
logic [ADDRESS_BITS-1:0] latched_address;
logic [HALF_BURST_LENGTH*DATA_BITS-1:0] shift_value[1:0];
enum { STATE_IDLE, STATE_WRITE, STATE_READ } state = STATE_IDLE;

assign data_cmd_ack = state==STATE_IDLE && reset_state[3] /* override off */;
assign ddr3_dq_o[0] = shift_value[0][DATA_BITS-1:0];
assign ddr3_dq_o[1] = shift_value[1][DATA_BITS-1:0];

enum { BS_PRECHARGED, BS_ACTIVATE_ROW, BS_OP, BS_READ, BS_WRITE, BS_OP_END } bank_state = BS_PRECHARGED;
reg[31:0] bank_state_counter = 0, bank_refersh_counter = 0;

// CPU clock domain
always_ff@(posedge cpu_clock_i) begin
    override_cmd_cpu_send <= 1'b0;

    if( ctrl_cmd_valid && ctrl_cmd_ack && ctrl_cmd_write ) begin
        case(ctrl_cmd_address[15:0])
            16'h0000: begin                             // Reset state
                reset_state <= ctrl_cmd_data;
            end
            16'h0004: begin                             // Override command
                override_cmd_cpu <= ctrl_cmd_data;
                override_cmd_cpu_send <= 1'b1;
            end
            16'h0008: begin                             // Override address
                override_addr <= ctrl_cmd_data;
            end
            16'h000c: begin                             // CL and CWL
                casReadLatency <= ctrl_cmd_data[15:0];
                casWriteLatency <= ctrl_cmd_data[31:16];
            end
            16'h0010: begin                             // Write recovery
                write_recovery <= ctrl_cmd_data;
            end
            16'h0014: begin                             // tRCD
                tRCD <= ctrl_cmd_data;
            end
            16'h0018: begin                             // tRC
                tRC <= ctrl_cmd_data;
            end
            16'h001c: begin                             // tRP
                tRP <= ctrl_cmd_data;
            end
            16'h0020: begin                             // tRFC
                tRFC <= ctrl_cmd_data;
            end
            16'h0024: begin                             // tREFI
                tREFI <= ctrl_cmd_data;
            end
        endcase
    end

    if( data_cmd_ack && data_cmd_valid ) begin
        if( data_cmd_write ) begin
            state <= STATE_WRITE;
            latched_write_value <= data_cmd_data_i;
        end else begin
            state <= STATE_READ;
        end
        latched_address <= data_cmd_address;
    end else if( bank_state==BS_OP_END && bank_state_counter==0 )
        state <= STATE_IDLE;
end

// DDR clock domain
always_ff@(posedge ddr_clock_i) begin
    data_rsp_ready <= 1'b0;

    if( !reset_state[3] /* override */ && override_cmd_cpu_send ) begin
        ddr3_addr_o <= override_addr;
        ddr3_ba_o <= override_addr[31:31-BANK_BITS+1];
        ddr3_cke_o <= reset_state[5];
        output_cmd <= override_cmd_cpu; // Remant from the days we had separate CPU and DDR clocks

        bank_refersh_counter <= tREFI;
    end else begin
        output_cmd <= 4'b0111; // NOP
        ddr3_addr_o <= 0;
        ddr3_ba_o <= 0;

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
                    end else if( state!=STATE_IDLE )
                        bank_state <= BS_ACTIVATE_ROW;
                end
                BS_ACTIVATE_ROW: begin
                    output_cmd <= 4'b0011; // Activate
                    ddr3_ba_o <= latched_address[ADDRESS_BITS-1:ADDRESS_BITS-BANK_BITS];
                    ddr3_addr_o <= latched_address[COL_BITS+ROW_BITS-1:COL_BITS];
                    bank_state <= BS_OP;
                    bank_state_counter <= tRCD;
                end
                BS_OP: begin
                    data_transfer_o <= 1'b1;
                    if( state==STATE_READ ) begin
                        output_cmd <= 4'b0101;  // Read
                        bank_state <= BS_READ;
                        bank_state_counter <= casReadLatency;
                    end else begin
                        output_cmd <= 4'b0100;  // Write
                        bank_state <= BS_WRITE;
                        bank_state_counter <= casWriteLatency;
                        data_write_o <= 1'b1;
                    end
                    ddr3_ba_o <= latched_address[ADDRESS_BITS-1:ADDRESS_BITS-BANK_BITS];
                    ddr3_addr_o <= 0;
                    ddr3_addr_o[9:0] <= latched_address[$clog2(DATA_BITS/8)+COL_BITS-1:$clog2(DATA_BITS/8)];
                    if( COL_BITS>10 )
                        ddr3_addr_o[11] = latched_address[$clog2(DATA_BITS/8)+10];
                    ddr3_addr_o[10] = 1'b1;       // Auto precharge
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
                    bank_state_counter <= tRP + (state==STATE_WRITE) ? write_recovery : 0;

                    data_rsp_ready <= 1;
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
            shift_value[0][(i+1)*DATA_BITS-1:i*DATA_BITS] = latched_write_value[ i*2*DATA_BITS+DATA_BITS-1:i*2*DATA_BITS ];
        end else begin
            if( i<HALF_BURST_LENGTH-1 )
                shift_value[0][(i+1)*DATA_BITS-1:i*DATA_BITS] = shift_value[0][(i+2)*DATA_BITS-1:(i+1)*DATA_BITS];
            else
                shift_value[0][HALF_BURST_LENGTH*DATA_BITS-1:(HALF_BURST_LENGTH-1)*DATA_BITS] = ddr3_dq_i[0];
        end
    end

    always_ff@(posedge ddr_clock_i) begin
        if( bank_state==BS_WRITE && bank_state_counter==0 ) begin
            shift_value[1][(i+1)*DATA_BITS-1:i*DATA_BITS] = latched_write_value[ (i+1)*2*DATA_BITS-1:i*2*DATA_BITS+DATA_BITS ];
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
