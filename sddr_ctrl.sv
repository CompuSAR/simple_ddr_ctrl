`timescale 1ns / 1ps

module sddr_ctrl#(
        BANK_BITS = 3,
        ROW_BITS = 14,
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
        output                                          ctrl_rsp_valid,
        output [31:0]                                   ctrl_rsp_data,

        // Data interfaces
        input                                           data_cmd_valid,
        input [BANK_BITS+ROW_BITS+COL_BITS+$clog2(DATA_BITS/8)-1:0]
                                                        data_cmd_address,
        input                                           data_cmd_write,
        output                                          data_cmd_ack,
        output logic                                    data_rsp_valid,
        input [BURST_LENGTH*DATA_BITS-1:0]              data_cmd_data_i,
        output [BURST_LENGTH*DATA_BITS-1:0]             data_rsp_data_o,

        // phy interfaces
        output logic                                    ddr3_cke_o,
        output                                          ddr3_ras_n_o,
        output                                          ddr3_cas_n_o,
        output                                          ddr3_we_n_o,

        output                                          ddr3_cs_n_o,

        output logic [BANK_BITS-1:0]                    ddr3_ba_o,
        output logic [ROW_BITS-1:0]                     ddr3_addr_o,
        output                                          ddr3_odt_o,
//        output [DATA_BITS/8-1:0]                        ddr3_dm_o,
        output [DATA_BITS-1:0]                          ddr3_dq_o[1:0],
        input [DATA_BITS-1:0]                           ddr3_dq_i[BURST_LENGTH-1:0],

        output logic                                    data_transfer_o,
        output                                          data_write_o,
        output                                          write_level_o,
        output [31:0]                                   delay_inc_o,
        output                                          dqs_out_o
    );

localparam PARITY_BITS = $clog2(DATA_BITS/8);
localparam ADDRESS_BITS = BANK_BITS+ROW_BITS+COL_BITS+PARITY_BITS;
localparam HALF_BURST_LENGTH = BURST_LENGTH/2;
localparam CMD_DATA_BITS = BURST_LENGTH*DATA_BITS;

logic [31:0] reset_state_cpu=0, reset_state_ddr; // State of the reset signals
xpm_cdc_array_single#(.SRC_INPUT_REG(0), .WIDTH(32))
reset_state_cdc(.src_clk(cpu_clock_i), .dest_clk(ddr_clock_i), .src_in(reset_state_cpu), .dest_out(reset_state_ddr));

// Bits: CS, RAS, CAS, WE.
logic [3:0] override_cmd_cpu, override_cmd_ddr, output_cmd;
logic override_cmd_cpu_send = 1'b0, override_cmd_ddr_ready, override_cmd_cpu_received;
logic [31:0] override_addr_cpu, override_addr_ddr;
logic odt_ddr = 1'b0;
logic data_write_reg;

logic[1:0] delay_inc_cpu=2'b0, delay_inc_ddr;
logic delay_inc_send_cpu = 1'b0, delay_in_send_ddr, delay_inc_ack_cpu;

assign data_write_o = data_write_reg;
assign ddr3_we_n_o = output_cmd[0];
assign ddr3_cas_n_o = output_cmd[1];
assign ddr3_ras_n_o = output_cmd[2];
assign ddr3_cs_n_o = output_cmd[3];

assign ctrl_cmd_ack = !override_cmd_cpu_received && !override_cmd_cpu_send && (!delay_inc_ack_cpu && !delay_inc_send_cpu);

assign ddr_reset_n_o            = reset_state_ddr[0];
assign ddr_phy_reset_n_o        = reset_state_ddr[1];
wire ctrl_reset                 = reset_state_cpu[2];
wire bypass_ddr                 = !reset_state_ddr[3];
wire bypass_cpu                 = !reset_state_cpu[3];
assign ddr3_odt_o               = reset_state_ddr[4] || !reset_state_ddr[4] && odt_ddr;
//assign ddr3_cke_o               = reset_state_ddr[5];
assign write_level_o            = reset_state_ddr[6];
assign dqs_out_o                = reset_state_ddr[7];

logic [CMD_DATA_BITS-1:0] latched_write_data, ordered_read_value, latched_read_value;
logic [HALF_BURST_LENGTH*DATA_BITS-1:0] shift_value[1:0];

assign ddr3_dq_o[0] = shift_value[0][DATA_BITS-1:0];
assign ddr3_dq_o[1] = shift_value[1][DATA_BITS-1:0];

localparam RefreshCounterBits = $clog2(tREFI);

enum { BS_PRECHARGED, BS_ACTIVATE_ROW, BS_OP, BS_READ, BS_READ_END, BS_WRITE, BS_WRITE_END } bank_state = BS_PRECHARGED;
reg[7:0] bank_state_counter = 0;
reg[RefreshCounterBits-1:0] refresh_counter = tREFI;
logic bank_state_counter_zero = 1'b0;

xpm_cdc_handshake#(
    .DEST_EXT_HSK(0),
    .WIDTH(32+4),
    .SRC_SYNC_FF(2),
    .DEST_SYNC_FF(3),
    .SIM_ASSERT_CHK(1)
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
logic data_cmd_write_ddr, current_op_write;
logic data_cmd_valid_cpu=1'b0, data_cmd_valid_ddr, data_cmd_ack_ddr=1'b0;
logic data_cmd_ack_cpu;
logic data_rsp_valid_ddr = 1'b0;
logic refresh_pending_cpu = 1'b0, refresh_pending_ddr;
logic refresh_pending_ack_ddr = 1'b0, refresh_pending_ack_cpu;
logic bypass_write = 1'b0;

assign data_cmd_ack = !data_cmd_valid_cpu && !data_cmd_ack_cpu && !bypass_cpu;

xpm_cdc_handshake#(
    .DEST_EXT_HSK(1),
    .WIDTH( ADDRESS_BITS + 1 /* Write */ + CMD_DATA_BITS ),
    .SRC_SYNC_FF(2),
    .DEST_SYNC_FF(3),
    .SIM_ASSERT_CHK(1)
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

logic rsp_valid;
xpm_cdc_handshake#(
    .DEST_EXT_HSK(0),
    .WIDTH( CMD_DATA_BITS ),
    .SRC_SYNC_FF(2),
    .DEST_SYNC_FF(3),
    .SIM_ASSERT_CHK(1)
) data_rsp_cdc(
    .src_clk(ddr_clock_i),
    .src_in( latched_read_value ),
    .src_rcv( data_rsp_cdc_ack_ddr ),
    .src_send( data_rsp_valid_ddr ),

    .dest_clk(cpu_clock_i),
    .dest_ack(),
    .dest_out( data_rsp_data_o ),
    .dest_req( rsp_valid )
);

assign ctrl_rsp_data = data_rsp_data_o;
assign data_rsp_valid = bypass_cpu ? 1'b0 : rsp_valid;
assign ctrl_rsp_valid = bypass_cpu ? rsp_valid : 1'b0;

xpm_cdc_single#(
    .SRC_INPUT_REG(0)
) refresh_pending_cdc(
    .src_clk(cpu_clock_i),
    .dest_clk(ddr_clock_i),

    .src_in(refresh_pending_cpu),
    .dest_out(refresh_pending_ddr)
);

xpm_cdc_single#(
    .SRC_INPUT_REG(0)
) refresh_pending_ack_cdc(
    .src_clk(ddr_clock_i),
    .dest_clk(cpu_clock_i),

    .src_in(refresh_pending_ack_ddr),
    .dest_out(refresh_pending_ack_cpu)
);

xpm_cdc_handshake#(
    .DEST_EXT_HSK(0),
    .DEST_SYNC_FF(2),
    .SRC_SYNC_FF(2),
    .WIDTH(2),
    .SIM_ASSERT_CHK(1)
) delay_inc_cdc(
    .src_clk(cpu_clock_i),
    .src_in(delay_inc_cpu),
    .src_send(delay_inc_send_cpu),
    .src_rcv(delay_inc_ack_cpu),

    .dest_clk(ddr_clock_i),
    .dest_req(delay_inc_send_ddr),
    .dest_out(delay_inc_ddr)
);
assign delay_inc_o = delay_inc_send_ddr ? { 30'b0, delay_inc_ddr } : 32'b0;

// CPU clock domain
always_ff@(posedge cpu_clock_i) begin
    if( override_cmd_cpu_send && !override_cmd_cpu_received )
        override_cmd_cpu_send <= 1'b1;
    else
        override_cmd_cpu_send <= 1'b0;

    if( delay_inc_ack_cpu ) begin
        delay_inc_send_cpu <= 1'b0;
        delay_inc_cpu <= 2'b0;
    end

    if( ctrl_cmd_valid && ctrl_cmd_ack ) begin
        if( ctrl_cmd_write ) begin
            case(ctrl_cmd_address[15:0])
                16'h0000: begin                                 // Reset state
                    reset_state_cpu <= ctrl_cmd_data;
                end
                16'h0004: begin                                 // Override command
                    override_cmd_cpu <= ctrl_cmd_data;
                    override_cmd_cpu_send <= 1'b1;
                end
                16'h0008: begin                                 // Override address
                    override_addr_cpu <= ctrl_cmd_data;
                end
                16'h0010: begin                                 // Delay increase
                    delay_inc_cpu <= ctrl_cmd_data;
                    delay_inc_send_cpu <= 1'b1;
                end
            endcase
        end else begin
            case(ctrl_cmd_address[15:0])
                16'h000c: begin                             // Override mode read
                    if(bypass_cpu) begin
                        override_cmd_cpu <= 4'b0101;        // READ
                        override_cmd_cpu_send <= 1'b1;
                    end
                end
            endcase
        end
    end

    if( data_cmd_ack && data_cmd_valid ) begin
        data_cmd_valid_cpu <= 1'b1;
    end
    if( data_cmd_valid_cpu && data_cmd_ack_cpu ) begin
        data_cmd_valid_cpu <= 1'b0;
    end
end

// Refresh counter management
always_ff@(posedge cpu_clock_i) begin
    if( refresh_pending_cpu && refresh_pending_ack_cpu )
        refresh_pending_cpu <= 1'b0;

    if( refresh_counter!=0 ) begin
        if( !bypass_cpu )
            refresh_counter <= refresh_counter-1;
    end else begin
        refresh_counter <= tREFI;
        refresh_pending_cpu <= 1'b1;
    end
end

// DDR clock domain
always_ff@(posedge ddr_clock_i) begin
    if( !bank_state_counter_zero ) begin
        bank_state_counter<=bank_state_counter-1;
        bank_state_counter_zero <= bank_state_counter==1;
    end

    output_cmd <= 4'b0111;      // NOP
    if( bypass_ddr && override_cmd_ddr_ready ) begin
        if( override_cmd_ddr==4'b0101 ) begin         // READ
            bank_state <= BS_OP;
            bypass_write <= 1'b0;
            data_rsp_valid_ddr <= 0;
        end else begin
            ddr3_addr_o <= override_addr_ddr;
            ddr3_ba_o <= override_addr_ddr[31:31-BANK_BITS+1];
            ddr3_cke_o <= reset_state_ddr[5];
            output_cmd <= override_cmd_ddr;
        end
    end else begin
        ddr3_addr_o <= 0;
        ddr3_ba_o <= 0;

        if( refresh_pending_ack_ddr && !refresh_pending_ddr )
            refresh_pending_ack_ddr <= 1'b0;
    end

    if( bank_state_counter_zero ) begin
        case( bank_state )
            BS_PRECHARGED: begin
                if( !bypass_ddr ) begin
                    if( refresh_pending_ddr ) begin
                        refresh_pending_ack_ddr <= 1'b1;

                        bank_state <= BS_PRECHARGED;
                        bank_state_counter <= tRFC;
                        bank_state_counter_zero <= 1'b0;

                        output_cmd <= 5'b0001;  // Refresh
                    end else if( data_cmd_valid_ddr && !data_cmd_ack_ddr ) begin
                        bank_state <= BS_ACTIVATE_ROW;
                        latched_write_data <= data_cmd_data_ddr;
                        data_cmd_ack_ddr <= 1'b1;
                    end
                end
            end
            BS_ACTIVATE_ROW: begin
                data_rsp_valid_ddr <= 0;

                output_cmd <= 4'b0011; // Activate
                ddr3_ba_o <= data_cmd_address_ddr[ADDRESS_BITS-1:ADDRESS_BITS-BANK_BITS];
                ddr3_addr_o <= data_cmd_address_ddr[COL_BITS+ROW_BITS+PARITY_BITS-1:COL_BITS+PARITY_BITS];
                current_op_write <= data_cmd_write_ddr;
                bank_state <= BS_OP;
                bank_state_counter <= tRCD;
                bank_state_counter_zero <= 1'b0;
            end
            BS_OP: begin
                data_cmd_ack_ddr <= 1'b0;
                data_transfer_o <= 1'b1;
                if( bypass_ddr ? !bypass_write : !data_cmd_write_ddr ) begin
                    output_cmd <= 4'b0101;  // Read
                    bank_state <= BS_READ;
                    bank_state_counter <= casReadLatency+1;
                    bank_state_counter_zero <= 1'b0;
                end else begin
                    output_cmd <= 4'b0100;  // Write
                    bank_state <= BS_WRITE;
                    bank_state_counter <= casWriteLatency-1;
                    bank_state_counter_zero <= 1'b0;
                    data_write_reg <= 1'b1;
                    odt_ddr <= 1'b1;
                end
                if( bypass_ddr ) begin
                    ddr3_addr_o <= override_addr_ddr;
                    ddr3_ba_o <= override_addr_ddr[31:31-BANK_BITS+1];
                end else begin
                    ddr3_ba_o <= data_cmd_address_ddr[ADDRESS_BITS-1:ADDRESS_BITS-BANK_BITS];
                    ddr3_addr_o <= 0;
                    ddr3_addr_o[9:0] <= data_cmd_address_ddr[COL_BITS+PARITY_BITS-1:PARITY_BITS];
                    if( COL_BITS>10 )
                        ddr3_addr_o[11] <= data_cmd_address_ddr[PARITY_BITS+10];
                    ddr3_addr_o[10] <= 1'b1;       // Auto precharge
                end
            end
            BS_WRITE: begin
                bank_state_counter <= HALF_BURST_LENGTH;
                bank_state_counter_zero <= 1'b0;
                bank_state <= BS_WRITE_END;
            end
            BS_READ: begin
                bank_state_counter <= HALF_BURST_LENGTH;
                bank_state_counter_zero <= 1'b0;
                bank_state <= BS_READ_END;
            end
            BS_WRITE_END: begin
                data_transfer_o <= 1'b0;
                data_write_reg <= 1'b0;

                bank_state <= BS_PRECHARGED;
                bank_state_counter <= tRP + write_recovery;
                bank_state_counter_zero <= 1'b0;

                odt_ddr <= 1'b0;
            end
            BS_READ_END: begin
                data_transfer_o <= 1'b0;
                data_write_reg <= 1'b0;

                bank_state <= BS_PRECHARGED;
                bank_state_counter <= tRP;
                bank_state_counter_zero <= 1'b0;

                latched_read_value <= ordered_read_value;
                data_rsp_valid_ddr <= 1;
            end
        endcase
    end
end


genvar i;
generate

for( i=0; i<HALF_BURST_LENGTH; i++ ) begin : shift_value_gen
    always_ff@(posedge ddr_clock_i) begin
        if( current_op_write && bank_state_counter_zero ) begin
            shift_value[0][(i+1)*DATA_BITS-1:i*DATA_BITS] <= latched_write_data[ i*2*DATA_BITS+DATA_BITS-1:i*2*DATA_BITS ];
        end else begin
            if( i<HALF_BURST_LENGTH-1 )
                shift_value[0][(i+1)*DATA_BITS-1:i*DATA_BITS] <= shift_value[0][(i+2)*DATA_BITS-1:(i+1)*DATA_BITS];
            else
                shift_value[0][HALF_BURST_LENGTH*DATA_BITS-1:(HALF_BURST_LENGTH-1)*DATA_BITS] <= ddr3_dq_i[0];
        end
    end

    always_ff@(posedge ddr_clock_i) begin
        if( current_op_write && bank_state_counter_zero ) begin
            shift_value[1][(i+1)*DATA_BITS-1:i*DATA_BITS] <= latched_write_data[ (i+1)*2*DATA_BITS-1:i*2*DATA_BITS+DATA_BITS ];
        end else begin
            if( i<HALF_BURST_LENGTH-1 )
                shift_value[1][(i+1)*DATA_BITS-1:i*DATA_BITS] <= shift_value[1][(i+2)*DATA_BITS-1:(i+1)*DATA_BITS];
            else
                shift_value[1][HALF_BURST_LENGTH*DATA_BITS-1:(HALF_BURST_LENGTH-1)*DATA_BITS] <= ddr3_dq_i[1];
        end
    end
end

for(i=0; i<BURST_LENGTH; ++i) begin : read_map
    assign ordered_read_value[DATA_BITS*(i+1)-1:DATA_BITS*i] = ddr3_dq_i[i];
end

endgenerate

endmodule
