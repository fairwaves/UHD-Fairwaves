//
// UmTRX TX chain with frontend, dsp, vita deframer and clock domain crossing
//

module umtrx_tx_chain
#(
    //the dsp unit number: 0, 1, 2...
    parameter DSPNO = 0,
    parameter FRONT_BASE = 0,
    parameter DSP_BASE = 0,
    parameter CTRL_BASE = 0,
    parameter FIFOSIZE = 10
)
(
    input sys_clk,
    input sys_rst,

    input dsp_clk,
    input dsp_rst,

    input fe_clk,
    input fe_rst,

    //settings bus dsp clock domain
    input set_stb_dsp,
    input [7:0] set_addr_dsp,
    input [31:0] set_data_dsp,

    //dsp clock domain
    output reg [11:0] dac_i,
    output reg [11:0] dac_q,
    input dac_stb,
    output run,

    //sys clock domain
    input [35:0] vita_data_sys,
    input vita_valid_sys,
    output vita_ready_sys,

    //sys clock domain
    output [35:0] err_data_sys,
    output err_valid_sys,
    input err_ready_sys,

    //vita time in dsp clock domain
    wire [63:0] vita_time
);

    /*******************************************************************
     * Create settings bus for fe clock domain
     ******************************************************************/
    wire set_stb_fe;
    wire [7:0] set_addr_fe;
    wire [31:0] set_data_fe;
    settings_bus_crossclock settings_bus_fe_crossclock
     (.clk_i(dsp_clk), .rst_i(dsp_rst), .set_stb_i(set_stb_dsp), .set_addr_i(set_addr_dsp), .set_data_i(set_data_dsp),
      .clk_o(fe_clk), .rst_o(fe_rst), .set_stb_o(set_stb_fe), .set_addr_o(set_addr_fe), .set_data_o(set_data_fe));

    /*******************************************************************
     * Cross DAC signals from fe to dsp clock domain
     * dac_i/q come from a register on the fe clock domain
     ******************************************************************/
    wire [15:0] dac_a_16, dac_b_16;
    always @(posedge fe_clk) begin
        dac_i <= dac_a_16[15:4];
        dac_q <= dac_b_16[15:4];
    end

    /*******************************************************************
     * TX frontend on fe clock domain
     ******************************************************************/
    wire [23:0] front_i, front_q;
    tx_frontend #(.BASE(FRONT_BASE)) tx_frontend
    (
        .clk(fe_clk), .rst(fe_rst),
        .set_stb(set_stb_fe),.set_addr(set_addr_fe),.set_data(set_data_fe),
        .tx_i(front_i), .tx_q(front_q), .run(1'b1),
        .dac_a(dac_a_16), .dac_b(dac_b_16)
    );

    /*******************************************************************
     * DUC chain on fe clock domain
     ******************************************************************/
    wire duc_strobe;
    reg duc_run;
    reg duc_clear;
    reg [31:0] duc_sample;
    duc_chain #(.BASE(DSP_BASE), .DSPNO(DSPNO)) duc_chain
    (
        .clk(fe_clk),.rst(fe_rst), .clr(duc_clear),
        .set_stb(set_stb_fe),.set_addr(set_addr_fe),.set_data(set_data_fe),
        .set_stb_user(), .set_addr_user(), .set_data_user(),
        .tx_fe_i(front_i),.tx_fe_q(front_q),
        .sample(duc_sample), .run(duc_run), .strobe(duc_strobe),
        .debug()
    );

    /*******************************************************************
     * Cross deframer signals from dsp to fe clock domain
     ******************************************************************/
    reg vita_strobe; //from fe to dsp
    always @(posedge fe_clk) vita_strobe <= duc_strobe;

    //from dsp to fe
    wire [31:0] vita_sample;
    wire vita_clear;
    wire vita_run;
    always @(posedge dsp_clk) begin
        if (dac_stb) begin
            duc_run <= vita_run;
            duc_clear <= vita_clear;
            duc_sample <= vita_sample;
        end
    end

    assign run = vita_run;

    /*******************************************************************
     * TX VITA deframer
     ******************************************************************/
    wire [35:0] vita_data_dsp, err_data_dsp;
    wire vita_valid_dsp, err_valid_dsp;
    wire vita_ready_dsp, err_ready_dsp;

    vita_tx_chain #(.BASE(CTRL_BASE), .FIFOSIZE(FIFOSIZE),
        .REPORT_ERROR(1), .DO_FLOW_CONTROL(1),
        .PROT_ENG_FLAGS(1), .USE_TRANS_HEADER(1),
        .DSP_NUMBER(DSPNO))
    vita_tx_chain
    (
        .clk(dsp_clk), .reset(dsp_rst),
        .set_stb(set_stb_dsp),.set_addr(set_addr_dsp),.set_data(set_data_dsp),
        .set_stb_user(), .set_addr_user(), .set_data_user(),
        .vita_time(vita_time),
        .tx_data_i(vita_data_dsp), .tx_src_rdy_i(vita_valid_dsp), .tx_dst_rdy_o(vita_ready_dsp),
        .err_data_o(err_data_dsp), .err_src_rdy_o(err_valid_dsp), .err_dst_rdy_i(err_ready_dsp),
        .sample(vita_sample), .strobe(vita_strobe && dac_stb), .run(vita_run), .clear_o(vita_clear),
        .debug()
    );

    /*******************************************************************
     * Cross clock fifo from sys to dsp clock domain
     ******************************************************************/
    fifo_2clock #(.WIDTH(36)) fifo_2clock_vita
    (
        .wclk(sys_clk), .datain(vita_data_sys), .src_rdy_i(vita_valid_sys), .dst_rdy_o(vita_ready_sys),
        .rclk(dsp_clk), .dataout(vita_data_dsp), .src_rdy_o(vita_valid_dsp), .dst_rdy_i(vita_ready_dsp),
        .arst(dsp_rst | sys_rst)
    );
    fifo_2clock #(.WIDTH(36)) fifo_2clock_err
    (
        .wclk(dsp_clk), .datain(err_data_dsp), .src_rdy_i(err_valid_dsp), .dst_rdy_o(err_ready_dsp),
        .rclk(sys_clk), .dataout(err_data_sys), .src_rdy_o(err_valid_sys), .dst_rdy_i(err_ready_sys),
        .arst(dsp_rst | sys_rst)
    );

endmodule //umtrx_tx_chain
