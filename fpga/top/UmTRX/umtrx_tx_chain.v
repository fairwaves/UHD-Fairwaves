//
// UmTRX TX chain with frontend, dsp, vita deframer and clock domain crossing
//

module umtrx_tx_chain
#(
    parameter PROT_DEST = 0, //framer index
    parameter DSPNO = 0, //the dsp unit number: 0, 1, 2...
    parameter DSP_BASE = 0,
    parameter CTRL_BASE = 0,
    parameter FIFOSIZE = 10,
    parameter DEBUG = 0
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

    //settings bus fe clock domain
    input set_stb_fe,
    input [7:0] set_addr_fe,
    input [31:0] set_data_fe,

    //fe clock domain
    output [23:0] front_i,
    output [23:0] front_q,
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

    vita_tx_chain #(.BASE(CTRL_BASE), .UNIT(PROT_DEST), .FIFOSIZE(FIFOSIZE),
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
    axi_fifo_2clk #(.WIDTH(36), .SIZE(0)) fifo_2clock_vita
    (
        .i_aclk(sys_clk), .i_tdata(vita_data_sys), .i_tvalid(vita_valid_sys), .i_tready(vita_ready_sys),
        .o_aclk(dsp_clk), .o_tdata(vita_data_dsp), .o_tvalid(vita_valid_dsp), .o_tready(vita_ready_dsp),
        .reset(dsp_rst | sys_rst)
    );
    axi_fifo_2clk #(.WIDTH(36), .SIZE(0)) fifo_2clock_err
    (
        .i_aclk(dsp_clk), .i_tdata(err_data_dsp), .i_tvalid(err_valid_dsp), .i_tready(err_ready_dsp),
        .o_aclk(sys_clk), .o_tdata(err_data_sys), .o_tvalid(err_valid_sys), .o_tready(err_ready_sys),
        .reset(dsp_rst | sys_rst)
    );

    /*******************************************************************
     * Debug
     ******************************************************************/
    generate
    if (DEBUG == 1) begin
        wire [35:0] CONTROL0;
        chipscope_icon chipscope_icon
        (
            .CONTROL0(CONTROL0)
        );
        wire [255:0] DATA;
        wire [7:0] TRIG0;
        chipscope_ila chipscope_ila
        (
            .CONTROL(CONTROL0),
            .CLK(dsp_clk),
            .DATA(DATA),
            .TRIG0(TRIG0)
        );
        assign TRIG0 =
        {
            vita_strobe, duc_strobe,
            vita_run, duc_run,
            dac_stb, run,
            2'b0
        };
        assign DATA[31:0] = vita_data_dsp[31:0];
        assign DATA[63:32] = vita_sample;
        assign DATA[95:64] = duc_sample;
        assign DATA[127:96] = {front_i[23:8], front_q[23:8]};
    end
    endgenerate

endmodule //umtrx_tx_chain
