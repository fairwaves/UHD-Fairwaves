//
// UmTRX RX chain with frontend, dsp, vita framer and clock domain crossing
//

module umtrx_rx_chain
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
    input [23:0] front_i,
    input [23:0] front_q,
    input adc_stb,
    output run,

    //sys clock domain
    output [35:0] vita_data_sys,
    output vita_valid_sys,
    input vita_ready_sys,

    //vita time in dsp clock domain
    wire [63:0] vita_time
);

    /*******************************************************************
     * DDC chain on fe clock domain
     ******************************************************************/
    wire ddc_strobe;
    reg ddc_run;
    reg ddc_clear;
    wire [31:0] ddc_sample;
    ddc_chain #(.BASE(DSP_BASE), .DSPNO(DSPNO)) ddc_chain
    (
        .clk(fe_clk), .rst(fe_rst), .clr(ddc_clear),
        .set_stb(set_stb_fe),.set_addr(set_addr_fe),.set_data(set_data_fe),
        .set_stb_user(), .set_addr_user(), .set_data_user(),
        .rx_fe_i(front_i),.rx_fe_q(front_q),
        .sample(ddc_sample), .run(ddc_run), .strobe(ddc_strobe),
        .debug()
    );

    /*******************************************************************
     * Cross framer signals from fe to dsp clock domain
     ******************************************************************/
    reg vita_strobe; //from fe to dsp
    reg [31:0] vita_sample;
    always @(posedge fe_clk) begin
        vita_strobe <= ddc_strobe;
        vita_sample <= ddc_sample;
    end

    //from dsp to fe
    wire vita_clear;
    wire vita_run;
    always @(posedge dsp_clk) begin
        if (adc_stb) begin
            ddc_run <= vita_run;
            ddc_clear <= vita_clear;
        end
    end

    assign run = vita_run;

    /*******************************************************************
     * RX VITA framer
     ******************************************************************/
    wire [35:0] vita_data_dsp;
    wire vita_valid_dsp;
    wire vita_ready_dsp;

    vita_rx_chain #(.BASE(CTRL_BASE),.UNIT(PROT_DEST),.FIFOSIZE(FIFOSIZE), .DSP_NUMBER(DSPNO)) vita_rx_chain
     (.clk(dsp_clk), .reset(dsp_rst),
      .set_stb(set_stb_dsp),.set_addr(set_addr_dsp),.set_data(set_data_dsp),
      .set_stb_user(), .set_addr_user(), .set_data_user(),
      .vita_time(vita_time), .overrun(),
      .sample(vita_sample), .run(vita_run), .strobe(vita_strobe && adc_stb), .clear_o(vita_clear),
      .rx_data_o(vita_data_dsp), .rx_src_rdy_o(vita_valid_dsp), .rx_dst_rdy_i(vita_ready_dsp),
      .debug() );

    /*******************************************************************
     * Cross clock fifo from sys to dsp clock domain
     ******************************************************************/
    wire [35:0] vita_data_sys0;
    wire vita_valid_sys0;
    wire vita_ready_sys0;

    axi_fifo_2clk #(.WIDTH(36), .SIZE(0)) fifo_2clock_vita
    (
        .i_aclk(dsp_clk), .i_tdata(vita_data_dsp), .i_tvalid(vita_valid_dsp), .i_tready(vita_ready_dsp),
        .o_aclk(sys_clk), .o_tdata(vita_data_sys0), .o_tvalid(vita_valid_sys0), .o_tready(vita_ready_sys0),
        .reset(dsp_rst | sys_rst)
    );

    axi_packet_gate #(.WIDTH(36), .SIZE(FIFOSIZE)) fully_buffer_sys_domain
    (
        .clk(sys_clk), .reset(sys_rst), .clear(0),
        .i_tdata(vita_data_sys0), .i_tvalid(vita_valid_sys0), .i_tready(vita_ready_sys0), .i_tlast(vita_data_sys0[33]), .i_terror(0),
        .o_tdata(vita_data_sys), .o_tvalid(vita_valid_sys), .o_tready(vita_ready_sys), .o_tlast()
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
            .CLK(sys_clk),
            .DATA(DATA),
            .TRIG0(TRIG0)
        );
        assign TRIG0 =
        {
            set_stb_dsp, set_stb_fe,
            ddc_run, vita_run,
            vita_valid_dsp, vita_ready_dsp,
            vita_valid_sys, vita_ready_sys
        };
        assign DATA[31:0] = vita_data_sys;
        assign DATA[63:32] = vita_data_dsp;
        assign DATA[95:64] = vita_sample;
        assign DATA[127:96] = ddc_sample;
        assign DATA[191:160] = set_data_dsp;
        assign DATA[223:192] = set_data_fe;
        assign DATA[255] = adc_stb;
        assign DATA[254] = ddc_strobe;
        assign DATA[253] = ddc_run;
        assign DATA[252] = ddc_clear;
        assign DATA[251] = vita_strobe;
        assign DATA[250] = vita_run;
        assign DATA[249] = vita_clear;
        assign DATA[248] = vita_valid_dsp;
        assign DATA[247] = vita_ready_dsp;
        assign DATA[246] = vita_valid_sys;
        assign DATA[245] = vita_ready_sys;
        assign DATA[244] = dsp_rst;
        assign DATA[243] = sys_rst;
        assign DATA[242] = fe_rst;
        assign DATA[241] = set_stb_dsp;
        assign DATA[240] = set_stb_fe;
        assign DATA[239:232] = set_addr_dsp;
        assign DATA[231:224] = set_addr_fe;
    end
    endgenerate

endmodule //umtrx_rx_chain
