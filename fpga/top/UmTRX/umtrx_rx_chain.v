//
// UmTRX RX chain with frontend, dsp, vita framer and clock domain crossing
//

module umtrx_rx_chain
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
    input [11:0] adc_i,
    input [11:0] adc_q,
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
     * Create settings bus for fe clock domain
     ******************************************************************/
    wire set_stb_fe;
    wire [7:0] set_addr_fe;
    wire [31:0] set_data_fe;
    settings_bus_crossclock settings_bus_fe_crossclock
     (.clk_i(dsp_clk), .rst_i(dsp_rst), .set_stb_i(set_stb_dsp), .set_addr_i(set_addr_dsp), .set_data_i(set_data_dsp),
      .clk_o(fe_clk), .rst_o(fe_rst), .set_stb_o(set_stb_fe), .set_addr_o(set_addr_fe), .set_data_o(set_data_fe));

    /*******************************************************************
     * Cross ADC signals from dsp to fe clock domain
     ******************************************************************/
    reg [15:0] adc_a_16, adc_b_16;
    always @(posedge dsp_clk) begin
        adc_a_16 <= {adc_i, 4'b0};
        adc_b_16 <= {adc_q, 4'b0};
    end

    /*******************************************************************
     * RX frontend on fe clock domain
     ******************************************************************/
    wire [23:0] front_i, front_q;
    rx_frontend #(.BASE(FRONT_BASE)) rx_frontend
    (
        .clk(fe_clk), .rst(fe_rst),
        .set_stb(set_stb_fe),.set_addr(set_addr_fe),.set_data(set_data_fe),
        .i_out(front_i), .q_out(front_q), .run(1'b1),
        .adc_a(adc_a_16), .adc_b(adc_b_16)
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

    vita_rx_chain #(.BASE(CTRL_BASE),.UNIT(DSPNO),.FIFOSIZE(FIFOSIZE), .DSP_NUMBER(DSPNO)) vita_rx_chain
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
    fifo_2clock #(.WIDTH(36)) fifo_2clock_vita
    (
        .wclk(dsp_clk), .datain(vita_data_dsp), .src_rdy_i(vita_valid_dsp), .dst_rdy_o(vita_ready_dsp),
        .rclk(sys_clk), .dataout(vita_data_sys), .src_rdy_o(vita_valid_sys), .dst_rdy_i(vita_ready_sys),
        .arst(dsp_rst | sys_rst)
    );

endmodule //umtrx_rx_chain
