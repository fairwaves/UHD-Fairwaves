//
// Copyright 2012 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

// Simple SPI core, the simplest, yet complete spi core I can think of

// Settings register controlled.
// 2 settings regs, control and data
// 1 32-bit readback and status signal

// Settings reg map:
//
// BASE+0 divider setting
// bits [15:0] spi clock divider
//
// BASE+1 configuration input
// bits [23:0] slave select, bit0 = slave0 enabled
// bits [29:24] num bits (1 through 32)
// bit [30] data input edge = in data bit latched on rising edge of clock
// bit [31] data output edge = out data bit latched on rising edge of clock
//
// BASE+2 input data
// Writing this register begins a spi transaction.
// Bits are latched out from bit 0.
// Therefore, load this register in reverse.
//
// Readback
// Bits are latched into bit 0.
// Therefore, data will be in-order.

module axis_spi_core
    #(
        //set to 1 for ILA
        parameter DEBUG = 0,

        //tdest width for number of core users
        parameter DESTW = 1,

        //width of serial enables (up to 24 is possible)
        parameter WIDTH = 8,

        //idle state of the spi clock
        parameter CLK_IDLE = 0,

        //idle state of the serial enables
        parameter SEN_IDLE = 24'hffffff
    )
    (
        //clock and synchronous reset
        input clock, input reset,

        //configuration settings bus
        input [DESTW-1:0] CONFIG_tdest,
        input [79:0] CONFIG_tdata,
        input CONFIG_tvalid,
        output CONFIG_tready,

        //32-bit data readback
        output reg [DESTW-1:0] READBACK_tdest,
        output [31:0] READBACK_tdata,
        output READBACK_tvalid,
        input READBACK_tready,

        //spi interface, slave selects, clock, data in, data out
        output [WIDTH-1:0] sen,
        output sclk,
        output mosi,
        input miso
    );

    //state
    localparam WAIT_CONFIG = 0;
    localparam PRE_IDLE = 1;
    localparam CLK_REG = 2;
    localparam CLK_INV = 3;
    localparam POST_IDLE = 4;
    localparam IDLE_SEN = 5;
    localparam WAIT_READBACK = 6;
    reg [2:0] state;

    //configuration settings
    reg [15:0] sclk_divider;
    reg [23:0] slave_select;
    reg [5:0] num_bits;
    reg datain_edge, dataout_edge;

    //output ready/valid signals
    assign CONFIG_tready = (state == WAIT_CONFIG);
    assign READBACK_tvalid = (state == WAIT_READBACK);

    //serial clock either idles or is in one of two clock states
    reg sclk_reg;
    assign sclk = sclk_reg;

    //serial enables either idle or enabled based on state
    wire sen_is_idle = (state == WAIT_CONFIG) || (state == IDLE_SEN);
    wire [23:0] sen24 = (sen_is_idle)? SEN_IDLE : (SEN_IDLE ^ slave_select);
    reg [WIDTH-1:0] sen_reg;
    always @(posedge clock) sen_reg <= sen24[WIDTH-1:0];
    assign sen = sen_reg;

    //data output shift register
    reg [31:0] dataout_reg;
    wire [31:0] dataout_next = {dataout_reg[30:0], 1'b0};
    assign mosi = dataout_reg[31];

    //data input shift register
    reg [31:0] datain_reg;
    wire [31:0] datain_next = {datain_reg[30:0], miso};
    assign READBACK_tdata = datain_reg;

    //counter for spi clock
    reg [15:0] sclk_counter;
    wire sclk_counter_done = (sclk_counter == sclk_divider);
    wire [15:0] sclk_counter_next = (sclk_counter_done)? 0 : sclk_counter + 1;

    //counter for latching bits miso/mosi
    reg [6:0] bit_counter;
    wire [6:0] bit_counter_next = bit_counter + 1;
    wire bit_counter_done = (bit_counter_next == num_bits);

    always @(posedge clock) begin
        if (reset) begin
            state <= WAIT_CONFIG;
            sclk_reg <= CLK_IDLE;
        end
        else begin
            case (state)

            WAIT_CONFIG: begin
                if (CONFIG_tvalid && CONFIG_tready) begin
                    state <= PRE_IDLE;
                end
                {sclk_divider, dataout_edge, datain_edge, num_bits, slave_select, dataout_reg} <= CONFIG_tdata;
                READBACK_tdest <= CONFIG_tdest;
                sclk_counter <= 0;
                bit_counter <= 0;
                sclk_reg <= CLK_IDLE;
            end

            PRE_IDLE: begin
                if (sclk_counter_done) state <= CLK_REG;
                sclk_counter <= sclk_counter_next;
                sclk_reg <= CLK_IDLE;
            end

            CLK_REG: begin
                if (sclk_counter_done) begin
                    state <= CLK_INV;
                    if (datain_edge  != CLK_IDLE)                     datain_reg  <= datain_next;
                    if (dataout_edge != CLK_IDLE && bit_counter != 0) dataout_reg <= dataout_next;
                    sclk_reg <= ~CLK_IDLE; //transition to rising when CLK_IDLE == 0
                end
                sclk_counter <= sclk_counter_next;
            end

            CLK_INV: begin
                if (sclk_counter_done) begin
                    state <= (bit_counter_done)? POST_IDLE : CLK_REG;
                    bit_counter <= bit_counter_next;
                    if (datain_edge  == CLK_IDLE)                      datain_reg  <= datain_next;
                    if (dataout_edge == CLK_IDLE && ~bit_counter_done) dataout_reg <= dataout_next;
                    sclk_reg <= CLK_IDLE; //transition to falling when CLK_IDLE == 0
                end
                sclk_counter <= sclk_counter_next;
            end

            POST_IDLE: begin
                if (sclk_counter_done) state <= IDLE_SEN;
                sclk_counter <= sclk_counter_next;
                sclk_reg <= CLK_IDLE;
            end

            IDLE_SEN: begin
                if (sclk_counter_done) state <= WAIT_READBACK;
                sclk_counter <= sclk_counter_next;
                sclk_reg <= CLK_IDLE;
            end

            WAIT_READBACK: begin
                if (READBACK_tready && READBACK_tvalid) begin
                    state <= WAIT_CONFIG;
                end
            end

            default: state <= WAIT_CONFIG;

            endcase //state
        end
    end

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
            .CLK(clock),
            .DATA(DATA),
            .TRIG0(TRIG0)
        );
        assign TRIG0 =
        {
            4'b0,
            CONFIG_tvalid, CONFIG_tready,
            READBACK_tvalid, READBACK_tready
        };

        assign DATA[79:0] = CONFIG_tdata;
        assign DATA[111:80] = READBACK_tdata;

        assign DATA[112] = CONFIG_tvalid;
        assign DATA[113] = CONFIG_tready;
        assign DATA[114] = READBACK_tvalid;
        assign DATA[115] = READBACK_tready;

        assign DATA[127:120] = state;
    end
    endgenerate

endmodule //axis_spi_core
