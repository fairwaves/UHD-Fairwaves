//
// Copyright 2011 Ettus Research LLC
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

module umtrx_router
    #(
        parameter BUF_SIZE = 9,
        parameter CTRL_BASE = 0
    )
    (
        //wishbone interface for memory mapped CPU frames
        input wb_clk_i,
        input wb_rst_i,
        input wb_we_i,
        input wb_stb_i,
        input [15:0] wb_adr_i,
        input [31:0] wb_dat_i,
        output [31:0] wb_dat_o,
        output wb_ack_o,
        output wb_err_o,
        output wb_rty_o,

        //setting register interface
        input set_stb, input [7:0] set_addr, input [31:0] set_data,
        input set_stb_udp, input [7:0] set_addr_udp, input [31:0] set_data_udp,

        input stream_clk,
        input stream_rst,
        input stream_clr,

        //output status register
        output [31:0] status,

        //eth interfaces
        input [35:0] eth_inp_data, input eth_inp_valid, output eth_inp_ready,
        output [35:0] eth_out_data, output eth_out_valid, input eth_out_ready,

        // Input Interfaces (in to router)
        input [35:0] ctrl_inp_data, input ctrl_inp_valid, output ctrl_inp_ready,
        input [35:0] dsp0_inp_data, input dsp0_inp_valid, output dsp0_inp_ready,
        input [35:0] dsp1_inp_data, input dsp1_inp_valid, output dsp1_inp_ready,
        input [35:0] dsp2_inp_data, input dsp2_inp_valid, output dsp2_inp_ready,
        input [35:0] dsp3_inp_data, input dsp3_inp_valid, output dsp3_inp_ready,
        input [35:0] err0_inp_data, input err0_inp_valid, output err0_inp_ready,
        input [35:0] err1_inp_data, input err1_inp_valid, output err1_inp_ready,

        // Output Interfaces (out of router)
        output [35:0] ctrl_out_data, output ctrl_out_valid, input ctrl_out_ready,
        output [35:0] dsp0_out_data, output dsp0_out_valid, input dsp0_out_ready,
        output [35:0] dsp1_out_data, output dsp1_out_valid, input dsp1_out_ready
    );

    assign wb_err_o = 1'b0;  // Unused for now
    assign wb_rty_o = 1'b0;  // Unused for now

    ////////////////////////////////////////////////////////////////////
    // CPU interface to this packet router
    ////////////////////////////////////////////////////////////////////
    wire [35:0] cpu_inp_data,  cpu_out_data;
    wire        cpu_inp_valid, cpu_out_valid;
    wire        cpu_inp_ready, cpu_out_ready;

    ////////////////////////////////////////////////////////////////////
    // Communication interfaces
    ////////////////////////////////////////////////////////////////////
    wire [35:0] com_inp_data,  com_out_data;
    wire        com_inp_valid, com_out_valid;
    wire        com_inp_ready, com_out_ready;

    ////////////////////////////////////////////////////////////////////
    // Control signals (setting registers and status signals)
    //    - handshake lines for the CPU communication
    //    - setting registers to program the inspector
    ////////////////////////////////////////////////////////////////////

    //assign status output signals
    wire [31:0] cpu_iface_status;
    assign status = {
        cpu_iface_status[31:9], 1'b1, cpu_iface_status[7:0]
    };

    ////////////////////////////////////////////////////////////////////
    // Communication input source
    ////////////////////////////////////////////////////////////////////

    //  short fifo in the packet inspection path to help timing
    fifo_short #(.WIDTH(36)) com_inp_fifo
     (.clk(stream_clk), .reset(stream_rst), .clear(stream_clr),
      .datain(eth_inp_data),  .src_rdy_i(eth_inp_valid), .dst_rdy_o(eth_inp_ready),
      .dataout(com_inp_data), .src_rdy_o(com_inp_valid), .dst_rdy_i(com_inp_ready),
      .space(), .occupied() );

    ////////////////////////////////////////////////////////////////////
    // Communication output source combiner (feeds UDP proto machine)
    ////////////////////////////////////////////////////////////////////
    axi_mux8 #(.WIDTH(36), .BUFFER(1)) combiner
    (
        .clk(stream_clk), .reset(stream_rst), .clear(stream_clr),
        .i0_tdata(cpu_inp_data), .i0_tlast(cpu_inp_data[33]), .i0_tvalid(cpu_inp_valid), .i0_tready(cpu_inp_ready),
        .i1_tdata(ctrl_inp_data), .i1_tlast(ctrl_inp_data[33]), .i1_tvalid(ctrl_inp_valid), .i1_tready(ctrl_inp_ready),
        .i2_tdata(dsp0_inp_data), .i2_tlast(dsp0_inp_data[33]), .i2_tvalid(dsp0_inp_valid), .i2_tready(dsp0_inp_ready),
        .i3_tdata(dsp1_inp_data), .i3_tlast(dsp1_inp_data[33]), .i3_tvalid(dsp1_inp_valid), .i3_tready(dsp1_inp_ready),
        .i4_tdata(dsp2_inp_data), .i4_tlast(dsp2_inp_data[33]), .i4_tvalid(dsp2_inp_valid), .i4_tready(dsp2_inp_ready),
        .i5_tdata(dsp3_inp_data), .i5_tlast(dsp3_inp_data[33]), .i5_tvalid(dsp3_inp_valid), .i5_tready(dsp3_inp_ready),
        .i6_tdata(err0_inp_data), .i6_tlast(err0_inp_data[33]), .i6_tvalid(err0_inp_valid), .i6_tready(err0_inp_ready),
        .i7_tdata(err1_inp_data), .i7_tlast(err1_inp_data[33]), .i7_tvalid(err1_inp_valid), .i7_tready(err1_inp_ready),
        .o_tdata(com_out_data), .o_tlast(), .o_tvalid(com_out_valid), .o_tready(com_out_ready)
    );

    ////////////////////////////////////////////////////////////////////
    // Interface CPU to memory mapped wishbone
    //   - Uses 1 setting register
    ////////////////////////////////////////////////////////////////////
    buffer_int2 #(.BASE(CTRL_BASE), .BUF_SIZE(BUF_SIZE)) cpu_to_wb(
        .clk(stream_clk), .rst(stream_rst | stream_clr),
        .set_stb(set_stb), .set_addr(set_addr), .set_data(set_data),
        .status(cpu_iface_status),
        // Wishbone interface to RAM
        .wb_clk_i(wb_clk_i), .wb_rst_i(wb_rst_i),
        .wb_we_i(wb_we_i),   .wb_stb_i(wb_stb_i),
        .wb_adr_i(wb_adr_i), .wb_dat_i(wb_dat_i),
        .wb_dat_o(wb_dat_o), .wb_ack_o(wb_ack_o),
        // Write FIFO Interface (from PR and into WB)
        .wr_data_i(cpu_out_data),
        .wr_ready_i(cpu_out_valid),
        .wr_ready_o(cpu_out_ready),
        // Read FIFO Interface (from WB and into PR)
        .rd_data_o(cpu_inp_data),
        .rd_ready_o(cpu_inp_valid),
        .rd_ready_i(cpu_inp_ready)
    );

    ////////////////////////////////////////////////////////////////////
    // Packet Dispatcher
    //   - provide buffering before cpu for random + small packet bursts
    ////////////////////////////////////////////////////////////////////
    wire [35:0] _cpu_out_data;
    wire        _cpu_out_valid;
    wire        _cpu_out_ready;

    wire [7:0] pd_dest;
    wire [35:0] pd_out_data;
    wire pd_out_valid, pd_out_ready;

    umtrx_packet_dispatcher #(.BASE(CTRL_BASE+1), .PORTS(7), .DROP_INDEX(7), .CPU_INDEX(0)) packet_dispatcher(
        .clk(stream_clk), .rst(stream_rst), .clr(stream_clr),
        .set_stb(set_stb), .set_addr(set_addr), .set_data(set_data),
        .com_inp_data(com_inp_data), .com_inp_valid(com_inp_valid), .com_inp_ready(com_inp_ready),
        .pd_out_data(pd_out_data), .pd_out_valid(pd_out_valid), .pd_out_ready(pd_out_ready), .pd_dest(pd_dest)
    );

    axi_demux4 #(.WIDTH(36), .BUFFER(1)) splitter
    (
        .clk(stream_clk), .reset(stream_rst), .clear(stream_clr),
        .header(), .dest(pd_dest[2:0]),
        .i_tdata(pd_out_data), .i_tlast(pd_out_data[33]), .i_tvalid(pd_out_valid), .i_tready(pd_out_ready),
        .o0_tdata(_cpu_out_data), .o0_tlast(), .o0_tvalid(_cpu_out_valid), .o0_tready(_cpu_out_ready),
        .o1_tdata(ctrl_out_data), .o1_tlast(), .o1_tvalid(ctrl_out_valid), .o1_tready(ctrl_out_ready),
        .o2_tdata(dsp0_out_data), .o2_tlast(), .o2_tvalid(dsp0_out_valid), .o2_tready(dsp0_out_ready),
        .o3_tdata(dsp1_out_data), .o3_tlast(), .o3_tvalid(dsp1_out_valid), .o3_tready(dsp1_out_ready)/*,
        .o4_tdata(), .o4_tlast(), .o4_tvalid(), .o4_tready(1'b1),
        .o5_tdata(), .o5_tlast(), .o5_tvalid(), .o5_tready(1'b1),
        .o6_tdata(), .o6_tlast(), .o6_tvalid(), .o6_tready(1'b1),
        .o7_tdata(), .o7_tlast(), .o7_tvalid(), .o7_tready(1'b1)*/
    );

    fifo_cascade #(.WIDTH(36), .SIZE(9/*512 lines plenty for short pkts*/)) cpu_out_fifo (
        .clk(stream_clk), .reset(stream_rst), .clear(stream_clr),
        .datain(_cpu_out_data), .src_rdy_i(_cpu_out_valid), .dst_rdy_o(_cpu_out_ready),
        .dataout(cpu_out_data), .src_rdy_o(cpu_out_valid),  .dst_rdy_i(cpu_out_ready)
    );

    ////////////////////////////////////////////////////////////////////
    // UDP TX Protocol framer
    ////////////////////////////////////////////////////////////////////
    prot_eng_tx #(.BASE(0)) udp_prot_eng_tx
     (.clk(stream_clk), .reset(stream_rst), .clear(stream_clr),
      .set_stb(set_stb_udp), .set_addr(set_addr_udp), .set_data(set_data_udp),
      .datain(com_out_data),  .src_rdy_i(com_out_valid), .dst_rdy_o(com_out_ready),
      .dataout(eth_out_data), .src_rdy_o(eth_out_valid), .dst_rdy_i(eth_out_ready) );

endmodule // umtrx_router
