//
// Copyright 2011-2012 Ettus Research LLC
//
// Packet dispatcher with fifo36 interface and 4 outputs.
//
// The packet dispatcher expects 2-byte padded ethernet frames.
// The frames will be inspected at ethernet, IPv4, UDP, and VRT layers.
//
// The following registers are used for dispatcher control:
//   * base + 0 = this ipv4 address (32 bits)
//   * base + 1 = udp server port (lower 16 bits)
//

module umtrx_packet_dispatcher
    #(
        parameter BASE = 0,
        parameter PORTS = 4,
        parameter DROP_INDEX = 0,
        parameter CPU_INDEX = 1
    )
    (
        //clocking and reset interface:
        input clk, input rst, input clr,

        //setting register interface:
        input set_stb, input [7:0] set_addr, input [31:0] set_data,

        //input stream interfaces:
        input [35:0] com_inp_data, input com_inp_valid, output com_inp_ready,

        //output stream interfaces:
        output [35:0] pd_out_data, output pd_out_valid, input pd_out_ready, output reg [7:0] pd_dest
    );

    //setting register to program the IP address
    wire [31:0] my_ip_addr;
    setting_reg #(.my_addr(BASE+0)) sreg_ip_addr(
        .clk(clk),.rst(rst),
        .strobe(set_stb),.addr(set_addr),.in(set_data),
        .out(my_ip_addr),.changed()
    );

    //setting register to program the UDP DSP port
    wire [15:0] my_udp_port;
    setting_reg #(.my_addr(BASE+1), .width(16)) sreg_udp_port(
        .clk(clk),.rst(rst),
        .strobe(set_stb),.addr(set_addr),.in(set_data),
        .out(my_udp_port),.changed()
    );

    ////////////////////////////////////////////////////////////////////
    // Communication input inspector
    //   - inspect com input and send it to DSP, EXT, CPU, or BOTH
    ////////////////////////////////////////////////////////////////////
    localparam PD_STATE_READ_COM_PRE = 0;
    localparam PD_STATE_READ_COM = 1;
    localparam PD_STATE_WRITE_REGS = 2;
    localparam PD_STATE_WRITE_LIVE = 3;

    localparam PD_MAX_NUM_DREGS = 14; //padded_eth + ip + udp + seq + vrt_hdr + vrt_sid
    localparam PD_DREGS_DSP_OFFSET = 11; //offset to start dsp at

    reg [1:0] pd_state;
    reg [3:0] pd_dreg_count; //data registers to buffer headers
    wire [3:0] pd_dreg_count_next = pd_dreg_count + 1'b1;
    wire pd_dreg_counter_done = (pd_dreg_count_next == PD_MAX_NUM_DREGS)? 1'b1 : 1'b0;
    reg [35:0] pd_dregs [PD_MAX_NUM_DREGS-1:0];

    reg is_eth_dst_mac_bcast;
    reg is_eth_type_ipv4;
    reg is_eth_ipv4_proto_udp;
    reg is_eth_ipv4_dst_addr_here;
    reg is_eth_udp_dst_port_here;
    reg is_eth_udp_ctl_port_here;
    reg is_vrt_size_zero;

    //Inspector output flags special case:
    //Inject SOF into flags at first DSP line.
    wire [3:0] pd_out_flags = (
        (pd_dreg_count == PD_DREGS_DSP_OFFSET) &&
        (pd_dest != CPU_INDEX)
    )? 4'b0001 : pd_dregs[pd_dreg_count][35:32];

    //The communication inspector ouput data and valid signals:
    //Mux between com input and data registers based on the state.
    assign pd_out_data = (pd_state == PD_STATE_WRITE_REGS)?
        {pd_out_flags, pd_dregs[pd_dreg_count][31:0]} : com_inp_data
    ;
    assign pd_out_valid =
        (pd_state == PD_STATE_WRITE_REGS)? 1'b1          : (
        (pd_state == PD_STATE_WRITE_LIVE)? com_inp_valid : (
    1'b0));

    //The communication inspector ouput ready signal:
    //Always ready when storing to data registers,
    //comes from inspector ready output when live,
    //and otherwise low.
    assign com_inp_ready =
        (pd_state == PD_STATE_READ_COM_PRE)  ? 1'b1         : (
        (pd_state == PD_STATE_READ_COM)      ? 1'b1         : (
        (pd_state == PD_STATE_WRITE_LIVE)    ? pd_out_ready : (
    1'b0)));

    //inspect the incoming data and mark register booleans
    always @(posedge clk)
    if (com_inp_ready & com_inp_valid) begin
        case(pd_dreg_count)
        0: begin
            is_eth_dst_mac_bcast <= (com_inp_data[15:0] == 16'hffff);
        end
        1: begin
            is_eth_dst_mac_bcast <= is_eth_dst_mac_bcast && (com_inp_data[31:0] == 32'hffffffff);
        end
        3: begin
            is_eth_type_ipv4 <= (com_inp_data[15:0] == 16'h800);
        end
        6: begin
            is_eth_ipv4_proto_udp <= (com_inp_data[23:16] == 8'h11);
        end
        8: begin
            is_eth_ipv4_dst_addr_here <= (com_inp_data[31:0] == my_ip_addr);
        end
        9: begin
            is_eth_udp_dst_port_here <= (com_inp_data[15:0] == my_udp_port);
        end
        12: begin
            is_vrt_size_zero <= (com_inp_data[15:0] == 16'h0);
        end
        endcase //pd_dreg_count
    end

    always @(posedge clk)
    if(rst | clr) begin
        pd_state <= PD_STATE_READ_COM_PRE;
        pd_dreg_count <= 0;
    end
    else begin
        case(pd_state)
        PD_STATE_READ_COM_PRE: begin
            if (com_inp_ready & com_inp_valid & com_inp_data[32]) begin
                pd_state <= PD_STATE_READ_COM;
                pd_dreg_count <= pd_dreg_count_next;
                pd_dregs[pd_dreg_count] <= com_inp_data;
            end
        end

        PD_STATE_READ_COM: begin
            if (com_inp_ready & com_inp_valid) begin
                pd_dregs[pd_dreg_count] <= com_inp_data;
                if (pd_dreg_counter_done | com_inp_data[33]) begin
                    pd_state <= PD_STATE_WRITE_REGS;
                    pd_dreg_count <= 0;

                    //---------- begin inspection decision -----------//
                    //EOF or bcast or not IPv4 or not UDP:
                    if (
                        com_inp_data[33] || is_eth_dst_mac_bcast ||
                        ~is_eth_type_ipv4 || ~is_eth_ipv4_proto_udp
                    ) begin
                        pd_dest <= CPU_INDEX;
                    end

                    //not my IP address:
                    else if (~is_eth_ipv4_dst_addr_here) begin
                        pd_dest <= DROP_INDEX;
                    end

                    //UDP data port and VRT:
                    else if (is_eth_udp_dst_port_here && ~is_vrt_size_zero) begin
                        pd_dest <= com_inp_data[7:0];
                        pd_dreg_count <= PD_DREGS_DSP_OFFSET;
                    end

                    //other:
                    else begin
                        pd_dest <= CPU_INDEX;
                    end
                    //---------- end inspection decision -------------//

                end
                else begin
                    pd_dreg_count <= pd_dreg_count_next;
                end
            end
        end

        PD_STATE_WRITE_REGS: begin
            if (pd_out_ready & pd_out_valid) begin
                if (pd_out_data[33]) begin
                    pd_state <= PD_STATE_READ_COM_PRE;
                    pd_dreg_count <= 0;
                end
                else if (pd_dreg_counter_done) begin
                    pd_state <= PD_STATE_WRITE_LIVE;
                    pd_dreg_count <= 0;
                end
                else begin
                    pd_dreg_count <= pd_dreg_count_next;
                end
            end
        end

        PD_STATE_WRITE_LIVE: begin
            if (pd_out_ready & pd_out_valid & pd_out_data[33]) begin
                pd_state <= PD_STATE_READ_COM_PRE;
            end
        end

        endcase //pd_state
    end

endmodule // packet_dispatcher36_x3
