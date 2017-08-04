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

// ////////////////////////////////////////////////////////////////////////////////
// Module Name:    umtrx_core
// ////////////////////////////////////////////////////////////////////////////////

module umtrx_core
  (// Clocks
   input sys_clk,
   input dsp_clk,
   input fe_clk,
   input wb_clk,
   input clk_icap, //ICAP timing fixes for UmTRX Spartan-6 FPGA.
   input clk_to_mac,
   input pps_in,
   
   // Misc, debug
   output [7:0] leds,
   output [31:0] debug,
   output [1:0] debug_clk,

   // GMII
   //   GMII-CTRL
   input GMII_COL,
   input GMII_CRS,

   //   GMII-TX
   output [7:0] GMII_TXD,
   output GMII_TX_EN,
   output GMII_TX_ER,
   output GMII_GTX_CLK,
   input GMII_TX_CLK,  // 100mbps clk

   //   GMII-RX
   input [7:0] GMII_RXD,
   input GMII_RX_CLK,
   input GMII_RX_DV,
   input GMII_RX_ER,

   //   GMII-Management
   inout MDIO,
   output MDC,
   input PHY_INTn,   // open drain
   output PHY_RESETn,

   // ADC
   input adc0_strobe,
   input [11:0] adc0_a,
   input [11:0] adc0_b,
   input adc1_strobe,
   input [11:0] adc1_a,
   input [11:0] adc1_b,
   output [1:0] lms_res,

   // DAC
   input dac0_strobe,
   output [11:0] dac0_a,
   output [11:0] dac0_b,
   input dac1_strobe,
   output [11:0] dac1_a,
   output [11:0] dac1_b,

   // I2C
   input scl_pad_i,
   output scl_pad_o,
   output scl_pad_oen_o,
   input sda_pad_i,
   output sda_pad_o,
   output sda_pad_oen_o,

   // Generic SPI
   output sclk,
   output mosi,
   input miso,
   output sen_dac,
   output sen_lms1,
   output sen_lms2,
   //Diversity switches
   output DivSw1,
   output DivSw2,
   input aux_scl_pad_i,
   output aux_scl_pad_o,
   output aux_scl_pad_oen_o,
   input aux_sda_pad_i,
   output aux_sda_pad_o,
   output aux_sda_pad_oen_o,
   //AUX SPI
   output aux_sdat,
   output aux_sclk,
   output aux_sen1,
   output aux_sen2,
   input aux_ld1,
   input aux_ld2,

   //PA control
   output enpa2,
   output enpa1,
   output lowpa,
   output en_dc_sync,

`ifndef NO_EXT_FIFO
   // External RAM
   input [35:0] RAM_D_pi,
   output [35:0] RAM_D_po,
   output RAM_D_poe,
   output [20:0] RAM_A,
   output RAM_CE1n,
   output RAM_CENn,
   output RAM_WEn,
   output RAM_OEn,
   output RAM_LDn,
`endif // !`ifndef NO_EXT_FIFO
   
   // Debug stuff
   output [3:0] uart_tx_o, 
   input [3:0] uart_rx_i,
   output [3:0] uart_baud_o,
   input sim_mode,
   input [3:0] clock_divider,
   input button,
   
   output spiflash_cs, output spiflash_clk, input spiflash_miso, output spiflash_mosi
   );

   localparam SR_MISC     =   0;   // 8 regs
   localparam SR_TIME64   =  10;   // 6
   localparam SR_BUF_POOL =  16;   // 4

   localparam SR_RX_FRONT0 =  20;   // 5
   localparam SR_RX_FRONT1 =  25;   // 5
   localparam SR_RX_CTRL0 =  30;   // 9
   localparam SR_RX_DSP0  =  40;   // 7
   localparam SR_RX_CTRL1 =  50;   // 9
   localparam SR_RX_DSP1  =  60;   // 7
   localparam SR_RX_CTRL2 =  70;   // 9
   localparam SR_RX_DSP2  =  80;   // 7
   localparam SR_RX_CTRL3 =  90;   // 9
   localparam SR_RX_DSP3  =  100;   // 7

   localparam SR_TX_FRONT0 = 110;   // ?
   localparam SR_TX_CTRL0  = 126;   // 6
   localparam SR_TX_DSP0   = 135;   // 5
   localparam SR_TX_FRONT1 = 145;   // ?
   localparam SR_TX_CTRL1  = 161;   // 6
   localparam SR_TX_DSP1   = 170;   // 5

   localparam SR_DIVSW    = 180;   // 2
   localparam SR_RX_FE_SW = 183;   // 1
   localparam SR_TX_FE_SW = 184;   // 1
   localparam SR_SPI_CORE = 185;   // 3
   
   // FIFO Sizes, 9 = 512 lines, 10 = 1024, 11 = 2048
   // all (most?) are 36 bits wide, so 9 is 1 BRAM, 10 is 2, 11 is 4 BRAMs
   localparam DSP_RX_FIFOSIZE = 9;
   localparam DSP_TX_FIFOSIZE = 9;
   localparam ETH_TX_FIFOSIZE = 9;
   localparam ETH_RX_FIFOSIZE = 11;
   
   wire [7:0] 	set_addr, set_addr_dsp, set_addr_sys, set_addr_fe, set_addr_udp_wb, set_addr_udp_sys;
   wire [31:0] 	set_data, set_data_dsp, set_data_sys, set_data_fe, set_data_udp_wb, set_data_udp_sys;
   wire 	set_stb, set_stb_dsp, set_stb_sys, set_stb_fe, set_stb_udp_wb, set_stb_udp_sys;

   wire set_stb_dsp0, set_stb_dsp1;
   wire [31:0] set_data_dsp0, set_data_dsp1;
   wire [7:0] set_addr_dsp0, set_addr_dsp1;
   
   reg 		wb_rst;
   wire 	dsp_rst, sys_rst, fe_rst;
   wire     net_clr;

   wire [31:0] 	status;
   wire 	bus_error, spi_int, i2c_int, aux_i2c_int, pps_int, onetime_int, periodic_int;
   wire 	proc_int, overrun0, overrun1, underrun;
   wire 	 underrun1;
   wire [3:0] 	uart_tx_int, uart_rx_int;

   wire [31:0] 	debug_gpio_0, debug_gpio_1;

   wire [31:0] 	debug_rx, debug_mac, debug_mac0, debug_mac1, debug_tx_dsp, debug_txc,
		debug_rx_dsp, debug_udp, debug_extfifo, debug_extfifo2;

   wire [15:0] 	dsp_rx_occ, dsp_tx_occ, eth_rx_occ, eth_tx_occ, eth_rx_occ2;
   wire 	dsp_rx_full, dsp_tx_full, eth_rx_full, eth_tx_full, eth_rx_full2;
   wire 	dsp_rx_empty, dsp_tx_empty, eth_rx_empty, eth_tx_empty, eth_rx_empty2;
	
   wire [31:0] 	irq;
   wire [63:0] 	vita_time, vita_time_pps;

    //generate sync reset signals for dsp and sys domains
    reset_sync sys_rst_sync(.clk(sys_clk), .reset_in(wb_rst), .reset_out(sys_rst));
    reset_sync dsp_rst_sync(.clk(dsp_clk), .reset_in(wb_rst), .reset_out(dsp_rst));
    reset_sync fe_rst_sync(.clk(fe_clk), .reset_in(wb_rst), .reset_out(fe_rst));

   // ///////////////////////////////////////////////////////////////////////////////////////////////
   // Wishbone Single Master INTERCON
   localparam 	dw = 32;  // Data bus width
   localparam 	aw = 16;  // Address bus width, for byte addressibility, 16 = 64K byte memory space
   localparam	sw = 4;   // Select width -- 32-bit data bus with 8-bit granularity.  
   
   wire [dw-1:0] m0_dat_o, m0_dat_i;
   wire [dw-1:0] s0_dat_o, s1_dat_o, s0_dat_i, s1_dat_i, s2_dat_o, s3_dat_o, s2_dat_i, s3_dat_i,
		 s4_dat_o, s5_dat_o, s4_dat_i, s5_dat_i, s6_dat_o, s7_dat_o, s6_dat_i, s7_dat_i,
		 s8_dat_o, s9_dat_o, s8_dat_i, s9_dat_i, sa_dat_o, sa_dat_i, sb_dat_i, sb_dat_o,
		 sc_dat_i, sc_dat_o, sd_dat_i, sd_dat_o, se_dat_i, se_dat_o, sf_dat_i, sf_dat_o;
   wire [aw-1:0] m0_adr,s0_adr,s1_adr,s2_adr,s3_adr,s4_adr,s5_adr,s6_adr,s7_adr,s8_adr,s9_adr,sa_adr,sb_adr,sc_adr, sd_adr, se_adr, sf_adr;
   wire [sw-1:0] m0_sel,s0_sel,s1_sel,s2_sel,s3_sel,s4_sel,s5_sel,s6_sel,s7_sel,s8_sel,s9_sel,sa_sel,sb_sel,sc_sel, sd_sel, se_sel, sf_sel;
   wire 	 m0_ack,s0_ack,s1_ack,s2_ack,s3_ack,s4_ack,s5_ack,s6_ack,s7_ack,s8_ack,s9_ack,sa_ack,sb_ack,sc_ack, sd_ack, se_ack, sf_ack;
   wire 	 m0_stb,s0_stb,s1_stb,s2_stb,s3_stb,s4_stb,s5_stb,s6_stb,s7_stb,s8_stb,s9_stb,sa_stb,sb_stb,sc_stb, sd_stb, se_stb, sf_stb;
   wire 	 m0_cyc,s0_cyc,s1_cyc,s2_cyc,s3_cyc,s4_cyc,s5_cyc,s6_cyc,s7_cyc,s8_cyc,s9_cyc,sa_cyc,sb_cyc,sc_cyc, sd_cyc, se_cyc, sf_cyc;
   wire 	 m0_err, m0_rty;
   wire 	 m0_we,s0_we,s1_we,s2_we,s3_we,s4_we,s5_we,s6_we,s7_we,s8_we,s9_we,sa_we,sb_we,sc_we,sd_we,se_we,sf_we;
   
   wb_1master #(.decode_w(8),
		.s0_addr(8'b0000_0000),.s0_mask(8'b1100_0000),  // Main RAM (0-16K)
		.s1_addr(8'b0100_0000),.s1_mask(8'b1111_0000),  // Packet Router (16-20K)
 		.s2_addr(8'b0101_0000),.s2_mask(8'b1111_1100),  // Unused
		.s3_addr(8'b0101_0100),.s3_mask(8'b1111_1100),  // I2C
		.s4_addr(8'b0101_1000),.s4_mask(8'b1111_1100),  // GPSDO
		.s5_addr(8'b0101_1100),.s5_mask(8'b1111_1100),  // Readback
		.s6_addr(8'b0110_0000),.s6_mask(8'b1111_0000),  // Ethernet MAC
		.s7_addr(8'b0111_0000),.s7_mask(8'b1111_0000),  // Settings Bus (only uses 1K)
		.s8_addr(8'b1000_0000),.s8_mask(8'b1111_1100),  // PIC
		.s9_addr(8'b1000_0100),.s9_mask(8'b1111_1100),  // I2C AUX
		.sa_addr(8'b1000_1000),.sa_mask(8'b1111_1100),  // UART
		.sb_addr(8'b1000_1100),.sb_mask(8'b1111_1100),  // Settings Bus for framer (1K)
		.sc_addr(8'b1001_0000),.sc_mask(8'b1111_0000),  // Unused
		.sd_addr(8'b1010_0000),.sd_mask(8'b1111_0000),  // ICAP
		.se_addr(8'b1011_0000),.se_mask(8'b1111_0000),  // SPI Flash
		.sf_addr(8'b1100_0000),.sf_mask(8'b1100_0000),  // 48K-64K, Boot RAM
		.dw(dw),.aw(aw),.sw(sw)) wb_1master
     (.clk_i(wb_clk),.rst_i(wb_rst),       
      .m0_dat_o(m0_dat_o),.m0_ack_o(m0_ack),.m0_err_o(m0_err),.m0_rty_o(m0_rty),.m0_dat_i(m0_dat_i),
      .m0_adr_i(m0_adr),.m0_sel_i(m0_sel),.m0_we_i(m0_we),.m0_cyc_i(m0_cyc),.m0_stb_i(m0_stb),
      .s0_dat_o(s0_dat_o),.s0_adr_o(s0_adr),.s0_sel_o(s0_sel),.s0_we_o	(s0_we),.s0_cyc_o(s0_cyc),.s0_stb_o(s0_stb),
      .s0_dat_i(s0_dat_i),.s0_ack_i(s0_ack),.s0_err_i(0),.s0_rty_i(0),
      .s1_dat_o(s1_dat_o),.s1_adr_o(s1_adr),.s1_sel_o(s1_sel),.s1_we_o	(s1_we),.s1_cyc_o(s1_cyc),.s1_stb_o(s1_stb),
      .s1_dat_i(s1_dat_i),.s1_ack_i(s1_ack),.s1_err_i(0),.s1_rty_i(0),
      .s2_dat_o(s2_dat_o),.s2_adr_o(s2_adr),.s2_sel_o(s2_sel),.s2_we_o	(s2_we),.s2_cyc_o(s2_cyc),.s2_stb_o(s2_stb),
      .s2_dat_i(s2_dat_i),.s2_ack_i(s2_ack),.s2_err_i(0),.s2_rty_i(0),
      .s3_dat_o(s3_dat_o),.s3_adr_o(s3_adr),.s3_sel_o(s3_sel),.s3_we_o	(s3_we),.s3_cyc_o(s3_cyc),.s3_stb_o(s3_stb),
      .s3_dat_i(s3_dat_i),.s3_ack_i(s3_ack),.s3_err_i(0),.s3_rty_i(0),
      .s4_dat_o(s4_dat_o),.s4_adr_o(s4_adr),.s4_sel_o(s4_sel),.s4_we_o	(s4_we),.s4_cyc_o(s4_cyc),.s4_stb_o(s4_stb),
      .s4_dat_i(s4_dat_i),.s4_ack_i(s4_ack),.s4_err_i(0),.s4_rty_i(0),
      .s5_dat_o(s5_dat_o),.s5_adr_o(s5_adr),.s5_sel_o(s5_sel),.s5_we_o	(s5_we),.s5_cyc_o(s5_cyc),.s5_stb_o(s5_stb),
      .s5_dat_i(s5_dat_i),.s5_ack_i(s5_ack),.s5_err_i(0),.s5_rty_i(0),
      .s6_dat_o(s6_dat_o),.s6_adr_o(s6_adr),.s6_sel_o(s6_sel),.s6_we_o	(s6_we),.s6_cyc_o(s6_cyc),.s6_stb_o(s6_stb),
      .s6_dat_i(s6_dat_i),.s6_ack_i(s6_ack),.s6_err_i(0),.s6_rty_i(0),
      .s7_dat_o(s7_dat_o),.s7_adr_o(s7_adr),.s7_sel_o(s7_sel),.s7_we_o	(s7_we),.s7_cyc_o(s7_cyc),.s7_stb_o(s7_stb),
      .s7_dat_i(s7_dat_i),.s7_ack_i(s7_ack),.s7_err_i(0),.s7_rty_i(0),
      .s8_dat_o(s8_dat_o),.s8_adr_o(s8_adr),.s8_sel_o(s8_sel),.s8_we_o	(s8_we),.s8_cyc_o(s8_cyc),.s8_stb_o(s8_stb),
      .s8_dat_i(s8_dat_i),.s8_ack_i(s8_ack),.s8_err_i(0),.s8_rty_i(0),
      .s9_dat_o(s9_dat_o),.s9_adr_o(s9_adr),.s9_sel_o(s9_sel),.s9_we_o	(s9_we),.s9_cyc_o(s9_cyc),.s9_stb_o(s9_stb),
      .s9_dat_i(s9_dat_i),.s9_ack_i(s9_ack),.s9_err_i(0),.s9_rty_i(0),
      .sa_dat_o(sa_dat_o),.sa_adr_o(sa_adr),.sa_sel_o(sa_sel),.sa_we_o(sa_we),.sa_cyc_o(sa_cyc),.sa_stb_o(sa_stb),
      .sa_dat_i(sa_dat_i),.sa_ack_i(sa_ack),.sa_err_i(0),.sa_rty_i(0),
      .sb_dat_o(sb_dat_o),.sb_adr_o(sb_adr),.sb_sel_o(sb_sel),.sb_we_o(sb_we),.sb_cyc_o(sb_cyc),.sb_stb_o(sb_stb),
      .sb_dat_i(sb_dat_i),.sb_ack_i(sb_ack),.sb_err_i(0),.sb_rty_i(0),
      .sc_dat_o(sc_dat_o),.sc_adr_o(sc_adr),.sc_sel_o(sc_sel),.sc_we_o(sc_we),.sc_cyc_o(sc_cyc),.sc_stb_o(sc_stb),
      .sc_dat_i(sc_dat_i),.sc_ack_i(sc_ack),.sc_err_i(0),.sc_rty_i(0),
      .sd_dat_o(sd_dat_o),.sd_adr_o(sd_adr),.sd_sel_o(sd_sel),.sd_we_o(sd_we),.sd_cyc_o(sd_cyc),.sd_stb_o(sd_stb),
      .sd_dat_i(sd_dat_i),.sd_ack_i(sd_ack),.sd_err_i(0),.sd_rty_i(0),
      .se_dat_o(se_dat_o),.se_adr_o(se_adr),.se_sel_o(se_sel),.se_we_o(se_we),.se_cyc_o(se_cyc),.se_stb_o(se_stb),
      .se_dat_i(se_dat_i),.se_ack_i(se_ack),.se_err_i(0),.se_rty_i(0),
      .sf_dat_o(sf_dat_o),.sf_adr_o(sf_adr),.sf_sel_o(sf_sel),.sf_we_o(sf_we),.sf_cyc_o(sf_cyc),.sf_stb_o(sf_stb),
      .sf_dat_i(sf_dat_i),.sf_ack_i(sf_ack),.sf_err_i(0),.sf_rty_i(0));

   // Unused Slaves
   assign s2_ack = 0;
   assign sc_ack = 0;

   // ////////////////////////////////////////////////////////////////////////////////////////
   // Reset Controller

   reg 		 cpu_bldr_ctrl_state;
   localparam CPU_BLDR_CTRL_WAIT = 0;
   localparam CPU_BLDR_CTRL_DONE = 1;
   
   wire 	 bldr_done;
   wire 	 por_rst;
   wire [aw-1:0] cpu_adr;

   // Swap boot ram and main ram when in bootloader mode
   assign m0_adr = (^cpu_adr[15:14] | (cpu_bldr_ctrl_state == CPU_BLDR_CTRL_DONE)) ? cpu_adr :
		   cpu_adr ^ 16'hC000;
   
   system_control sysctrl 
     (.wb_clk_i(wb_clk), .wb_rst_o(por_rst), .ram_loader_done_i(1'b1) );
   
   always @(posedge wb_clk)
     if(por_rst) begin
        cpu_bldr_ctrl_state <= CPU_BLDR_CTRL_WAIT;
        wb_rst <= 1'b1;
     end
     else begin
        case(cpu_bldr_ctrl_state)
	  
          CPU_BLDR_CTRL_WAIT: begin
             wb_rst <= 1'b0;
             if (bldr_done == 1'b1) begin //set by the bootloader
                cpu_bldr_ctrl_state <= CPU_BLDR_CTRL_DONE;
                wb_rst <= 1'b1;
             end
          end
	  
          CPU_BLDR_CTRL_DONE: begin //stay here forever
             wb_rst <= 1'b0;
          end
	  
        endcase //cpu_bldr_ctrl_state
     end
   
   // /////////////////////////////////////////////////////////////////////////
   // Processor

   assign 	 bus_error = m0_err | m0_rty;

   wire [63:0] zpu_status;
   zpu_wb_top #(.dat_w(dw), .adr_w(aw), .sel_w(sw))
     zpu_top0 (.clk(wb_clk), .rst(wb_rst), .enb(~wb_rst),
	   // Data Wishbone bus to system bus fabric
	   .we_o(m0_we),.stb_o(m0_stb),.dat_o(m0_dat_i),.adr_o(cpu_adr),
	   .dat_i(m0_dat_o),.ack_i(m0_ack),.sel_o(m0_sel),.cyc_o(m0_cyc),
	   // Interrupts and exceptions
	   .zpu_status(zpu_status), .interrupt(proc_int & 1'b0));
   
   // /////////////////////////////////////////////////////////////////////////
   // Dual Ported Boot RAM -- D-Port is Slave #0 on main Wishbone
   // Dual Ported Main RAM -- D-Port is Slave #F on main Wishbone
   // I-port connects directly to processor

   bootram bootram(.clk(wb_clk), .reset(wb_rst),
		   .if_adr(14'b0), .if_data(),
		   .dwb_adr_i(sf_adr[13:0]), .dwb_dat_i(sf_dat_o), .dwb_dat_o(sf_dat_i),
		   .dwb_we_i(sf_we), .dwb_ack_o(sf_ack), .dwb_stb_i(sf_stb), .dwb_sel_i(sf_sel));

////blinkenlights v0.1
//defparam bootram.RAM0.INIT_00=256'hbc32fff0_aa43502b_b00000fe_30630001_80000000_10600000_a48500ff_10a00000;
//defparam bootram.RAM0.INIT_01=256'ha48500ff_b810ffd0_f880200c_30a50001_10830000_308000ff_be23000c_a4640001;

`include "bootloader_umtrx.rmi"

   ram_harvard2 #(.AWIDTH(14),.RAM_SIZE(16384))
   sys_ram(.wb_clk_i(wb_clk),.wb_rst_i(wb_rst),	     
	   .if_adr(14'b0), .if_data(),
	   .dwb_adr_i(s0_adr[13:0]), .dwb_dat_i(s0_dat_o), .dwb_dat_o(s0_dat_i),
	   .dwb_we_i(s0_we), .dwb_ack_o(s0_ack), .dwb_stb_i(s0_stb), .dwb_sel_i(s0_sel));
   
   // /////////////////////////////////////////////////////////////////////////
   // Buffer Pool, slave #1
   wire 	 dsp_tx0_ready, dsp_tx0_valid;
   wire 	 dsp_tx1_ready, dsp_tx1_valid;
   wire 	 eth_tx_ready, eth_tx_valid;
   wire [35:0] 	 dsp_tx0_data, dsp_tx1_data, eth_tx_data;

   wire 	 eth_rx_valid, eth_rx_ready;
   wire 	 dsp_rx0_valid, dsp_rx0_ready;
   wire 	 dsp_rx1_valid, dsp_rx1_ready;
   wire 	 dsp_rx2_valid, dsp_rx2_ready;
   wire 	 dsp_rx3_valid, dsp_rx3_ready;
   wire [35:0] 	 eth_rx_data;
   wire [35:0] 	 dsp_rx0_data, dsp_rx1_data;
   wire [35:0] 	 dsp_rx2_data, dsp_rx3_data;

   wire [35:0] 	 err_tx0_data;
   wire 	 err_tx0_valid, err_tx0_ready;
   wire [35:0] 	 err_tx1_data;
   wire 	 err_tx1_valid, err_tx1_ready;

   wire [35:0] resp_data, ctrl_data;
   wire resp_valid, ctrl_valid;
   wire resp_ready, ctrl_ready;

    umtrx_router #(.BUF_SIZE(9), .CTRL_BASE(SR_BUF_POOL)) router
    (
        .wb_clk_i(wb_clk),.wb_rst_i(wb_rst),
        .wb_we_i(s1_we),.wb_stb_i(s1_stb),.wb_adr_i(s1_adr),.wb_dat_i(s1_dat_o),
        .wb_dat_o(s1_dat_i),.wb_ack_o(s1_ack),.wb_err_o(),.wb_rty_o(),

        .set_stb(set_stb_sys), .set_addr(set_addr_sys), .set_data(set_data_sys),
        .set_stb_udp(set_stb_udp_sys), .set_addr_udp(set_addr_udp_sys), .set_data_udp(set_data_udp_sys),

        .stream_clk(sys_clk), .stream_rst(sys_rst), .stream_clr(net_clr),

        .status(status),

        .eth_inp_data(eth_rx_data), .eth_inp_valid(eth_rx_valid), .eth_inp_ready(eth_rx_ready),
        .eth_out_data(eth_tx_data), .eth_out_valid(eth_tx_valid), .eth_out_ready(eth_tx_ready),

        .ctrl_inp_data(resp_data), .ctrl_inp_valid(resp_valid), .ctrl_inp_ready(resp_ready),
        .dsp0_inp_data(dsp_rx0_data), .dsp0_inp_valid(dsp_rx0_valid), .dsp0_inp_ready(dsp_rx0_ready),
        .dsp1_inp_data(dsp_rx1_data), .dsp1_inp_valid(dsp_rx1_valid), .dsp1_inp_ready(dsp_rx1_ready),
        .dsp2_inp_data(dsp_rx2_data), .dsp2_inp_valid(dsp_rx2_valid), .dsp2_inp_ready(dsp_rx2_ready),
        .dsp3_inp_data(dsp_rx3_data), .dsp3_inp_valid(dsp_rx3_valid), .dsp3_inp_ready(dsp_rx3_ready),
        .err0_inp_data(err_tx0_data), .err0_inp_valid(err_tx0_valid), .err0_inp_ready(err_tx0_ready),
        .err1_inp_data(err_tx1_data), .err1_inp_valid(err_tx1_valid), .err1_inp_ready(err_tx1_ready),

        .ctrl_out_data(ctrl_data), .ctrl_out_valid(ctrl_valid), .ctrl_out_ready(ctrl_ready),
        .dsp0_out_data(dsp_tx0_data), .dsp0_out_valid(dsp_tx0_valid), .dsp0_out_ready(dsp_tx0_ready),
        .dsp1_out_data(dsp_tx1_data), .dsp1_out_valid(dsp_tx1_valid), .dsp1_out_ready(dsp_tx1_ready)
    );

   // /////////////////////////////////////////////////////////////////////////
   // SPI -- Slave #2
    reg [31:0] spi_readback0;
    reg [31:0] spi_readback1;
    wire spi_ready;

    wire [0:0] AXIS_SPI_CONFIG_tdest;
    wire [79:0] AXIS_SPI_CONFIG_tdata;
    wire AXIS_SPI_CONFIG_tvalid;
    wire AXIS_SPI_CONFIG_tready;

    wire [0:0] AXIS_SPI_READBACK_tdest;
    wire [31:0] AXIS_SPI_READBACK_tdata;
    wire AXIS_SPI_READBACK_tvalid;
    wire AXIS_SPI_READBACK_tready;

    axis_spi_core #(.DESTW(1), .WIDTH(5), .DEBUG(0)) axis_shared_spi(
        .clock(dsp_clk), .reset(dsp_rst),

        .CONFIG_tdest(AXIS_SPI_CONFIG_tdest),
        .CONFIG_tdata(AXIS_SPI_CONFIG_tdata),
        .CONFIG_tvalid(AXIS_SPI_CONFIG_tvalid),
        .CONFIG_tready(AXIS_SPI_CONFIG_tready),

        .READBACK_tdest(AXIS_SPI_READBACK_tdest),
        .READBACK_tdata(AXIS_SPI_READBACK_tdata),
        .READBACK_tvalid(AXIS_SPI_READBACK_tvalid),
        .READBACK_tready(AXIS_SPI_READBACK_tready),

        .sen({aux_sen2,aux_sen1,sen_dac,sen_lms2,sen_lms1}),
        .sclk(sclk), .mosi(mosi), .miso(miso)
    );

    //setting register block for spi dest 0 (wishbone) and spi dest 1 (ctrl fifo)
    //Note: the strobes are exclusive (settings fifo cross clock)
    wire [79:0] spi_config [0:1];
    wire [0:1] spi_trigger;
    wire [0:1] set_stb_dsp_n = {set_stb_dsp0, set_stb_dsp1};
    genvar i;
    generate for (i=0; i <= 1; i=i+1) begin
        setting_reg #(.my_addr(SR_SPI_CORE+2),.width(32)) axis_shared_spi_sr0(
            .clk(dsp_clk),.rst(dsp_rst),.strobe(set_stb_dsp_n[i]),.addr(set_addr_dsp),.in(set_data_dsp),
            .out(spi_config[i][31:0]),.changed(spi_trigger[i]));

        setting_reg #(.my_addr(SR_SPI_CORE+1),.width(32)) axis_shared_spi_sr1(
            .clk(dsp_clk),.rst(dsp_rst),.strobe(set_stb_dsp_n[i]),.addr(set_addr_dsp),.in(set_data_dsp),
            .out(spi_config[i][63:32]),.changed());

        setting_reg #(.my_addr(SR_SPI_CORE+0),.width(16)) axis_shared_spi_sr2(
            .clk(dsp_clk),.rst(dsp_rst),.strobe(set_stb_dsp_n[i]),.addr(set_addr_dsp),.in(set_data_dsp),
            .out(spi_config[i][79:64]),.changed());
    end endgenerate

    //assign config bus from setting register sources
    //Note: the triggers are exclusive (settings fifo cross clock)
    assign AXIS_SPI_CONFIG_tdest = (spi_trigger[0])?1'b0:1'b1;
    assign AXIS_SPI_CONFIG_tdata = (spi_trigger[0])?spi_config[0]:spi_config[1];
    assign AXIS_SPI_CONFIG_tvalid = (spi_trigger != 0);

    //create spi ready to block the ctrl fifo ASAP
    wire spi_ready_now = AXIS_SPI_CONFIG_tready && !AXIS_SPI_CONFIG_tvalid;
    assign spi_ready = spi_ready_now && spi_ready_prev;
    reg spi_ready_prev;
    always @(posedge dsp_clk) begin
        spi_ready_prev <= spi_ready_now;
    end

    //readback output bus latches values into readback register
    assign AXIS_SPI_READBACK_tready = 1'b1;
    always @(posedge dsp_clk) begin
        if (AXIS_SPI_READBACK_tvalid && AXIS_SPI_READBACK_tready) begin
            if (AXIS_SPI_READBACK_tdest == 1'b0) spi_readback0 <= AXIS_SPI_READBACK_tdata;
            if (AXIS_SPI_READBACK_tdest == 1'b1) spi_readback1 <= AXIS_SPI_READBACK_tdata;
        end
    end

   // /////////////////////////////////////////////////////////////////////////
   // I2C -- Slave #3
   i2c_master_top #(.ARST_LVL(1)) 
     i2c (.wb_clk_i(wb_clk),.wb_rst_i(wb_rst),.arst_i(1'b0), 
	  .wb_adr_i(s3_adr[4:2]),.wb_dat_i(s3_dat_o[7:0]),.wb_dat_o(s3_dat_i[7:0]),
	  .wb_we_i(s3_we),.wb_stb_i(s3_stb),.wb_cyc_i(s3_cyc),
	  .wb_ack_o(s3_ack),.wb_inta_o(i2c_int),
	  .scl_pad_i(scl_pad_i),.scl_pad_o(scl_pad_o),.scl_padoen_o(scl_pad_oen_o),
	  .sda_pad_i(sda_pad_i),.sda_pad_o(sda_pad_o),.sda_padoen_o(sda_pad_oen_o) );
     
   i2c_master_top #(.ARST_LVL(1)) 
     aux_i2c (.wb_clk_i(wb_clk),.wb_rst_i(wb_rst),.arst_i(1'b0), 
	  .wb_adr_i(s9_adr[4:2]),.wb_dat_i(s9_dat_o[7:0]),.wb_dat_o(s9_dat_i[7:0]),
	  .wb_we_i(s9_we),.wb_stb_i(s9_stb),.wb_cyc_i(s9_cyc),
	  .wb_ack_o(s9_ack),.wb_inta_o(aux_i2c_int),
	  .scl_pad_i(aux_scl_pad_i),.scl_pad_o(aux_scl_pad_o),.scl_padoen_o(aux_scl_pad_oen_o),
	  .sda_pad_i(aux_sda_pad_i),.sda_pad_o(aux_sda_pad_o),.sda_padoen_o(aux_sda_pad_oen_o) );

   assign 	 s3_dat_i[31:8] = 24'd0;
   assign 	 s9_dat_i[31:8] = 24'd0;

   // /////////////////////////////////////////////////////////////////////////
   // Buffer Pool Status -- Slave #5   
   
   //compatibility number -> increment when the fpga has been sufficiently altered
   localparam compat_num = {16'd9, 16'd3}; //major, minor

   wire [31:0] irq_readback = {16'b0, aux_ld2, aux_ld1, button, spi_ready, 12'b0};

   wb_readback_mux buff_pool_status
     (.wb_clk_i(wb_clk), .wb_rst_i(wb_rst), .wb_stb_i(s5_stb),
      .wb_adr_i(s5_adr), .wb_dat_o(s5_dat_i), .wb_ack_o(s5_ack),

      .word00(spi_readback0),.word01(`NUMDDC),.word02(`NUMDUC),.word03(32'b0),
      .word04(32'b0),.word05(32'b0),.word06(32'b0),.word07(32'b0),
      .word08(status),.word09(32'b0),.word10(vita_time[63:32]),
      .word11(vita_time[31:0]),.word12(compat_num),.word13(irq_readback),
      .word14(vita_time_pps[63:32]),.word15(vita_time_pps[31:0])
      );

   // /////////////////////////////////////////////////////////////////////////
   // Ethernet MAC  Slave #6

   simple_gemac_wrapper #(.RXFIFOSIZE(ETH_RX_FIFOSIZE), 
			  .TXFIFOSIZE(ETH_TX_FIFOSIZE)) simple_gemac_wrapper
     (.clk125(clk_to_mac),  .reset(wb_rst | net_clr),
      .GMII_GTX_CLK(GMII_GTX_CLK), .GMII_TX_EN(GMII_TX_EN),  
      .GMII_TX_ER(GMII_TX_ER), .GMII_TXD(GMII_TXD),
      .GMII_RX_CLK(GMII_RX_CLK), .GMII_RX_DV(GMII_RX_DV),  
      .GMII_RX_ER(GMII_RX_ER), .GMII_RXD(GMII_RXD),
      .sys_clk(sys_clk),
      .rx_f36_data(eth_rx_data), .rx_f36_src_rdy(eth_rx_valid), .rx_f36_dst_rdy(eth_rx_ready),
      .tx_f36_data(eth_tx_data), .tx_f36_src_rdy(eth_tx_valid), .tx_f36_dst_rdy(eth_tx_ready),
      .wb_clk(wb_clk), .wb_rst(wb_rst), .wb_stb(s6_stb), .wb_cyc(s6_cyc), .wb_ack(s6_ack),
      .wb_we(s6_we), .wb_adr(s6_adr), .wb_dat_i(s6_dat_o), .wb_dat_o(s6_dat_i),
      .mdio(MDIO), .mdc(MDC),
      .debug(debug_mac));

   // /////////////////////////////////////////////////////////////////////////
   // Settings Bus -- Slave #7
   settings_bus settings_bus
     (.wb_clk(wb_clk),.wb_rst(wb_rst),.wb_adr_i(s7_adr),.wb_dat_i(s7_dat_o),
      .wb_stb_i(s7_stb),.wb_we_i(s7_we),.wb_ack_o(s7_ack),
      .strobe(set_stb),.addr(set_addr),.data(set_data));
   
   assign 	 s7_dat_i = 32'd0;

   settings_bus_crossclock settings_bus_sys_crossclock
     (.clk_i(wb_clk), .rst_i(wb_rst), .set_stb_i(set_stb), .set_addr_i(set_addr), .set_data_i(set_data),
      .clk_o(sys_clk), .rst_o(sys_rst), .set_stb_o(set_stb_sys), .set_addr_o(set_addr_sys), .set_data_o(set_data_sys));

   settings_bus_crossclock settings_bus_fe_crossclock
     (.clk_i(dsp_clk), .rst_i(dsp_rst), .set_stb_i(set_stb_dsp), .set_addr_i(set_addr_dsp), .set_data_i(set_data_dsp),
      .clk_o(fe_clk), .rst_o(fe_rst), .set_stb_o(set_stb_fe), .set_addr_o(set_addr_fe), .set_data_o(set_data_fe));

   //mux settings_bus_crossclock and settings_readback_bus_fifo_ctrl with prio
   assign set_stb_dsp = set_stb_dsp0 | set_stb_dsp1;
   assign set_addr_dsp = set_stb_dsp1? set_addr_dsp1 : set_addr_dsp0;
   assign set_data_dsp = set_stb_dsp1? set_data_dsp1 : set_data_dsp0;

   settings_bus_crossclock #(.FLOW_CTRL(1/*on*/)) settings_bus_dsp_crossclock
     (.clk_i(wb_clk), .rst_i(wb_rst), .set_stb_i(set_stb), .set_addr_i(set_addr), .set_data_i(set_data),
      .clk_o(dsp_clk), .rst_o(dsp_rst), .set_stb_o(set_stb_dsp0), .set_addr_o(set_addr_dsp0), .set_data_o(set_data_dsp0),
      .blocked(set_stb_dsp1));

   // /////////////////////////////////////////////////////////////////////////
   // Settings + Readback Bus -- FIFO controlled

    wire [35:0] ctrl_data_dsp, resp_data_dsp;
    wire ctrl_valid_dsp, resp_valid_dsp;
    wire ctrl_ready_dsp, resp_ready_dsp;

    axi_fifo_2clk #(.WIDTH(36), .SIZE(0)) fifo_2clock_ctrl
    (
        .i_aclk(sys_clk), .i_tdata(ctrl_data), .i_tvalid(ctrl_valid), .i_tready(ctrl_ready),
        .o_aclk(dsp_clk), .o_tdata(ctrl_data_dsp), .o_tvalid(ctrl_valid_dsp), .o_tready(ctrl_ready_dsp),
        .reset(dsp_rst | sys_rst)
    );

    axi_fifo_2clk #(.WIDTH(36), .SIZE(0)) fifo_2clock_resp
    (
        .i_aclk(dsp_clk), .i_tdata(resp_data_dsp), .i_tvalid(resp_valid_dsp), .i_tready(resp_ready_dsp),
        .o_aclk(sys_clk), .o_tdata(resp_data), .o_tvalid(resp_valid), .o_tready(resp_ready),
        .reset(dsp_rst | sys_rst)
    );

    wire [31:0] sfc_debug;
    wire sfc_clear;
    settings_fifo_ctrl #(.PROT_DEST(3), .PROT_HDR(1)) sfc
    (
        .clock(dsp_clk), .reset(dsp_rst), .clear(sfc_clear),
        .vita_time(vita_time), .perfs_ready(spi_ready),
        .in_data(ctrl_data_dsp), .in_valid(ctrl_valid_dsp), .in_ready(ctrl_ready_dsp),
        .out_data(resp_data_dsp), .out_valid(resp_valid_dsp), .out_ready(resp_ready_dsp),
        .strobe(set_stb_dsp1), .addr(set_addr_dsp1), .data(set_data_dsp1),
        .word00(spi_readback1),.word01(32'b0),.word02(32'b0),.word03(32'b0),
        .word04(32'b0),.word05(32'b0),.word06(32'b0),.word07(32'b0),
        .word08(32'b0),.word09(32'b0),.word10(vita_time[63:32]),
        .word11(vita_time[31:0]),.word12(32'b0),.word13(irq_readback),
        .word14(vita_time_pps[63:32]),.word15(vita_time_pps[31:0]),
        .debug(sfc_debug)
    );

   // Output control lines
   wire 	 phy_reset;
   assign 	 PHY_RESETn = ~phy_reset;

   setting_reg #(.my_addr(SR_MISC+0),.width(6)) sr_lms_res
     (.clk(wb_clk),.rst(wb_rst),.strobe(set_stb),.addr(set_addr),.in(set_data),.out({en_dc_sync, enpa2, enpa1, lowpa, lms_res}),.changed());

   setting_reg #(.my_addr(SR_MISC+1),.width(1)) sr_clear_sfc
     (.clk(dsp_clk),.rst(dsp_rst),.strobe(set_stb_dsp),.addr(set_addr_dsp),.in(set_data_dsp),.changed(sfc_clear));

   wire sram_clear;
   setting_reg #(.my_addr(SR_MISC+2),.width(1)) sr_sram_clear
     (.clk(sys_clk),.rst(sys_rst),.strobe(set_stb_sys),.addr(set_addr_sys),.in(set_data_sys),.changed(sram_clear));

   setting_reg #(.my_addr(SR_MISC+4),.width(1)) sr_phy
     (.clk(wb_clk),.rst(wb_rst),.strobe(set_stb),.addr(set_addr),.in(set_data),.out(phy_reset),.changed());

   setting_reg #(.my_addr(SR_MISC+5),.width(1)) sr_bld
     (.clk(wb_clk),.rst(wb_rst),.strobe(set_stb),.addr(set_addr),.in(set_data),.out(bldr_done),.changed());

   setting_reg #(.my_addr(SR_MISC+7),.width(1)) net_reset_sr
     (.clk(wb_clk),.rst(wb_rst),.strobe(set_stb),.addr(set_addr),.in(set_data),.out(net_clr),.changed());

   // Diversity switches
   setting_reg #(.my_addr(SR_DIVSW+0),.width(1), .at_reset(32'd1)) sr_divsw1
     (.clk(wb_clk),.rst(wb_rst),.strobe(set_stb),.addr(set_addr),.in(set_data),.out(DivSw1),.changed());

   setting_reg #(.my_addr(SR_DIVSW+1),.width(1), .at_reset(32'd1)) sr_divsw2
     (.clk(wb_clk),.rst(wb_rst),.strobe(set_stb),.addr(set_addr),.in(set_data),.out(DivSw2),.changed());

   // /////////////////////////////////////////////////////////////////////////
   //  LEDS
   //    register 8 determines whether leds are controlled by SW or not
   //    1 = controlled by HW, 0 = by SW
   //    In Rev3 there are only 6 leds, and the highest one is on the ETH connector

   wire [7:0] 	 led_src, led_sw;
   wire LEDA, LEDB, LEDC, LEDE;
   wire [7:0] 	 led_hw = {LEDA, LEDC, LEDE, LEDB, 1'b0};
   
   setting_reg #(.my_addr(SR_MISC+3),.width(8)) sr_led
     (.clk(wb_clk),.rst(wb_rst),.strobe(set_stb),.addr(set_addr),.in(set_data),.out(led_sw),.changed());

   setting_reg #(.my_addr(SR_MISC+6),.width(8), .at_reset(8'b0001_1110)) sr_led_src
     (.clk(wb_clk),.rst(wb_rst),.strobe(set_stb),.addr(set_addr),.in(set_data),.out(led_src),.changed());

   assign 	 leds = (led_src & led_hw) | (~led_src & led_sw);

   // /////////////////////////////////////////////////////////////////////////
   // GPSDO unit

   wire gpsdo_int;

   gpsdo gpsdo(
      .pps_in(pps_in),
      .clk_in(wb_clk),
      .wb_adr_i(s4_adr[4:2]),
      .wb_dat_i(s4_dat_o),
      .wb_dat_o(s4_dat_i),
      .wb_we_i(s4_we),
      .wb_stb_i(s4_stb),
      .wb_cyc_i(s4_cyc),
      .wb_ack_o(s4_ack),
      .wb_err_o(s4_err),
      .wb_int_o(gpsdo_int),
      .wb_clk_i(wb_clk),
      .wb_rst_i(wb_rst)
   );
 
   // /////////////////////////////////////////////////////////////////////////
   // Interrupt Controller, Slave #8

   assign irq= {{8'b0},
		{uart_tx_int[3:0], uart_rx_int[3:0]},
		{8'b0},
		{2'b0,aux_i2c_int, PHY_INTn, i2c_int,spi_int,gpsdo_int,1'b0}};
   
   pic pic(.clk_i(wb_clk),.rst_i(wb_rst),.cyc_i(s8_cyc),.stb_i(s8_stb),.adr_i(s8_adr[4:2]),
	   .we_i(s8_we),.dat_i(s8_dat_o),.dat_o(s8_dat_i),.ack_o(s8_ack),.int_o(proc_int),
	   .irq(irq) );
 	 
   // /////////////////////////////////////////////////////////////////////////
   // UART, Slave #10

   quad_uart #(.TXDEPTH(3),.RXDEPTH(3)) uart  // depth of 3 is 128 entries
     (.clk_i(wb_clk),.rst_i(wb_rst),
      .we_i(sa_we),.stb_i(sa_stb),.cyc_i(sa_cyc),.ack_o(sa_ack),
      .adr_i(sa_adr[6:2]),.dat_i(sa_dat_o),.dat_o(sa_dat_i),
      .rx_int_o(uart_rx_int),.tx_int_o(uart_tx_int),
      .tx_o(uart_tx_o),.rx_i(uart_rx_i),.baud_o(uart_baud_o));

   // /////////////////////////////////////////////////////////////////////////
   // Settings Bus Framer -- Slave #B
   settings_bus settings_bus_framer
     (.wb_clk(wb_clk),.wb_rst(wb_rst),.wb_adr_i(sb_adr),.wb_dat_i(sb_dat_o),
      .wb_stb_i(sb_stb),.wb_we_i(sb_we),.wb_ack_o(sb_ack),
      .strobe(set_stb_udp_wb),.addr(set_addr_udp_wb),.data(set_data_udp_wb));
   
   assign 	 sb_dat_i = 32'd0;

   settings_bus_crossclock settings_bus_udp_sys_crossclock
     (.clk_i(wb_clk), .rst_i(wb_rst), .set_stb_i(set_stb_udp_wb), .set_addr_i(set_addr_udp_wb), .set_data_i(set_data_udp_wb),
      .clk_o(sys_clk), .rst_o(sys_rst), .set_stb_o(set_stb_udp_sys), .set_addr_o(set_addr_udp_sys), .set_data_o(set_data_udp_sys));

   // /////////////////////////////////////////////////////////////////////////
   // ICAP for reprogramming the FPGA, Slave #13 (D)

   s6_icap_wb s6_icap_wb
     (.clk(wb_clk), .clk_icap(clk_icap),.reset(wb_rst), .cyc_i(sd_cyc), .stb_i(sd_stb),
      .we_i(sd_we), .ack_o(sd_ack), .dat_i(sd_dat_o), .dat_o(sd_dat_i));
   
   // /////////////////////////////////////////////////////////////////////////
   // SPI for Flash -- Slave #14 (E)
   spi_top flash_spi
     (.wb_clk_i(wb_clk),.wb_rst_i(wb_rst),.wb_adr_i(se_adr[4:0]),.wb_dat_i(se_dat_o),
      .wb_dat_o(se_dat_i),.wb_sel_i(se_sel),.wb_we_i(se_we),.wb_stb_i(se_stb),
      .wb_cyc_i(se_cyc),.wb_ack_o(se_ack),.wb_err_o(se_err),.wb_int_o(spiflash_int),
      .ss_pad_o(spiflash_cs),
      .sclk_pad_o(spiflash_clk),.mosi_pad_o(spiflash_mosi),.miso_pad_i(spiflash_miso) );

   // /////////////////////////////////////////////////////////////////////////
   // RX Frontend
    wire [23:0] rx_front0_i, rx_front0_q;
    rx_frontend #(.BASE(SR_RX_FRONT0)) rx_frontend0
    (
        .clk(fe_clk), .rst(fe_rst),
        .set_stb(set_stb_fe),.set_addr(set_addr_fe),.set_data(set_data_fe),
        .i_out(rx_front0_i), .q_out(rx_front0_q), .run(1'b1),
        .adc_a({adc0_a, 4'b0}), .adc_b({adc0_b, 4'b0})
    );
    wire [23:0] rx_front1_i, rx_front1_q;
    rx_frontend #(.BASE(SR_RX_FRONT1)) rx_frontend1
    (
        .clk(fe_clk), .rst(fe_rst),
        .set_stb(set_stb_fe),.set_addr(set_addr_fe),.set_data(set_data_fe),
        .i_out(rx_front1_i), .q_out(rx_front1_q), .run(1'b1),
        .adc_a({adc1_a, 4'b0}), .adc_b({adc1_b, 4'b0})
    );

   // /////////////////////////////////////////////////////////////////////////
   // RX chains

    //switch to select frontend used per DSP
    wire [3:0] run_rx_dsp;
    wire [3:0] rx_fe_sw;
    setting_reg #(.my_addr(SR_RX_FE_SW),.width(4)) sr_rx_fe_sw
     (.clk(dsp_clk),.rst(dsp_rst),.strobe(set_stb_dsp),.addr(set_addr_dsp),.in(set_data_dsp),.out(rx_fe_sw),.changed());

    generate
    if (`NUMDDC > 0) begin
    umtrx_rx_chain
    #(
        .PROT_DEST(4),
        .DSPNO(0),
        .DSP_BASE(SR_RX_DSP0),
        .CTRL_BASE(SR_RX_CTRL0),
        .FIFOSIZE(DSP_RX_FIFOSIZE),
        .DEBUG(0)
    )
    umtrx_rx_chain0
    (
        .sys_clk(sys_clk), .sys_rst(sys_rst),
        .dsp_clk(dsp_clk), .dsp_rst(dsp_rst),
        .fe_clk(fe_clk), .fe_rst(fe_rst),
        .set_stb_dsp(set_stb_dsp), .set_addr_dsp(set_addr_dsp), .set_data_dsp(set_data_dsp),
        .set_stb_fe(set_stb_fe), .set_addr_fe(set_addr_fe), .set_data_fe(set_data_fe),
        .front_i(rx_fe_sw[0]?rx_front1_i:rx_front0_i),
        .front_q(rx_fe_sw[0]?rx_front1_q:rx_front0_q),
        .adc_stb(rx_fe_sw[0]?adc1_strobe:adc0_strobe),
        .run(run_rx_dsp[0]),
        .vita_data_sys(dsp_rx0_data), .vita_valid_sys(dsp_rx0_valid), .vita_ready_sys(dsp_rx0_ready),
        .vita_time(vita_time)
    );
    end else begin
        assign dsp_rx0_valid = 0;
        assign run_rx_dsp[0] = 0;
    end
    if (`NUMDDC > 1) begin
    umtrx_rx_chain
    #(
        .PROT_DEST(5),
        .DSPNO(1),
        .DSP_BASE(SR_RX_DSP1),
        .CTRL_BASE(SR_RX_CTRL1),
        .FIFOSIZE(DSP_RX_FIFOSIZE)
    )
    umtrx_rx_chain1
    (
        .sys_clk(sys_clk), .sys_rst(sys_rst),
        .dsp_clk(dsp_clk), .dsp_rst(dsp_rst),
        .fe_clk(fe_clk), .fe_rst(fe_rst),
        .set_stb_dsp(set_stb_dsp), .set_addr_dsp(set_addr_dsp), .set_data_dsp(set_data_dsp),
        .set_stb_fe(set_stb_fe), .set_addr_fe(set_addr_fe), .set_data_fe(set_data_fe),
        .front_i(rx_fe_sw[1]?rx_front1_i:rx_front0_i),
        .front_q(rx_fe_sw[1]?rx_front1_q:rx_front0_q),
        .adc_stb(rx_fe_sw[1]?adc1_strobe:adc0_strobe),
        .run(run_rx_dsp[1]),
        .vita_data_sys(dsp_rx1_data), .vita_valid_sys(dsp_rx1_valid), .vita_ready_sys(dsp_rx1_ready),
        .vita_time(vita_time)
    );
    end else begin
        assign dsp_rx1_valid = 0;
        assign run_rx_dsp[1] = 0;
    end
    if (`NUMDDC > 2) begin
    umtrx_rx_chain
    #(
        .PROT_DEST(6),
        .DSPNO(2),
        .DSP_BASE(SR_RX_DSP2),
        .CTRL_BASE(SR_RX_CTRL2),
        .FIFOSIZE(DSP_RX_FIFOSIZE)
    )
    umtrx_rx_chain2
    (
        .sys_clk(sys_clk), .sys_rst(sys_rst),
        .dsp_clk(dsp_clk), .dsp_rst(dsp_rst),
        .fe_clk(fe_clk), .fe_rst(fe_rst),
        .set_stb_dsp(set_stb_dsp), .set_addr_dsp(set_addr_dsp), .set_data_dsp(set_data_dsp),
        .set_stb_fe(set_stb_fe), .set_addr_fe(set_addr_fe), .set_data_fe(set_data_fe),
        .front_i(rx_fe_sw[2]?rx_front1_i:rx_front0_i),
        .front_q(rx_fe_sw[2]?rx_front1_q:rx_front0_q),
        .adc_stb(rx_fe_sw[2]?adc1_strobe:adc0_strobe),
        .run(run_rx_dsp[2]),
        .vita_data_sys(dsp_rx2_data), .vita_valid_sys(dsp_rx2_valid), .vita_ready_sys(dsp_rx2_ready),
        .vita_time(vita_time)
    );
    end else begin
        assign dsp_rx2_valid = 0;
        assign run_rx_dsp[2] = 0;
    end
    if (`NUMDDC > 3) begin
    umtrx_rx_chain
    #(
        .PROT_DEST(7),
        .DSPNO(3),
        .DSP_BASE(SR_RX_DSP3),
        .CTRL_BASE(SR_RX_CTRL3),
        .FIFOSIZE(DSP_RX_FIFOSIZE)
    )
    umtrx_rx_chain3
    (
        .sys_clk(sys_clk), .sys_rst(sys_rst),
        .dsp_clk(dsp_clk), .dsp_rst(dsp_rst),
        .fe_clk(fe_clk), .fe_rst(fe_rst),
        .set_stb_dsp(set_stb_dsp), .set_addr_dsp(set_addr_dsp), .set_data_dsp(set_data_dsp),
        .set_stb_fe(set_stb_fe), .set_addr_fe(set_addr_fe), .set_data_fe(set_data_fe),
        .front_i(rx_fe_sw[3]?rx_front1_i:rx_front0_i),
        .front_q(rx_fe_sw[3]?rx_front1_q:rx_front0_q),
        .adc_stb(rx_fe_sw[3]?adc1_strobe:adc0_strobe),
        .run(run_rx_dsp[3]),
        .vita_data_sys(dsp_rx3_data), .vita_valid_sys(dsp_rx3_valid), .vita_ready_sys(dsp_rx3_ready),
        .vita_time(vita_time)
    );
    end else begin
        assign dsp_rx3_valid = 0;
        assign run_rx_dsp[3] = 0;
    end
    endgenerate

   // /////////////////////////////////////////////////////////////////////////
   // TX Frontend
    wire [23:0] tx_front0_i, tx_front0_q;
    wire [3:0] dac0_a_pad, dac0_b_pad;
    tx_frontend #(.BASE(SR_TX_FRONT0)) tx_frontend0
    (
        .clk(fe_clk), .rst(fe_rst),
        .set_stb(set_stb_fe),.set_addr(set_addr_fe),.set_data(set_data_fe),
        .tx_i(tx_front0_i), .tx_q(tx_front0_q), .run(1'b1),
        .dac_a({dac0_a, dac0_a_pad}), .dac_b({dac0_b, dac0_b_pad})
    );

    wire [23:0] tx_front1_i, tx_front1_q;
    wire [3:0] dac1_a_pad, dac1_b_pad;
    tx_frontend #(.BASE(SR_TX_FRONT1)) tx_frontend1
    (
        .clk(fe_clk), .rst(fe_rst),
        .set_stb(set_stb_fe),.set_addr(set_addr_fe),.set_data(set_data_fe),
        .tx_i(tx_front1_i), .tx_q(tx_front1_q), .run(1'b1),
        .dac_a({dac1_a, dac1_a_pad}), .dac_b({dac1_b, dac1_b_pad})
    );

   // /////////////////////////////////////////////////////////////////////////
   // TX chains
    wire [35:0] sram0_data, sram1_data;
    wire sram0_valid, sram1_valid;
    wire sram0_ready, sram1_ready;
    wire run_tx_dsp0, run_tx_dsp1;

    //switch to select frontend used per DSP
    wire tx_fe_sw;
    setting_reg #(.my_addr(SR_TX_FE_SW),.width(1)) sr_tx_fe_sw
     (.clk(dsp_clk),.rst(dsp_rst),.strobe(set_stb_dsp),.addr(set_addr_dsp),.in(set_data_dsp),.out(tx_fe_sw),.changed());

    //assign dac switch
    wire [23:0] dac0_a_int, dac0_b_int;
    wire [23:0] dac1_a_int, dac1_b_int;
    assign {tx_front0_i, tx_front0_q} = (tx_fe_sw == 0)? {dac0_a_int, dac0_b_int} : {dac1_a_int, dac1_b_int};
    assign {tx_front1_i, tx_front1_q} = (tx_fe_sw == 1)? {dac0_a_int, dac0_b_int} : {dac1_a_int, dac1_b_int};

    generate
    if (`NUMDUC > 0) begin
    umtrx_tx_chain
    #(
        .PROT_DEST(0),
        .DSPNO(0),
        .DSP_BASE(SR_TX_DSP0),
        .CTRL_BASE(SR_TX_CTRL0),
        .FIFOSIZE(DSP_TX_FIFOSIZE)
    )
    umtrx_tx_chain0
    (
        .sys_clk(sys_clk), .sys_rst(sys_rst),
        .dsp_clk(dsp_clk), .dsp_rst(dsp_rst),
        .fe_clk(fe_clk), .fe_rst(fe_rst),
        .set_stb_dsp(set_stb_dsp), .set_addr_dsp(set_addr_dsp), .set_data_dsp(set_data_dsp),
        .set_stb_fe(set_stb_fe), .set_addr_fe(set_addr_fe), .set_data_fe(set_data_fe),
        .front_i(dac0_a_int), .front_q(dac0_b_int), .dac_stb(dac0_strobe), .run(run_tx_dsp0),
        .vita_data_sys(sram0_data), .vita_valid_sys(sram0_valid), .vita_ready_sys(sram0_ready),
        .err_data_sys(err_tx0_data), .err_valid_sys(err_tx0_valid), .err_ready_sys(err_tx0_ready),
        .vita_time(vita_time)
    );
    end else begin
        assign sram0_ready = 1;
        assign err_tx0_valid = 0;
        assign run_tx_dsp0 = 0;
    end
    if (`NUMDUC > 1) begin
    umtrx_tx_chain
    #(
        .PROT_DEST(1),
        .DSPNO(1),
        .DSP_BASE(SR_TX_DSP1),
        .CTRL_BASE(SR_TX_CTRL1),
        .FIFOSIZE(DSP_TX_FIFOSIZE)
    )
    umtrx_tx_chain1
    (
        .sys_clk(sys_clk), .sys_rst(sys_rst),
        .dsp_clk(dsp_clk), .dsp_rst(dsp_rst),
        .fe_clk(fe_clk), .fe_rst(fe_rst),
        .set_stb_dsp(set_stb_dsp), .set_addr_dsp(set_addr_dsp), .set_data_dsp(set_data_dsp),
        .set_stb_fe(set_stb_fe), .set_addr_fe(set_addr_fe), .set_data_fe(set_data_fe),
        .front_i(dac1_a_int), .front_q(dac1_b_int), .dac_stb(dac1_strobe), .run(run_tx_dsp1),
        .vita_data_sys(sram1_data), .vita_valid_sys(sram1_valid), .vita_ready_sys(sram1_ready),
        .err_data_sys(err_tx1_data), .err_valid_sys(err_tx1_valid), .err_ready_sys(err_tx1_ready),
        .vita_time(vita_time)
    );
    end else begin
        assign sram1_ready = 1;
        assign err_tx1_valid = 0;
        assign run_tx_dsp1 = 0;
    end
    endgenerate

   // /////////////////////////////////////////////////////////////////////////
   // configuration specific LED mapping

    generate
    if (`NUMDUC == 0) begin
        //no tx case? -- each RX DSP gets a LED
        assign {LEDE, LEDB, LEDA, LEDC} = run_rx_dsp;
    end else begin
        //default -- map active frontends to LED based on which DSP is active and how they are mapped
        assign LEDC = |(run_rx_dsp & (~rx_fe_sw));
        assign LEDB = |(run_rx_dsp & rx_fe_sw);
        assign {LEDA, LEDE} = (tx_fe_sw == 0)? {run_tx_dsp0, run_tx_dsp1} : {run_tx_dsp1, run_tx_dsp0};
    end
    endgenerate


   // ///////////////////////////////////////////////////////////////////////////////////
   // DSP TX

`ifndef NO_EXT_FIFO
   assign 	 RAM_A[20:19] = 2'b0;
`endif // !`ifndef NO_EXT_FIFO
   
   ext_fifo #(.EXT_WIDTH(36),.INT_WIDTH(36),.RAM_DEPTH(19),.FIFO_DEPTH(19)) 
     ext_fifo_i1
       (.int_clk(sys_clk),
	.ext_clk(sys_clk),
	.rst(sys_rst | sram_clear),
`ifndef NO_EXT_FIFO
	.RAM_D_pi(RAM_D_pi),
	.RAM_D_po(RAM_D_po),
	.RAM_D_poe(RAM_D_poe),
	.RAM_A(RAM_A[18:0]),
	.RAM_WEn(RAM_WEn),
	.RAM_CENn(RAM_CENn),
	.RAM_LDn(RAM_LDn),
	.RAM_OEn(RAM_OEn),
	.RAM_CE1n(RAM_CE1n),
`else
	.RAM_D_pi(),
	.RAM_D_po(),
	.RAM_D_poe(),
	.RAM_A(),
	.RAM_WEn(),
	.RAM_CENn(),
	.RAM_LDn(),
	.RAM_OEn(),
	.RAM_CE1n(),
`endif // !`ifndef NO_EXT_FIFO
	.datain(dsp_tx0_valid?dsp_tx0_data:dsp_tx1_data),
	.src_rdy_i(dsp_tx0_valid),
	.dst_rdy_o(dsp_tx0_ready),
	.dataout(sram0_data),
	.src_rdy_o(sram0_valid),
	.dst_rdy_i(sram0_ready),
	.src1_rdy_i(dsp_tx1_valid),
	.dst1_rdy_o(dsp_tx1_ready),
	.src1_rdy_o(sram1_valid),
	.dst1_rdy_i(sram1_ready),
	.dataout_1(sram1_data),
	.debug(debug_extfifo),
	.debug2(debug_extfifo2) );

   // /////////////////////////////////////////////////////////////////////////
   // VITA Timing

   time_64bit #(.BASE(SR_TIME64)) time_64bit
     (.clk(dsp_clk), .rst(dsp_rst), .set_stb(set_stb_dsp), .set_addr(set_addr_dsp), .set_data(set_data_dsp),
      .pps(pps_in), .vita_time(vita_time), .vita_time_pps(vita_time_pps), .pps_int(pps_int));

   // /////////////////////////////////////////////////////////////////////////////////////////
   // Debug Pins
  
   assign debug_clk = 2'b00; // {dsp_clk, clk_to_mac};
   assign debug = 32'd0;
   assign debug_gpio_0 = 32'd0;
   assign debug_gpio_1 = 32'd0;
   
endmodule // u2_core
