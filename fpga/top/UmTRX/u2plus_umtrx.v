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

`timescale 1ns / 1ps
//`define LVDS 1
//`define DCM_FOR_RAMCLK
//////////////////////////////////////////////////////////////////////////////////

module u2plus_umtrx
  (
   input CLK_FPGA_P, input CLK_FPGA_N,  // Diff
   
   // ADC 1
   input ADC_CLK_O1,
   input RX1IQSEL,
   output RX1_EN,
   input [11:0] RX1D,
   // ADC 2
   input ADC_CLK_O2,
   input RX2IQSEL,
   output RX2_EN,
   input [11:0] RX2D,
   // DAC 1
   input TX1CLK,
   output TX1EN,
   output reg TX1IQSEL,
   output reg [11:0] TX1D,
   // DAC 2
   input TX2CLK,
   output TX2EN,
   output reg TX2IQSEL,
   output reg [11:0] TX2D,
   //LMS 1 Control
   output SCLK1,
   output LMS1nRST,
   output SEN1,
   input MISO1,
   output MOSI1,
   //LMS 2 Control
   output SCLK2,
   output LMS2nRST,
   output SEN2,
   input MISO2,
   output MOSI2,

   //Diversity switches
   output DivSw1_N,
   output DivSw1_P,
   output DivSw2_N,
   output DivSw2_P,

   // Misc, debug
   output [5:1] leds,  // LED4 is shared w/INIT_B
   input FPGA_RESET,
   output [1:0] debug_clk,
   output [31:0] debug,
   output [3:1] TXD, input [3:1] RXD, // UARTs

   inout SCL, inout SDA,   // I2C

   // PPS
   input PPS_IN,

   // SPI
   output SEN_DAC, output SCLK_DAC, output MOSI_DAC,

   // GigE PHY
   input CLK_TO_MAC,

   output reg [7:0] GMII_TXD,
   output reg GMII_TX_EN,
   output reg GMII_TX_ER,
   output GMII_GTX_CLK,
   input GMII_TX_CLK,  // 100mbps clk

   input GMII_RX_CLK,
   input [7:0] GMII_RXD,
   input GMII_RX_DV,
   input GMII_RX_ER,
   input GMII_COL,
   input GMII_CRS,

   input PHY_INTn,   // open drain
   inout MDIO,
   output MDC,
   output PHY_RESETn,
   output ETH_LED,
   output ETH_LEDG,
   
   // SRAM
   inout [35:0] RAM_D,
   output [20:0] RAM_A,
   output [3:0] RAM_BWn,
   output RAM_ZZ,
   output RAM_LDn,
   output RAM_OEn,
   output RAM_WEn,
   output RAM_CENn,
   output RAM_CLK,
   
   // SPI Flash
   output flash_cs,
   output flash_clk,
   output flash_mosi,
   input flash_miso
   );

   wire  CLK_TO_MAC_int, CLK_TO_MAC_int2;
      
   // FPGA-specific pins connections
   wire 	clk_fpga, dsp_clk, clk_div, dcm_out, wb_clk, clk_icap, lms_clk, clock_ready;

wire DivSw1, DivSw2;
   OBUF DIVSW1_P_pin (.I(DivSw1),.O(DivSw1_P));
   OBUF DIVSW1_N_pin (.I(~DivSw1),.O(DivSw1_N));
   OBUF DIVSW2_P_pin (.I(DivSw2),.O(DivSw2_P));
   OBUF DIVSW2_N_pin (.I(~DivSw2),.O(DivSw2_N));

   
   wire 	exp_time_in;
   
   wire 	exp_time_out;

   wire 	exp_user_in;
   
   wire 	exp_user_out;

   reg [5:0] 	clock_ready_d;
   always @(posedge clk_fpga)
     clock_ready_d[5:0] <= {clock_ready_d[4:0],clock_ready};
   wire 	dcm_rst = ~&clock_ready_d & |clock_ready_d;

`ifdef LVDS
   wire [13:0] 	adc_a, adc_a_inv, adc_b;
   capture_ddrlvds #(.WIDTH(14)) capture_ddrlvds
     (.clk(dsp_clk), .ssclk_p(ADC_clkout_p), .ssclk_n(ADC_clkout_n), 
      .in_p({{ADCA_12_p, ADCA_10_p, ADCA_8_p, ADCA_6_p, ADCA_4_p, ADCA_2_p, ADCA_0_p},
	     {ADCB_12_p, ADCB_10_p, ADCB_8_p, ADCB_6_p, ADCB_4_p, ADCB_2_p, ADCB_0_p}}), 
      .in_n({{ADCA_12_n, ADCA_10_n, ADCA_8_n, ADCA_6_n, ADCA_4_n, ADCA_2_n, ADCA_0_n},
	     {ADCB_12_n, ADCB_10_n, ADCB_8_n, ADCB_6_n, ADCB_4_n, ADCB_2_n, ADCB_0_n}}), 
      .out({adc_a_inv,adc_b}));
   assign adc_a = ~adc_a_inv;
`else
   reg [13:0] 	adc_a, adc_b;
   always @(posedge dsp_clk)
     begin
	adc_a <= ~{ADCA_12_p,ADCA_12_n, ADCA_10_p,ADCA_10_n, ADCA_8_p,ADCA_8_n, ADCA_6_p,ADCA_6_n,
		   ADCA_4_p,ADCA_4_n, ADCA_2_p,ADCA_2_n, ADCA_0_p,ADCA_0_n };
	adc_b <= {ADCB_12_p,ADCB_12_n, ADCB_10_p,ADCB_10_n, ADCB_8_p,ADCB_8_n, ADCB_6_p,ADCB_6_n,
		   ADCB_4_p,ADCB_4_n, ADCB_2_p,ADCB_2_n, ADCB_0_p,ADCB_0_n };
     end
`endif // !`ifdef LVDS
   // Interface to ADC of LMS
   reg [13:0] 	adc_a_0, adc_b_0, adc_a_1, adc_b_1;

   assign RX1_EN = 1'b1;
   assign RX2_EN = 1'b1;

   always @(posedge lms_clk)
     begin
         if (RX1IQSEL == 1'b1)
            adc_a_0 = {RX1D, 2'b00}; //ADC_I signal
         else
            adc_b_0 <= {RX1D, 2'b00}; // ADC_Q signal
     end
   always @(posedge lms_clk)
     begin
         if (RX2IQSEL == 1'b1)
            adc_a_1 = {RX2D, 2'b00}; //ADC_I signal
         else
            adc_b_1 <= {RX2D, 2'b00}; // ADC_Q signal
     end
   
   // Handle Clocks

   pll_clk pll_clk_instance
   (// Clock in ports
    .clk_in(CLK_FPGA_P),      // IN
    // Clock out ports
    .wb_clk(wb_clk),     // OUT 52 MHz
    .dsp_clk(dsp_clk),     // OUT 104 MHz
    .clk270_100(clk270_100_buf),     // OUT 104 MHz
    .clk_fpga(clk_fpga),     // OUT 104 MHz
    .clk_icap(clk_icap),     // OUT 13 MHz, 180 deg 
    .lms_clk(lms_clk),     // OUT 26 MHz
    // Status and control signals
    .LOCKED_OUT(LOCKED_OUT));      // OUT

   pll_rx pll_rx_instance
   (// Clock in ports
    .gmii_rx_clk(GMII_RX_CLK),      // IN
    // Clock out ports
    .clk_rx(clk_rx),     // OUT
    .clk_to_mac(CLK_TO_MAC_int2), // OUT
    .clk_rx_180(clk_rx_180));    // OUT


   OFDDRRSE RAM_CLK_i1 (.Q(RAM_CLK),
			.C0(clk270_100_buf),
			.C1(~clk270_100_buf),
			.CE(1'b1),
			.D0(1'b1),
			.D1(1'b0),
			.R(1'b0),
			.S(1'b0));
  
   // I2C -- Don't use external transistors for open drain, the FPGA implements this
   IOBUF scl_pin(.O(scl_pad_i), .IO(SCL), .I(scl_pad_o), .T(scl_pad_oen_o));
   IOBUF sda_pin(.O(sda_pad_i), .IO(SDA), .I(sda_pad_o), .T(sda_pad_oen_o));

   // LEDs are active low outputs
   wire [6:0] leds_int;
   assign     {ETH_LEDG,ETH_LED,leds} = {7'b0011111 ^ leds_int};  // drive low to turn on leds
   
   // SPI
   wire       miso, mosi, sclk;

   assign 	{SCLK_DAC,MOSI_DAC} = ~SEN_DAC ? {sclk,mosi} : 2'B0;
   assign 	{SCLK1,MOSI1}       = ~SEN1    ? {sclk,mosi} : 2'B0;
   assign 	{SCLK2,MOSI2}       = ~SEN2    ? {sclk,mosi} : 2'B0;
   
   assign 	miso 	= (~SEN1 & MISO1) | (~SEN2 & MISO2) ;
   
   wire 	GMII_TX_EN_unreg, GMII_TX_ER_unreg;
   wire [7:0] 	GMII_TXD_unreg;
   wire 	GMII_GTX_CLK_int;
   
   always @(posedge GMII_GTX_CLK_int)
     begin
	GMII_TX_EN <= GMII_TX_EN_unreg;
	GMII_TX_ER <= GMII_TX_ER_unreg;
	GMII_TXD <= GMII_TXD_unreg;
     end

   OFDDRRSE OFDDRRSE_gmii_inst 
     (.Q(GMII_GTX_CLK),      // Data output (connect directly to top-level port)
      .C0(GMII_GTX_CLK_int),    // 0 degree clock input
      .C1(~GMII_GTX_CLK_int),    // 180 degree clock input
      .CE(1),    // Clock enable input
      .D0(0),    // Posedge data input
      .D1(1),    // Negedge data input
      .R(0),      // Synchronous reset input
      .S(0)       // Synchronous preset input
      );
   

   //
   // Instantiate IO for Bidirectional bus to SRAM
   //
   wire [35:0] RAM_D_pi;
   wire [35:0] RAM_D_po;
   wire        RAM_D_poe;
   
   genvar      i;
   
   generate  
      for (i=0;i<36;i=i+1)
        begin : gen_RAM_D_IO
`ifndef NO_EXT_FIFO

	   IOBUF #(
		   .DRIVE(12),
		   .IOSTANDARD("LVCMOS25"),
		   .SLEW("FAST")
		   )
	     RAM_D_i (
		      .O(RAM_D_pi[i]),
		      .I(RAM_D_po[i]),
		      .IO(RAM_D[i]),
		      .T(RAM_D_poe)
		      );
`endif // !`ifndef NO_EXT_FIFO
	end // block: gen_RAM_D_IO
   endgenerate

   
   
   wire [11:0] dac_a_int1, dac_b_int1, dac_a_int2, dac_b_int2;
   // Interface to DAC of LMS

   assign TX1EN = 1'b1;
   assign TX2EN = 1'b1;

   reg dsp_clk_div2_tx=0; // DSP clock signal devided by 2
   always @(negedge lms_clk)
   begin
      dsp_clk_div2_tx = ~dsp_clk_div2_tx;
      if (dsp_clk_div2_tx)
         begin
            TX1D <= dac_a_int1[11:0]; //DAC_I signal
            TX1IQSEL = 1'b0;
            TX2D <= dac_a_int2[11:0]; //DAC_I signal
            TX2IQSEL = 1'b0;
         end
      else
         begin
            TX1D <= dac_b_int1[11:0]; //DAC_Q signal
            TX1IQSEL = 1'b1;
            TX2D <= dac_b_int2[11:0]; //DAC_Q signal
            TX2IQSEL = 1'b1;
         end
      end

   wire 	pps;
   assign pps = PPS_IN;

   
   u2plus_core u2p_c(.dsp_clk           (dsp_clk),
		     .wb_clk            (wb_clk),
		     .lms_clk  (clk_icap),
		     .clk_icap		(clk_icap),
		     .clock_ready       (clock_ready),
		     .clk_to_mac	(CLK_TO_MAC_int2),
		     .pps_in		(pps),
		     .leds		(leds_int),
		     .debug		(debug[31:0]),
		     .debug_clk		(debug_clk[1:0]),
		     .exp_time_in	(exp_time_in),
		     .exp_time_out	(exp_time_out),
		     .GMII_COL		(GMII_COL),
		     .GMII_CRS		(GMII_CRS),
		     .GMII_TXD		(GMII_TXD_unreg[7:0]),
		     .GMII_TX_EN	(GMII_TX_EN_unreg),
		     .GMII_TX_ER	(GMII_TX_ER_unreg),
		     .GMII_GTX_CLK	(GMII_GTX_CLK_int),
		     .GMII_TX_CLK	(GMII_TX_CLK),
		     .GMII_RXD		(GMII_RXD[7:0]),
		     .GMII_RX_CLK	(clk_rx),
		     .GMII_RX_DV	(GMII_RX_DV),
		     .GMII_RX_ER	(GMII_RX_ER),
		     .MDIO		(MDIO),
		     .MDC		(MDC),
		     .PHY_INTn		(PHY_INTn),
		     .PHY_RESETn	(PHY_RESETn),
		     .ser_enable  (),
		     .ser_prbsen  (),
		     .ser_loopen  (),
		     .ser_rx_en   (),
		     .ser_tx_clk  (),
		     .ser_t       (),
		     .ser_tklsb   (),
		     .ser_tkmsb   (),
		     .ser_rx_clk  (),
		     .ser_r       (),
		     .ser_rklsb   (),
		     .ser_rkmsb   (),
		     .adc_a_0		(adc_a_0[13:0]),
		     .adc_ovf_a_0		(1'b0),
		     .adc_on_a_0		(),
		     .adc_oe_a_0		(),
		     .adc_b_0		(adc_b_0[13:0]),
		     .adc_ovf_b_0		(1'b0),
		     .adc_on_b_0		(),
		     .adc_oe_b_0		(),
		     .adc_a_1		(adc_a_1[13:0]),
		     .adc_ovf_a_1		(1'b0),
		     .adc_on_a_1		(),
		     .adc_oe_a_1		(),
		     .adc_b_1		(adc_b_1[13:0]),
		     .adc_ovf_b_1		(1'b0),
		     .adc_on_b_1		(),
		     .adc_oe_b_1		(),
		     .lms_res ({LMS2nRST,LMS1nRST}),
		     .dac1_a		(dac_a_int2),
		     .dac1_b		(dac_b_int2),
		     .dac_a		(dac_a_int1),
		     .dac_b		(dac_b_int1),
		     .scl_pad_i		(scl_pad_i),
		     .scl_pad_o		(scl_pad_o),
		     .scl_pad_oen_o	(scl_pad_oen_o),
		     .sda_pad_i		(sda_pad_i),
		     .sda_pad_o		(sda_pad_o),
		     .sda_pad_oen_o	(sda_pad_oen_o),
		     .clk_en		(),
		     .clk_sel		(),
		     .clk_func		(),
		     .clk_status	(),
		     .sclk		(sclk),
		     .mosi		(mosi),
		     .miso		(miso),
		     .sen_dac	 (SEN_DAC),
		     .sen_lms1   (SEN1),
		     .sen_lms2 (SEN2),
		     .io_tx		 (),
		     .io_rx		 (),
//Diversity switches
		     .DivSw1(DivSw1),
		     .DivSw2(DivSw2),
`ifndef NO_EXT_FIFO
		     .RAM_D_po          (RAM_D_po),
		     .RAM_D_pi          (RAM_D_pi),
		     .RAM_D_poe         (RAM_D_poe),
		     .RAM_A             (RAM_A),
		     .RAM_CE1n          (RAM_CE1n),
		     .RAM_CENn          (RAM_CENn),
		     .RAM_WEn           (RAM_WEn),
		     .RAM_OEn           (RAM_OEn),
		     .RAM_LDn           (RAM_LDn), 
`endif // !`ifndef NO_EXT_FIFO
		     .uart_tx_o         (TXD[3:1]),
		     .uart_rx_i         ({1'b1,RXD[3:1]}),
		     .uart_baud_o       (),
		     .sim_mode          (1'b0),
		     .clock_divider     (2),
		     .button            (FPGA_RESET),
		     .spiflash_cs       (flash_cs),
		     .spiflash_clk      (flash_clk),
		     .spiflash_miso     (flash_miso),
		     .spiflash_mosi     (flash_mosi)
		     );

`ifndef NO_EXT_FIFO
   // Drive low so that RAM does not sleep.
   assign RAM_ZZ = 0;
   // Byte Writes are qualified by the global write enable
   // Always do 36bit operations to extram.
   assign RAM_BWn = 4'b0000;
`endif // !`ifndef NO_EXT_FIFO
   
endmodule // u2plus
