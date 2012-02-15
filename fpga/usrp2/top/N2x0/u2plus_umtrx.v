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
   output reg LMS1nRST,
   output SEN1,
   input MISO1,
   output MOSI1,
   //LMS 2 Control
   output SCLK2,
   output reg LMS2nRST,
   output SEN2,
   input MISO2,
   output MOSI2,

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
   output SEN_DAC, output SCLK_DAC, input MISO_DAC,

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
`ifndef UMTRX
   IBUFG phyclk (.O(CLK_TO_MAC_int), .I(CLK_TO_MAC));
   BUFG phyclk2 (.O(CLK_TO_MAC_int2), .I(CLK_TO_MAC_int));
`endif // !`ifndef UMTRX
      
   // FPGA-specific pins connections
   wire 	clk_fpga, dsp_clk, clk_div, dcm_out, wb_clk, clk_icap, clock_ready;

`ifndef UMTRX
   IBUFGDS clk_fpga_pin (.O(clk_fpga),.I(CLK_FPGA_P),.IB(CLK_FPGA_N));
   defparam 	clk_fpga_pin.IOSTANDARD = "LVPECL_25";
`endif // !`ifndef UMTRX
   
   wire 	exp_time_in;
`ifndef UMTRX
   IBUFDS exp_time_in_pin (.O(exp_time_in),.I(exp_time_in_p),.IB(exp_time_in_n));
   defparam 	exp_time_in_pin.IOSTANDARD = "LVDS_25";
`endif // !`ifndef UMTRX
   
   wire 	exp_time_out;
`ifndef UMTRX
   OBUFDS exp_time_out_pin (.O(exp_time_out_p),.OB(exp_time_out_n),.I(exp_time_out));
   defparam 	exp_time_out_pin.IOSTANDARD  = "LVDS_25";
`endif // !`ifndef UMTRX

   wire 	exp_user_in;
`ifndef UMTRX
   IBUFDS exp_user_in_pin (.O(exp_user_in),.I(exp_user_in_p),.IB(exp_user_in_n));
   defparam 	exp_user_in_pin.IOSTANDARD = "LVDS_25";
`endif // !`ifndef UMTRX
   
   wire 	exp_user_out;
`ifndef UMTRX
   OBUFDS exp_user_out_pin (.O(exp_user_out_p),.OB(exp_user_out_n),.I(exp_user_out));
   defparam 	exp_user_out_pin.IOSTANDARD  = "LVDS_25";
`endif // !`ifndef UMTRX

   reg [5:0] 	clock_ready_d;
   always @(posedge clk_fpga)
     clock_ready_d[5:0] <= {clock_ready_d[4:0],clock_ready};
   wire 	dcm_rst = ~&clock_ready_d & |clock_ready_d;

`ifndef LMS602D_FRONTEND
   // ADC A is inverted on the schematic to facilitate a clean layout
   //  We account for that here by inverting it
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
`else
   // Interface to ADC of LMS
   reg [13:0] 	adc_a_0, adc_b_0, adc_a_1, adc_b_1;

   assign RX1_EN = 1'b1;
   assign RX2_EN = 1'b1;

   always @(posedge dsp_clk)
     begin
         LMS1nRST = 1'b1;
         if (RX1IQSEL == 1'b0)
            adc_a_0 = {2'b00, RX1D}; //ADC_I signal
         else
            adc_b_0 <= {2'b00, RX1D}; // ADC_Q signal
     end
   always @(posedge dsp_clk)
     begin
         LMS2nRST = 1'b1;
         if (RX2IQSEL == 1'b0)
            adc_a_1 = {2'b00, RX2D}; //ADC_I signal
         else
            adc_b_1 <= {2'b00, RX2D}; // ADC_Q signal
     end
`endif // !`ifndef LMS602D_FRONTEND
   
   // Handle Clocks
`ifndef UMTRX
   DCM DCM_INST (.CLKFB(dsp_clk), 
                 .CLKIN(clk_fpga), 
                 .DSSEN(0), 
                 .PSCLK(0), 
                 .PSEN(0), 
                 .PSINCDEC(0), 
                 .RST(dcm_rst), 
                 .CLKDV(clk_div), 
                 .CLKFX(), 
                 .CLKFX180(), 
                 .CLK0(dcm_out), 
                 .CLK2X(), 
                 .CLK2X180(), 
                 .CLK90(), 
                 .CLK180(), 
                 .CLK270(clk270_100), 
                 .LOCKED(LOCKED_OUT), 
                 .PSDONE(), 
                 .STATUS());
   defparam DCM_INST.CLK_FEEDBACK = "1X";
   defparam DCM_INST.CLKDV_DIVIDE = 2.0;
   defparam DCM_INST.CLKFX_DIVIDE = 1;
   defparam DCM_INST.CLKFX_MULTIPLY = 4;
   defparam DCM_INST.CLKIN_DIVIDE_BY_2 = "FALSE";
   defparam DCM_INST.CLKIN_PERIOD = 10.000;
   defparam DCM_INST.CLKOUT_PHASE_SHIFT = "NONE";
   defparam DCM_INST.DESKEW_ADJUST = "SYSTEM_SYNCHRONOUS";
   defparam DCM_INST.DFS_FREQUENCY_MODE = "LOW";
   defparam DCM_INST.DLL_FREQUENCY_MODE = "LOW";
   defparam DCM_INST.DUTY_CYCLE_CORRECTION = "TRUE";
   defparam DCM_INST.FACTORY_JF = 16'h8080;
   defparam DCM_INST.PHASE_SHIFT = 0;
   defparam DCM_INST.STARTUP_WAIT = "FALSE";

   BUFG dspclk_BUFG (.I(dcm_out), .O(dsp_clk));
   BUFG wbclk_BUFG (.I(clk_div), .O(wb_clk));
   // Create clock for external SRAM thats -90degree phase to DSPCLK (i.e) 2nS earlier at 100MHz.
   BUFG  clk270_100_buf_i1 (.I(clk270_100), 
			    .O(clk270_100_buf));
`else

   pll_clk pll_clk_instance
   (// Clock in ports
    .clk_in(CLK_FPGA_P),      // IN
    // Clock out ports
    .wb_clk(wb_clk),     // OUT 52 MHz
    .dsp_clk(dsp_clk),     // OUT 104 MHz
    .clk270_100(clk270_100_buf),     // OUT 104 MHz
    .clk_fpga(clk_fpga),     // OUT 104 MHz
    .clk_icap(clk_icap),     // OUT 13 MHz, 180 deg 
    // Status and control signals
    .LOCKED_OUT(LOCKED_OUT));      // OUT

   pll_rx pll_rx_instance
   (// Clock in ports
    .gmii_rx_clk(GMII_RX_CLK),      // IN
    // Clock out ports
    .clk_rx(clk_rx),     // OUT
    .clk_to_mac(CLK_TO_MAC_int2), // OUT
    .clk_rx_180(clk_rx_180));    // OUT

`endif // !`ifndef UMTRX

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
`ifndef UMTRX
   wire [5:0] leds_int;
   assign     {ETH_LED,leds} = {6'b011111 ^ leds_int};  // drive low to turn on leds
`else
   wire [6:0] leds_int;
   assign     {ETH_LEDG,ETH_LED,leds} = {7'b0011111 ^ leds_int};  // drive low to turn on leds
`endif // !`ifndef UMTRX
   
   // SPI
   wire       miso, mosi, sclk;

`ifndef UMTRX
   assign 	{SCLK_CLK,MOSI_CLK} 	   = ~SEN_CLK ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_DAC,MOSI_DAC} 	   = ~SEN_DAC ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_ADC,MOSI_ADC} 	   = ~SEN_ADC ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_TX_DB,MOSI_TX_DB}    = ~SEN_TX_DB ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_TX_DAC,MOSI_TX_DAC}  = ~SEN_TX_DAC ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_TX_ADC,MOSI_TX_ADC}  = ~SEN_TX_ADC ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_RX_DB,MOSI_RX_DB}    = ~SEN_RX_DB ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_RX_DAC,MOSI_RX_DAC}  = ~SEN_RX_DAC ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_RX_ADC,MOSI_RX_ADC}  = ~SEN_RX_ADC ? {sclk,mosi} : 2'B0;
   
   assign 	miso 			   = (~SEN_CLK & MISO_CLK) | (~SEN_DAC & MISO_DAC) |
					     (~SEN_TX_DB & MISO_TX_DB) | (~SEN_TX_ADC & MISO_TX_ADC) |
					     (~SEN_RX_DB & MISO_RX_DB) | (~SEN_RX_ADC & MISO_RX_ADC);
`else
   assign 	SCLK_DAC = sclk;
   assign 	{SCLK1,MOSI1} 	   = ~SEN1 ? {sclk,mosi} : 2'B0;
   assign 	{SCLK2,MOSI2} 	   = ~SEN2 ? {sclk,mosi} : 2'B0;
   
   assign 	miso 	= (~SEN_DAC & MISO_DAC) | (~SEN1 & MISO1) | (~SEN2 & MISO2) ;
`endif // !`ifndef UMTRX
   
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
   
`ifndef NO_SERDES
   wire ser_tklsb_unreg, ser_tkmsb_unreg;
   wire [15:0] ser_t_unreg;
   wire        ser_tx_clk_int;
   
   always @(posedge ser_tx_clk_int)
     begin
	ser_tklsb <= ser_tklsb_unreg;
	ser_tkmsb <= ser_tkmsb_unreg;
	ser_t <= ser_t_unreg;
     end

   assign ser_tx_clk = clk_fpga;

   reg [15:0] ser_r_int;
   reg 	      ser_rklsb_int, ser_rkmsb_int;

   always @(posedge ser_rx_clk)
     begin
	ser_r_int <= ser_r;
	ser_rklsb_int <= ser_rklsb;
	ser_rkmsb_int <= ser_rkmsb;
     end
   
   /*
   OFDDRRSE OFDDRRSE_serdes_inst 
     (.Q(ser_tx_clk),      // Data output (connect directly to top-level port)
      .C0(ser_tx_clk_int),    // 0 degree clock input
      .C1(~ser_tx_clk_int),    // 180 degree clock input
      .CE(1),    // Clock enable input
      .D0(0),    // Posedge data input
      .D1(1),    // Negedge data input
      .R(0),      // Synchronous reset input
      .S(0)       // Synchronous preset input
      );
   */

`endif // !`ifndef NO_SERDES

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

   
   
   wire [15:0] dac_a_int1, dac_b_int1, dac_a_int2, dac_b_int2;
`ifndef LMS602D_FRONTEND
   // DAC A and B are swapped in schematic to facilitate clean layout
   // DAC A is also inverted in schematic to facilitate clean layout
   always @(negedge dsp_clk) DACA <= ~dac_b_int;
   always @(negedge dsp_clk) DACB <= dac_a_int;
`else
   // Interface to DAC of LMS

   assign TX1EN = 1'b1;
   assign TX2EN = 1'b1;

   reg dsp_clk_div2_tx=0; // DSP clock signal devided by 2
   always @(negedge dsp_clk)
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
`endif // !`ifndef LMS602D_FRONTEND

   wire 	pps;
`ifndef UMTRX
   assign pps = PPS_IN ^ PPS2_IN;
`else
   assign pps = PPS_IN;
`endif // !`ifndef UMTRX

   
   u2plus_core u2p_c(.dsp_clk           (dsp_clk),
		     .wb_clk            (wb_clk),
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
`ifndef UMTRX
		     .GMII_RX_CLK	(GMII_RX_CLK),
`else
		     .GMII_RX_CLK	(clk_rx),
`endif // !`ifndef UMTRX
		     .GMII_RX_DV	(GMII_RX_DV),
		     .GMII_RX_ER	(GMII_RX_ER),
		     .MDIO		(MDIO),
		     .MDC		(MDC),
		     .PHY_INTn		(PHY_INTn),
		     .PHY_RESETn	(PHY_RESETn),
`ifndef NO_SERDES
		     .ser_enable	(ser_enable),
		     .ser_prbsen	(ser_prbsen),
		     .ser_loopen	(ser_loopen),
		     .ser_rx_en		(ser_rx_en),
		     .ser_tx_clk	(ser_tx_clk_int),
		     .ser_t		(ser_t_unreg[15:0]),
		     .ser_tklsb		(ser_tklsb_unreg),
		     .ser_tkmsb		(ser_tkmsb_unreg),
		     .ser_rx_clk	(ser_rx_clk),
		     .ser_r		(ser_r_int[15:0]),
		     .ser_rklsb		(ser_rklsb_int),
		     .ser_rkmsb		(ser_rkmsb_int),
`else
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
`endif // !`ifndef NO_SERDES
`ifndef LMS602D_FRONTEND
		     .adc_a		(adc_a[13:0]),
		     .adc_ovf_a		(1'b0),
		     .adc_on_a		(),
		     .adc_oe_a		(),
		     .adc_b		(adc_b[13:0]),
		     .adc_ovf_b		(1'b0),
		     .adc_on_b		(),
		     .adc_oe_b		(),
`else
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
`endif // !`ifndef LMS602D_FRONTEND
		     .dac_a		(dac_a_int1[15:0]),
		     .dac_b		(dac_b_int1[15:0]),
		     .scl_pad_i		(scl_pad_i),
		     .scl_pad_o		(scl_pad_o),
		     .scl_pad_oen_o	(scl_pad_oen_o),
		     .sda_pad_i		(sda_pad_i),
		     .sda_pad_o		(sda_pad_o),
		     .sda_pad_oen_o	(sda_pad_oen_o),
`ifndef UMTRX
		     .clk_en		(clk_en[1:0]),
		     .clk_sel		(clk_sel[1:0]),
		     .clk_func		(clk_func),
		     .clk_status	(clk_status),
`else
		     .clk_en		(),
		     .clk_sel		(),
		     .clk_func		(),
		     .clk_status	(),
`endif // !`ifndef UMTRX
		     .sclk		(sclk),
		     .mosi		(mosi),
		     .miso		(miso),
`ifndef UMTRX
		     .sen_clk		(SEN_CLK),
		     .sen_dac		(SEN_DAC),
		     .sen_adc           (SEN_ADC),
		     .sen_tx_db		(SEN_TX_DB),
		     .sen_tx_adc	(SEN_TX_ADC),
		     .sen_tx_dac	(SEN_TX_DAC),
		     .sen_rx_db		(SEN_RX_DB),
		     .sen_rx_adc	(SEN_RX_ADC),
		     .sen_rx_dac	(SEN_RX_DAC),
		     .io_tx		(io_tx[15:0]),
		     .io_rx		(io_rx[15:0]),
`else
		     .sen_dac	 (SEN_DAC),
		     .sen_lms1   (SEN1),
		     .sen_lms2 (SEN2),
		     .io_tx		 (),
		     .io_rx		 (),
`endif // !`ifndef UMTRX
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
