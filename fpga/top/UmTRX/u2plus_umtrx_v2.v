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

module u2plus_umtrx_v2
  (
   input CLK_FPGA_P, input CLK_FPGA_N,  // Diff
   
   // ADC 1
   input RX1IQSEL,
   input [11:0] RX1D,
   // ADC 2
   input RX2IQSEL,
   input [11:0] RX2D,
   // DAC 1
   output reg TX1IQSEL,
   output reg [11:0] TX1D,
   // DAC 2
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
   output led_stat,
   input FPGA_RESET,
   output debug_clk_p,
   output debug_clk_n,
   output [31:0] debug,
   output [3:1] TXD, input [3:1] RXD, // UARTs

   inout SCL, inout SDA,   // I2C
   
   //AUX
   output AUX_SDAT,
   output AUX_SCLK,
   output AUX_SEN1,
   output AUX_SEN2,
   input AUX_LD1,
   input AUX_LD2,
   //input AUX_XX,
   inout AUX_SCL,
   inout AUX_SDA,

   //PA control
   output ENPA2,
   output ENPA1,
   output LOWPA,
   output DCSYNC,

   // PPS
   input PPS_IN,
   output GPS_ON,

   // SPI
   output SEN_DAC, output SCLK_DAC, output MISO_DAC,

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
   wire 	clk_fpga, dsp_clk, clk_div, dcm_out, wb_clk, clk_icap, lms_clk;

wire DivSw1, DivSw2;
   OBUF DIVSW1_P_pin (.I(DivSw1),.O(DivSw1_P));
   OBUF DIVSW1_N_pin (.I(~DivSw1),.O(DivSw1_N));
   OBUF DIVSW2_P_pin (.I(DivSw2),.O(DivSw2_P));
   OBUF DIVSW2_N_pin (.I(~DivSw2),.O(DivSw2_N));

    //register iogress for all adc/dac signals to force IOB
    reg RX1IQSEL_reg, RX2IQSEL_reg,TX1IQSEL_reg, TX2IQSEL_reg;
    reg [11:0] RX1D_reg, RX2D_reg, TX1D_reg, TX2D_reg;
    always @(posedge lms_clk) begin
        {RX1IQSEL_reg, RX2IQSEL_reg, RX1D_reg, RX2D_reg} <= {RX1IQSEL, RX2IQSEL, RX1D, RX2D};
    end
    always @(negedge lms_clk) begin
        {TX1IQSEL, TX2IQSEL, TX1D, TX2D} <= {TX1IQSEL_reg, TX2IQSEL_reg, TX1D_reg, TX2D_reg};
    end

   /////////////////////////////////////////////////////////////////////
   // Interface to ADC of LMS
   reg [11:0] 	adc1_a, adc1_b, adc2_a, adc2_b;
   reg adc1_strobe, adc2_strobe;

    always @(posedge lms_clk) begin
        if (RX1IQSEL_reg == 1'b1)
            adc1_a <= RX1D_reg; //ADC_I signal
        else
            adc1_b <= RX1D_reg; // ADC_Q signal

        if (RX2IQSEL_reg == 1'b1)
            adc2_a <= RX2D_reg; //ADC_I signal
        else
            adc2_b <= RX2D_reg; // ADC_Q signal

	//FIXME Either the LMS1 and LMS2 can align the RXIQSEL* framing lines,
	//otherwise we need to use an identical strobe for MIMO time alignment.
        //adc1_strobe <= ~RX1IQSEL_reg;
        //adc2_strobe <= ~RX2IQSEL_reg;
        adc1_strobe <= dsp_clk_div2_tx;
        adc2_strobe <= dsp_clk_div2_tx;
    end

   /////////////////////////////////////////////////////////////////////
   // Interface to DAC of LMS
   wire [11:0] dac1_a, dac1_b, dac2_a, dac2_b;
   wire dac1_strobe, dac2_strobe;

    always @(posedge lms_clk) begin
        if (dac1_strobe == 1'b1)
            TX1D_reg <= dac1_a; //DAC_I signal
        else
            TX1D_reg <= dac1_b; //DAC_Q signal

        if (dac2_strobe == 1'b1)
            TX2D_reg <= dac2_a; //DAC_I signal
        else
            TX2D_reg <= dac2_b; //DAC_Q signal

        TX1IQSEL_reg <= ~dac1_strobe;
        TX2IQSEL_reg <= ~dac2_strobe;
    end

    reg dsp_clk_div2_tx=0; // DSP clock signal devided by 2
    always @(negedge lms_clk) dsp_clk_div2_tx = ~dsp_clk_div2_tx;
    assign dac1_strobe = dsp_clk_div2_tx;
    assign dac2_strobe = dsp_clk_div2_tx;

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
    .LOCKED_OUT(led_stat));      // OUT

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
   
   IOBUF aux_scl_pin(.O(aux_scl_pad_i), .IO(AUX_SCL), .I(aux_scl_pad_o), .T(aux_scl_pad_oen_o));
   IOBUF aux_sda_pin(.O(aux_sda_pad_i), .IO(AUX_SDA), .I(aux_sda_pad_o), .T(aux_sda_pad_oen_o));

   // LEDs are active low outputs
   wire [6:0] leds_int;
   assign     {ETH_LEDG,ETH_LED,leds} = {7'b0011111 ^ leds_int};  // drive low to turn on leds
   
   // SPI
   wire       miso, mosi, sclk;

   assign 	{SCLK_DAC,MISO_DAC} = ~SEN_DAC ? {sclk,mosi} : 2'B0;
   assign 	{SCLK1,MOSI1}       = ~SEN1    ? {sclk,mosi} : 2'B0;
   assign 	{SCLK2,MOSI2}       = ~SEN2    ? {sclk,mosi} : 2'B0;
   assign 	{AUX_SCLK,AUX_SDAT} = (~AUX_SEN1 | ~AUX_SEN2) ? {sclk,mosi} : 2'B0;
   
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

   wire 	pps;
   assign GPS_ON = 1'b1;
   assign pps = PPS_IN;

   wire enpa2_o, enpa1_o, lowpa_o;
   OBUF enpa2_pin (.I(enpa2_o),.O(ENPA2));
   OBUF enpa1_pin (.I(enpa1_o),.O(ENPA1));
   OBUF lowpa_pin (.I(lowpa_o),.O(LOWPA));

   reg dcsync_o;
   wire en_dc_sync_o;
   
   IOBUF DCSYNC_pin (.O(), .IO(DCSYNC), .I(dcsync_o), .T(~en_dc_sync_o));
   reg [5:0] dc_count;
   always @(posedge lms_clk) begin
       if (en_dc_sync_o == 1'b1) begin
           if (dc_count == 23) begin
               dcsync_o <= ~dcsync_o;
               dc_count <= 0;
           end else
               dc_count <= dc_count + 1;
       end else begin
           dc_count <= 0;
           dcsync_o <= 1'b0;
       end
   end

   umtrx_core u2p_c(
		     .sys_clk           (dsp_clk),
		     .dsp_clk           (lms_clk),
		     .fe_clk            (clk_icap), //1/2 dsp rate
		     .wb_clk            (wb_clk),
		     .clk_icap		(clk_icap),
		     .clk_to_mac	(CLK_TO_MAC_int2),
		     .pps_in		(pps),
		     .leds		(leds_int),
		     .debug		(debug[31:0]),
		     .debug_clk		({debug_clk_p, debug_clk_n}),
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
		     .adc0_strobe(adc1_strobe),
		     .adc0_a		(adc1_a),
		     .adc0_b		(adc1_b),
		     .adc1_strobe(adc2_strobe),
		     .adc1_a		(adc2_a),
		     .adc1_b		(adc2_b),
		     .lms_res ({LMS2nRST,LMS1nRST}),
		     .dac0_strobe(dac1_strobe),
		     .dac0_a		(dac1_a),
		     .dac0_b		(dac1_b),
		     .dac1_strobe(dac2_strobe),
		     .dac1_a		(dac2_a),
		     .dac1_b		(dac2_b),
		     .scl_pad_i		(scl_pad_i),
		     .scl_pad_o		(scl_pad_o),
		     .scl_pad_oen_o	(scl_pad_oen_o),
		     .sda_pad_i		(sda_pad_i),
		     .sda_pad_o		(sda_pad_o),
		     .sda_pad_oen_o	(sda_pad_oen_o),
		     .aux_scl_pad_i		(aux_scl_pad_i),
		     .aux_scl_pad_o		(aux_scl_pad_o),
		     .aux_scl_pad_oen_o	(aux_scl_pad_oen_o),
		     .aux_sda_pad_i		(aux_sda_pad_i),
		     .aux_sda_pad_o		(aux_sda_pad_o),
		     .aux_sda_pad_oen_o	(aux_sda_pad_oen_o),
           .aux_sen1 (AUX_SEN1),
           .aux_sen2 (AUX_SEN2),
           .aux_ld1 (AUX_LD1),
           .aux_ld2 (AUX_LD2),         
		     .sclk		(sclk),
		     .mosi		(mosi),
		     .miso		(miso),
		     .sen_dac	 (SEN_DAC),
		     .sen_lms1   (SEN1),
		     .sen_lms2 (SEN2),
//Diversity switches
		     .DivSw1(DivSw1),
		     .DivSw2(DivSw2),
//PA control
		     .enpa2(enpa2_o),
		     .enpa1(enpa1_o),
		     .lowpa(lowpa_o),
		     .en_dc_sync(en_dc_sync_o),
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
   
endmodule // u2plus_umtrx_v2
