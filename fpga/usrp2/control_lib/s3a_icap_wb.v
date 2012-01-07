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



module s3a_icap_wb
  (input clk, input reset,
   input cyc_i, input stb_i, input we_i, output ack_o,
   input [31:0] dat_i, output [31:0] dat_o);//, output [31:0] debug_out);

`ifndef SPARTAN6
   assign dat_o[31:8] = 24'd0;
`else
   assign dat_o[31:16] = 16'd0;
`endif // !`ifndef SPARTAN6
   
   wire 	BUSY, CE, WRITE, ICAPCLK;
   
   //changed this to gray-ish code to prevent glitching
   reg [2:0] 	icap_state;
   localparam ICAP_IDLE  = 0;
   localparam ICAP_WR0 	 = 1;
   localparam ICAP_WR1 	 = 5;
   localparam ICAP_RD0 	 = 2;
   localparam ICAP_RD1 	 = 3;

   always @(posedge clk)
     if(reset)
       icap_state 	<= ICAP_IDLE;
     else
       case(icap_state)
	 ICAP_IDLE :
	   begin
	      if(stb_i & cyc_i)
		if(we_i)
		  icap_state <= ICAP_WR0;
		else
		  icap_state <= ICAP_RD0;
	   end
	 ICAP_WR0 :
	   icap_state <= ICAP_WR1;
	 ICAP_WR1 :
	   icap_state <= ICAP_IDLE;
	 ICAP_RD0 :
	   icap_state <= ICAP_RD1;
	 ICAP_RD1 :
	   icap_state <= ICAP_IDLE;
       endcase // case (icap_state)

   assign WRITE 	 = (icap_state == ICAP_WR0) | (icap_state == ICAP_WR1);
   assign CE 		 = (icap_state == ICAP_WR0) | (icap_state == ICAP_RD0);
`ifndef SPARTAN6
   assign ICAPCLK        = CE & (~clk);
`else
   wire 	clk_fb, clk_out, clk_inv;
   DCM DCM_INST_ICAP (.CLKFB(clk_fb),
                 .CLKIN(clk),
                 .DSSEN(0),
                 .PSCLK(0),
                 .PSEN(0),
                 .PSINCDEC(0),
                 .RST(reset),
                 .CLKDV(),
                 .CLKFX(),
                 .CLKFX180(),
                 .CLK0(clk_out),
                 .CLK2X(),
                 .CLK2X180(),
                 .CLK90(),
                 .CLK180(clk_inv),
                 .CLK270(),
                 .LOCKED(),
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

   BUFG wbclk_BUFG (.I(clk_out), .O(clk_fb));

   BUFGCE BUFGCE_inst (
      .O(ICAPCLK),   // 1-bit output: Clock buffer output
      .CE(CE), // 1-bit input: Clock buffer select
      .I(clk_inv)    // 1-bit input: Clock buffer input (S=0)
   );
`endif // !`ifndef SPARTAN6

   assign ack_o = (icap_state == ICAP_WR1) | (icap_state == ICAP_RD1);
   //assign debug_out = {17'd0, BUSY, dat_i[7:0], ~CE, ICAPCLK, ~WRITE, icap_state};
   
`ifndef SPARTAN6
   ICAP_SPARTAN3A ICAP_SPARTAN3A_inst 
     (.BUSY(BUSY),          // Busy output
      .O(dat_o[7:0]),            // 32-bit data output
      .CE(~CE),              // Clock enable input
      .CLK(ICAPCLK),            // Clock input
      .I(dat_i[7:0]),            // 32-bit data input
      .WRITE(~WRITE)         // Write input
      );
`else
   ICAP_SPARTAN6 ICAP_SPARTAN6_inst
     (.BUSY(BUSY),          // Busy output
      .O(dat_o[15:0]),            // 32-bit data output
      .CE(~CE),              // Clock enable input
      .CLK(ICAPCLK),            // Clock input
      .I(dat_i[15:0]),            // 32-bit data input
      .WRITE(~WRITE)         // Write input
      );
`endif // !`ifndef SPARTAN6

endmodule // s3a_icap_wb
