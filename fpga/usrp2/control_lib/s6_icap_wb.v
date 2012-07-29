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


//ATTENTION!! Maximum ICAP clock in Spartan 6 is 20 MHz!!!

module s6_icap_wb
  (input clk, input clk_icap, input reset,
   input cyc_i, input stb_i, input we_i, output reg ack_o,
   input [31:0] dat_i, output reg[31:0] dat_o);//, output [31:0] debug_out);

	wire 	BUSY, CE, WRITE, ICAPCLK;
	reg s_cyc_i;
	reg s1_cyc_i;
	reg s_stb_i;
	reg s1_stb_i;
	reg s_we_i;
	reg s1_we_i;
	wire s1_ack_o;
	reg s_ack_o;
	reg[31:0] s_dat_i;
	reg[31:0] s1_dat_i;
	
	reg[31:0] s_dat_o;
	wire[31:0] s1_dat_o;
	
   assign s1_dat_o[31:16] = 16'd0;
  
 	
   //changed this to gray-ish code to prevent glitching
   reg [2:0] 	icap_state;
   localparam ICAP_IDLE  = 0;
   localparam ICAP_WR0 	 = 1;
   localparam ICAP_WR1 	 = 5;
   localparam ICAP_RD0 	 = 2;
   localparam ICAP_RD1 	 = 3;

	always @(negedge clk_icap)
	 if (reset)
	   begin
			s_stb_i <= 1'b0;
			s_cyc_i <= 1'b0;
			s_we_i <= 1'b0;
			s1_stb_i <= 1'b0;
			s1_cyc_i <= 1'b0;
			s1_we_i <= 1'b0;
			s_dat_i <= 32'b0;
			s1_dat_i <= 32'b0;
		end 
	 else
		begin
			s_stb_i <= stb_i;
			s_cyc_i <= cyc_i;
			s_we_i <= we_i;
			s1_stb_i <= s_stb_i;
			s1_cyc_i <= s_cyc_i;
			s1_we_i <= s_we_i;
			s_dat_i <= dat_i;
			s1_dat_i <= s_dat_i;
		end
	
   always @(negedge clk_icap) //13 MHz ICAP clock w/180 degree phase shift from source 26 MHz clock
     if(reset)
       icap_state 	<= ICAP_IDLE;
     else
       case(icap_state)
	 ICAP_IDLE :
	   begin
	   if(s1_stb_i & s1_cyc_i)
		if(s1_we_i)
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

   BUFGCE BUFGCE_inst (
      .O(ICAPCLK),   // 1-bit output: Clock buffer output
      .CE(CE), // 1-bit input: Clock buffer select
      .I(clk_icap)    // 1-bit input: Clock buffer input (S=0)
   );

   assign s1_ack_o = (icap_state == ICAP_WR1) | (icap_state == ICAP_RD1);
   //assign debug_out = {17'd0, BUSY, dat_i[7:0], ~CE, ICAPCLK, ~WRITE, icap_state};
   
   ICAP_SPARTAN6 ICAP_SPARTAN6_inst
     (.BUSY(BUSY),          // Busy output
      .O(s1_dat_o[15:0]),            // 32-bit data output
      .CE(~CE),              // Clock enable input
      .CLK(ICAPCLK),            // Clock input
      .I(s1_dat_i[15:0]),            // 32-bit data input
      .WRITE(~WRITE)         // Write input
      );

//cross back to 54 MHz clock domain
	always @(posedge clk)
	if (reset)
   	begin
			s_dat_o <= 32'd0;
		   dat_o <= 32'd0;
			s_ack_o <= 1'b0;
			ack_o <= 1'b0;
		end
	else 
		begin
			s_ack_o <= s1_ack_o;
			ack_o <= s_ack_o;
		   s_dat_o <= s1_dat_o;
		   dat_o[15:0] <= s_dat_o[15:0];
		end
		
endmodule // s6_icap_wb
