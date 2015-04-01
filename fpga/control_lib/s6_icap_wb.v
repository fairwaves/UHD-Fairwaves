//
// Copyright 2013 Andrew Karpenkov, Fairwaves LLC
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

    reg slow_clk_icap;
    always @(posedge clk_icap) slow_clk_icap <= ~slow_clk_icap;
	
	wire 	BUSY, CE, WRITE;
	wire[31:0] s1_dat_i;
	reg[31:0] s_dat_o;
	wire[31:0] s1_dat_o;
	
   assign s1_dat_o[31:16] = 16'd0;
  
   wire  full, empty;

   fifo_xlnx_16x40_2clk icap_fifo
     (.rst(reset),
      .wr_clk(clk), .din(dat_i), .wr_en(we_i & stb_i & ~ack_o & ~full), .full(full),
      .rd_clk(slow_clk_icap), .dout(s1_dat_i), .rd_en(~empty), .empty(empty));

   assign WRITE 	 = empty;
   assign CE 		 = empty;
   
   ICAP_SPARTAN6 ICAP_SPARTAN6_inst
     (.BUSY(BUSY),          // Busy output
      .O(s1_dat_o[15:0]),            // 16-bit data output
      .CE(CE),              // Clock enable input
      .CLK(slow_clk_icap),            // Clock input
      .I(s1_dat_i[15:0]),            // 16-bit data input
      .WRITE(WRITE)         // Write input
      );

//cross back to Wishbone clock domain
	always @(posedge clk)
	if (reset)
   	begin
			s_dat_o <= 32'd0;
		   dat_o <= 32'd0;
			ack_o <= 1'b0;
		end
	else 
		begin
		   s_dat_o <= s1_dat_o;
		   dat_o[15:0] <= s_dat_o[15:0];
         ack_o <= stb_i & ~ack_o;
		end
		
endmodule // s6_icap_wb
