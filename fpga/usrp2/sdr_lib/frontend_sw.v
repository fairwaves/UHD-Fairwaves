// Copyright 2013 Fairwaves
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

module frontend_sw
  #(parameter BASE=0)
   (input clk, input rst,
    input set_stb, input [7:0] set_addr, input [31:0] set_data,
    input [23:0] i_0_in, input [23:0] q_0_in, 
    input [23:0] i_1_in, input [23:0] q_1_in,
    input run_0_in, input run_1_in,
    input adc_ovf_i_0_in, input adc_ovf_q_0_in,
    input adc_ovf_i_1_in, input adc_ovf_q_1_in,    
    output reg [23:0] i_0_mux, output reg [23:0] q_0_mux, 
    output reg [23:0] i_1_mux, output reg [23:0] q_1_mux,
    output reg run_0_mux, output reg run_1_mux,
    output reg adc_ovf_i_0_mux, output reg adc_ovf_q_0_mux,
    output reg adc_ovf_i_1_mux, output reg adc_ovf_q_1_mux);

   wire front_sw;
   setting_reg #(.my_addr(BASE),.width(1), .at_reset(32'd0)) sr_front_sw
     (.clk(clk),.rst(wb_rst),.strobe(set_stb),.addr(set_addr),.in(set_data),.out(front_sw),.changed());
     
   always @(posedge clk)
      if(~front_sw)
         begin
            i_0_mux <= i_0_in;
            q_0_mux <= q_0_in;
            
            i_1_mux <= i_1_in;
            q_1_mux <= q_1_in;
            
            run_0_mux <= run_0_in;
            run_1_mux <= run_1_in;
            
            adc_ovf_i_0_mux <= adc_ovf_i_0_in;
            adc_ovf_q_0_mux <= adc_ovf_q_0_in;
            
            adc_ovf_i_1_mux <= adc_ovf_i_1_in;
            adc_ovf_q_1_mux <= adc_ovf_q_1_in;
         end
      else
         begin
            i_0_mux <= i_1_in;
            q_0_mux <= q_1_in;
            
            i_1_mux <= i_0_in;
            q_1_mux <= q_0_in;
            
            run_0_mux <= run_1_in;
            run_1_mux <= run_0_in;
            
            adc_ovf_i_0_mux <= adc_ovf_i_1_in;
            adc_ovf_q_0_mux <= adc_ovf_q_1_in;
            
            adc_ovf_i_1_mux <= adc_ovf_i_0_in;
            adc_ovf_q_1_mux <= adc_ovf_q_0_in;
         end
     
endmodule
