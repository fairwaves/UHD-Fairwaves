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
  #(parameter BASE=0,
    parameter WIDTH=4)
   (input clk, input rst,
    input set_stb, input [7:0] set_addr, input [31:0] set_data,
    input [23:0] i_0_in, input [23:0] q_0_in, 
    input [23:0] i_1_in, input [23:0] q_1_in,
    input run_0_in, input run_1_in,
    input run_2_in, input run_3_in,
    input adc_ovf_i_0_in, input adc_ovf_q_0_in,
    input adc_ovf_i_1_in, input adc_ovf_q_1_in,    
    output [23:0] i_0_mux, output [23:0] q_0_mux, 
    output [23:0] i_1_mux, output [23:0] q_1_mux,
    output [23:0] i_2_mux, output [23:0] q_2_mux, 
    output [23:0] i_3_mux, output [23:0] q_3_mux,
    output run_0_mux, output run_1_mux,
    output run_2_mux, output run_3_mux,
    output adc_ovf_i_0_mux, output adc_ovf_q_0_mux,
    output adc_ovf_i_1_mux, output adc_ovf_q_1_mux,
    output adc_ovf_i_2_mux, output adc_ovf_q_2_mux,
    output adc_ovf_i_3_mux, output adc_ovf_q_3_mux);

   wire [WIDTH-1:0] front_sw;
   setting_reg #(.my_addr(BASE),.width(WIDTH), .at_reset(32'd0)) sr_front_sw
     (.clk(clk),.rst(wb_rst),.strobe(set_stb),.addr(set_addr),.in(set_data),.out(front_sw),.changed());
     
   reg [23:0] i_mux [WIDTH-1:0], q_mux [WIDTH-1:0];
   reg [WIDTH-1:0] run_mux ;
   reg [WIDTH-1:0] adc_ovf_i_mux, adc_ovf_q_mux ;
   wire [WIDTH-1:0] run_in;
   
   assign i_0_mux = i_mux[0];   assign q_0_mux = q_mux[0]; 
   assign i_1_mux = i_mux[1];   assign q_1_mux = q_mux[1];
   assign i_2_mux = i_mux[2];   assign q_2_mux = q_mux[2];
   assign i_3_mux = i_mux[3];   assign q_3_mux = q_mux[3];

   assign run_0_mux = run_mux[0];   assign run_1_mux = run_mux[1];
   assign run_2_mux = run_mux[2];   assign run_3_mux = run_mux[3];
   
   assign run_in[0] = run_0_in;   assign run_in[1] = run_1_in;
   assign run_in[2] = run_2_in;   assign run_in[3] = run_3_in;
   
   assign adc_ovf_i_0_mux = adc_ovf_i_mux[0];   assign adc_ovf_q_0_mux = adc_ovf_q_mux[0];
   assign adc_ovf_i_1_mux = adc_ovf_i_mux[1];   assign adc_ovf_q_1_mux = adc_ovf_q_mux[1];
   assign adc_ovf_i_2_mux = adc_ovf_i_mux[2];   assign adc_ovf_q_2_mux = adc_ovf_q_mux[2];
   assign adc_ovf_i_3_mux = adc_ovf_i_mux[3];   assign adc_ovf_q_3_mux = adc_ovf_q_mux[3];

  genvar i;
  generate for (i=0; i<WIDTH; i=i+1)
  begin
   always @(posedge clk)
   begin
      if(~front_sw[i])
            begin
               i_mux[i] <= i_0_in;
               q_mux[i] <= q_0_in;
               
               adc_ovf_i_mux[i] <= adc_ovf_i_0_in;
               adc_ovf_q_mux[i] <= adc_ovf_q_0_in;
            end
         else
            begin
               i_mux[i] <= i_1_in;
               q_mux[i] <= q_1_in;
                              
               adc_ovf_i_mux[i] <= adc_ovf_i_1_in;
               adc_ovf_q_mux[i] <= adc_ovf_q_1_in;
            end
         run_mux[i] <= run_in[i];
   end
  end
  endgenerate

endmodule
