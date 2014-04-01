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

// Since this FIFO uses a ZBT/NoBL SRAM for its storage which is a since port 
// device it can only sustain data throughput at half the RAM clock rate.
// Fair arbitration to ensure this occurs is included in this logic and
// requests for transactions that can not be completed are held off.
// This FIFO requires a an external signal driving read_strobe that assures space for at least 6
// reads since this the theopretical maximum number in flight due to pipeling.

module nobl_fifo
  #(parameter WIDTH=18,RAM_DEPTH=19,FIFO_DEPTH=19)
    (   
	input clk,
	input rst,
	input [WIDTH-1:0] RAM_D_pi,
	output [WIDTH-1:0] RAM_D_po,
	output RAM_D_poe,
	output [RAM_DEPTH-1:0] RAM_A,
	output RAM_WEn,
	output RAM_CENn,
	output RAM_LDn,
	output RAM_OEn,
	output RAM_CE1n,
	input [WIDTH-1:0] write_data,
	input write_strobe,
	output space_avail,
	output  [WIDTH-1:0] read_data,
	input read_strobe,                    // Triggers a read, result in approximately 6 cycles.
`ifdef LMS602D_FRONTEND
	input [WIDTH-1:0] write_data_1,
	input write_strobe_1,
	output space_avail_1,
	output  [WIDTH-1:0] read_data_1,
	input read_strobe_1,                    // Triggers a read, result in approximately 6 cycles.
	output reg [FIFO_DEPTH-1:0] capacity_1,
	output  data_avail_1,                    // Qulaifys read data available this cycle on read_data_1.
`endif // !`ifdef LMS602D_FRONTEND
	output  data_avail,                    // Qulaifys read data available this cycle on read_data.
	output reg [FIFO_DEPTH-1:0] capacity
	);

   //reg [FIFO_DEPTH-1:0] capacity;
   reg [FIFO_DEPTH-1:0] wr_pointer;
   reg [FIFO_DEPTH-1:0] rd_pointer;
   wire [RAM_DEPTH-1:0] address;
   reg 			data_avail_int;  // Internal not empty flag.
   
   assign 	    write = write_strobe && space_avail;
`ifndef LMS602D_FRONTEND
   assign 	    read = read_strobe && data_avail_int;
`else
   assign 	    read_one_ch = read_strobe && data_avail_int;
   assign 	    read_dual_ch = (read_strobe && read_strobe_1) ? ((queue_rd) ? 1'b0 : read_one_ch ): read_one_ch;
   assign 	    read = (data_avail_int && data_avail_int_1) ? read_dual_ch : read_one_ch;
   
   reg [FIFO_DEPTH-1:0] wr_pointer_1;
   reg [FIFO_DEPTH-1:0] rd_pointer_1;
   
   wire id_dsp_o;   
   reg 			data_avail_int_1;  // Internal not empty flag.
   
   wire data_avail_0_1;
   assign data_avail = data_avail_0_1 && ~id_dsp_o;
   assign data_avail_1 = data_avail_0_1 && id_dsp_o;
   
   wire [WIDTH-1:0] write_data_0_1;
   
   reg queue_wr, queue_rd, space_avail_out, space_avail_1_out;
   
   assign 	    read_one_ch_1 = read_strobe_1 && data_avail_int_1;
   assign 	    read_dual_ch_1 = (read_strobe && read_strobe_1) ? ((queue_rd) ? read_one_ch_1 : 1'b0 ): read_one_ch_1;
   assign 	    read_1 = (data_avail_int && data_avail_int_1) ? read_dual_ch_1 : read_one_ch_1;
   assign 	    write_1 = write_strobe_1 && space_avail_1;
`endif // !`ifdef LMS602D_FRONTEND

   // When a read and write collision occur, supress the space_avail flag next cycle
   // and complete write followed by read over 2 cycles. This forces balanced arbitration
   // and makes for a simple logic design.

`ifndef LMS602D_FRONTEND
   always @(posedge clk)
     if (rst)
       begin
	  capacity <= (1 << FIFO_DEPTH) - 1;
	  wr_pointer <= 0;
	  rd_pointer <= 0;
	  space_avail <= 1;
	  data_avail_int <= 0;
       end
     else	  
       begin
	  // No space available if:
	  // Capacity is already zero; Capacity is 1 and write is asserted (lookahead); both read and write are asserted (collision)
	  space_avail <= ~((capacity == 0) || (read&&write) || ((capacity == 1) && write) );
	  // Capacity has 1 cycle delay so look ahead here for corner case of read of last item in FIFO.
	  data_avail_int <= ~((capacity == ((1 << FIFO_DEPTH)-1))  || ((capacity == ((1 << FIFO_DEPTH)-2)) && (~write && read))  );
	  wr_pointer <= wr_pointer + write;
	  rd_pointer <= rd_pointer + (~write && read); 
	  capacity <= capacity - write + (~write && read) ;
       end // else: !if(rst)

   assign address = write ? wr_pointer : rd_pointer;
   assign enable = write || read; 

`else
   always @(posedge clk)
     if (rst)
       begin
	  capacity <= (1 << (FIFO_DEPTH-1)) - 1;
	  wr_pointer <= 0;
	  rd_pointer <= 0;
	  space_avail_out <= 1;
	  data_avail_int <= 0;
     
	  capacity_1 <= (1 << (FIFO_DEPTH-1)) - 1;
	  wr_pointer_1 <= 0;
	  rd_pointer_1 <= 0;
	  space_avail_1_out <= 1;
	  data_avail_int_1 <= 0;
     
	  queue_wr <= 0;
	  queue_rd <= 0;
       end
     else	  
       begin
       
   // TX 0
	  // No space available if:
	  // Capacity is already zero; Capacity is 1 and write is asserted (lookahead); both read and write are asserted (collision)
	  space_avail_out <= ~((capacity == 0) || (read&&write) || ((capacity == 1) && write) );
	  // Capacity has 1 cycle delay so look ahead here for corner case of read of last item in FIFO.
	  data_avail_int <= ~((capacity == ((1 << (FIFO_DEPTH-1))-1))  || ((capacity == ((1 << (FIFO_DEPTH-1))-2)) && (~(write | write_1) && read))  );
	  wr_pointer[FIFO_DEPTH-1] <= 1'b0;
	  wr_pointer[FIFO_DEPTH-2:0] <= wr_pointer[FIFO_DEPTH-2:0] + write;
	  rd_pointer[FIFO_DEPTH-1] <= 1'b0;
	  rd_pointer[FIFO_DEPTH-2:0] <= rd_pointer[FIFO_DEPTH-2:0] + (~(write | write_1 | read_1) && read); 
	  capacity <= capacity - write + (~(write | write_1) && read) ;
   // TX 1
	  // No space available if:
	  // Capacity is already zero; Capacity is 1 and write is asserted (lookahead); both read and write are asserted (collision)
	  space_avail_1_out <= ~((capacity_1 == 0) || (read_1&&write_1) || ((capacity_1 == 1) && write_1) );
	  // Capacity has 1 cycle delay so look ahead here for corner case of read of last item in FIFO.
	  data_avail_int_1 <= ~((capacity_1 == ((1 << (FIFO_DEPTH-1))-1))  || ((capacity_1 == ((1 << (FIFO_DEPTH-1))-2)) && (~(write_1 | write) && read_1))  );
	  wr_pointer_1[FIFO_DEPTH-1] <= 1'b1;
	  wr_pointer_1[FIFO_DEPTH-2:0] <= wr_pointer_1[FIFO_DEPTH-2:0] + write_1;
	  rd_pointer_1[FIFO_DEPTH-1] <= 1'b1;
	  rd_pointer_1[FIFO_DEPTH-2:0] <= rd_pointer_1[FIFO_DEPTH-2:0] + (~(write_1 | write | read) && read_1);
	  capacity_1 <= capacity_1 - write_1 + (~(write | write_1) && read_1) ;
     
	  if (write || write_1)
	       queue_wr <= ~queue_wr;
	  if (read || read_1)
	       queue_rd <= ~queue_rd;
     
       end // else: !if(rst)

   wire space_avail_buf = (queue_wr) ? 1'b0 : space_avail_out;
   wire space_avail_1_buf = (queue_wr) ? space_avail_1_out : 1'b0; 
   
   assign space_avail = (write_strobe && write_strobe_1) ? space_avail_buf : space_avail_out; //  
   assign space_avail_1 = (write_strobe && write_strobe_1) ? space_avail_1_buf :  space_avail_1_out; // 

   assign address = (write || write_1) ? (write ? wr_pointer : wr_pointer_1) : (read ? rd_pointer : rd_pointer_1);
   assign enable = write || read || write_1 || read_1; 
   
   assign write_data_0_1 = write ? write_data : write_data_1; // Multiplexer for TX DSP data (write)
   assign read_data_1 = read_data;
`endif // !`ifdef LMS602D_FRONTEND


   //
   // Simple NoBL SRAM interface, 4 cycle read latency.
   // Read/Write arbitration via temprary application of empty/full flags.
   //
   nobl_if #(.WIDTH(WIDTH),.DEPTH(RAM_DEPTH))
     nobl_if_i1
       (
	.clk(clk),
	.rst(rst),
	.RAM_D_pi(RAM_D_pi),
	.RAM_D_po(RAM_D_po),
	.RAM_D_poe(RAM_D_poe),
	.RAM_A(RAM_A),
	.RAM_WEn(RAM_WEn),
	.RAM_CENn(RAM_CENn),
	.RAM_LDn(RAM_LDn),
	.RAM_OEn(RAM_OEn),
	.RAM_CE1n(RAM_CE1n),
	.address(address),
	.data_in(read_data),
`ifndef LMS602D_FRONTEND
	.data_out(write_data),
	.data_in_valid(data_avail),
	.write(write),
`else
	.id_dsp_o(id_dsp_o),
	.data_out(write_data_0_1),
	.data_in_valid(data_avail_0_1),
	.write(write || write_1),
`endif // !`ifdef LMS602D_FRONTEND
	.enable(enable)
	);

   

endmodule // nobl_fifo
