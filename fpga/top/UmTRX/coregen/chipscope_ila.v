///////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2014 Xilinx, Inc.
// All Rights Reserved
///////////////////////////////////////////////////////////////////////////////
//   ____  ____
//  /   /\/   /
// /___/  \  /    Vendor     : Xilinx
// \   \   \/     Version    : 14.4
//  \   \         Application: Xilinx CORE Generator
//  /   /         Filename   : chipscope_ila.v
// /___/   /\     Timestamp  : Wed Apr 16 09:24:08 PDT 2014
// \   \  /  \
//  \___\/\___\
//
// Design Name: Verilog Synthesis Wrapper
///////////////////////////////////////////////////////////////////////////////
// This wrapper is used to integrate with Project Navigator and PlanAhead

`timescale 1ns/1ps

module chipscope_ila(
    CONTROL,
    CLK,
    DATA,
    TRIG0) /* synthesis syn_black_box syn_noprune=1 */;


inout [35 : 0] CONTROL;
input CLK;
input [255 : 0] DATA;
input [7 : 0] TRIG0;

endmodule
