-------------------------------------------------------------------------------
-- Copyright (c) 2014 Xilinx, Inc.
-- All Rights Reserved
-------------------------------------------------------------------------------
--   ____  ____
--  /   /\/   /
-- /___/  \  /    Vendor     : Xilinx
-- \   \   \/     Version    : 14.4
--  \   \         Application: Xilinx CORE Generator
--  /   /         Filename   : chipscope_icon.vhd
-- /___/   /\     Timestamp  : Wed Apr 16 09:06:54 PDT 2014
-- \   \  /  \
--  \___\/\___\
--
-- Design Name: XST Instantiation template
-------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.std_logic_1164.all;

LIBRARY chipscope_icon_v1_06_a;

ENTITY chipscope_icon IS
  PORT (
    CONTROL0 : INOUT STD_LOGIC_VECTOR(35 DOWNTO 0)
  );
END chipscope_icon;

ARCHITECTURE spartan6 OF chipscope_icon IS

  ATTRIBUTE X_CORE_INFO : STRING;
  ATTRIBUTE X_CORE_INFO OF spartan6 : ARCHITECTURE IS "chipscope_icon_v1_06_a, Xilinx CORE Generator 14.4";

  ATTRIBUTE CHECK_LICENSE_TYPE : STRING;
  ATTRIBUTE CHECK_LICENSE_TYPE OF spartan6 : ARCHITECTURE IS "chipscope_icon,chipscope_icon_v1_06_a,NONE,NONE";

  ATTRIBUTE CORE_GENERATION_INFO : STRING;
  ATTRIBUTE CORE_GENERATION_INFO OF spartan6 : ARCHITECTURE IS "chipscope_icon,chipscope_icon_v1_06_a,{c_use_new_parser=0,c_xco_list=Number_Control_Ports=1;Use_Ext_Bscan=false;User_Scan_Chain=USER1;Enable_Jtag_Bufg=true;Use_Unused_Bscan=false;Use_Softbscan=false,c_xdevicefamily=spartan6,c_core_type=1,c_major_version=14,c_minor_version=4,c_build_revision=0,c_core_major_ver=1,c_core_minor_ver=2,c_core_minor_alpha_ver=97,c_mfg_id=1,c_use_bufr=0,c_use_sim=0,c_use_softbscan=0,c_part_idcode_register=0,c_use_jtag_bufg=1,c_num_control_ports=1,c_use_control0=1,c_use_control1=0,c_use_control2=0,c_use_control3=0,c_use_control4=0,c_use_control5=0,c_use_control6=0,c_use_control7=0,c_use_control8=0,c_use_control9=0,c_use_control10=0,c_use_control11=0,c_use_control12=0,c_use_control13=0,c_use_control14=0,c_use_ext_bscan=0,c_use_unused_bscan=0,c_use_xst_tck_workaround=1,c_example_design=false,c_constraint_type=external,c_user_scan_chain=1,}";


BEGIN
  U0: ENTITY chipscope_icon_v1_06_a.chipscope_icon
  GENERIC MAP (
    c_use_new_parser => 0,
    c_xco_list => "Number_Control_Ports=1;Use_Ext_Bscan=false;User_Scan_Chain=USER1;Enable_Jtag_Bufg=true;Use_Unused_Bscan=false;Use_Softbscan=false",
    c_xdevicefamily => "spartan6",
    c_core_type => 1,
    c_major_version => 14,
    c_minor_version => 4,
    c_build_revision => 0,
    c_core_major_ver => 1,
    c_core_minor_ver => 2,
    c_core_minor_alpha_ver => 97,
    c_mfg_id => 1,
    c_use_bufr => 0,
    c_use_sim => 0,
    c_use_softbscan => 0,
    c_part_idcode_register => 0,
    c_use_jtag_bufg => 1,
    c_num_control_ports => 1,
    c_use_control0 => 1,
    c_use_control1 => 0,
    c_use_control2 => 0,
    c_use_control3 => 0,
    c_use_control4 => 0,
    c_use_control5 => 0,
    c_use_control6 => 0,
    c_use_control7 => 0,
    c_use_control8 => 0,
    c_use_control9 => 0,
    c_use_control10 => 0,
    c_use_control11 => 0,
    c_use_control12 => 0,
    c_use_control13 => 0,
    c_use_control14 => 0,
    c_use_ext_bscan => 0,
    c_use_unused_bscan => 0,
    c_use_xst_tck_workaround => 1,
    c_example_design => false,
    c_constraint_type => "external",
    c_user_scan_chain => 1
  )
  PORT MAP (
    CONTROL0 => CONTROL0
  );
END spartan6;
