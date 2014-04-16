-------------------------------------------------------------------------------
-- Copyright (c) 2014 Xilinx, Inc.
-- All Rights Reserved
-------------------------------------------------------------------------------
--   ____  ____
--  /   /\/   /
-- /___/  \  /    Vendor     : Xilinx
-- \   \   \/     Version    : 14.4
--  \   \         Application: Xilinx CORE Generator
--  /   /         Filename   : chipscope_ila.vhd
-- /___/   /\     Timestamp  : Wed Apr 16 09:22:06 PDT 2014
-- \   \  /  \
--  \___\/\___\
--
-- Design Name: XST Instantiation template
-------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.std_logic_1164.all;

LIBRARY chipscope_ila_v1_05_a;

ENTITY chipscope_ila IS
  PORT (
    CONTROL : INOUT STD_LOGIC_VECTOR(35 DOWNTO 0);
    CLK : IN STD_LOGIC;
    DATA : IN STD_LOGIC_VECTOR(255 DOWNTO 0);
    TRIG0 : IN STD_LOGIC_VECTOR(7 DOWNTO 0)
  );
END chipscope_ila;

ARCHITECTURE spartan6 OF chipscope_ila IS

  ATTRIBUTE X_CORE_INFO : STRING;
  ATTRIBUTE X_CORE_INFO OF spartan6 : ARCHITECTURE IS "chipscope_ila_v1_05_a, Xilinx CORE Generator 14.4";

  ATTRIBUTE CHECK_LICENSE_TYPE : STRING;
  ATTRIBUTE CHECK_LICENSE_TYPE OF spartan6 : ARCHITECTURE IS "chipscope_ila,chipscope_ila_v1_05_a,NONE,NONE";

  ATTRIBUTE CORE_GENERATION_INFO : STRING;
  ATTRIBUTE CORE_GENERATION_INFO OF spartan6 : ARCHITECTURE IS "chipscope_ila,chipscope_ila_v1_05_a,{c_xco_list=Component_Name=chipscope_ila;Number_Of_Trigger_Ports=1;Max_Sequence_Levels=1;Use_RPMs=false;Enable_Trigger_Output_Port=false;Sample_On=Rising;Sample_Data_Depth=2048;Enable_Storage_Qualification=true;Data_Same_As_Trigger=false;Data_Port_Width=256;Trigger_Port_Width_1=8;Match_Units_1=1;Counter_Width_1=Disabled;Match_Type_1=basic_with_edges;Exclude_From_Data_Storage_1=true;Trigger_Port_Width_2=8;Match_Units_2=1;Counter_Width_2=Disabled;Match_Type_2=basic_with_edges;Exclude_From_Data_Storage_2=true;Trigger_Port_Width_3=8;Match_Units_3=1;Counter_Width_3=Disabled;Match_Type_3=basic_with_edges;Exclude_From_Data_Storage_3=true;Trigger_Port_Width_4=8;Match_Units_4=1;Counter_Width_4=Disabled;Match_Type_4=basic_with_edges;Exclude_From_Data_Storage_4=true;Trigger_Port_Width_5=8;Match_Units_5=1;Counter_Width_5=Disabled;Match_Type_5=basic_with_edges;Exclude_From_Data_Storage_5=true;Trigger_Port_Width_6=8;Match_Units_6=1;Counter_Width_6=Disabled;Match_Type_6=basic_with_edges;Exclude_From_Data_Storage_6=true;Trigger_Port_Width_7=8;Match_Units_7=1;Counter_Width_7=Disabled;Match_Type_7=basic_with_edges;Exclude_From_Data_Storage_7=true;Trigger_Port_Width_8=8;Match_Units_8=1;Counter_Width_8=Disabled;Match_Type_8=basic_with_edges;Exclude_From_Data_Storage_8=true;Trigger_Port_Width_9=8;Match_Units_9=1;Counter_Width_9=Disabled;Match_Type_9=basic_with_edges;Exclude_From_Data_Storage_9=true;Trigger_Port_Width_10=8;Match_Units_10=1;Counter_Width_10=Disabled;Match_Type_10=basic_with_edges;Exclude_From_Data_Storage_10=true;Trigger_Port_Width_11=8;Match_Units_11=1;Counter_Width_11=Disabled;Match_Type_11=basic_with_edges;Exclude_From_Data_Storage_11=true;Trigger_Port_Width_12=8;Match_Units_12=1;Counter_Width_12=Disabled;Match_Type_12=basic_with_edges;Exclude_From_Data_Storage_12=true;Trigger_Port_Width_13=8;Match_Units_13=1;Counter_Width_13=Disabled;Match_Type_13=basic_with_edges;Exclude_From_Data_Storage_13=true;Trigger_Port_Width_14=8;Match_Units_14=1;Counter_Width_14=Disabled;Match_Type_14=basic_with_edges;Exclude_From_Data_Storage_14=true;Trigger_Port_Width_15=8;Match_Units_15=1;Counter_Width_15=Disabled;Match_Type_15=basic_with_edges;Exclude_From_Data_Storage_15=true;Trigger_Port_Width_16=8;Match_Units_16=1;Counter_Width_16=Disabled;Match_Type_16=basic_with_edges;Exclude_From_Data_Storage_16=true,c_xdevicefamily=spartan6,c_core_type=2,c_mfg_id=1,c_major_version=14,c_minor_version=4,c_build_revision=0,c_core_major_ver=1,c_core_minor_ver=4,c_core_minor_alpha_ver=97,c_ram_type=1,c_srl16_type=2,c_use_inv_clk=0,c_use_rpm=0,c_use_trig_out=0,c_use_atc_clkin=0,c_use_data=1,c_use_trig0=1,c_use_trig1=0,c_use_trig2=0,c_use_trig3=0,c_use_trig4=0,c_use_trig5=0,c_use_trig6=0,c_use_trig7=0,c_use_trig8=0,c_use_trig9=0,c_use_trig10=0,c_use_trig11=0,c_use_trig12=0,c_use_trig13=0,c_use_trig14=0,c_use_trig15=0,c_use_trigdata0=0,c_use_trigdata1=0,c_use_trigdata2=0,c_use_trigdata3=0,c_use_trigdata4=0,c_use_trigdata5=0,c_use_trigdata6=0,c_use_trigdata7=0,c_use_trigdata8=0,c_use_trigdata9=0,c_use_trigdata10=0,c_use_trigdata11=0,c_use_trigdata12=0,c_use_trigdata13=0,c_use_trigdata14=0,c_use_trigdata15=0,c_data_width=256,c_data_depth=2048,c_use_gap=0,c_timestamp_type=0,c_timestamp_width=32,c_timestamp_depth=512,c_use_storage_qual=1,c_tseq_type=0,c_num_tseq_cnt=0,c_tseq_cnt1_width=1,c_tseq_cnt0_width=1,c_num_tseq_states=1,c_use_tc_mcnt=0,c_tc_mcnt_width=1,c_ext_cap_rate_mode=0,c_ext_cap_pin_mode=0,c_num_ext_cap_pins=8,c_ext_cap_use_reg=1,c_num_match_units=1,c_trig0_width=8,c_trig1_width=1,c_trig2_width=1,c_trig3_width=1,c_trig4_width=1,c_trig5_width=1,c_trig6_width=1,c_trig7_width=1,c_trig8_width=1,c_trig9_width=1,c_trig10_width=1,c_trig11_width=1,c_trig12_width=1,c_trig13_width=1,c_trig14_width=1,c_trig15_width=1,c_m0_tpid=0,c_m1_tpid=1,c_m2_tpid=2,c_m3_tpid=3,c_m4_tpid=4,c_m5_tpid=5,c_m6_tpid=6,c_m7_tpid=7,c_m8_tpid=8,c_m9_tpid=9,c_m10_tpid=10,c_m11_tpid=11,c_m12_tpid=12,c_m13_tpid=13,c_m14_tpid=14,c_m15_tpid=15,c_m0_type=1,c_m1_type=0,c_m2_type=0,c_m3_type=0,c_m4_type=0,c_m5_type=0,c_m6_type=0,c_m7_type=0,c_m8_type=0,c_m9_type=0,c_m10_type=0,c_m11_type=0,c_m12_type=0,c_m13_type=0,c_m14_type=0,c_m15_type=0,c_use_mcnt0=0,c_use_mcnt1=0,c_use_mcnt2=0,c_use_mcnt3=0,c_use_mcnt4=0,c_use_mcnt5=0,c_use_mcnt6=0,c_use_mcnt7=0,c_use_mcnt8=0,c_use_mcnt9=0,c_use_mcnt10=0,c_use_mcnt11=0,c_use_mcnt12=0,c_use_mcnt13=0,c_use_mcnt14=0,c_use_mcnt15=0,c_mcnt0_width=1,c_mcnt1_width=1,c_mcnt2_width=1,c_mcnt3_width=1,c_mcnt4_width=1,c_mcnt5_width=1,c_mcnt6_width=1,c_mcnt7_width=1,c_mcnt8_width=1,c_mcnt9_width=1,c_mcnt10_width=1,c_mcnt11_width=1,c_mcnt12_width=1,c_mcnt13_width=1,c_mcnt14_width=1,c_mcnt15_width=1,c_constraint_type=external,c_example_design=false,}";


BEGIN
  U0: ENTITY chipscope_ila_v1_05_a.chipscope_ila
  GENERIC MAP (
    c_xco_list => "Component_Name=chipscope_ila;Number_Of_Trigger_Ports=1;Max_Sequence_Levels=1;Use_RPMs=false;Enable_Trigger_Output_Port=false;Sample_On=Rising;Sample_Data_Depth=2048;Enable_Storage_Qualification=true;Data_Same_As_Trigger=false;Data_Port_Width=256;Trigger_Port_Width_1=8;Match_Units_1=1;Counter_Width_1=Disabled;Match_Type_1=basic_with_edges;Exclude_From_Data_Storage_1=true;Trigger_Port_Width_2=8;Match_Units_2=1;Counter_Width_2=Disabled;Match_Type_2=basic_with_edges;Exclude_From_Data_Storage_2=true;Trigger_Port_Width_3=8;Match_Units_3=1;Counter_Width_3=Disabled;Match_Type_3=basic_with_edges;Exclude_From_Data_Storage_3=true;Trigger_Port_Width_4=8;Match_Units_4=1;Counter_Width_4=Disabled;Match_Type_4=basic_with_edges;Exclude_From_Data_Storage_4=true;Trigger_Port_Width_5=8;Match_Units_5=1;Counter_Width_5=Disabled;Match_Type_5=basic_with_edges;Exclude_From_Data_Storage_5=true;Trigger_Port_Width_6=8;Match_Units_6=1;Counter_Width_6=Disabled;Match_Type_6=basic_with_edges;Exclude_From_Data_Storage_6=true;Trigger_Port_Width_7=8;Match_Units_7=1;Counter_Width_7=Disabled;Match_Type_7=basic_with_edges;Exclude_From_Data_Storage_7=true;Trigger_Port_Width_8=8;Match_Units_8=1;Counter_Width_8=Disabled;Match_Type_8=basic_with_edges;Exclude_From_Data_Storage_8=true;Trigger_Port_Width_9=8;Match_Units_9=1;Counter_Width_9=Disabled;Match_Type_9=basic_with_edges;Exclude_From_Data_Storage_9=true;Trigger_Port_Width_10=8;Match_Units_10=1;Counter_Width_10=Disabled;Match_Type_10=basic_with_edges;Exclude_From_Data_Storage_10=true;Trigger_Port_Width_11=8;Match_Units_11=1;Counter_Width_11=Disabled;Match_Type_11=basic_with_edges;Exclude_From_Data_Storage_11=true;Trigger_Port_Width_12=8;Match_Units_12=1;Counter_Width_12=Disabled;Match_Type_12=basic_with_edges;Exclude_From_Data_Storage_12=true;Trigger_Port_Width_13=8;Match_Units_13=1;Counter_Width_13=Disabled;Match_Type_13=basic_with_edges;Exclude_From_Data_Storage_13=true;Trigger_Port_Width_14=8;Match_Units_14=1;Counter_Width_14=Disabled;Match_Type_14=basic_with_edges;Exclude_From_Data_Storage_14=true;Trigger_Port_Width_15=8;Match_Units_15=1;Counter_Width_15=Disabled;Match_Type_15=basic_with_edges;Exclude_From_Data_Storage_15=true;Trigger_Port_Width_16=8;Match_Units_16=1;Counter_Width_16=Disabled;Match_Type_16=basic_with_edges;Exclude_From_Data_Storage_16=true",
    c_xdevicefamily => "spartan6",
    c_core_type => 2,
    c_mfg_id => 1,
    c_major_version => 14,
    c_minor_version => 4,
    c_build_revision => 0,
    c_core_major_ver => 1,
    c_core_minor_ver => 4,
    c_core_minor_alpha_ver => 97,
    c_ram_type => 1,
    c_srl16_type => 2,
    c_use_inv_clk => 0,
    c_use_rpm => 0,
    c_use_trig_out => 0,
    c_use_atc_clkin => 0,
    c_use_data => 1,
    c_use_trig0 => 1,
    c_use_trig1 => 0,
    c_use_trig2 => 0,
    c_use_trig3 => 0,
    c_use_trig4 => 0,
    c_use_trig5 => 0,
    c_use_trig6 => 0,
    c_use_trig7 => 0,
    c_use_trig8 => 0,
    c_use_trig9 => 0,
    c_use_trig10 => 0,
    c_use_trig11 => 0,
    c_use_trig12 => 0,
    c_use_trig13 => 0,
    c_use_trig14 => 0,
    c_use_trig15 => 0,
    c_use_trigdata0 => 0,
    c_use_trigdata1 => 0,
    c_use_trigdata2 => 0,
    c_use_trigdata3 => 0,
    c_use_trigdata4 => 0,
    c_use_trigdata5 => 0,
    c_use_trigdata6 => 0,
    c_use_trigdata7 => 0,
    c_use_trigdata8 => 0,
    c_use_trigdata9 => 0,
    c_use_trigdata10 => 0,
    c_use_trigdata11 => 0,
    c_use_trigdata12 => 0,
    c_use_trigdata13 => 0,
    c_use_trigdata14 => 0,
    c_use_trigdata15 => 0,
    c_data_width => 256,
    c_data_depth => 2048,
    c_use_gap => 0,
    c_timestamp_type => 0,
    c_timestamp_width => 32,
    c_timestamp_depth => 512,
    c_use_storage_qual => 1,
    c_tseq_type => 0,
    c_num_tseq_cnt => 0,
    c_tseq_cnt1_width => 1,
    c_tseq_cnt0_width => 1,
    c_num_tseq_states => 1,
    c_use_tc_mcnt => 0,
    c_tc_mcnt_width => 1,
    c_ext_cap_rate_mode => 0,
    c_ext_cap_pin_mode => 0,
    c_num_ext_cap_pins => 8,
    c_ext_cap_use_reg => 1,
    c_num_match_units => 1,
    c_trig0_width => 8,
    c_trig1_width => 1,
    c_trig2_width => 1,
    c_trig3_width => 1,
    c_trig4_width => 1,
    c_trig5_width => 1,
    c_trig6_width => 1,
    c_trig7_width => 1,
    c_trig8_width => 1,
    c_trig9_width => 1,
    c_trig10_width => 1,
    c_trig11_width => 1,
    c_trig12_width => 1,
    c_trig13_width => 1,
    c_trig14_width => 1,
    c_trig15_width => 1,
    c_m0_tpid => 0,
    c_m1_tpid => 1,
    c_m2_tpid => 2,
    c_m3_tpid => 3,
    c_m4_tpid => 4,
    c_m5_tpid => 5,
    c_m6_tpid => 6,
    c_m7_tpid => 7,
    c_m8_tpid => 8,
    c_m9_tpid => 9,
    c_m10_tpid => 10,
    c_m11_tpid => 11,
    c_m12_tpid => 12,
    c_m13_tpid => 13,
    c_m14_tpid => 14,
    c_m15_tpid => 15,
    c_m0_type => 1,
    c_m1_type => 0,
    c_m2_type => 0,
    c_m3_type => 0,
    c_m4_type => 0,
    c_m5_type => 0,
    c_m6_type => 0,
    c_m7_type => 0,
    c_m8_type => 0,
    c_m9_type => 0,
    c_m10_type => 0,
    c_m11_type => 0,
    c_m12_type => 0,
    c_m13_type => 0,
    c_m14_type => 0,
    c_m15_type => 0,
    c_use_mcnt0 => 0,
    c_use_mcnt1 => 0,
    c_use_mcnt2 => 0,
    c_use_mcnt3 => 0,
    c_use_mcnt4 => 0,
    c_use_mcnt5 => 0,
    c_use_mcnt6 => 0,
    c_use_mcnt7 => 0,
    c_use_mcnt8 => 0,
    c_use_mcnt9 => 0,
    c_use_mcnt10 => 0,
    c_use_mcnt11 => 0,
    c_use_mcnt12 => 0,
    c_use_mcnt13 => 0,
    c_use_mcnt14 => 0,
    c_use_mcnt15 => 0,
    c_mcnt0_width => 1,
    c_mcnt1_width => 1,
    c_mcnt2_width => 1,
    c_mcnt3_width => 1,
    c_mcnt4_width => 1,
    c_mcnt5_width => 1,
    c_mcnt6_width => 1,
    c_mcnt7_width => 1,
    c_mcnt8_width => 1,
    c_mcnt9_width => 1,
    c_mcnt10_width => 1,
    c_mcnt11_width => 1,
    c_mcnt12_width => 1,
    c_mcnt13_width => 1,
    c_mcnt14_width => 1,
    c_mcnt15_width => 1,
    c_constraint_type => "external",
    c_example_design => false
  )
  PORT MAP (
    CONTROL => CONTROL,
    CLK => CLK,
    DATA => DATA,
    TRIG0 => TRIG0
  );
END spartan6;
