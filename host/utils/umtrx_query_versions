#!/usr/bin/env python
# -*- coding: utf-8 -*-

##########################
###  Query sensors
##########################

from umtrx_property_tree import umtrx_property_tree

s = umtrx_property_tree()
s.connect()

mb_path="/mboards/0"
fpga_version = s.query_string_value(mb_path+"/fpga_version")
fw_version = s.query_string_value(mb_path+"/fw_version")

print "FPGA bitstream version: %s" % fpga_version
print "ZPU firmware version: %s" % fw_version

s.close()
