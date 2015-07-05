#!/usr/bin/env python
# -*- coding: utf-8 -*-

##########################
###  Query sensors
##########################

from umtrx_property_tree import umtrx_property_tree
from umtrx_vswr import umtrx_vswr

s = umtrx_property_tree()
s.connect()

sensors_path="/mboards/0/sensors"
res = s.list_path_raw(sensors_path)
sensors_list = res.get('result', [])

print "Sensors:"
for sensor in sensors_list:
  reply = s.query_sensor_raw(sensors_path+"/"+sensor)
  if reply.has_key('result'):
    res = reply['result']
    print "  %15s = %9s %s" % (res['name'], res['value'], res['unit'])
  else:
    print "Can't read sensor %s" % sensor

#vswr_calibration = TM10_VSWR_cal
#vswr_calibration = TM3_VSWR_cal
vswr_calibration = 0
for num in [1, 2]:
  vpr_name = 'voltagePR'+str(num)
  vpf_name = 'voltagePF'+str(num)
  if vpr_name in sensors_list and vpf_name in sensors_list:
    vpr = float(s.query_sensor_value(sensors_path+'/'+vpr_name))
    vpf = float(s.query_sensor_value(sensors_path+'/'+vpf_name))
    vswr = umtrx_vswr(vpf, vpr, vswr_calibration)
    print "TRX %d power detector:" % num
    print "              VPF =  %5.2f  V"   % vpf
    print "               PF = %5.1f   dBm" % vswr.pf()
    print "              VPR =  %5.2f  V"   % vpr
    print "               PR = %5.1f   dBm" % vswr.pr()
    print "             VSWR = %6.2f"       % vswr.vswr()
    print "            Gamma =   %5.3f"     % vswr.gamma()
    print "      Return Loss = %5.1f   dB"  % vswr.return_loss()
    print "    Mismatch Loss =   %5.3f dB"  % vswr.mismatch_loss()
    print "    Through power =  %5.2f  %%"  % (100.0*vswr.pf_rate())
    print "  Reflected power =  %5.2f  %%"  % (100.0*vswr.pr_rate())

s.close()
