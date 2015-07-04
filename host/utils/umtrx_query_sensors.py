#!/usr/bin/env python
# -*- coding: utf-8 -*-

##########################
###  VSWR calculations
##########################

# Implemented as described at:
# http://www.markimicrowave.com/assets/data/return%20loss%20to%20vswr.pdf

import math

TM10_VSWR_cal=0.3

def vswr_volts_to_db(VPF, VPR, calibration=0, coef=0.05):
  return (VPF-VPR-calibration)/coef

def db_to_vswr(db):
  gamma = math.pow(10, -db/20.0)
  return (1+gamma)/(1-gamma)


##########################
###  Query sensors
##########################

from umtrx_property_tree import umtrx_property_tree

s = umtrx_property_tree()
s.connect()

sensors_path="/mboards/0/sensors"
res = s.list_path_raw(sensors_path)
sensors_list = res.get('result', [])

for sensor in sensors_list:
  print s.query_sensor_raw(sensors_path+"/"+sensor)

#vswr_calibration = TM10_VSWR_cal
vswr_calibration = 0
for num in [1, 2]:
  vpr_name = 'voltagePR'+str(num)
  vpf_name = 'voltagePF'+str(num)
  if vpr_name in sensors_list and vpf_name in sensors_list:
    vpr = float(s.query_sensor_value(sensors_path+'/'+vpr_name))
    vpf = float(s.query_sensor_value(sensors_path+'/'+vpf_name))
    vswr_db = vswr_volts_to_db(vpf, vpr, vswr_calibration)
    if vswr_db == 0.0:
      vswr = float("inf")
    else:
      vswr = db_to_vswr(vswr_db)
    print "Channel %d VSWR = %.2f = %.1f dB" % (num, vswr, vswr_db)

s.close()
