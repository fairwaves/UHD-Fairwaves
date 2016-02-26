#!/usr/bin/python -u
# -*- coding: utf-8 -*-

import os
import sched,time

from umtrx_property_tree import umtrx_property_tree
from umtrx_vswr import umtrx_vswr

HOSTNAME = os.environ['COLLECTD_HOSTNAME'] if 'COLLECTD_HOSTNAME' in os.environ  else 'localhost'
INTERVAL = os.environ['COLLECTD_INTERVAL'] if 'COLLECTD_INTERVAL' in os.environ  else '60'


umtrx = umtrx_property_tree()
umtrx.connect()

sensors_path = "/mboards/0/sensors"
res = umtrx.list_path_raw(sensors_path)
# ['tempA', 'tempB', 'voltagePR1', 'voltagePF1', 'voltagePR2', 'voltagePF2', 'voltagezero', 'voltageVin', 'voltageVinPA', 'voltageDCOUT']
sensors_list = res.get('result', [])


# Voltages1     PR1:GAUGE:0:32, PR2:GAUGE:0:32, PF1:GAUGE:0:32, PF2:GAUGE:0:32
# Voltages2     Zero:GAUGE:0:32, Vin:GAUGE:0:32, VinPA:GAUGE:0:32, DCOUT:GAUGE:0:32
# Temperature   A:GAUGE:-40:120, B:GAUGE:-40:120,
# ReturnLoss    C1:GAUGE:0:32, C2:GAUGE:0:32
# VSWR          C1:GAUGE:0:32, C2:GAUGE:0:32

def publish_set(name, values):
    res = [HOSTNAME, name, INTERVAL]
    res.extend(values)
    src = "PUTVAL \"%s/UmTRX/%s\" interval=%s N:" + ":".join(["%s"] * len(values))
    print src % tuple(res)


def publish():
    sets = {'Voltages1': ['VoltagePR1', 'VoltagePR2', 'VoltagePF1', 'VoltagePF2'],
            'Voltages2': ['Voltagezero', 'VoltageVin', 'VoltageVinPA', 'VoltageDCOUT'],
            'Temperature': ['TempA', 'TempB'], 'VSWR': ['Channel_1_VSWR', 'Channel_2_VSWR'],
            'ReturnLoss': ['Channel_1_ReturnLoss', 'Channel_2_ReturnLoss']}

    current_sensors = {}

    for sensor in sensors_list:
        sensor_data = umtrx.query_sensor_raw(sensors_path + "/" + sensor)['result']
        current_sensors[sensor_data['name']] = sensor_data['value']

    # vswr_calibration = TM10_VSWR_cal
    vswr_calibration = 0

    for num in [1, 2]:
        vpr_name = 'voltagePR' + str(num)
        vpf_name = 'voltagePF' + str(num)
        if vpr_name in sensors_list and vpf_name in sensors_list:
            vpr = float(umtrx.query_sensor_value(sensors_path + '/' + vpr_name))
            vpf = float(umtrx.query_sensor_value(sensors_path + '/' + vpf_name))

            vswr = umtrx_vswr(vpf, vpr, vswr_calibration)

            current_sensors['Channel_%d_VSWR' % num] = vswr.vswr()
            current_sensors['Channel_%d_ReturnLoss' % num] = vswr.return_loss()

    for key, values in sets.iteritems():
        publish_set(key, [current_sensors[name] for name in values])


s = sched.scheduler(time.time, time.sleep)

def timer_loop():
    s.enter(float(INTERVAL), 1, timer_loop, ())
    publish()

timer_loop()
s.run()

umtrx.close()
