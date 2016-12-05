#!/usr/bin/python -u
# -*- coding: utf-8 -*-

import os
import sched, time

from umtrx_property_tree import umtrx_property_tree
from umtrx_vswr import umtrx_vswr

BOARD_ID = "0"
SENSORS_PATH = "/mboards/{id}/sensors".format(id=BOARD_ID)
VSWR_CALIBRATION = 0  # = TM10_VSWR_cal

HOSTNAME = os.environ['COLLECTD_HOSTNAME'] if 'COLLECTD_HOSTNAME' in os.environ  else 'localhost'
INTERVAL = os.environ['COLLECTD_INTERVAL'] if 'COLLECTD_INTERVAL' in os.environ  else '60'

umtrx = umtrx_property_tree()
umtrx.connect()

# typically this yields: ['tempA', 'tempB', 'voltagePR1', 'voltagePF1', 'voltagePR2', 'voltagePF2', 'voltagezero', 'voltageVin', 'voltageVinPA', 'voltageDCOUT']
sensors_list = umtrx.list_path_raw(SENSORS_PATH).get("result", [])


def publish():
    now = time.time()

    current_sensors = {sensor: umtrx.query_sensor_value(SENSORS_PATH + "/" + sensor) for sensor in sensors_list}

    for channel in ["1", "2"]:
        vpf_name = "voltagePF" + channel
        vpr_name = "voltagePR" + channel

        if vpf_name in current_sensors and vpr_name in current_sensors:
            vswr = umtrx_vswr(float(current_sensors[vpf_name]), float(current_sensors[vpr_name]), VSWR_CALIBRATION)
            current_sensors["VSWR" + channel] = vswr.vswr()
            current_sensors["ReturnLoss" + channel] = vswr.return_loss()

    for name, value in current_sensors.items():
        print "PUTVAL {host}/umtrx-{id}/sensor-{name} interval={interval} {now}:{value}".format(
            host=HOSTNAME, id=BOARD_ID, name=name.lower(), interval=INTERVAL, now=now, value=value)


s = sched.scheduler(time.time, time.sleep)


def timer_loop():
    s.enter(float(INTERVAL), 1, timer_loop, ())
    publish()


timer_loop()
s.run()

umtrx.close()
