#!/usr/bin/env python
# -*- coding: utf-8 -*-

##########################
###  Property tree API
##########################

import socket
import json

class umtrx_property_tree:

  def connect(self, host="localhost", port=12345):
    self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.s.connect((host, port))
    self.f = self.s.makefile()

  def close(self):
    self.s.close()

  #
  # Helper methods
  #

  def _send_request(self, action, path, value_type=None, value=None):
    d = dict(action=action, path=path)
    if value_type is not None: d['type'] = value_type
    if value is not None: d['value'] = value
    return self.s.send((json.dumps(d)+'\n').encode('UTF-8'))

  def _recv_response(self):
    resp = self.f.readline().strip()
    if len(resp)>0:
      return json.loads(resp)
    else:
      return None

  #
  # Getters (raw)
  #

  def query_bool_raw(self, path):
    self._send_request('GET', path, value_type='BOOL')
    return self._recv_response()

  def query_int_raw(self, path):
    self._send_request('GET', path, value_type='INT')
    return self._recv_response()

  def query_double_raw(self, path):
    self._send_request('GET', path, value_type='DOUBLE')
    return self._recv_response()

  def query_sensor_raw(self, path):
    self._send_request('GET', path, value_type='SENSOR')
    return self._recv_response()

  def query_range_raw(self, path):
    self._send_request('GET', path, value_type='RANGE')
    return self._recv_response()

  def query_string_raw(self, path):
    self._send_request('GET', path, value_type='STRING')
    return self._recv_response()

  def query_complex_raw(self, path):
    self._send_request('GET', path, value_type='COMPLEX')
    return self._recv_response()

  #
  # Getters (value)
  #

  def query_bool_value(self, path):
    res = self.query_bool_raw(path)
    return res['result']

  def query_int_value(self, path):
    res = self.query_int_raw(path)
    return int(res['result'])

  def query_double_value(self, path):
    res = self.query_double_raw(path)
    return float(res['result'])

  def query_sensor_value(self, path):
    res = self.query_sensor_raw(path)
    return res['result']['value']

  def query_range_value(self, path):
    res = self.query_range_raw(path)
    return res['result']

  def query_string_value(self, path):
    res = self.query_string_raw(path)
    return res['result']

  def query_complex_value(self, path):
    res = self.query_complex_raw(path)
    i = float(res['result'][0])
    q = float(res['result'][1])
    return complex(i, q)

  #
  # Setters
  #

  def set_bool(self, path, val):
    self._send_request('SET', path, value_type='BOOL', value=val)
    return self._recv_response()

  def set_int(self, path, val):
    self._send_request('SET', path, value_type='INT', value=val)
    return self._recv_response()

  def set_double(self, path, val):
    self._send_request('SET', path, value_type='DOUBLE', value=val)
    return self._recv_response()

  def set_string(self, path, val):
    self._send_request('SET', path, value_type='STRING', value=val)
    return self._recv_response()

  def set_complex(self, path, val):
    if type(val) is complex:
      # Convert complex to an array
      val = [val.real, val.imag]
    self._send_request('SET', path, value_type='COMPLEX', value=val)
    return self._recv_response()

  #
  # Check path presence and list paths
  #

  def has_path_raw(self, path):
    self._send_request('HAS', path)
    return self._recv_response()

  def list_path_raw(self, path):
    self._send_request('LIST', path)
    return self._recv_response()
