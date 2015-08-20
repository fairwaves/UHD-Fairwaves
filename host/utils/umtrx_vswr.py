#!/usr/bin/env python
# -*- coding: utf-8 -*-

##########################
###  VSWR calculations
##########################

# Implemented as described at:
# http://www.markimicrowave.com/assets/data/return%20loss%20to%20vswr.pdf

import math

# TODO: requires better calibration
TM10_VSWR_cal=0.3/2

class umtrx_vswr:
  def __init__(self, VPF, VPR, calibration=0, coef=0.05):
    self.vpf = VPF
    self.vpr = VPR
    self.calibration = calibration
    self.coef = coef
    self._gamma = self._calc_gamma()

  def _calc_gamma(self):
    ''' Internal function: calculate Gamma '''
    return math.pow(10, -self.return_loss()/20.0)

  def pf(self):
    ''' Estimated through power, dBm '''
    return (self.vpf-self.calibration)/self.coef

  def pr(self):
    ''' Estimated reflected power, dBm '''
    return (self.vpr-self.calibration)/self.coef

  def return_loss(self):
    ''' Estimated return loss, dB '''
    return self.pf()-self.pr()

  def gamma(self):
    ''' Estimated Gamma '''
    return self._gamma

  def vswr(self):
    ''' Estimated VSWR '''
    gamma = self._gamma
    if gamma == 1.0:
      return float("inf")
    elif gamma > 1.0:
      return float("nan")
    else:
      return (1+gamma)/(1-gamma)

  def mismatch_loss(self):
    ''' Estimated mismatch loss, dB '''
    gamma = self._gamma
    if gamma == 1.0:
      return float("-inf")
    elif gamma > 1.0:
      return float("nan")
    else:
      return -10.0 * math.log(1.0-gamma*gamma, 10)

  def pf_rate(self):
    ''' Estimated reflected power rate, % '''
    gamma = self._gamma
    if gamma > 1.0:
      return float("nan")
    return 1.0 - gamma*gamma

  def pr_rate(self):
    ''' Estimated reflected power rate, % '''
    gamma = self._gamma
    if gamma > 1.0:
      return float("nan")
    return gamma*gamma
