import SoapySDR
from SoapySDR import *
import math
import numpy
import matplotlib.pyplot as plt
from scipy import signal
import random

SAMP_RATE = 13e6/4

def calcAvgPs(umtrx, rxStream, fftSize = 4096, numFFT = 50, numSkips=2):
    avgFFT = numpy.array([0]*fftSize, numpy.complex64)
    umtrx.activateStream(rxStream, SOAPY_SDR_END_BURST, 0, fftSize*(numFFT+numSkips))

    for i in range(numFFT+numSkips):
        samps = numpy.array([0]*fftSize, numpy.complex64)
        assert(umtrx.readStream(rxStream, [samps], fftSize).ret == fftSize)
        if i < numSkips: continue #skip first for transients
        samps *= numpy.blackman(fftSize)
        avgFFT += numpy.fft.fft(samps)
    avgFFT /= numFFT
    assert(len(avgFFT) == fftSize)

    ps = 10*numpy.log10(numpy.abs(avgFFT)) - 20*math.log10(fftSize)
    freqs = numpy.fft.fftfreq(fftSize, 1.0/SAMP_RATE)
    return ps, freqs

def measureToneFromPs(ps_freqs, toneFreq, BW=SAMP_RATE/25):
    ps, freqs = ps_freqs
    tonePower = None
    for idx in range(len(ps)):
        if freqs[idx] > toneFreq-BW/2 and freqs[idx] < toneFreq+BW/2:
            if tonePower is None: tonePower = ps[idx]
            tonePower = max(ps[idx], tonePower)
    return tonePower

umtrx = SoapySDR.Device(dict(driver='uhd', type='umtrx'))

#frontend map selects side A on ch0
umtrx.setFrontendMapping(SOAPY_SDR_RX, "A:0")
umtrx.setFrontendMapping(SOAPY_SDR_TX, "A:0")

#cal antennas for loopback
umtrx.setAntenna(SOAPY_SDR_RX, 0, "CAL")
umtrx.setAntenna(SOAPY_SDR_TX, 0, "CAL")

#set a low sample rate
umtrx.setSampleRate(SOAPY_SDR_RX, 0, SAMP_RATE)
umtrx.setSampleRate(SOAPY_SDR_TX, 0, SAMP_RATE)

#tune rx with offset so we can see tx DC
umtrx.setFrequency(SOAPY_SDR_TX, 0, 806e6)
umtrx.setFrequency(SOAPY_SDR_RX, 0, 806.3e6)

rxStream = umtrx.setupStream(SOAPY_SDR_RX, "CF32")

if True:

    best_correction = 0.0
    best_dc_power = None

    for bound in (0.2, 0.1, 0.05, 0.01):
        this_correction = best_correction

        for searchNo in range(int(-50*math.log10(bound))):

            #print 'searchNo',searchNo

            corr_i = random.uniform(this_correction.real-bound, this_correction.real+bound)
            corr_q = random.uniform(this_correction.imag-bound, this_correction.imag+bound)

            correction = complex(min(max(corr_i, -1), 1), min(max(corr_q, -1), 1))

            umtrx.setDCOffset(SOAPY_SDR_TX, 0, correction)

            ps, freqs = calcAvgPs(umtrx, rxStream)
            dc_power = measureToneFromPs((ps, freqs), -0.3e6)

            if best_dc_power is None or best_dc_power > dc_power:
                best_dc_power = dc_power
                best_correction = correction

        print 'search says best_dc_power', best_dc_power, ' best_correction', best_correction

    #prove that its really the best...
    umtrx.setDCOffset(SOAPY_SDR_TX, 0, best_correction)
    for i in range(5):
        ps, freqs = calcAvgPs(umtrx, rxStream)
        dc_power = measureToneFromPs((ps, freqs), -0.3e6)
        print dc_power


    #idx = numpy.argsort(freqs)
    #plt.plot(freqs[idx]/1e6, ps[idx])
    #plt.show()

umtrx.closeStream(rxStream)
