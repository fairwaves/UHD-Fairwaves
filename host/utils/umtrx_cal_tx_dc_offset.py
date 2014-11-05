import SoapySDR
from SoapySDR import *
import math
import numpy
import matplotlib.pyplot as plt
from scipy import signal
import random
import time
import os

SAMP_RATE = 13e6/4
FREQ_OFFSET = 0.3e6
FREQ_START = 700e6
FREQ_STOP = 1200e6
FREQ_CORRECTION_STEP = 7e6
FREQ_VALIDATION_STEP = 2e6
NUM_AVG_POINTS = 5

import numpy as np
def find_nearest(array,value):
    idx = (np.abs(array-value)).argmin()
    return array[idx]

def calcAvgPs(umtrx, rxStream, fftSize = 4096, numFFT = 10, numSkips=2):
    samps = numpy.array([0]*fftSize, numpy.complex64)
    avgFFT = numpy.array([0]*fftSize, numpy.complex64)

    while umtrx.readStream(rxStream, [samps], fftSize, 0, 1000).ret != SOAPY_SDR_TIMEOUT: pass #read until timeout
    umtrx.activateStream(rxStream, SOAPY_SDR_END_BURST, 0, fftSize*(numFFT+numSkips))

    numActualFFTs = 0
    for i in range(numFFT+numSkips):
        if umtrx.readStream(rxStream, [samps], fftSize).ret != fftSize:
            print 'D'
            return calcAvgPs(umtrx, rxStream, fftSize, numFFT, numSkips)
        if i < numSkips: continue #skip first for transients
        samps *= signal.flattop(fftSize)
        avgFFT += numpy.fft.fft(samps)
        numActualFFTs += 1
    avgFFT /= numActualFFTs
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

def validateWithMeasurements(umtrx, rxStream, toneFreq, numAvgPts=5):
    powers = list()
    for i in range(NUM_AVG_POINTS):
        ps, freqs = calcAvgPs(umtrx, rxStream)
        powers.append(measureToneFromPs((ps, freqs), toneFreq))
    return 10*numpy.log(numpy.average(numpy.exp(numpy.array(powers)/10))), numpy.std(powers)

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

rxStream = umtrx.setupStream(SOAPY_SDR_RX, "CF32")

#calibrate out dc offset at select frequencies
best_correction_per_freq = dict()
best_dc_power_per_freq = dict()
initial_dc_power_per_freq = dict()
stddev_dc_power_per_freq = dict()
average_dc_power_per_freq = dict()

for freq in numpy.arange(FREQ_START, FREQ_STOP, FREQ_CORRECTION_STEP):

    print 'Doing freq:', freq/1e6, 'MHz'

    #tune rx with offset so we can see tx DC
    umtrx.setFrequency(SOAPY_SDR_TX, 0, freq)
    umtrx.setFrequency(SOAPY_SDR_RX, 0, freq + FREQ_OFFSET)
    umtrx.getHardwareTime() #readback so commands are processed

    best_correction = 0.0
    best_dc_power = None

    #grab values before correction
    umtrx.setDCOffset(SOAPY_SDR_TX, 0, best_correction)
    averagePowers, stddevPowers = validateWithMeasurements(umtrx, rxStream, -FREQ_OFFSET)
    initial_dc_power_per_freq[freq] = averagePowers

    for bound in (0.1, 0.05, 0.01):
        this_correction = best_correction

        for searchNo in range(50):

            #print 'searchNo',searchNo

            corr_i = random.uniform(this_correction.real-bound, this_correction.real+bound)
            corr_q = random.uniform(this_correction.imag-bound, this_correction.imag+bound)

            correction = complex(min(max(corr_i, -1), 1), min(max(corr_q, -1), 1))

            umtrx.setDCOffset(SOAPY_SDR_TX, 0, correction)

            ps, freqs = calcAvgPs(umtrx, rxStream)
            dc_power = measureToneFromPs((ps, freqs), -FREQ_OFFSET)

            if best_dc_power is None or best_dc_power > dc_power:
                best_dc_power = dc_power
                best_correction = correction

    print 'best_dc_power', best_dc_power, ' best_correction', best_correction

    #prove that its really the best...
    umtrx.setDCOffset(SOAPY_SDR_TX, 0, best_correction)
    averagePowers, stddevPowers = validateWithMeasurements(umtrx, rxStream, -FREQ_OFFSET)
    print 'averagePowers', averagePowers
    print 'stddevPowers', stddevPowers

    best_correction_per_freq[freq] = best_correction
    best_dc_power_per_freq[freq] = best_dc_power
    stddev_dc_power_per_freq[freq] = stddevPowers
    average_dc_power_per_freq[freq] = averagePowers

########################################################################
## produce tx cal serial format
########################################################################
serial = umtrx.getHardwareInfo()['tx0_serial']
cal_dest = os.path.join(os.path.expanduser("~"), '.uhd', 'cal', "tx_dc_cal_v0.2_%s.csv"%serial)
print 'cal_dest', cal_dest
cal_data = open(cal_dest, 'w')

cal_data.write("name, TX Frontend Calibration\n")
cal_data.write("serial, %s\n"%serial)
cal_data.write("timestamp, %d\n"%int(time.time()))
cal_data.write("version, 0, 1\n")
cal_data.write("DATA STARTS HERE\n")
cal_data.write("lo_frequency, correction_real, correction_imag, measured, delta\n")
for freq in sorted(best_correction_per_freq.keys()):
    cal_data.write(', '.join(map(str, [
        freq,
        best_correction_per_freq[freq].real,
        best_correction_per_freq[freq].imag,
        best_dc_power_per_freq[freq],
        initial_dc_power_per_freq[freq]-best_dc_power_per_freq[freq],
    ])) + '\n')

"""
#recollect dc offsets with corrections applied:
validation_average_dc_offsets_per_freq = dict()
validation_stddev_dc_offsets_per_freq = dict()
original_dc_power_per_freq = dict()

for freq in numpy.arange(FREQ_START, FREQ_STOP, FREQ_VALIDATION_STEP):

    #tune rx with offset so we can see tx DC
    umtrx.setFrequency(SOAPY_SDR_TX, 0, freq)
    umtrx.setFrequency(SOAPY_SDR_RX, 0, freq + FREQ_OFFSET)
    umtrx.getHardwareTime() #readback so commands are processed

    umtrx.setDCOffset(SOAPY_SDR_TX, 0, 0.0)

    #get the original value before corrections
    averagePowers, stddevPowers = validateWithMeasurements(umtrx, rxStream, -FREQ_OFFSET)
    original_dc_power_per_freq[freq] = averagePowers

    #pick the correction to use (closest in freq)
    correction_freq = find_nearest(best_correction_per_freq.keys(), freq)
    correction = best_correction_per_freq[correction_freq]

    #perform the measurements again
    umtrx.setDCOffset(SOAPY_SDR_TX, 0, correction)
    averagePowers, stddevPowers = validateWithMeasurements(umtrx, rxStream, -FREQ_OFFSET)

    validation_average_dc_offsets_per_freq[freq] = averagePowers
    validation_stddev_dc_offsets_per_freq[freq] = stddevPowers

umtrx.closeStream(rxStream)

plt.figure(1)

plt.subplot(311)
freqs = sorted(average_dc_power_per_freq.keys())
vals = [average_dc_power_per_freq[f] for f in freqs]
cor_data = plt.plot(numpy.array(freqs)/1e6, vals, label='Correction')
freqs = sorted(validation_average_dc_offsets_per_freq.keys())
vals = [validation_average_dc_offsets_per_freq[f] for f in freqs]
val_data = plt.plot(numpy.array(freqs)/1e6, vals, label='Validation')
legend = plt.legend(loc='upper right', shadow=True)
plt.title("Freq (MHz) vs average power (dB)")
plt.grid(True)

plt.subplot(312)
freqs = sorted(stddev_dc_power_per_freq.keys())
vals = [stddev_dc_power_per_freq[f] for f in freqs]
cor_data = plt.plot(numpy.array(freqs)/1e6, vals, label='Correction')
freqs = sorted(validation_stddev_dc_offsets_per_freq.keys())
vals = [validation_stddev_dc_offsets_per_freq[f] for f in freqs]
val_data = plt.plot(numpy.array(freqs)/1e6, vals, label='Validation')
legend = plt.legend(loc='upper right', shadow=True)
plt.title("Freq (MHz) vs stddev power (dB)")
plt.grid(True)

plt.subplot(313)
freqs = sorted(original_dc_power_per_freq.keys())
vals = [abs(original_dc_power_per_freq[f]-validation_average_dc_offsets_per_freq[f]) for f in freqs]
plt.plot(numpy.array(freqs)/1e6, vals, label='Correction')
plt.title("Freq (MHz) vs correction (dB)")
plt.grid(True)

plt.show()
"""
