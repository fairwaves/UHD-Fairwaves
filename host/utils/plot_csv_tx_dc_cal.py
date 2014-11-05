import sys
import csv
import numpy as np
import matplotlib.pyplot as plt

def plot_csv(file_path):

    freq_vals = list()
    dc_i_vals = list()
    dc_q_vals = list()
    dc_i_vals_per_freq = dict()
    dc_q_vals_per_freq = dict()

    #extract values into the lists
    with open(file_path) as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if 'DATA STARTS HERE' in row[0]: break
        for i, row in enumerate(reader):
            if not i: continue #skip titles
            tx_lo, icor, qcor, meadured, delta = row
            tx_lo = float(tx_lo)/1e9
            icor = float(icor)
            qcor = float(qcor)
            freq_vals.append(tx_lo)
            dc_i_vals.append(icor)
            dc_q_vals.append(qcor)
            if tx_lo not in dc_i_vals_per_freq: dc_i_vals_per_freq[tx_lo] = list()
            if tx_lo not in dc_q_vals_per_freq: dc_q_vals_per_freq[tx_lo] = list()
            dc_i_vals_per_freq[tx_lo].append(icor)
            dc_q_vals_per_freq[tx_lo].append(qcor)

    #plot of raw correction values
    plt.figure(1)
    plt.subplot(311)
    plt.plot(freq_vals, dc_i_vals, freq_vals, dc_q_vals,)
    #plt.ylim(0, 250)
    plt.title("Freq (GHz) vs raw IQ corrections")
    #plt.xlabel('Freq (GHz)')
    plt.grid(True)

    #plot of variance
    plt.subplot(312)
    freqs = sorted(dc_i_vals_per_freq.keys())
    dc_i_std = [np.std(dc_i_vals_per_freq[f]) for f in freqs]
    dc_q_std = [np.std(dc_q_vals_per_freq[f]) for f in freqs]
    plt.plot(freqs, dc_i_std, freqs, dc_q_std)
    #plt.ylim(0, 150)
    plt.title("Freq (GHz) vs stddev IQ corrections")
    #plt.xlabel('Freq (GHz)')
    plt.grid(True)

    #plot of average values
    """
    avg_len = 10
    def moving_average(a, n=avg_len) :
        ret = np.cumsum(a, dtype=float)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n - 1:] / n

    plt.subplot(313)
    plt.plot(freq_vals[:-avg_len+1], moving_average(dc_i_vals), freq_vals[:-avg_len+1], moving_average(dc_q_vals),)
    plt.ylim(0, 250)
    plt.title("Freq (GHz) vs averaged IQ corrections")
    #plt.xlabel('Freq (GHz)')
    plt.grid(True)
    """
    plt.subplot(313)
    freqs = sorted(dc_i_vals_per_freq.keys())
    dc_i_avg = [np.mean(dc_i_vals_per_freq[f]) for f in freqs]
    dc_q_avg = [np.mean(dc_q_vals_per_freq[f]) for f in freqs]
    plt.plot(freqs, dc_i_avg, freqs, dc_q_avg)
    #plt.ylim(0, 250)
    plt.title("Freq (GHz) vs averaged IQ corrections")
    #plt.xlabel('Freq (GHz)')
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    plot_csv(sys.argv[1])
