import sys
import csv
import numpy as np
import matplotlib.pyplot as plt

def plot_csv(file_path):

    freq_vals = list()
    dc_i_vals = list()
    dc_q_vals = list()

    #extract values into the lists
    with open(file_path) as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if 'DATA STARTS HERE' in row[0]: break
        for i, row in enumerate(reader):
            if not i: continue #skip titles
            tx_lo, icor, qcor, meadured, delta = row
            freq_vals.append(float(tx_lo)/1e9)
            dc_i_vals.append(int(icor))
            dc_q_vals.append(int(qcor))

    #plot of raw correction values
    plt.figure(1)
    plt.subplot(211)
    plt.plot(freq_vals, dc_i_vals, freq_vals, dc_q_vals,)
    plt.ylim(0, 250)
    plt.title("Freq vs IQ corrections")
    plt.xlabel('Freq (GHz)')
    plt.grid(True)

    #plot of average values
    avg_len = 10
    def moving_average(a, n=avg_len) :
        ret = np.cumsum(a, dtype=float)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n - 1:] / n

    plt.subplot(212)
    plt.plot(freq_vals[:-avg_len+1], moving_average(dc_i_vals), freq_vals[:-avg_len+1], moving_average(dc_q_vals),)
    plt.ylim(0, 250)
    plt.title("Freq vs averaged IQ corrections")
    plt.xlabel('Freq (GHz)')
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    plot_csv(sys.argv[1])
