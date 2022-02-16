
from pylab import *
from rtlsdr import *
import numpy as np
import matplotlib.pyplot as plt

# rtl_power

# Quick setup guide:
# https://github.com/roger-/pyrtlsdr


# SDR attached to computer
sdr = RtlSdr()
# SDR attached to Raspberry Pi
#sdr = RtlSdrTcpClient(hostname='192.168.102.210', port=55366)

# Configure device
sample_rate = 2.4e6 # Hz
center_freq = 96e6 # Hz

sdr.sample_rate = sample_rate  
sdr.center_freq = center_freq     
sdr.freq_correction = 60   # PPM
sdr.gain = 'auto'
num_samples = 256*1024

samples = sdr.read_samples(num_samples)

sdr.close()


def calc_max_power(samples):

    max_pwr = 0
    max_pwr_idx = 0
    for idx, sample in enumerate(samples):
        power = np.abs(sample)**2
        if power >= max_pwr:
            max_pwr = power
            max_pwr_idx = idx
    return max_pwr, max_pwr_idx

def calc_power_range(samples, range=0.00001):

    num = len(samples)
    _ , max_pwr_idx = calc_max_power(samples)
    
    # Select % range centered at highest power reading for calculation
    max_idx = int(max_pwr_idx + np.floor((range/2)*num))
    min_idx = int(max_pwr_idx - np.floor((range/2)*num))
    
    pwr_samples = samples[min_idx:max_idx]

    return np.mean(np.abs(pwr_samples)**2)

def power_for_frequency_range(samples, sample_rate, center_freq, min_freq=95.99999e6, max_freq=96.00001e6):

    num_samples = len(samples)
    center_idx = num_samples//2.

    if center_freq > min_freq:
        min_idx = np.floor(center_idx - abs(((center_freq-min_freq)/(sample_rate*0.5))*(num_samples/2)))
    elif center_freq < min_freq:
        min_idx = np.floor(center_idx + abs(((center_freq-min_freq)/(sample_rate*0.5))*(num_samples/2)))
    else:
        min_idx = center_idx

    if center_freq > max_freq:
        max_idx = np.floor(center_idx - abs(((center_freq-max_freq)/(sample_rate*0.5))*(num_samples/2)))
    elif center_freq < max_freq:
        max_idx = np.floor(center_idx + abs(((center_freq-max_freq)/(sample_rate*0.5))*(num_samples/2)))
    else:
        max_idx = center_idx
    
    pwr_samples = samples[int(min_idx):int(max_idx)]

    return np.mean(np.abs(pwr_samples)**2)

# dB Conversion
# avg_pwr = 10*np.log10(avg_pwr) # in dB
# avg_pwr = 10*np.log10(avg_pwr/1e-3)  # in dBm

def dB(pwr):
    return 10*np.log10(pwr)


# Calculate Average signal power
avg_pwr = np.mean(np.abs(samples)**2)   # Watts

max_power, _ = calc_max_power(samples)

rel_strength = max_power/avg_pwr
# If signal havs roughly zero mean
# avg_pwr = np.var(samples) 

print("Signal Power centered on max = {} dB".format(dB(calc_power_range(samples))))
print("Signal Power for freqency range between {} and {} MHz = {} dB".format(95, 97, dB(power_for_frequency_range(samples, sample_rate, center_freq))))
print("Average signal Power = {} dB".format(dB(avg_pwr)))
print("Max signal Power = {} dB".format(dB(max_power)))
print("Relative strength Factor = {}".format(rel_strength))
print("Relative Power gain = {} dB".format(10*np.log10(max_power/avg_pwr)))


# Use matplotlib to estimate and plot the PSD


psd(samples, NFFT=1024, Fs=sdr.sample_rate/1e6, Fc=sdr.center_freq/1e6)
xlabel('Frequency (MHz)')
ylabel('Relative power (dB)')

show()


"""
Fs = sdr.sample_rate # lets say we sampled at 1 MHz
# Assume x contains your array of IQ samples
N = 512
samples = samples[0:N] # we will only take the FFT of the first N samples 
samples = samples * np.hamming(len(samples)) # apply a Hamming window
PSD = (np.abs(np.fft.fft(samples))/N)**2
PSD_log = 10.0*np.log10(PSD)
PSD_shifted = np.fft.fftshift(PSD_log)

center_freq = sdr.center_freq # frequency we tuned our SDR to
f = np.arange(Fs/-2.0, Fs/2.0, Fs/N) # start, stop, step.  centered around 0 Hz
f += center_freq # now add center frequency
plt.plot(f, PSD_shifted)

plt.xlabel("Frequency [Hz]")
plt.ylabel("Magnitude [dB]")
plt.grid(True)

plt.show()
"""
