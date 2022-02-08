
from pylab import *
from rtlsdr import *
import numpy as np
import matplotlib.pyplot as plt

# rtl_power

# Quick setup guide:
# https://github.com/roger-/pyrtlsdr

#sdr = RtlSdr()
sdr = RtlSdrTcpClient(hostname='192.168.102.210', port=55366)

# Configure device
sdr.sample_rate = 1e6  # Hz
sdr.center_freq = 70e6     # Hz
sdr.freq_correction = 60   # PPM
sdr.gain = 'auto'
num_samples = 512

samples = sdr.read_samples(num_samples)

sdr.close()

# Calculate Average signal power
avg_pwr = np.mean(np.abs(samples)**2)   # Watts

# If signal havs roughly zero mean
# avg_pwr = np.var(samples) 

# dB Conversion
avg_pwr = 10*np.log10(avg_pwr)  # in dB
# avg_pwr = 10*np.log10(avg_pwr/1e-3)  # in dBm

print(avg_pwr)

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