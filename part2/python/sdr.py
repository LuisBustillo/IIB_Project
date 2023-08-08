
from pylab import *
from rtlsdr import *
import numpy as np
import matplotlib.pyplot as plt

# rtl_power

# Quick setup guide:
# https://github.com/roger-/pyrtlsdr

# Configure device
def configure_device(sdr, sample_rate=2.4e6, center_freq=914e6):
    
    sdr.sample_rate = sample_rate  
    sdr.center_freq = center_freq     
    sdr.freq_correction = 60   # PPM
    sdr.gain = 10

# Read Samples
def receive_samples(sdr, num_samples=256*1024):
    samples = sdr.read_samples(num_samples)
    return samples

# Stop connection
def close_connection(sdr):
    sdr.close()


# dB Conversion
# avg_pwr = 10*np.log10(avg_pwr) # in dB
# avg_pwr = 10*np.log10(avg_pwr/1e-3)  # in dBm

def dB(pwr):
    return 10*np.log10(pwr)

# Helper Functions

def GetSpacedElements(arr, numElems, returnIndices=False):
    indices = np.round(np.linspace(0, len(arr) - 1, numElems)).astype(int)
    values = arr[indices]
    return (values, indices) if returnIndices else (values)

def find_closest_index(arr, val):
    diff = 1e9
    closest_idx = 0
    for idx in range(len(arr)):
       
        if abs(arr[idx] - val) < diff:
            diff = abs(arr[idx] - val)
            closest_idx = idx

    return closest_idx

# Calculating Max power and signal power at specific frequency from PSD
def get_power_from_PSD(samples, sdr, freq, plot=False):

    # Use matplotlib to estimate and plot the PSD
    Pxx, freqs = psd(samples, NFFT=1024, Fs=sdr.sample_rate/1e6, Fc=sdr.center_freq/1e6)
    
    f = freq/1e6
    f_idx = find_closest_index(freqs, f)

    power_at_freq = dB(Pxx[f_idx])
    max_power = dB(max(Pxx))

    #TODO write algorithm for second peak
    # for sample in samples

    if plot:
        xlabel('Frequency (MHz)')
        ylabel('Relative power (dB)')
        show()

    return max_power, power_at_freq


# Signal power measurements from received samples

def power_for_range(samples, sdr, min_freq, max_freq):

    Pxx, freqs = psd(samples, NFFT=1024, Fs=sdr.sample_rate/1e6, Fc=sdr.center_freq/1e6)
    
    min_idx = find_closest_index(freqs, min_freq/1e6)
    max_idx = find_closest_index(freqs, max_freq/1e6)
    
    pwr_samples = Pxx[min_idx:max_idx]
    pwr_range = np.mean(np.abs(pwr_samples)**2)

    return pwr_range


# Low-level PSD plotting implementation

def plot_PSD(sdr, samples):

    Fs = sdr.sample_rate # lets say we sampled at 1 MHz
    # Assume samples contains your array of IQ samples
    N = 1024
    # N = len(samples)
    # samples = samples[0:N] # we will only take the FFT of the first N samples
    samples = GetSpacedElements(samples, N)
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

def power_for_frequency_range(samples, sample_rate, center_freq, min_freq, max_freq):

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


if __name__ == '__main__':
    
    # SDR attached to computer
    sdr = RtlSdr()
    # SDR attached to Raspberry Pi
    #sdr = RtlSdrTcpClient(hostname='192.168.228.210', port=55366)
    
    configure_device(sdr, center_freq=914.6e6)

    samples = receive_samples(sdr)

    # close_connection(sdr)

    measurment_freq = 915.0e6

    # Calculate Average signal power
    avg_pwr = np.mean(np.abs(samples)**2)   # Watts
    # If signal havs roughly zero mean
    # avg_pwr = np.var(samples) 
    
    # Measurments from PSD

    max_power, power_at_freq = get_power_from_PSD(samples, sdr, freq=measurment_freq, plot=True)

    #print("Signal Power for freqency range between {} and {} MHz = {} dB".format(95, 97, dB(power_for_frequency_range(samples, sample_rate=2.4e6, center_freq=915e6, min_freq=914e6, max_freq=916e6))))
    print("Average signal Power = {} dB".format(dB(avg_pwr)))
    print("Max signal Power = {} dB".format(max_power))
    print("Signal Power at {} MHz = {} dB".format(measurment_freq/1e6, power_at_freq))
    #print("Relative strength Factor = {}".format(rel_strength))
    #print("Relative Power gain = {} dB".format(10*np.log10(max_power/avg_pwr)))

    
