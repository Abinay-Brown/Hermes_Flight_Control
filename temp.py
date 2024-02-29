
from scipy.signal import butter, lfilter
import matplotlib.pyplot as plt
import numpy as np

# start of sample signal

# Parameters for the base sine wave signal
num_samples = 10000
sampling_rate = 10000  # Hz
signal_frequency = 2  # Hz for the base sine wave signal

# Parameters for the high-frequency noise
noise_mean = 1  # Mean of the Gaussian noise
noise_std_dev = 1.5  # Standard deviation of the Gaussian noise
carrier_frequency = 2500  # Hz, frequency of the high-frequency noise component

# Time vector
t = np.linspace(0, num_samples / sampling_rate, num_samples, endpoint=False)

# Generate the base sine wave signal
base_signal = np.sin(2 * np.pi * signal_frequency * t)
test_signal = np.sin(2 * np.pi * signal_frequency * 3 * t)


# Generate Gaussian noise
noise = np.random.normal(noise_mean, noise_std_dev, num_samples)

# Generate a high-frequency sinusoidal carrier for the noise
carrier = np.sin(2 * np.pi * carrier_frequency * t)

# Modulate the noise with the high-frequency carrier to create high-frequency noise
high_freq_noise = noise * carrier

# Combine the base sine wave signal with the high-frequency noise
combined_signal = base_signal + high_freq_noise + test_signal

# End of sample signal

# Apply FFT to the combined signal
fft_result = np.fft.fft(combined_signal)
fft_freq = np.fft.fftfreq(num_samples, 1/sampling_rate)

# Bandstop filter parameters
lowcut = 2200  # Low cutoff frequency of the bandstop filter
highcut = 2700  # High cutoff frequency of the bandstop filter
order = 3  # Filter order


# Function to design a Butterworth bandstop filter
def butter_bandstop(lowcut, highcut, fs, order=2):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='bandstop')
    return b, a

# Function to apply a bandstop filter
def butter_bandstop_filter(data, lowcut, highcut, fs, order=2):
    b, a = butter_bandstop(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

# Apply the filter
filtered_signal = butter_bandstop_filter(combined_signal, lowcut, highcut, sampling_rate , order)

# Apply FFT to the combined signal
fft_result = np.fft.fft(filtered_signal)
fft_freq = np.fft.fftfreq(num_samples, 1/sampling_rate)


# Plotting
plt.figure(figsize=(15, 9))

# Plot the base sine wave signal
plt.subplot(3, 1, 1)
plt.plot(t[:1000], base_signal[:1000])
plt.title('Base Sine Wave Signal')
plt.xlabel('Time (seconds)')
plt.ylabel('Amplitude')

# Plot the combined signal
plt.subplot(3, 1, 2)
plt.plot(t[:1000], combined_signal[:1000])
plt.title('Sine Wave Signal with High-Frequency Noise')
plt.xlabel('Time (seconds)')
plt.ylabel('Amplitude')

# Plotting the FFT result
plt.subplot(3, 1, 3)
# Plot only the positive frequencies
positive_freqs = fft_freq > 0
plt.plot(fft_freq[positive_freqs], np.abs(fft_result[positive_freqs]))
plt.title('FFT of the Combined Signal')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Magnitude')
plt.xlim(0, sampling_rate / 2)  # Nyquist limit
plt.grid(True)

plt.tight_layout()
plt.show()