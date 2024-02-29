import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq


# Open the text file for reading
with open('imu_cal_249.txt', 'r') as file:
    # Read each line in the 
    first_column_data = []
    time_data = []
    for line in file:
        # Split the line by the delimiter ','
        data = line.strip().split(',')
        # Print the row-wise data
        print(data)
        first_column_data.append(float(data[3]))
        # Store time data
        time_data.append(float(data[-1]))

N = len(first_column_data
        )
first_column_data = np.array(first_column_data)
time_data = np.array(time_data)

# Perform FFT on the first column data
fft_result = np.fft.fft(first_column_data)

yf = fft(first_column_data)
xf = fftfreq(N, 1/1000)[:N//2]

# Plot FFT result
plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))
plt.grid()
plt.show()