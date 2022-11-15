# %%
import numpy as np
import matplotlib.pyplot as plt

# %%
END_TIME = 0.5 # s
SAMPLING_FREQUENCY = 5e3
SAMPLES = int(SAMPLING_FREQUENCY*END_TIME)
FFT_RESOL = 0.1 # Hz
freqBins_FFT = int(2**np.ceil(np.log2(abs(SAMPLING_FREQUENCY/2/FFT_RESOL))))
timeAxis = np.linspace(start=0,stop=END_TIME,num=SAMPLES,endpoint=False)

freq_LO = 77e9 # Hz
wavelength = 3e8 / freq_LO # m
target_velocity = 0 # m/s
scaling_factor = 100
target_vibration_amplitude = 100e-6 # m
target_vibration_frequency = 50 # Hz

x_LO = np.exp(2j*(np.pi*freq_LO*timeAxis))
x_RX = np.exp(2j*(np.pi*freq_LO*timeAxis + 2*np.pi*target_velocity/wavelength*timeAxis + 2*target_vibration_amplitude/wavelength*2*np.pi*np.cos(2*np.pi*target_vibration_frequency*timeAxis)))
x_IF = x_LO * x_RX
x_IF = x_IF - np.mean(x_IF)

# %%
ifi = np.real(x_IF)
ifq = np.imag(x_IF)
plt.plot(timeAxis, ifi)
plt.plot(timeAxis, ifq)
plt.legend(("ifi","ifq"))
plt.show()

# %%
if_complex = np.add(np.asarray(ifi), 1j*np.asarray(ifq))
plt.plot(timeAxis, np.abs(if_complex))
plt.plot(timeAxis, np.angle(if_complex))
plt.legend(("magnitude","phase"))
plt.show()

# %%
FFT = np.fft.fftshift(np.fft.fft(if_complex, n = freqBins_FFT)) # FFT of complex signal
FFT_magn = np.abs(1/(SAMPLES)*FFT) # FFT magnitude
FFT_dB = 20*np.log10(FFT_magn)
freqAxis = np.fft.fftshift(np.fft.fftfreq(freqBins_FFT)) # freqBins+1
freqAxis_Hz = freqAxis * SAMPLING_FREQUENCY
plt.plot(freqAxis_Hz, FFT_dB)
plt.ylabel('spectrum magnitude (dBV)')
plt.xlabel('frequency (Hz)')
plt.axis([-1000, 1000, -200, 0])
plt.grid(True)
plt.show()


