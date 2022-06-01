# %%
import numpy as np
import matplotlib.pyplot as plt

# %%
END_TIME = 1 # s
SAMPLING_FREQUENCY = 1e3
SAMPLES = int(SAMPLING_FREQUENCY*END_TIME)
FFT_RESOL = 0.1 # Hz
freqBins_FFT = int(2**np.ceil(np.log2(abs(SAMPLING_FREQUENCY/2/FFT_RESOL))))

DC_i = 0.1
eps_i = 1
A_i = 1
freq_i = 27 # Hz
phi_i = np.deg2rad(30)

DC_q = 0
eps_q = 1
A_q = 1
delta_phi = np.deg2rad(2)
freq_q = 27 # Hz
phi_q = np.deg2rad(30)

# %%
timeAxis = np.linspace(start=0,stop=END_TIME,num=SAMPLES,endpoint=False)
ifi = DC_i+eps_i*A_i*np.cos(2*np.pi*freq_i*timeAxis+phi_i)
ifq = DC_q+eps_q*A_q*np.cos(2*np.pi*freq_q*timeAxis+phi_q-np.pi/2+delta_phi)
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
plt.axis([-50, 50, -80, 0])
plt.grid(True)
plt.show()


