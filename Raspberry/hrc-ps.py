
#!/usr/bin/python
# Hydro Radar Controller with acquisition system based on Picoscope.

import spidev
import time
from gpiozero import OutputDevice
import ctypes
import numpy as np
from picosdk.ps2000a import ps2000a as ps
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from picosdk.functions import adc2mV, assert_pico_ok
import csv
from datetime import datetime
from itertools import zip_longest
import os.path
from math import log
import adafruit_adxl34x
import busio
import board
from struct import unpack

i2c = busio.I2C(board.SCL, board.SDA, frequency=100e3)
accelerometer = adafruit_adxl34x.ADXL345(i2c)
accelerometer.range = adafruit_adxl34x.Range.RANGE_16_G # Range

# Offset compensation (MANUAL CALIBRATION)
# OFSX = 0x2A # X-axis offset in LSBs (sensore amazon #1)
# OFSY = 0x26 # Y-axis offset in LSBs (sensore amazon #1)
# OFSZ = 0x80 # Z-axis offset in LSBs (sensore amazon #1)
OFSX = 0x00 # X-axis offset in LSBs (sensore adafruit #1)
OFSY = 0x00 # Y-axis offset in LSBs (sensore adafruit #1)
OFSZ = 0x00 # Z-axis offset in LSBs (sensore adafruit #1)
accelerometer._write_register_byte(adafruit_adxl34x._REG_OFSX, OFSX)
accelerometer._write_register_byte(adafruit_adxl34x._REG_OFSY, OFSY) 
accelerometer._write_register_byte(adafruit_adxl34x._REG_OFSZ, OFSZ)

# Data rate and power mode control
# bit5: low power (disabled)
# bit4-1: output data rate (100 Hz)
accelerometer._write_register_byte(adafruit_adxl34x._REG_BW_RATE, 0b00001010)

# Power-saving feature control
# bit6: link (disabled)
# bit5: auto sleep (disabled)
# bit4: measure (disabled --> enabled)
# bit3: sleep (disabled)
# bit2-1: wakeup bits (8 Hz) 
accelerometer._write_register_byte(adafruit_adxl34x._REG_BW_RATE, 0b00000000)
accelerometer._write_register_byte(adafruit_adxl34x._REG_BW_RATE, 0b00001000)

# Data format control
# bit8: self test (disabled)
# bit7: SPI mode (4 wire)
# bit6: interrupt active level (active high)
# bit4: full resolution bit (enabled)
# bit3: justify (right)
# bit2-1: range (16g)
accelerometer._write_register_byte(adafruit_adxl34x._REG_DATA_FORMAT, 0b00001011)

# FIFO control
# bit8-7: FIFO mode (stream)
# bit6: trigger (INT1, but unuseful)
# bit5-1: samples needed to trigger watermark interrupt (immediately)  
accelerometer._write_register_byte(adafruit_adxl34x._REG_FIFO_CTL, 0b10000000)

# Edit 'offset' and 'calibrated full-scale' sections after "calibration.py" test.
# Sensor: Adafruit #1
x_min = -249
x_max = 260
delta_x = 509
y_min = -244
y_max = 261
delta_y = 505
z_min = -258
z_max = 242
delta_z = 500

# Offset expressed in LSB.
# We use this instead of OFS registers for finer tuning.
# These numbers depends on calibration.py results for a specific sensor.
x_offset = round((x_min + x_max) / 2)
y_offset = round((y_min + y_max) / 2)
z_offset = round((z_min + z_max) / 2)

# Calibrated full-scale factor, for rescaling.
# We assume that LSB:g relation in linear after rescaling.
# These numbers depends on calibration.py results for a specific sensor.
x_cfs = numpy.ceil(delta_x / 2)
y_cfs = numpy.ceil(delta_y / 2)
z_cfs = numpy.ceil(delta_z / 2)

time.sleep(1)

averages = 20
x_g_avg = 0
y_g_avg = 0
z_g_avg = 0
tiltAngle_1st_avg = 0
tiltAngle_2nd_avg = 0

chipSelectNeg = OutputDevice('BOARD11')
chipSelectNeg.active_high = False
chipSelectNeg.off()
chipEnable = OutputDevice('BOARD12')
chipEnable.active_high = True
chipEnable.off()
loadEnable = OutputDevice('BOARD7')
loadEnable.active_high = True
loadEnable.off()

MSB = 0b00000000
# bit7: Active-low LNA Gain Reduction (OFF)
# bit6: Not Used
# bit5: Not Used
# bit4: TX power disable (OFF)
#       TX is controlled by active-low "TXOFF" pin of the IC.
# bit3: Analog multiplexer control bit "AMUX2" (0)
# bit2: Test bit, must be LOW (0)
# bit1: Test bit, must be lOW (0)
# bit0: Analog multiplexer control bit "AMUX1" (0)

LSB = 0b00001000 # TX full power
# LSB = 0b00001101 # TX power reduced of 4dB
# LSB = 0b00001110 # TX power reduced of 6dB
# LSB = 0b00001111 # TX power reduced of 9dB

# bit7: Analog multiplexer control bit "AMUX0" (0)
# bit6: Active-low 64k divider (ON)
# bit5: Active-low 16 divider (ON)
# bit4: Active-low LO buffer output power in high mode (ON)
# bit3: Active-low TX buffer output power in high mode (ON)
# bit2: TW power reduction bit2
# bit1: TW power reduction bit1
# bit0: TW power reduction bit0
# TW output power reduction factors [dB] : 0, 0.4, 0.8, 1.4, 2.5, 4, 6, 9

# Register values (obtained using ADIsimPLL and ADF4158 evaluation software)

# Choose R0:
print('Enter trasmitter frequency: (23500:50:24500 MHz)')
VCOfreq = input()

if VCOfreq == '23500':
    R0, VCOfreq =        [0x00, 0x24, 0xB8, 0x00], '23500MHz'
elif VCOfreq == '23550':
    R0, VCOfreq =        [0x00, 0x24, 0xCC, 0x00], '23550MHz'
elif VCOfreq == '23600':
    R0, VCOfreq =        [0x00, 0x24, 0xE0, 0x00], '23600MHz'
elif VCOfreq == '23650':
    R0, VCOfreq =        [0x00, 0x24, 0xF4, 0x00], '23650MHz'
elif VCOfreq == '23700':
    R0, VCOfreq =        [0x00, 0x25, 0x08, 0x00], '23700MHz'
elif VCOfreq == '23750':
    R0, VCOfreq =        [0x00, 0x25, 0x1C, 0x00], '23750MHz'
elif VCOfreq == '23800':
    R0, VCOfreq =        [0x00, 0x25, 0x30, 0x00], '23800MHz'
elif VCOfreq == '23850':
    R0, VCOfreq =        [0x00, 0x25, 0x44, 0x00], '23850MHz'
elif VCOfreq == '23900':
    R0, VCOfreq =        [0x00, 0x25, 0x58, 0x00], '23900MHz'
elif VCOfreq == '23950':
    R0, VCOfreq =        [0x00, 0x25, 0x6C, 0x00], '23950MHz'
elif VCOfreq == '24000':
    R0, VCOfreq =        [0x00, 0x25, 0x80, 0x00], '24000MHz'
elif VCOfreq == '24050':
    R0, VCOfreq =        [0x00, 0x25, 0x94, 0x00], '24050MHz'
elif VCOfreq == '24100':
    R0, VCOfreq =        [0x00, 0x25, 0xA8, 0x00], '24100MHz'
elif VCOfreq == '24150':
    R0, VCOfreq =        [0x00, 0x25, 0xBC, 0x00], '24150MHz'
elif VCOfreq == '24200':
    R0, VCOfreq =        [0x00, 0x25, 0xD0, 0x00], '24200MHz'
elif VCOfreq == '24250':
    R0, VCOfreq =        [0x00, 0x25, 0xE4, 0x00], '24250MHz'
elif VCOfreq == '24300':
    R0, VCOfreq =        [0x00, 0x25, 0xF8, 0x00], '24300MHz'
elif VCOfreq == '24350':
    R0, VCOfreq =        [0x00, 0x26, 0x0C, 0x00], '24350MHz'
elif VCOfreq == '24400':
    R0, VCOfreq =        [0x00, 0x26, 0x20, 0x00], '24400MHz'
elif VCOfreq == '24450':
    R0, VCOfreq =        [0x00, 0x26, 0x34, 0x00], '24450MHz'
elif VCOfreq == '24500':
    R0, VCOfreq =        [0x00, 0x26, 0x48, 0x00], '24500MHz'
else:
    raise ValueError('Enter a valid tranmitter frequency value.')

R1 =        [0x00, 0x00, 0x00, 0x01]
R2 =        [0x02, 0x10, 0xFF, 0xFA]
R3 =        [0x00, 0x00, 0x00, 0x43]
R4 =        [0x00, 0x18, 0xC8, 0x04]
R5_load1 =  [0x00, 0x4B, 0xFF, 0xFD]
R5_load2 =  [0x00, 0x80, 0x00, 0x7D]
R6_load1 =  [0x00, 0x00, 0x00, 0x56]
R6_load2 =  [0x00, 0x80, 0x00, 0x06]
R7 =        [0x00, 0x02, 0xFF, 0xFF]

def transferRegister(register):
    spi0.xfer(register)
    # Load into the specific 32-bits latch:
    time.sleep(1e-4)
    loadEnable.on()
    time.sleep(1e-4)
    loadEnable.off()
    time.sleep(1e-4)

print('BGT24MTR11 programming started...')
spi0 = spidev.SpiDev()
# Use of /dev/spidev0.0, SPI0 with CE0 not used.
# SCLK: BOARD23
# MOSI: BOARD19
spi0.open(0,0)
spi0.max_speed_hz = 122000
# CPOL=0, CPHA=1
spi0.mode = 0b01
# Enable BGT24MTR11 for SPI programming
chipSelectNeg.on()
time.sleep(1e-4)
spi0.xfer([MSB,LSB])
spi0.close()
time.sleep(1e-4)
# Disable BGT24MTR11 after successfull programming
chipSelectNeg.off()
print('BGT24MTR11 programming ended.')

# ADF4158 power-on
chipEnable.on()

# ADF4158 programming
print('ADF4158 programming started...')
spi0.open(0,0)
spi0.max_speed_hz = 122000
# CPOL=0, CPHA=1
spi0.mode = 0b00
time.sleep(1e-4)

transferRegister(R7)
transferRegister(R6_load1)
transferRegister(R6_load2)
transferRegister(R5_load1)
transferRegister(R5_load2)
transferRegister(R4)
transferRegister(R3)
transferRegister(R2)
transferRegister(R1)
transferRegister(R0) # last one to be loaded (double-buffered)

spi0.close()
time.sleep(1e-4)
print('ADF4158 programming ended.')

# Specify sampling frequency
SAMPLING_FREQUENCY = 100e3 # Hz
if SAMPLING_FREQUENCY >= 125e6:
    timebase = round(log(500e6/SAMPLING_FREQUENCY,2))
    print('Sampling frequency: {:,}'.format(1/(2**timebase/5)*1e8) + ' Hz')
else:
    timebase=round(62.5e6/SAMPLING_FREQUENCY+2)
    print('Sampling frequency: {:,}'.format(62.5e6/(timebase-2)) + ' Hz')

# PICOSCOPE ACQUISITION

# Specify acquisition time
ACQUISITION_TIME = 2 # s
samplingInterval = 1/SAMPLING_FREQUENCY
totalSamples = round(ACQUISITION_TIME/samplingInterval)
print('Number of total samples (for each channel): {:,}'.format(totalSamples))
# Buffer memory size: 32 M

FFT_FREQ_BINS = 2**18
print('Number of frequency bins for FFT computation: {:,}'.format(FFT_FREQ_BINS))

# Create chandle and status ready for use.
# The c_int16 constructor accepts an optional integer initializer. Default: 0.
chandle = ctypes.c_int16()
status = {}

# Open 2000 series PicoScope
print('Setting up PiscoScope 2206B unit...')
# Returns handle to chandle for use in future API functions
# First argument: number that uniquely identifies the scope (address of chandle)
# Second argument:  first scope found (None)
status["openunit"] = ps.ps2000aOpenUnit(ctypes.byref(chandle), None)
# If any error, the following line will raise one.
assert_pico_ok(status["openunit"])

# Set up channel A
# handle = chandle
# channel = PS2000A_CHANNEL_A = 0
# enabled = 1
# coupling type = PS2000A_DC = 1
# range = PS2000A_2V = 7
# analogue offset = -2 V = -2
chARange = 5
status["setChA"] = ps.ps2000aSetChannel(chandle, 0, 1, 0, chARange, 0)
assert_pico_ok(status["setChA"])
# Set up channel B
# handle = chandle
# channel = PS2000A_CHANNEL_B = 1
# enabled = 1
# coupling type = PS2000A_DC = 1
# range = PS2000A_2V = 7
# analogue offset = 0 V
chBRange = 5
status["setChB"] = ps.ps2000aSetChannel(chandle, 1, 1, 0, chBRange, 0)
assert_pico_ok(status["setChB"])

# Set up single trigger
# handle = chandle
# enabled = 1
# source = PS2000A_CHANNEL_A = 0
# threshold = 1024 ADC counts
# direction = PS2000A_RISING = 2
# delay = 0 sample periods
# auto Trigger = 1000 ms (if no trigger events occurs)
status["trigger"] = ps.ps2000aSetSimpleTrigger(chandle, 1, 0, 0, 2, 1000000, 5000)
assert_pico_ok(status["trigger"])
# Set number of pre and post trigger samples to be collected
preTriggerSamples = round(totalSamples/2)
postTriggerSamples = totalSamples-preTriggerSamples
# Get timebase information
# handle = chandle
# timebase: obtained by samplingFrequency (sample interval formula: (timebase-2)*16 ns [for timebase>=3])
# noSamples = totalSamples
# pointer to timeIntervalNanoseconds = ctypes.byref(timeIntervalNs)
# oersample: not used, just initialized
# pointer to totalSamples = ctypes.byref(returnedMaxSamples)
# segment index = 0 (index of the memory segment to use, only 1 segment by default)
timeIntervalns = ctypes.c_float()
returnedMaxSamples = ctypes.c_int32()
oversample = ctypes.c_int16(0)
status["getTimebase2"] = ps.ps2000aGetTimebase2(chandle,
                                                timebase,
                                                totalSamples,
                                                ctypes.byref(timeIntervalns),
                                                oversample,
                                                ctypes.byref(returnedMaxSamples),
                                                0)
assert_pico_ok(status["getTimebase2"])
time.sleep(1) # Wait transient after switch to AC coupling.
print('Done.')

# Block sampling mode
# The scope stores data in internal buffer memory and then transfer it to the PC via USB.
# The data is lost when a new run is started in the same segment.
# For PicoScope 2206B the buffer memory is 32 MS, maximum sampling rate 500 MS/s.
print('Running block capture...')
# Run block capture
# handle = chandle
# number of pre-trigger samples = preTriggerSamples
# number of post-trigger samples = PostTriggerSamples
# timebase (already defined when using ps2000aGetTimebase2)
# oversample: not used
# time indisposed ms = None (not needed, it's the time the scope will spend collecting samples)
# segment index = 0 (the only one defined by default, this index is zero-based)
# lpReady = None (using ps2000aIsReady rather than ps2000aBlockReady; callback functions that the driver will call when the data has been collected).
# pParameter = None (void pointer passed to ps2000aBlockReady() to return arbitrary data to the application)
status["runBlock"] = ps.ps2000aRunBlock(chandle,
                                        preTriggerSamples,
                                        postTriggerSamples,
                                        timebase,
                                        oversample,
                                        None,
                                        0,
                                        None,
                                        None)
assert_pico_ok(status["runBlock"])

# Check for data collection to finish using ps2000aIsReady
ready = ctypes.c_int16(0)
check = ctypes.c_int16(0)
while ready.value == check.value:
    status["isReady"] = ps.ps2000aIsReady(chandle, ctypes.byref(ready))

# Create buffers ready for assigning pointers for data collection
bufferAMax = (ctypes.c_int16 * totalSamples)()
bufferAMin = (ctypes.c_int16 * totalSamples)() # used for downsampling which isn't in the scope of this example
bufferBMax = (ctypes.c_int16 * totalSamples)()
bufferBMin = (ctypes.c_int16 * totalSamples)() # used for downsampling which isn't in the scope of this example

# Set data buffer location for data collection from channel A
# handle = chandle
# source = PS2000A_CHANNEL_A = 0
# pointer to buffer max = ctypes.byref(bufferDPort0Max)
# pointer to buffer min = ctypes.byref(bufferDPort0Min)
# buffer length = totalSamples
# segment index = 0
# ratio mode = PS2000A_RATIO_MODE_NONE = 0
status["setDataBuffersA"] = ps.ps2000aSetDataBuffers(chandle,
                                                     0,
                                                     ctypes.byref(bufferAMax),
                                                     ctypes.byref(bufferAMin),
                                                     totalSamples,
                                                     0,
                                                     0)
assert_pico_ok(status["setDataBuffersA"])

# Set data buffer location for data collection from channel B
# handle = chandle
# source = PS2000A_CHANNEL_B = 1
# pointer to buffer max = ctypes.byref(bufferBMax)
# pointer to buffer min = ctypes.byref(bufferBMin)
# buffer length = totalSamples
# segment index = 0
# ratio mode = PS2000A_RATIO_MODE_NONE = 0
status["setDataBuffersB"] = ps.ps2000aSetDataBuffers(chandle,
                                                     1,
                                                     ctypes.byref(bufferBMax),
                                                     ctypes.byref(bufferBMin),
                                                     totalSamples,
                                                     0,
                                                     0)
assert_pico_ok(status["setDataBuffersB"])

# Create overflow location
overflow = ctypes.c_int16()
# Create converted type totalSamples
cTotalSamples = ctypes.c_int32(totalSamples)

# Retrived data from scope to buffers assigned above
# handle = chandle
# start index = 0 (zero-based index, sample intervals from the start of the buffer)
# pointer to number of samples = ctypes.byref(cTotalSamples)
# downsample ratio = 0
# downsample ratio mode = PS2000A_RATIO_MODE_NONE (downsampling disabled)
# pointer to overflow = ctypes.byref(overflow))
status["getValues"] = ps.ps2000aGetValues(chandle, 0, ctypes.byref(cTotalSamples), 0, 0, 0, ctypes.byref(overflow))
assert_pico_ok(status["getValues"])


# find maximum ADC count value
# handle = chandle
# pointer to value = ctypes.byref(maxADC)
maxADC = ctypes.c_int16()
status["maximumValue"] = ps.ps2000aMaximumValue(chandle, ctypes.byref(maxADC))
assert_pico_ok(status["maximumValue"])

# convert ADC counts data to mV
adc2mVChAMax =  adc2mV(bufferAMax, chARange, maxADC)
adc2mVChBMax =  adc2mV(bufferBMax, chBRange, maxADC)

# Create time data
timeAxis = 1e-9*(np.linspace(0, (cTotalSamples.value) * (timeIntervalns.value-1), cTotalSamples.value))
print('Done.')

# Stop the scope
print('Closing the scope...')
# handle = chandle
status["stop"] = ps.ps2000aStop(chandle)
assert_pico_ok(status["stop"])
# Close unitDisconnect the scope
# handle = chandle
status["close"] = ps.ps2000aCloseUnit(chandle)
assert_pico_ok(status["close"])
print('Done.')
print(status)

# Save raw data to .csv file (with timestamp);
# Save time domain plots to .png files;
# Save frequency domain plots to .png files.
startTime = time.time()
print('Saving raw samples to .csv file...')
print('Computing FFT...')
print('Generating .png plots...')
timestamp = datetime.now().strftime("%Y%m%d_%I%M%S_%p")

# ChA raw samples
samplesFileNameChA = timestamp + "__" + VCOfreq + "__ChA.csv"
completeFileNameChA = os.path.join('./data-acquired/raw-samples',samplesFileNameChA)
with open(completeFileNameChA,'w') as file:
    writer = csv.writer(file)
    writer.writerows(zip(adc2mVChAMax,timeAxis))
# ChA time plot - Full length
timePlotNameChA = os.path.join('./data-acquired/png-graphs', timestamp + "__" + VCOfreq + "__ChA_time-full.png")
plt.plot(timeAxis, adc2mVChAMax)
plt.ylabel('ChA (mV)')
plt.xlabel('Time (s)')
plt.grid(True)
plt.savefig(timePlotNameChA)
# plt.show()
plt.close()
# ChA time plot - Zoom
timePlotNameChA = os.path.join('./data-acquired/png-graphs', timestamp + "__" + VCOfreq + "__ChA_time-zoom.png")
plt.plot(timeAxis, adc2mVChAMax)
plt.ylabel('Signal (mV)')
plt.xlabel('Time (s)')
plt.grid(True)
plt.axis([0, 5e-3, -500, 500])
plt.savefig(timePlotNameChA)
# plt.show()
plt.close()
# FFT ChA
ChA_FFT = np.fft.rfft(adc2mVChAMax, n = FFT_FREQ_BINS) # FFT of real signal
ChA_FFT_mV = np.abs(2/(totalSamples)*ChA_FFT) # FFT magnitude
ChA_FFT_dBV = 20*np.log10(ChA_FFT_mV/1000)
# ChA_PSD = numpy.abs(ChA_FFT)**2
# ChA_PSD_dBm = 10*numpy.log10(ChA_PSD/1e-3)
freqAxis = np.fft.rfftfreq(FFT_FREQ_BINS) # freqBins/2+1
freqAxis_Hz = freqAxis * SAMPLING_FREQUENCY
print('Channel A - Estimated Doppler Frequency (spectrum peak): ' + str(freqAxis_Hz[ChA_FFT_dBV.argmax()]) + ' Hz')
# ChA spectrum - Full
freqPlotNameChA = os.path.join('./data-acquired/png-graphs', timestamp + "__" + VCOfreq + "__ChA_FFT-full.png")
plt.plot(freqAxis_Hz, ChA_FFT_dBV)
plt.ylabel('ChA spectrum (dBV)')
plt.xlabel('Frequency (Hz)')
plt.grid(True)
plt.savefig(freqPlotNameChA)
# plt.show()
plt.close()
# ChA spectrum - Zoom
freqPlotNameChA = os.path.join('./data-acquired/png-graphs', timestamp + "__" + VCOfreq + "__ChA_FFT-zoom.png")
plt.plot(freqAxis_Hz, ChA_FFT_dBV)
plt.ylabel('ChA spectrum (dBV)')
plt.xlabel('Frequency (Hz)')
plt.grid(True)
plt.axis([0, 10e3, -100, 0])
plt.savefig(freqPlotNameChA)
# plt.show()
plt.close()

# ChB raw samples
samplesFileNameChB = timestamp + "__" + VCOfreq + "__ChB.csv"
completeFileNameChB = os.path.join('./data-acquired/raw-samples',samplesFileNameChB)
with open(completeFileNameChB,'w') as file:
    writer = csv.writer(file)
    writer.writerows(zip(adc2mVChBMax,timeAxis))
# ChB time plot - Full length
timePlotNameChB = os.path.join('./data-acquired/png-graphs', timestamp + "__" + VCOfreq + "__ChB_time-full.png")
plt.plot(timeAxis, adc2mVChBMax)
plt.ylabel('ChB (mV)')
plt.xlabel('Time (s)')
plt.grid(True)
plt.savefig(timePlotNameChB)
# plt.show()
plt.close()
# ChB time plot - Zoom
timePlotNameChB = os.path.join('./data-acquired/png-graphs', timestamp + "__" + VCOfreq + "__ChB_time-zoom.png")
plt.plot(timeAxis, adc2mVChBMax)
plt.ylabel('ChB (mV)')
plt.xlabel('Time (s)')
plt.grid(True)
plt.axis([0, 5e-3, -500, 500])
plt.savefig(timePlotNameChB)
# plt.show()
plt.close()
# FFT ChB
ChB_FFT = np.fft.rfft(adc2mVChBMax, n = FFT_FREQ_BINS) # FFT of real signal
ChB_FFT_mV = np.abs(2/(totalSamples)*ChB_FFT) # FFT magnitude
ChB_FFT_dBV = 20*np.log10(ChB_FFT_mV/1000)
# ChB_PSD = numpy.abs(ChB_FFT)**2
# ChB_PSD_dBm = 10*numpy.log10(ChB_PSD/1e-3)
freqAxis = np.fft.rfftfreq(FFT_FREQ_BINS) # freqBins/2+1
freqAxis_Hz = freqAxis * SAMPLING_FREQUENCY
print('Channel B - Estimated Doppler Frequency (spectrum peak): ' + str(freqAxis_Hz[ChB_FFT_dBV.argmax()]) + ' Hz')
# ChB spectrum - Full
freqPlotNameChB = os.path.join('./data-acquired/png-graphs', timestamp + "__" + VCOfreq + "__ChB_FFT-full.png")
plt.plot(freqAxis_Hz, ChB_FFT_dBV)
plt.ylabel('ChB spectrum (dBV)')
plt.xlabel('Frequency (Hz)')
plt.grid(True)
plt.savefig(freqPlotNameChB)
# plt.show()
plt.close()
# ChA spectrum - Zoom
freqPlotNameChB = os.path.join('./data-acquired/png-graphs', timestamp + "__" + VCOfreq + "__ChB_FFT-zoom.png")
plt.plot(freqAxis_Hz, ChB_FFT_dBV)
plt.ylabel('ChB spectrum (dBV)')
plt.xlabel('Frequency (Hz)')
plt.grid(True)
plt.axis([0, 10e3, -100, 0])
plt.savefig(freqPlotNameChB)
# plt.show()
plt.close()

elapsedTime = time.time() - startTime
print('Done. Elapsed time: {:.1f}'.format(elapsedTime) + ' s.')

