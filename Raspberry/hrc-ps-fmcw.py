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
from scipy import stats

### ACQUISITION SETTINGS ###

PLL_ON = True # Doppler radar controlled by PLL

RAW_DATA = True # Set to 'False' to disable saving of raw data in .csv format
SAMPLING_FREQUENCY = 100e3 # Hz
ACQUISITION_TIME = 10e-3 # s
CHA_RANGE = 8 # Picoscope Ch.A ranges (1:10): 20m, 50m, 100m, 200m, 500m, 1, 2, 5, 10, 20
CHB_RANGE = 8 # Picoscope Ch.B ranges (1:10): 20m, 50m, 100m, 200m, 500m, 1, 2, 5, 10, 20
TRIGGER_DELAY_SEC = 1 # trigger delay in seconds
triggerDelay_samples = int(SAMPLING_FREQUENCY * TRIGGER_DELAY_SEC) # trigger delay in number of samples

# Acquisition time
print("Acquisition time: ", ACQUISITION_TIME, ' s')
# Sampling frequency
if SAMPLING_FREQUENCY >= 125e6:
    timebase = round(log(500e6/SAMPLING_FREQUENCY,2))
    SAMPLING_FREQUENCY = 1/(2**timebase/5)*1e8
    print('Sampling frequency: {:,}'.format(SAMPLING_FREQUENCY) + ' Hz')
else:
    timebase=round(62.5e6/SAMPLING_FREQUENCY+2)
    SAMPLING_FREQUENCY = 62.5e6/(timebase-2)
    print('Sampling frequency: {:,}'.format(SAMPLING_FREQUENCY) + ' Hz')
samplingInterval = 1/SAMPLING_FREQUENCY
totalSamples = round(ACQUISITION_TIME/samplingInterval)
print('Number of total samples (for each channel): {:,}'.format(totalSamples))
print('Full-scale ranges for PicoScope channels (V) ([1:10]: 20m, 50m, 100m, 200m, 500m, 1, 2, 5, 10, 20):')
print('Ch.A (IFI): ', CHA_RANGE)
print('Ch.B (IFQ): ', CHB_RANGE)

### TRX and PLL ###

chipSelectNeg = OutputDevice('BOARD11')
chipSelectNeg.active_high = False
chipSelectNeg.off()
chipEnable = OutputDevice('BOARD12')
chipEnable.active_high = True
chipEnable.off()
loadEnable = OutputDevice('BOARD7')
loadEnable.active_high = True
loadEnable.off()

### MSB ###
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

### LSB ###
# LSB = 0b00001000 # TX full power
# LSB = 0b00001101 # TX power reduced of 4dB
# LSB = 0b00001110 # TX power reduced of 6dB
LSB = 0b00001111 # TX power reduced of 9dB

# bit7: Analog multiplexer control bit "AMUX0" (0)
# bit6: Active-low 64k divider (ON)
# bit5: Active-low 16 divider (ON)
# bit4: Active-low LO buffer output power in high mode (ON)
# bit3: Active-low TX buffer output power in high mode (ON)
# bit2: TW power reduction bit2
# bit1: TW power reduction bit1
# bit0: TW power reduction bit0
# TW output power reduction factors [dB] : 0, 0.4, 0.8, 1.4, 2.5, 4, 6, 9

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


### PICOSCOPE ###

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
# ranges (1:10): 20m, 50m, 100m, 200m, 500m, 1, 2, 5, 10, 20
# analogue offset = -2 V = -2
status["setChA"] = ps.ps2000aSetChannel(chandle, 0, 1, 0, CHA_RANGE, 0)
assert_pico_ok(status["setChA"])
# Set up channel B
# handle = chandle
# channel = PS2000A_CHANNEL_B = 1
# enabled = 1
# coupling type = PS2000A_DC = 1
# range = PS2000A_2V = 7
# ranges (1:10): 20m, 50m, 100m, 200m, 500m, 1, 2, 5, 10, 20
# analogue offset = 0 V
status["setChB"] = ps.ps2000aSetChannel(chandle, 1, 1, 0, CHB_RANGE, 0)
assert_pico_ok(status["setChB"])

# Set up single trigger
# handle = chandle
# enabled = 1
# source = PS2000A_CHANNEL_A = 0
# threshold = 1024 ADC counts
# direction = PS2000A_RISING = 2
# delay = 0 sample periods
# auto Trigger = 1000 ms (if no trigger events occurs)
status["trigger"] = ps.ps2000aSetSimpleTrigger(chandle, 1, 0, 0, 2, triggerDelay_samples, 5000)
assert_pico_ok(status["trigger"])
# Set number of pre and post trigger samples to be collected
preTriggerSamples = round(0)
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
print('Done.')

# ADF4158 registers
R0 =        [0x80, 0x49, 0x70, 0x00]
R1 =        [0x00, 0x00, 0x00, 0x01]
R2 =        [0x07, 0x50, 0x80, 0x2A]
R3 =        [0x00, 0x00, 0x04, 0x03]
R4 =        [0x00, 0x07, 0xFF, 0x84]
R5_load1 =  [0x00, 0x13, 0x33, 0x35]
R5_load2 =  [0x00, 0x13, 0x33, 0x35]
R6_load1 =  [0x00, 0x00, 0x1F, 0x46]
R6_load2 =  [0x00, 0x00, 0x1F, 0x46]
R7 =        [0x00, 0x00, 0x00, 0x07]

def transferRegister(register):
    spi0.xfer(register)
    # Load into the specific 32-bits latch:
    time.sleep(1e-4)
    loadEnable.on()
    time.sleep(1e-4)
    loadEnable.off()
    time.sleep(1e-4)

if PLL_ON == True:
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
#    transferRegister(R6_load2)
    transferRegister(R5_load1)
#    transferRegister(R5_load2)
    transferRegister(R4)
    transferRegister(R3)
    transferRegister(R2)
    transferRegister(R1)
    transferRegister(R0) # last one to be loaded (double-buffered)

    spi0.close()
    time.sleep(1e-4)
    print('ADF4158 programming ended.')

# Block sampling mode
# The scope stores data in internal buffer memory and then transfer it to the PC via USB.
# The data is lost when a new run is started in the same segment.
# For PicoScope 2206B the buffer memory is 32 MS, maximum sampling rate 500 MS/s.
print('Running block capture...')
startTime = time.time()
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
adc2mVChAMax =  adc2mV(bufferAMax, CHA_RANGE, maxADC)
adc2mVChBMax =  adc2mV(bufferBMax, CHB_RANGE, maxADC)

# Create time data
timeAxis = 1e-9*(np.linspace(0, (cTotalSamples.value) * (timeIntervalns.value-1), cTotalSamples.value))
print('Done.')

### DATA MANAGEMENT ##

# Save raw data to .csv file (with timestamp);
# Save time domain plots to .png files;
# Save frequency domain plots to .png files.
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

if RAW_DATA == True:
    # ChA raw samples
    print('Saving ChA raw samples to .csv file...')
    samplesFileNameChA = timestamp + "__ChA.csv"
    completeFileNameChA = os.path.join('./data-acquired/raw-samples',samplesFileNameChA)
    with open(completeFileNameChA,'w') as file:
        writer = csv.writer(file)
        writer.writerows(zip(adc2mVChAMax,timeAxis))
    print('Done.')
    # ChB raw samples
    print('Saving ChB raw samples to .csv file...')
    samplesFileNameChB = timestamp + "__ChB.csv"
    completeFileNameChB = os.path.join('./data-acquired/raw-samples',samplesFileNameChB)
    with open(completeFileNameChB,'w') as file:
        writer = csv.writer(file)
        writer.writerows(zip(adc2mVChBMax,timeAxis))
    print('Done.')

elapsedTime = time.time() - startTime
print('Acquisition completed. Elapsed time (block acquisition and data management): {:.1f}'.format(elapsedTime) + ' s.')
        
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