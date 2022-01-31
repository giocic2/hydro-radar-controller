
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

### ACQUISITION SETTINGS ###

PIVOT_HEIGHT = 0.56 + 0.0 # m. Vertical distance between pivot and ground level.
ANTENNA_CENTER_POSITION = 0.12 # m. Distance between pivot and center of RX antenna.
MAX_SCAN_ANGLE = np.deg2rad(15)
ACCELL_AVERAGES = 20
SAMPLING_FREQUENCY = 100e3 # Hz
ACQUISITION_TIME = 1 # s
FFT_RESOL = 1 # Hz
REAL_TIME_MEAS = True # Set to 'False' to disable real time signal processing (FFT and surface velocity computation)
RAW_DATA = True # Set to 'False' to disable saving of raw data in .csv format
SMOOTHING_WINDOW = 10 # Hz
FFT_THRESHOLD = -80 # dBV
CHA_RANGE = 6 # Picoscope Ch.A ranges (1:10): 20m, 50m, 100m, 200m, 500m, 1, 2, 5, 10, 20
CHB_RANGE = 6 # Picoscope Ch.B ranges (1:10): 20m, 50m, 100m, 200m, 500m, 1, 2, 5, 10, 20
FREQUENCY_MIN = -50_000
BANDWIDTH_THRESHOLD = 6 # dB

print("*** GRID SCAN SETTINGS ***")
# Acquisition time
print("Acquisition time (for each direction): ", ACQUISITION_TIME, ' s')
# Sampling frequency
if SAMPLING_FREQUENCY >= 125e6:
    timebase = round(log(500e6/SAMPLING_FREQUENCY,2))
    print('Sampling frequency: {:,}'.format(1/(2**timebase/5)*1e8) + ' Hz')
else:
    timebase=round(62.5e6/SAMPLING_FREQUENCY+2)
    print('Sampling frequency: {:,}'.format(62.5e6/(timebase-2)) + ' Hz')
samplingInterval = 1/SAMPLING_FREQUENCY
totalSamples = round(ACQUISITION_TIME/samplingInterval)
print('Number of total samples (for each channel): {:,}'.format(totalSamples))
print('Full-scale ranges for PicoScope channels (V) ([1:10]: 20m, 50m, 100m, 200m, 500m, 1, 2, 5, 10, 20):')
print('Ch.A (IFI): ', CHA_RANGE)
print('Ch.B (IFQ): ', CHB_RANGE)
# FFT bins and resolution
freqBins_FFT = int(2**np.ceil(np.log2(abs(SAMPLING_FREQUENCY/2/FFT_RESOL))))
if REAL_TIME_MEAS == True:
    print('FFT resolution: ' + str(SAMPLING_FREQUENCY / freqBins_FFT) + ' Hz')
    print('FFT bins: ' + str(freqBins_FFT))
smoothingBins = int(round(SMOOTHING_WINDOW / (SAMPLING_FREQUENCY / freqBins_FFT)))
if SMOOTHING == True:
    print('Size of smoothing window (moving average): ' + str(smoothingBins) + ' bins')
print('Threshold for detection: ' + str(FFT_THRESHOLD) + ' dBV')

### ACCELEROMETER ###

i2c = busio.I2C(board.SCL, board.SDA, frequency=100e3)
accelerometer = adafruit_adxl34x.ADXL345(i2c)
accelerometer.range = adafruit_adxl34x.Range.RANGE_16_G # Range

# Offset compensation (MANUAL CALIBRATION)
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
# We use this instead of OFS_ registers for finer tuning.
# These numbers depends on calibration.py results for a specific sensor.
x_offset = round((x_min + x_max) / 2)
y_offset = round((y_min + y_max) / 2)
z_offset = round((z_min + z_max) / 2)

# Calibrated full-scale factor, for rescaling.
# We assume that LSB:g relation is linear after rescaling.
# These numbers depends on calibration.py results for a specific sensor.
x_cfs = np.ceil(delta_x / 2)
y_cfs = np.ceil(delta_y / 2)
z_cfs = np.ceil(delta_z / 2)

time.sleep(1)

x_g_avg = 0
y_g_avg = 0
z_g_avg = 0
tiltAngle_1st_avg = 0
tiltAngle_2nd_avg = 0
continueCalibration = True

while (continueCalibration == True):
	for index in range(ACCELL_AVERAGES):
		DATA_XYZ = accelerometer._read_register(adafruit_adxl34x._REG_DATAX0, 6)
		time.sleep(0.1)
		x, y, z = unpack("<hhh",DATA_XYZ)
		x_g = x - x_offset
		y_g = y - y_offset
		z_g = z - z_offset
		x_g_avg = x_g_avg + x_g/ACCELL_AVERAGES
		y_g_avg = y_g_avg + y_g/ACCELL_AVERAGES
		z_g_avg = z_g_avg + z_g/ACCELL_AVERAGES
		# Two formulas to evaluate the same tilt angle
		tiltAngle_1st = np.arcsin(- z_g / z_cfs)
		tiltAngle_2nd = np.arccos(+ y_g / y_cfs)
		tiltAngle_1st_avg = tiltAngle_1st_avg + tiltAngle_1st/ACCELL_AVERAGES
		tiltAngle_2nd_avg = tiltAngle_2nd_avg + tiltAngle_2nd/ACCELL_AVERAGES
	tiltAngle_avg = (tiltAngle_1st_avg + tiltAngle_2nd_avg) / 2
# 	print("x y z [LSB]:", round(x_g_avg), round(y_g_avg), round(z_g_avg))
# 	print("Tilt angle [deg] (two values that should be equal): {0:.2f} {1:.2f}".format(numpy.rad2deg(tiltAngle_1st_avg), numpy.rad2deg(tiltAngle_2nd_avg)))
	print("Estimated tilt angle [deg]: {0:.1f}".format(np.rad2deg(tiltAngle_avg)))
	if round(x_g_avg) != 0:
		print("WARNING: gravity along X-axis should be 0! Please align the sensor horizontally.")
		print("x: ", round(x_g_avg))
	accelerometer._write_register_byte(adafruit_adxl34x._REG_BW_RATE, 0b00000000)
	accelerometer._write_register_byte(adafruit_adxl34x._REG_BW_RATE, 0b00001000)
	whatNext = input("Repeat calibration? [y/n]: ")
	continueTakingUserInput = True
	while continueTakingUserInput == True:
		if whatNext == 'y':
			continueCalibration = True
			continueTakingUserInput = False
		if whatNext == 'n':
			continueCalibration = False
			continueTakingUserInput = False
		if whatNext != 'y' and whatNext != 'n':
			print("Please type correctly [y/n].")
			continueTakingUserInput = True
	time.sleep(0.5)
	x_g_avg = 0
	y_g_avg = 0
	z_g_avg = 0
	tiltAngle_1st_avg = 0
	tiltAngle_2nd_avg = 0
tiltAngle = str("{0:.1f}".format(np.rad2deg(tiltAngle_avg))) + "deg"

# Accelerometer in sleep mode
accelerometer._write_register_byte(adafruit_adxl34x._REG_BW_RATE, 0b00000100)

# Only for my particular radar prototype
#    /
#   /. RX antenna
#  /    .
# *pivot   .
# |           .
# |              . d (@ beam direction)
# |                 .
# |                    .
# |                       .
# ____                       ._____ground level
#    <----------------------->
#                l
antennaHeight = PIVOT_HEIGHT + ANTENNA_CENTER_POSITION * np.cos(tiltAngle_avg)
antennaHeight_str = str("{0:.2f}".format(antennaHeight)) + "m"
maxSwath = 2 * antennaHeight / np.tan(tiltAngle_avg) * np.tan(MAX_SCAN_ANGLE)
print("Vertical distance between RX antenna and ground level: {0:.2f} m".format(antennaHeight))
# d
print("Distance between RX antenna and ground level @ beam direction: {0:.2f} m".format(antennaHeight / np.sin(tiltAngle_avg)))
# l
print("Projection of that distance on ground level: {0:.2f} m".format(antennaHeight / np.tan(tiltAngle_avg)))
# swath
print("Horizontal swath @ ground level: {0:.2f} m".format(maxSwath))

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

print('BGT24MTR11 programming started...', end = ' ')
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

# Register values (obtained using ADIsimPLL and ADF4158 evaluation software)
# Choose R0:
scanningDirections = int(input('Enter how many scanning directions: (1, 2, 3, 5, 6, 11, 21): '))
thisDirection = 1
if scanningDirections != 1:
    VCOfreq_step = 1000 / (scanningDirections - 1)
    VCOfreq = 23500
if scanningDirections == 1:
    VCOfreq = int(input('Enter transmitter frequency (23500:50:24500): '))
    VCOfreq_step = 1001 # Just to put VCOfreq > 24500 at next cycle
VCOfreq_str = str(VCOfreq)

### PICOSCOPE ###

# Create chandle and status ready for use.
# The c_int16 constructor accepts an optional integer initializer. Default: 0.
chandle = ctypes.c_int16()
status = {}

# Open 2000 series PicoScope
print('Setting up PiscoScope 2206B unit...', end = ' ')
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
status["trigger"] = ps.ps2000aSetSimpleTrigger(chandle, 1, 0, 0, 2, 200_000, 5000)
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

### GRID SCAN ###
while VCOfreq <= 24500:
    print("*****************")
    print("Scanning direction " + str(thisDirection) + " of " + str(scanningDirections) + "...")
    thisDirection += 1
    if VCOfreq == 23500:
        R0, VCOfreq_str =        [0x00, 0x24, 0xB8, 0x00], '23500MHz'
    elif VCOfreq == 23550:
        R0, VCOfreq_str =        [0x00, 0x24, 0xCC, 0x00], '23550MHz'
    elif VCOfreq == 23600:
        R0, VCOfreq_str =        [0x00, 0x24, 0xE0, 0x00], '23600MHz'
    elif VCOfreq == 23650:
        R0, VCOfreq_str =        [0x00, 0x24, 0xF4, 0x00], '23650MHz'
    elif VCOfreq == 23700:
        R0, VCOfreq_str =        [0x00, 0x25, 0x08, 0x00], '23700MHz'
    elif VCOfreq == 23750:
        R0, VCOfreq_str =        [0x00, 0x25, 0x1C, 0x00], '23750MHz'
    elif VCOfreq == 23800:
        R0, VCOfreq_str =        [0x00, 0x25, 0x30, 0x00], '23800MHz'
    elif VCOfreq == 23850:
        R0, VCOfreq_str =        [0x00, 0x25, 0x44, 0x00], '23850MHz'
    elif VCOfreq == 23900:
        R0, VCOfreq_str =        [0x00, 0x25, 0x58, 0x00], '23900MHz'
    elif VCOfreq == 23950:
        R0, VCOfreq_str =        [0x00, 0x25, 0x6C, 0x00], '23950MHz'
    elif VCOfreq == 24000:
        R0, VCOfreq_str =        [0x00, 0x25, 0x80, 0x00], '24000MHz' # Rcounter=1, Rdoubler=ON, Ref/2=OFF (default)
        # R0, VCOfreq_str =        [0x00, 0x4B, 0x00, 0x00], '24000MHz' # Rcounter=1, Rdoubler=OFF, Ref/2=OFF
        # R0, VCOfreq_str =        [0x00, 0x96, 0x00, 0x00], '24000MHz' # Rcounter=1, Rdoubler=OFF, Ref/2=ON
    elif VCOfreq == 24050:
        R0, VCOfreq_str =        [0x00, 0x25, 0x94, 0x00], '24050MHz'
    elif VCOfreq == 24100:
        R0, VCOfreq_str =        [0x00, 0x25, 0xA8, 0x00], '24100MHz'
    elif VCOfreq == 24150:
        R0, VCOfreq_str =        [0x00, 0x25, 0xBC, 0x00], '24150MHz'
    elif VCOfreq == 24200:
        R0, VCOfreq_str =        [0x00, 0x25, 0xD0, 0x00], '24200MHz'
    elif VCOfreq == 24250:
        R0, VCOfreq_str =        [0x00, 0x25, 0xE4, 0x00], '24250MHz'
    elif VCOfreq == 24300:
        R0, VCOfreq_str =        [0x00, 0x25, 0xF8, 0x00], '24300MHz'
    elif VCOfreq == 24350:
        R0, VCOfreq_str =        [0x00, 0x26, 0x0C, 0x00], '24350MHz'
    elif VCOfreq == 24400:
        R0, VCOfreq_str =        [0x00, 0x26, 0x20, 0x00], '24400MHz'
    elif VCOfreq == 24450:
        R0, VCOfreq_str =        [0x00, 0x26, 0x34, 0x00], '24450MHz'
    elif VCOfreq == 24500:
        R0, VCOfreq_str =        [0x00, 0x26, 0x48, 0x00], '24500MHz'
    else:
        raise ValueError('Transmitter frequency value not valid.')
    
    VCOfreq += VCOfreq_step # Update VCOfreq value for the next loop cycle.

    R1 =        [0x00, 0x00, 0x00, 0x01]

    R2 =        [0x02, 0x10, 0xFF, 0xFA] # Rcounter=1, Rdoubler=ON, Ref/2=OFF (default)
    # R2 =        [0x02, 0x00, 0xFF, 0xFA] # Rcounter=1, Rdoubler=OFF, Ref/2=OFF
    # R2 =        [0x02, 0x20, 0xFF, 0xFA] # Rcounter=1, Rdoubler=OFF, Ref/2=ON

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

    # ADF4158 power-on
    chipEnable.on()

    # ADF4158 programming
    print('ADF4158 programming started...', end = ' ')
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

    # Block sampling mode
    # The scope stores data in internal buffer memory and then transfer it to the PC via USB.
    # The data is lost when a new run is started in the same segment.
    # For PicoScope 2206B the buffer memory is 32 MS, maximum sampling rate 500 MS/s.
    print('Running block capture...', end = ' ')
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
        print('Saving ChA raw samples to .csv file...', end = ' ')
        samplesFileNameChA = timestamp + "__" + tiltAngle + "__" + antennaHeight_str + "__" + VCOfreq_str + "__ChA.csv"
        completeFileNameChA = os.path.join('./data-acquired/raw-samples',samplesFileNameChA)
        with open(completeFileNameChA,'w') as file:
            writer = csv.writer(file)
            writer.writerows(zip(adc2mVChAMax,timeAxis))
        print('Done.')
        # ChB raw samples
        print('Saving ChB raw samples to .csv file...', end = ' ')
        samplesFileNameChB = timestamp + "__" + tiltAngle + "__" + antennaHeight_str + "__" + VCOfreq_str + "__ChB.csv"
        completeFileNameChB = os.path.join('./data-acquired/raw-samples',samplesFileNameChB)
        with open(completeFileNameChB,'w') as file:
            writer = csv.writer(file)
            writer.writerows(zip(adc2mVChBMax,timeAxis))
        print('Done.')
    
    if REAL_TIME_MEAS == True:
        print('Real time measurements of surface velocity...', end = ' ')
        # FFT computation
        complexSignal_mV = np.add(np.asarray(adc2mVChAMax), 1j*np.asarray(adc2mVChBMax))
        FFT = np.fft.fftshift(np.fft.fft(complexSignal_mV, n = freqBins_FFT)) # FFT of complex signal
        FFT_mV = np.abs(1/(totalSamples)*FFT) # FFT magnitude
        FFT_max = np.amax(FFT_mV)
        FFT_dBV = 20*np.log10(FFT_mV/1000)
        freqAxis = np.fft.fftshift(np.fft.fftfreq(freqBins_FFT)) # freqBins+1
        freqAxis_Hz = freqAxis * SAMPLING_FREQUENCY
        
        # FFT computation: normalized and smoothed 
        FFT_norm = FFT_mV / FFT_max
        FFT_norm_dB = 20*np.log10(FFT_norm)
        FFT_norm_smooth = np.convolve(FFT_norm, np.ones(smoothingBins), 'same') / smoothingBins
        FFT_norm_dB_smooth = 20*np.log10(FFT_norm_smooth)
        FFT_norm_dB_max = np.amax(FFT_norm_dB)
        FFT_norm_dB_smooth_max = np.amax(FFT_norm_dB_smooth)
        peakFreq = freqAxis_Hz[FFT_norm_dB.argmax()] # If two identical maxima, only the first occurrence is shown (negative frequency)
        # Doppler centroid bandwidth
        if np.amax(FFT_dBV) < FFT_THRESHOLD:
            print('WARNING: Target not detected.')
        else:
            freqIndex = 0
            stopIndex = 0
            start_detected = False
            startBand = 0
            stopBand = 0
            centroidDetected = False
            while centroidDetected == False:
                for element in FFT_norm_dB_smooth:
                    if element >= (FFT_norm_dB_smooth_max - BANDWIDTH_THRESHOLD):
                        if start_detected == False:
                            startBand = max(freqAxis_Hz[freqIndex],FREQUENCY_MIN)
                            start_detected = True
                        stopIndex = max(stopIndex,freqIndex)
                        stopBand = freqAxis_Hz[stopIndex]
                    else:
                        start_detected = False
                    freqIndex += 1
                    if startBand < peakFreq and stopBand > peakFreq:
                        centroidDetected = True
                        break
            print('Detected Doppler frequency: {:.1f}'.format(peakFreq) + ' Hz')
            print('Amplitude of this FFT peak (norm.smooth.): {:.1f}'.format(FFT_norm_dB_smooth_max) + ' dB')
            print('Bandwidth threshold: {:.1f}'.format(BANDWIDTH_THRESHOLD) + ' dB')
            print('Bandwidth: {:.1f}'.format(stopBand - startBand) + ' Hz')
            print('Bandwidth starts at {:.1f}'.format(startBand) + ' Hz')
            print('Bandwidth stops at {:.1f}'.format(stopBand) + ' Hz')
    elapsedTime = time.time() - startTime
    print('Acquisition completed. Elapsed time (block acquisition and data management): {:.1f}'.format(elapsedTime) + ' s.')
# Stop the scope
print('Closing the scope...', end = ' ')
# handle = chandle
status["stop"] = ps.ps2000aStop(chandle)
assert_pico_ok(status["stop"])
# Close unitDisconnect the scope
# handle = chandle
status["close"] = ps.ps2000aCloseUnit(chandle)
assert_pico_ok(status["close"])
print('Done.')
print(status)
