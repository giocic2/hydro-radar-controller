#!/usr/bin/python
# Code based on the sample https://www.takaitra.com/spi-device-raspberry-pi/

import spidev
import time
from gpiozero import OutputDevice

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
R0 =        [0x00, 0x24, 0xB8, 0x00] # 23.50 GHz
# R0 =        [0x00, 0x24, 0xB8, 0x00] # 23.50 GHz + 10 kHz
# R0 =        [0x00, 0x24, 0xCC, 0x00] # 23.55 GHz
# R0 =        [0x00, 0x24, 0xE0, 0x00] # 23.60 GHz
# R0 =        [0x00, 0x24, 0xF4, 0x00] # 23.65 GHz
# R0 =        [0x00, 0x25, 0x08, 0x00] # 23.70 GHz
# R0 =        [0x00, 0x25, 0x1C, 0x00] # 23.75 GHz
# R0 =        [0x00, 0x25, 0x30, 0x00] # 23.80 GHz
# R0 =        [0x00, 0x25, 0x44, 0x00] # 23.85 GHz
# R0 =        [0x00, 0x25, 0x58, 0x00] # 23.90 GHz
# R0 =        [0x00, 0x25, 0x6C, 0x00] # 23.95 GHz

R0 =        [0x00, 0x25, 0x80, 0x00] # 24.00 GHz: Rcounter=1, Rdoubler=ON, Ref/2=OFF (default)
# R0 =        [0x00, 0x4B, 0x00, 0x00] # 24.00 GHz: Rcounter=1, Rdoubler=OFF, Ref/2=OFF
# R0 =        [0x00, 0x96, 0x00, 0x00] # 24.00 GHz: Rcounter=1, Rdoubler=OFF, Ref/2=ON

# R0 =        [0x00, 0x25, 0x94, 0x00] # 24.05 GHz
# R0 =        [0x00, 0x25, 0xA8, 0x00] # 24.10 GHz
# R0 =        [0x00, 0x25, 0xBC, 0x00] # 24.15 GHz
# R0 =        [0x00, 0x25, 0xD0, 0x00] # 24.20 GHz
# R0 =        [0x00, 0x25, 0xE4, 0x00] # 24.25 GHz
# R0 =        [0x00, 0x25, 0xF8, 0x00] # 24.30 GHz
# R0 =        [0x00, 0x26, 0x0C, 0x00] # 24.35 GHz
# R0 =        [0x00, 0x26, 0x20, 0x00] # 24.40 GHz
# R0 =        [0x00, 0x26, 0x34, 0x00] # 24.45 GHz
# R0 =        [0x00, 0x26, 0x48, 0x00] # 24.50 GHz

# Choose R1:
R1 =        [0x00, 0x00, 0x00, 0x01] # 23.50 + n*0.05 GHz
# R1 =        [0x02, 0x26, 0x00, 0x01] # 23.50 GHz + 10 kHz


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