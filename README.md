# hydro-radar-controller
BGT24MTR11 synchronized in frequency by ADF4158.
BGT24MTR11 and ADF4158 programming with RaspberryPi (Python) or Arduino.
## Configure your Raspberry Pi from terminal
### 1. Enable SPI interface
1. From Raspberry Pi terminal:
```
sudo raspi-config
```
2. Select "Interface options" and enable SPI interface.
3. Load kernel module:
```
sudo modprobe spi-2835
```
4. Install _spidev_ Python module:
```
sudo apt install python/dev python3/dev
```
```
cd ~
```
```
git clone https://github.com/doceme/py-spidev.git
```
```
cd py-spidev
```
```
make
```
```
sudo make install
```

### 2. Clone this repository
From terminal:
```
git clone https://github.com/giocic2/hydro-radar-controller.git
```
### 3. Execute the script, after made it executable
```
chmod +x ./hydro-radar-controller/Raspberry/hydro-radar-controller.py
python3 ./hydro-radar-controller/Raspberry/hydro-radar-controller.py
```
You can do the same with example scripts.
### 4. (Optional) Update repository discarding local changes, and make the Python script executable again
```
git add --all
git reset --hard
git pull
chmod +x /Raspberry/hydro-radar-controller.py
chmod +x /Raspberry/example_20steps_50MHz_800ms.py
chmod +x /Raspberry/constant-frequency.py
```
## Configure Raspberry to use PicoScope
Follow the instructions on the repository picotech/picosdk-python-wrappers. I have my own forked version, with the code to use PicoScope 2206B.