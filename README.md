# PskOnline X20 USB 2.0 fingertip PPG waveform sensor based on STM32F103C8T6 + MAX30102

## Copyright and license

The software contained in the repository is a part of PskOnline PSK-X20 open source hardware project, which follows the below definition: https://www.oshwa.org/definition/

* The generated portions of the files contained in the repository and other files generated 
by STM32CubeMX tool are copyright STMicroelectronics. These files are licensed under the license 
terms included into each of the files.
* Other files are created by Alexey Adadurov and licensed under MIT license (see the LICENSE file)

## Features

* Obtain PPG waveform samples using MAX30102 sensor
* Transmit PPG waveform samples to the USB host
* Sampling rate: 400 Hz / 18 bits per sample
* FIFO for 1000 4-byte samples
* WinUSB compatible - requires no custom drivers on Windows 7 and later
* Unique serial numbers based on the STM32 MCUs' unique IDs
* Open source .Net client (https://github.com/adadurov/PskOnline.X20.DotNet) 
* Custom PCB and enclosure (work in progress - to be published as open hardware)

## Hardware Documentation

Schematics: https://easyeda.com/editor#id=0f54b359eb3346139e6319dc807d9f70|301b5fc02a874223bf19331fa95dc1f2

PCB Gerber files: see the ./hardware folder.

## Using VID & PID with derived products

When customizing the hardware or the firmware, it is your responsibility to obtain a unique value for VID & PID to use in the products derived from this project. 

## Data retrieval

The host or the client application should retrieve PPG waveform samples by sending 'Start' command and continuously reading endpoint 0x81 afterwards.
The client should expect the package of at least ```%bytes_per_transfer%``` bytes. Client can find out this value by reading ```x20_capabilities``` structure (see Commands section below).

## Commands

The sensor supports several commands that provide the sensor's capabilities information and control its operation.
The commands are sent to the USB Control endpoint (EP 0) as Vendor requests to Interface.

The command codes are defined in ./Inc/psk_x20.h 

### GetCapabilitiesDescriptor

Retrieves the Capabilities Descriptor (a vendor-defined structure -- see ./Inc/psk_x20.h) containing information 
about firmware revision, sampling rate, bits per sample, bytes per waveform transfer etc.

### UsePpg

Switches to PPG waveform mode. In this mode the device transmits PPG waveform data. This is the default mode.

### UseRamp

Switches to the ramp mode. In this mode the device generates and transmits a ramp signal instead of the PPG waveform to assist with testing of the client data retrieval code.

### Start

Clears the internal FIFO and starts continuous transmission of the PPG waveform data (or ramp) on endpoint 0x81.

### Stop

Stops transmission of the PPG waveform data.

## Prototyping Components

* Bluepill board (https://wiki.stm32duino.com/index.php?title=Blue_Pill) based on STM32F103C8T6 chip or a replacement.
* MAX30102 (https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf) breakout board (https://ru.aliexpress.com/item/TZT-MH-ET-LIVE-MAX30102/32902336311.html?spm=a2g0s.9042311.0.0.3b0033edxrZMlh)

## Development tips and acknowledgements

The code is based on USB CDC (VCP) middleware generated by STM32CubeMX with a fix to the USB ISR to fix 'lost packages' issue. The fix comes from 
David Siorpaes_O in this thread: https://community.st.com/s/question/0D50X00009XkXmg/usb-cdc-breaks-on-heavy-load 

### Development environment

* System Workbench for STM32 (Eclipse-based): http://www.openstm32.org
* STLink v.2 (or compatible) module with SWD feature
* USB to Serial Converter based on CH340

Remember to set the target to software based reset (Project -> Debug Congiruations -> Debuggin -> Show generation parameters)


### Filter for basic USB debugging with Microsoft Message Analyzer

Filter: UsbSpec.UsbDeviceId == "VID_0x0483&PID_0x5760"

Adjust as needed for your own VID and PID.

### Baud rate settings for debugging console

CH340-based bridge
Windows baud rate: 1843200

STM32 baud rate: 1943200 
(a bit higher to help fit into the clock rate margin)

### Connection table for prototyping

Use with Bluepill and MAX30102 breakout board + serial to USB connector.

MAX30102 Breakout Board Connections

BluePill | MAX30102
B10 | SDC
B11 | SDA
G   | GND
3.3 | VCC 

Serial port connections

Bluepill | Serial
A3 RX | TX 
A2 TX | RX
G     | GND
