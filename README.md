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
* Power consumption: no more than 45 mA
* WinUSB compatible - requires no custom drivers on Windows 7 and later
* Unique serial numbers based on the STM32 MCUs' unique IDs
* Open source .Net client (https://github.com/adadurov/PskOnline.X20.DotNet) 
* Prototype enclosure for MH-ET_LIVE and BluePill -- https://grabcad.com/library/mh-et_live-box-max30102-ppg-waveform-sensor-1
* Custom PCB and enclosure (work in progress - to be published as open hardware)

## Hardware Documentation

Schematics: https://easyeda.com/editor#id=0f54b359eb3346139e6319dc807d9f70|301b5fc02a874223bf19331fa95dc1f2

PCB Gerber files: see the ./hardware folder.

## Using VID, PID and WinUSB compatible GUID with derived products

VID & PID used in this product are licensed from https://pid.codes team for use with this specific project.

In case you create a work derived from this project and introduce a breaking change in either 
the semantics of the custom USB requests implemeneted in this project or by changing the format for transferring
PPG waveform data, it is your responsibility to obtain unique VID and PID and use a new GUID in the WinUSB 
descriptors of the derived product.


## Data retrieval

The host or the client application should retrieve PPG waveform samples by sending 'Start' command and continuously reading endpoint 0x81 afterwards.
The client should expect the package of at least ```%bytes_per_transfer%```  bytes. Client can find out this value by reading ```x20_capabilities``` structure (see Commands section below).

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

## BOM used for Prototyping

* Bluepill board (https://wiki.stm32duino.com/index.php?title=Blue_Pill) based on STM32F103C8T6 chip or a replacement.
* MAX30102 (https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf) breakout board (https://ru.aliexpress.com/item/TZT-MH-ET-LIVE-MAX30102/32902336311.html?spm=a2g0s.9042311.0.0.3b0033edxrZMlh)
* 800 nm IR pass filter (Purchased from Aliexpress. As an alternative, a regular glass would be fine as well.
https://m.aliexpress.ru/item/4000190603427.html?sku_id=10000000711889116)

The code is also compatible with Olimext STM32-P103 board (https://www.olimex.com/Products/ARM/ST/STM32-P103/) with controllable USB pull-up circuitry.

## Development tips and acknowledgements

First of all, this project uses a 'software' I2C as a workaround of a bug in STM32F103C8T6 that causes sporadic failures of USB functionality if USB and I2C are used simultaneously.

* The code is based on USB CDC (VCP) middleware generated by STM32CubeMX.
* A free USB VID & PID http://pid.codes/1209/5252/ provided by http://pid.codes
* A fix to the USB ISR to fix 'lost packages' issue comes from David Siorpaes_O in this thread: https://community.st.com/s/question/0D50X00009XkXmg/usb-cdc-breaks-on-heavy-load.
* I2C bit-banging implementation is derived from GouMinghao at https://github.com/GouMinghao/STM32_Software_I2C_Master
* Elightments about I2C + USB issues in STM32F103 come from https://community.st.com/s/question/0D50X00009Xki78/stm32f103-usb-stall

### Development environment

* System Workbench for STM32 (Eclipse-based): http://www.openstm32.org
* STLink v.2 (or compatible) module with SWD feature
* USB to Serial Converter based on CH340

Remember to set the target to software based reset (Project -> Debug Congiruations -> Debuggin -> Show generation parameters)


### Filter for basic USB debugging with Microsoft Message Analyzer

Filters:

        UsbSpec.UsbDeviceId == "VID_0x0483&PID_0x5760"
        UsbSpec.UsbDeviceId == "VID_0x1209&PID_0x5252"      
        (UsbSpec.UsbDeviceId == "VID_0x1209&PID_0x5252") or (UsbSpec.UsbDeviceId == "VID_0x0483&PID_0x5760")

Adjust as needed for your own VID and PID.

### Baud rate settings for debugging console

CH340-based bridge
Windows baud rate: 1843200

STM32 baud rate: 1943200 
(a bit higher to help fit into the clock rate margin)

### Connection table for prototyping

Use with Bluepill and MAX30102 breakout board + serial to USB connector + .

#### Power connections
 
| BluePill / breadboard line   | breadboard power line   | 
| ---------------------------- | ----------------------- |
| G / R-1                      | -                       |
| 3.3 / R-3                    | +                       |

#### MAX30102 connections

| BluePill / breadboard line   | MAX30102 / breadboard line   | 
| ---------------------------- | ---------------------------- |
| B10 / R-6                    | SDC / R-28                   |
| B11 / R-5                    | SDA / R-27                   |
| -- / +                       | R-26                         |
| -- / -                       | R-29                         |

#### Serial Port Connection

| Bluepill / breadboard line   | Serial adapter   |
| ---------------------------- | ---------------- |
| TX / A2 / R-14               | RX               |
| RX / A3 / R-13               | TX               |
| A0+GND  / R-16               | GND              |
