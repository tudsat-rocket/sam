# I/O module

<img src="assets/io_board_front.jpg" title="" alt="" width="314">

The I/O module is built as a general board for various sensor inputs and outputs. It is run by an STM32F103.

The module features 4 output connectors, with each being usable both as a servo output or 2 switchable outputs, powered by one of a variety of power sources. They also feature a variety of communication ports, allowing the connection to UART, I2C systems as well as the use of the STM32's ADC.

The outputs of the I/O module can be powered straight from the FC battery, a 5v line powered by a linear regulator, an external source like the charge bus or an auxiliary battery, or an onboard, programmable boost converter.

## Components

- STM32F103
- 4x output connector
- 2x switchable outputs
- see schematics for complete list

## Firmware

The frimware for the IO-board is written in rust, uses [embassy](https://github.com/embassy-rs/embassy) and can be found in the `frimware` folder. 


## Getting started

- clone the repository and run cargo build?
- flash the firmware to the module
