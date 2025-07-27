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

The firmware for the IO-board is written in rust, uses [embassy](https://github.com/embassy-rs/embassy) and can be found in the `firmware` folder. 

## Getting started

- clone the repository

- install STLink drivers
  
  - **Linux**: Install via package manager if possible, e.g. `pacman -S stlink`
  
  - **Windows**:
    
    - Download drivers from [ST's website](https://www.st.com/en/development-tools/stsw-link009.html) (requires account)
    - Extract .zip and run `dpinst_amd64.exe` for 64-bit systems, and `dpinst_x86.exe` on 32-bit systems.
    
    After installation, it *may* be necessary to unplug the STLink and plug it back in, or even reboot (or reload udev rules on Linux).

- install **Rustup**
  
  - A tool used to manage different Rust versions and targets (e.g. x86, ARM,
     etc.). Download and run installer: follow the instructions on [https://rustup.rs/](https://rustup.rs/) (Use rustup-init.exe for Windows, defaults are fine)

- **Rust**
  
  - Using `rustup`, we can download the target needed for the STM32 and some other tools we need.
    
    - `rustup target add thumbv7em-none-eabi` (Add ARM toolchain)
    
    - `cargo install probe-rs-tools --locked`
    
    - `cargo install flip-link` (Needed for cargo run)

- **Compiling & Flashing**
  
  - run `cargo run --release`
