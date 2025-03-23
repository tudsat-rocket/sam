# Hephaistos Engine Controller Board

The engine board is responsible for igniting and maintaining the combustion of our SRAD hybrid engine.

Hephaistos is the god of vulcanism and fire in greek mythology. 

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
