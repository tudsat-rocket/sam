Sam
===

Ground station software for the Sting FC.

# Setup

(Assuming rust/rustup is already installed)

```
cargo install --path .
````

# Usage

To run the GUI, run the compiled binary without any arguments.

```
USAGE:
    sam [SUBCOMMAND]

OPTIONS:
    -h, --help       Print help information
    -V, --version    Print version information

SUBCOMMANDS:
    bootloader    Reboot the FC into bootloader
    gui           Launch the main gui [default]
    help          Print this message or the help of the given subcommand(s)
    logcat        Attach to FC and tail logs
    reboot        Reboot the FC
```
