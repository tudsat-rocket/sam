Sam
===

Ground station software for the Sting FC, includes both command line tools and GUI.

![](https://raw.githubusercontent.com/tudsat-rocket/sam/develop/gui/assets/screenshot.png)

# Running

To run for debugging purposes (`--release` recommended for simulations):

```
$ cargo run
```

# Installation

(Assuming rust/rustup is already installed)

```
$ cargo install --path gui
````

# Building Web Assembly Version

## Setup

```
$ rustup target add wasm32-unknown-unknown
$ cargo install wasm-pack --features curl/static-curl
```

## Building

```
$ wasm-pack build gui --target no-modules
```

This will store the compiled application in `gui/pkg/`.

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
