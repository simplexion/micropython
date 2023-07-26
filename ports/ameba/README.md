MicroPython port to Ameba
==========================

This MicroPython port is for AmebaD platform, details of the platform can be found here  https://www.amebaiot.com/en/amebad/

## Compile and Flash

Prerequisite steps for building the Ameba port:

```shell
git clone <URL>.git micropython
cd micropython
make -C ports/ameba submodules

# for Nix(OS) only
nix-shell lib/ambd_sdk/shell.nix
# otherwise set the toolchain path the following way
ASDK_TOOLCHAIN = $(dirname $(dirname $(which arm-none-eabi-gcc)))

make -C mpy-cross
make -C ports/ameba
make -C ports/ameba flash
make -C ports/ameba term
```
