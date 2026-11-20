#!/usr/bin/env sh
# Flash an ELF onto the nRF52840 Dongle through its USB DFU bootloader.
# Usage: ./flash.sh <elf> [serial-port]   (cargo run passes the ELF)
# Put the dongle in DFU mode first: press its RESET button (red LED pulses).
set -eu

elf="$1"
port="${2:-${DONGLE_DFU_PORT:-}}"
out="$(dirname "$elf")"
hex="$out/dongle.hex"
pkg="$out/dongle-dfu.zip"

sysroot="$(rustc --print sysroot)"
objcopy="$sysroot/lib/rustlib/$(rustc -vV | sed -n 's/^host: //p')/bin/rust-objcopy"
# rust-objcopy links against the toolchain's libLLVM.
LD_LIBRARY_PATH="$sysroot/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" "$objcopy" -O ihex "$elf" "$hex"

nrfutil nrf5sdk-tools pkg generate --hw-version 52 --sd-req 0x00 \
    --application "$hex" --application-version 1 "$pkg" >/dev/null

if [ -z "$port" ]; then
    # The Open DFU bootloader enumerates as 1915:521f.
    for dev in /dev/serial/by-id/*Open_DFU_Bootloader*; do
        if [ -e "$dev" ]; then port="$dev"; fi
    done
fi
[ -n "$port" ] || { echo "no DFU bootloader found: press the dongle's RESET button" >&2; exit 1; }

nrfutil nrf5sdk-tools dfu usb-serial -pkg "$pkg" -p "$port"
