# catupload

Command-line uploader for a WCH CH32V203 CAT bootloader.

https://github.com/verylowfreq/catbootloader_ch32v20x

It speaks a simple USB Vendor-class bulk protocol to erase, program, verify, and reset the device after flashing.

## Features
- Writes a raw binary to a target address (with optional offset).
- Automatically erases the required flash pages before programming.
- Optional CRC16-CCITT verification after programming.
- Resets the device when done.

## Requirements
- A CH32V20x device running a compatible bootloader that exposes a USB Vendor-class bulk interface.
- USB access for your OS user account, with a driver that allows Vendor-class bulk transfers.


## Usage and example
```sh
catupload --bin PATH --address ADDR [options]
```

```sh
# Program with an offset, then verify
catupload --bin firmware.bin --address 0x08000000 --offset 0x2000 --verify
```

## Build
```sh
cargo build --release
```

## Options
- `--bin PATH` Path to the binary to write.
- `--address ADDR` Base address to write to (hex or decimal).
- `--offset OFFSET` Offset added to the base address (default: 0).
- `--vid VID` USB VID (default: 0xf055).
- `--pid PID` USB PID (default: 0x6585).
- `--verify` Verify with CRC16-CCITT after programming.

## Notes
- Erase is always performed automatically before programming.
- The erase range is expanded to 4 KB page boundaries as needed.
- The tool resets the device after a successful run.
