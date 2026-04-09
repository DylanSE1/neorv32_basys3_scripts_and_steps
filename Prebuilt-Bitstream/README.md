# Prebuilt Demos — Ready-to-Flash Bitstreams & Binaries

This directory will contain prebuilt bitstreams so users can try the NPU **without installing any build tools**. Just program the FPGA and go

## Contents
- /ECP5U5MG-85F-EVN-Board-Bistreams:
- NPU bitstream
- NPU + Camera Controller Bitstream
These bitstreams are made using the VHDL files shown in the video guides
- More to go

## How to Use (once files are added)

### 1. Flash the bitstream

Program the `.bit` file to your FPGA using the appropriate tool

### 2. Upload the firmware

1. Connect a serial terminal: `gtkterm --port /dev/ttyUSB0 --speed 19200`
2. Enable **Configuration → CR LF Auto** in GTKTerm.
3. Reset the board — you'll see the NEORV32 bootloader menu.
4. Press `u` → **Ctrl+Shift+R** → select the `.exe` file → press `e` to execute.

### 3. Run Programs
1. Build and upload any of the Ada demos OR
2. Write your NPU (+ camera controller) programs
