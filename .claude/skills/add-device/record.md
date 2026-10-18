# <Product name> (<codename>)

Keep this record in the device's workspace, not in the kernel tree.

## Device

- Codename:
- Product:
- SoC, and its upstream dtsi:
- SoC vendor:

## Recovery

- How to reach the bootloader:
- Slot that stays stock:
- Partitions of the boot chain, with backup path, size and SHA-256:

| Partition | Size | SHA-256 | Backup |
| --- | --- | --- | --- |

## Boot chain

- Stock images examined (path, SHA-256):
- Header version; which image carries the kernel, which the ramdisks:
- Where the bootloader takes the device tree from:
- AVB:
- devices/boot adapter:

## Bootloader tree

- Captured or reconstructed; from which boot or partitions, path, SHA-256:
- Root compatible:
- Model:
- Boot argument words that name the device:
- Kernel physical range (/proc/iomem):

## Reference kernel

- Tree and exact base:
- Delta patch, SHA-256:
- .config and loaded modules:

| File | Class | Action |
| --- | --- | --- |

## Device tree nodes left out in step 1

| Node | Needs |
| --- | --- |

## Gates

- Gate 1 (commands, results, image hashes):
- Gate 2 (boot log, uname -r, handoff marker):

## Features

| Feature | Reference kernel | This kernel | Item |
| --- | --- | --- | --- |
| Display | | | |
| Touch | | | |
| Storage | | | |
| USB | | | |
| Wi-Fi | | | |
| Bluetooth | | | |
| Audio | | | |
| Charging | | | |
| Battery | | | |
| Sensors | | | |
| Suspend | | | |
| Modem | | | |
| Camera | | | |

## Fixes

| Item | Evidence | Fix (commit) | Verified |
| --- | --- | --- | --- |
