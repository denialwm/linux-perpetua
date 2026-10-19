# linux-perpetua

Perpetua is mainline Linux for phones and tablets that ship with an Android
bootloader. It follows the latest Linux release and carries what a small set
of devices needs to run it: their device trees, the drivers of their
components and the fixes they need. The goal is to keep these devices working
on every new release with as little work per release as possible. Perpetua is
part of the [Denial](https://github.com/denialwm) project.

Each release has its own branch, `perpetua/v<version>`: the upstream tag
`v<version>` with Perpetua's commits on top. The upstream README is
[README](../README).

## Devices

| Codename  | Device                      | SoC                         |
|-----------|-----------------------------|-----------------------------|
| `iceland` | OnePlus Pad 4               | Qualcomm Kaanapali (SM8850) |
| `roadstr` | Motorola Edge 70            | Qualcomm Eliza              |
| `aston`   | OnePlus Ace 3 / OnePlus 12R | Qualcomm SM8550             |

Every device has its own build: its own kernel image, with its own device tree
built in, and its own configuration. `devices/<codename>/` holds each device,
and [devices/README](../devices/README) describes the layout.

## Building

Perpetua builds like any arm64 kernel, with LLVM or a GCC cross compiler
([requirements](../Documentation/process/changes.rst)). For the OnePlus Pad 4:

```sh
git clone -b perpetua/v7.3 https://github.com/denialwm/linux-perpetua.git
cd linux-perpetua
ARCH=arm64 LLVM=1 devices/config iceland ../build-iceland
make ARCH=arm64 LLVM=1 O=../build-iceland Image modules
make ARCH=arm64 LLVM=1 O=../build-iceland \
     INSTALL_MOD_PATH="$PWD/../modules-iceland" modules_install
```

`devices/config` merges defconfig with the device's configuration fragments,
and fails when Kconfig drops an option that one of them asks for. The kernel is
`../build-iceland/arch/arm64/boot/Image`, with the device tree built in.

## Booting

Perpetua leaves the bootloader alone. It must be unlocked, and it keeps loading
its own boot images and passing its own device tree. The kernel recognises that
tree from the `/chosen/fdt-handoff` node of its built-in one, carries over what
only the bootloader knows (the memory map, the firmware reservations, the
ramdisk and the board addresses) and boots the built-in tree instead
(`ARM64_FDT_HANDOFF`,
[fdt-handoff.yaml](../Documentation/devicetree/bindings/arm/fdt-handoff.yaml)).
On a running system, `/sys/firmware/devicetree/base/chosen/linux,fdt-handoff`
shows that it did.

The devices use the GKI boot chain of Android 13 and later: the boot partition
holds the kernel and init_boot holds the ramdisk. `devices/boot/android-gki`
writes either image from the device's stock one, keeping its header and AVB
properties. It needs `avbtool` and `unpack_bootimg`:

```sh
devices/boot/android-gki boot stock/boot.img \
    ../build-iceland/arch/arm64/boot/Image boot.img
```

Perpetua provides neither the ramdisk nor the root filesystem. Each device's
configuration builds in what the ramdisk needs before the root filesystem is
mounted; the modules go on the root filesystem.

## Flashing

From fastboot, writing the boot image replaces the kernel of the active slot:

```sh
fastboot flash boot boot.img
```

Keep the stock `boot.img` and `init_boot.img`: the bootloader is untouched, so
writing them back with fastboot brings the stock kernel back. Write one
partition at a time. When the ramdisk changes too, write init_boot first and
boot it with the stock kernel, then write boot. Never change the active slot,
and never write vbmeta, dtbo, vendor_boot or any bootloader or firmware
partition.

## Adding a device

A device is added in three steps: import only what this tree is missing for
it, boot it as is, then fix only what that boot shows broken. Fixes from the
device's own kernel are not imported because they exist there.
[devices/README](../devices/README) summarises the procedure, and
[the add-device skill](../.claude/skills/add-device/SKILL.md) describes it in
full, with its checks.
