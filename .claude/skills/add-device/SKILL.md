---
name: add-device
description: Integrate a new phone or tablet into this kernel tree by a strict procedure - import only what the tree is missing for it, boot it as is, then fix only what is proven broken. Use when adding a device, porting a device from another kernel version, fork or vendor tree, or adding a new SoC, SoC vendor or boot chain to devices/.
---

# Add a device

This tree builds one kernel per device from one shared source (devices/README).
A new device usually arrives with a kernel of its own: a vendor tree, a fork,
another mainline version. That kernel is the **reference**: evidence of what
the device needs and of what works on it. It is never a base.

The procedure has three steps, each closed by a gate:

1. **Import** what this tree is missing for the device, and nothing else.
2. **Boot it as is**, with no fixes.
3. **Fix** only what step 2 proves broken, one fix at a time.

Fixes from the reference kernel are not imported because they exist. Many
are not needed here: the shared tree is a different version, already has
the fix, or fixes it differently. A fix enters only when step 3 shows the
problem on this tree.

Keep the device's record from [record.md](record.md) in the device's own
workspace, not in this tree: it holds serial numbers, addresses and backups.
Every step below says what it adds to the record.

## Rules for every step

- Never base work on the reference kernel: do not merge it, rebase onto its
  version or copy its versions of files that this tree also has.
- Prefer upstream. Take a commit from mainline or linux-next with
  `git cherry-pick -x`, then a patch posted for review, and only then the
  reference kernel's code, rewritten for this tree.
- Added code must not contain board checks (`of_machine_is_compatible()` in
  drivers), textually included code (`#include "x.c"`, `.inc` files), module
  parameters that enable hardware, or out-of-tree modules. Describe the
  hardware in the device tree, with its binding. `devices/check` enforces
  this.
- A vendor stack (a vendor charger, sensor or touch framework, a `drivers/*/vendor`
  drop) is never imported. The device runs without it, with the standard
  behaviour, and replacing it with proper drivers is a separate project with
  its own design.
- If the workspace has rules (AGENTS.md, CLAUDE.md, device procedures), they
  apply on top of these, and they win for building, flashing and device
  access.
- Device safety: no partition write without a recovery path in the record.
  Before every write, state the partition, the image, the slot, why it is
  needed and how to recover. Write one partition at a time. Never write
  partitions outside the device's boot chain, never change the active slot
  and never touch the bootloader, vbmeta or firmware partitions.
- Charging: the reference kernel stays the device's working kernel until
  this one charges it, and its images stay bootable. Start a test boot
  only with the battery charged enough for the test and a recovery, and go
  back to the reference kernel to charge.
- The user validates what is on screen. Do not judge rendered output, and do
  not trigger visible or interactive events (notifications, vibration,
  pairing) unless the user asks for that event.
- Stop and ask when a gate fails for a reason the procedure does not cover.

## Step 0: survey (read-only)

Nothing is written to the device and nothing is committed in this step.

1. Identify the device: codename (the lower-case name its vendor uses, which
   becomes `devices/<codename>`), product name, SoC and the name of its
   upstream dtsi (for example `sm8550`), SoC vendor.
2. Establish recovery before anything else: how to reach the bootloader
   (keys, commands), which slot stays stock and untouched, and verified
   backups (size and SHA-256, read back) of every partition of the boot
   chain. No recovery path: stop.
3. Describe the boot chain from the stock images: `unpack_bootimg` and
   `avbtool info_image` on boot, init_boot and vendor_boot. Record the
   header version, which image carries the kernel and which the ramdisks,
   where the device tree comes from and the AVB algorithm. Pick the
   `devices/boot` adapter that writes this chain, or note that it needs a
   new one.
4. Capture the bootloader's own tree: `/sys/firmware/fdt` from a boot in
   which the bootloader passed its own device tree, such as stock Android.
   A kernel booted with an appended or built-in tree shows that tree
   instead, and its capture is not the bootloader's. Record the root
   compatible, the model and the boot arguments, and pick the words that
   name this device rather than its SoC (a project or board identifier).

   When the device has no such boot left, for example when it has run only
   Linux for a while, reconstruct the tree and mark it "reconstructed" in
   the record:
   - Read the slot's vendor_boot and dtbo partitions (read-only) and
     extract the device trees the bootloader picks from (`unpack_bootimg`,
     then split the dtb section).
   - On the running Linux, `/proc/cmdline` and `/proc/bootconfig` hold the
     boot arguments the bootloader generated, and `/sys/firmware/fdt` holds
     its memory map and `/chosen` as it fixed them up in the appended tree.
   - Name the device by words of those boot arguments, which the bootloader
     wrote at run time, and use a `stock-compatible` that every extracted
     tree carries. The description then does not depend on which variant
     the bootloader picks.
   - For `devices/handoff/test`, build the fixture from the variant the
     bootloader picks for this device, with its overlays applied
     (`fdtoverlay`), and its `/memory` and `/chosen` taken from the running
     tree.

   The kernel reads the memory map and reservations from the real tree at
   every boot; the fixture only feeds the host test. The first boot of
   step 2 proves the description.
5. Record the kernel's physical range from `/proc/iomem` ("Kernel code"
   through "Kernel data") on any Linux booted on the device.
6. Pin the reference kernel: its exact base (`git describe`, or the
   Makefile version plus a diff against that upstream tag), its complete
   delta as a patch with its SHA-256, including untracked files, its
   `.config` and its loaded modules. Do not edit or build in its tree.
7. Fill the record's feature matrix for the reference kernel: for each
   feature (display, touch, storage, USB, Wi-Fi, Bluetooth, audio, charging,
   battery, sensors, suspend, modem, camera), whether it works, partly works
   or does not.

## Step 1: import what is missing

Classify every file of the reference delta in the record:

| Class | What | Action in step 1 |
| --- | --- | --- |
| device | its device tree and binding entry; drivers for components this tree has no driver for (panel, touchscreen, ...) | import |
| present | files and drivers this tree already has, at any version | use this tree's |
| shared | changes to files this tree has: drivers, SoC dtsi, core code | do not import; step-3 hints |
| vendor | vendor stacks, out-of-tree modules | do not import |
| glue | appended device trees, boot hacks, defconfig edits | replaced by `devices/` |

Then:

1. **Device tree.** Port the device's DTS onto this tree's SoC dtsi as it
   is. Leave out the nodes that need something this SoC dtsi does not have,
   and list them in the record as step-3 items. Add the root compatible to
   its dt-bindings, and a vendor prefix when the vendor is new.
2. **Missing drivers.** For each driver class "device", take the upstream
   commit if one exists. Otherwise port the reference driver as its own
   commit, adapted to this tree's APIs, with a binding and its Kconfig
   entry. Check that it builds without new warnings (`W=1` for its
   directory).
3. **Handoff.** In the DTS, give `/chosen` the boot arguments and a
   `fdt-handoff` node (Documentation/devicetree/bindings/arm/fdt-handoff.yaml):
   `stock-compatible` for the root compatible, and `stock-model` or
   `stock-bootargs` for what names the device. Add `keep-initrd` when the
   bootloader's ramdisk must be kept, and one `*-address` node per board
   address that the bootloader passes (Bluetooth, Wi-Fi).
4. **Registration.** Add `devices/<codename>/device` and
   `devices/<codename>/kernel.config` (devices/README). The fragment sets
   `CONFIG_BUILTIN_DTB_NAME`, enables the device's own drivers, and builds in
   what its boot userspace needs before the root filesystem. Do not copy the
   reference `.config`. A new SoC adds `arch/arm64/configs/<soc>.config`,
   named after the SoC's dtsi. A new SoC vendor adds
   `arch/arm64/configs/<vendor>.config`, which turns its platform back on
   after mobile.config.
5. **Boot chain.** A chain that no adapter writes gets a new
   `devices/boot/<chain>` with the same contract as `android-gki`: it writes
   the image from the device's stock image, keeps its header and AVB
   properties, verifies the result and prints its size and SHA-256.
6. **Commits.** One logical change per commit, each in its topic's place in
   the series: upstream backports, SoC support, bindings, drivers, the
   device's DTS, its handoff description, configuration fragments, then
   `devices/`. Follow the subject conventions of each subsystem. Keep the
   author of picked commits and note any change in a bracketed line.

**Gate 1.** All of these must pass before anything goes to the device:

- `devices/config <codename> OBJDIR`, then the build of `Image modules dtbs`
  in OBJDIR, with no new warnings.
- `devices/check <codename> OBJDIR BASE`, where BASE is the commit before
  the first one of this device.
- `devices/handoff/test`:
  - with this device's dtb and its captured stock tree, it exits 0;
  - with this device's dtb and each other captured stock tree you have, it
    exits 2;
  - with each other device's dtb and this device's stock tree, it exits 2.
- `scripts/checkpatch.pl` on each new commit.
- When a commit changed files that other devices build, rebuild each of
  them and rerun `devices/check` for it.
- The boot chain's adapter writes the images and its own verification
  passes. Record the image hashes.

## Step 2: boot it as is

Boot the kernel of gate 1 unchanged. Do not fix anything in this step, not
even a known problem: a fix belongs to step 3, where it can be checked
against the device as it boots without it.

1. Install the modules of the new release on the device's root filesystem
   next to the reference kernel's, and keep the reference kernel bootable.
2. Write the images, one partition at a time, stating each write as the
   rules require. When the ramdisk changes too, write it first and boot the
   reference kernel with it; then write the kernel.
3. Boot. Record how far it gets: the console or SSH, `uname -r`, and the
   empty `/sys/firmware/devicetree/base/chosen/linux,fdt-handoff`, which
   shows that the built-in tree was booted.
4. When it does not boot, recover as the record says. Find the cause
   offline (`devices/handoff/test`, `pstore`, `earlycon`) and correct the
   import of step 1. Do not import shared changes to get it booting.

**Gate 2.** The device reaches its userspace on this kernel, the handoff
marker is present, and the record holds the boot log and whether the
device charges (the battery's power supply status and current with a
charger attached).

## Step 3: fix what needs fixing

Fill the feature matrix for this kernel. The work list is every feature
that works on the reference kernel and fails here, plus what the user asks
for. Nothing else goes on it. Charging comes first when it fails: until it
works, every test boot drains the battery.

Take one item at a time:

1. Reproduce it and find its cause on this kernel: logs, register reads,
   traces. Record the evidence.
2. Find the fix, in this order: an upstream commit (`cherry-pick -x`); the
   part of the reference delta that addresses this cause, rewritten for
   this tree as an upstream-shaped commit; a new fix. SoC dtsi additions
   (the nodes left out in step 1) go to the SoC's topic, because every
   device on that SoC shares them.
3. Build, run `devices/check` with BASE, deploy, and confirm the feature
   works. Update the record.
4. When the fix changed code that other devices use, rebuild them, check
   them, and verify each device whose behaviour it can change.

A feature that only a vendor stack provides (fast charging, for example)
becomes a rework project: a design for standalone drivers with bindings,
reviewed with the user before any code.

## Done

The device is integrated when:

- `devices/check <codename> OBJDIR BASE` passes;
- every `devices/handoff/test` case of gate 1 gives its expected result;
- every regression in the feature matrix is fixed or tracked as a named
  item, such as a vendor-stack rework;
- other devices affected by shared changes were rebuilt and verified;
- the record is complete and the commits are in their topics' places.
