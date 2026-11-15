# Contributing a device

A device comes as a pull request against the current release branch,
`perpetua/v<version>`. Each release branch is the upstream tag with
Perpetua's commits on top, and it is rebased for every new release, so a pull
request is not merged as it is: its commits are placed into their topics'
places in the series. You stay their author.

## Layout

One folder per device, `devices/<codename>/`, with `device` and
`kernel.config`. The board's device tree is
`arch/arm64/boot/dts/<vendor>/<soc>-<maker>-<codename>.dts`, and its root
compatible goes in the bindings. [devices/README](../devices/README) describes
the layout.

[The add-device skill](../.claude/skills/add-device/SKILL.md) describes the
procedure in full: import only what this tree is missing for the device, boot
it as is, then fix only what that boot shows broken. The device's own kernel
shows what the device needs. It is never a base, and its fixes do not come in
because they exist there.

## Commits

One logical change per commit, in this order:

1. upstream backports, with `git cherry-pick -x`
2. SoC dtsi additions, apart from the board's commits: other devices share
   them
3. bindings, then drivers, each driver with its binding and Kconfig entry
4. the board's device tree, then one commit per feature on top
5. configuration fragments: `arm64: configs: <soc>: ...` for the SoC,
   `devices: <codename>: ...` for the device

Follow the subject conventions of each subsystem; `git log` on the release
branch has examples. Signed-off-by lines are not needed.
`scripts/checkpatch.pl` should pass on each commit.

## What does not go in

- vendor stacks: `drivers/*/vendor` drops, vendor audio, thermal, charger or
  sensor frameworks
- out-of-tree modules
- `of_machine_is_compatible()` in drivers
- textually included code, such as `#include "x.c"` or `.inc` files
- module parameters that enable hardware

Describe the hardware in the device tree instead; `devices/check` enforces
this. When a feature works only through a vendor stack, leave it out and say
so in the pull request: replacing the stack is a project of its own.

## Handoff

The kernel boots its built-in device tree and recognises the bootloader's tree
from the `fdt-handoff` node in `/chosen`
([fdt-handoff.yaml](../Documentation/devicetree/bindings/arm/fdt-handoff.yaml)).
Capture the stock `/sys/firmware/fdt` from a stock Android boot and describe
it with words that name the device, not only its SoC. `devices/handoff/test`
must exit 0 with your device tree and your stock tree, and 2 for every pairing
with another device: your device tree with their stock trees, and theirs with
yours. Attach the stock tree to the pull request.

## The pull request

Its description holds:

- the device, its SoC and the stock firmware version the tree was captured
  from
- the output of `devices/config <codename> OBJDIR`, of the build of
  `Image modules dtbs` (no new warnings) and of
  `devices/check <codename> OBJDIR BASE`, where BASE is the commit before
  your first one
- the `devices/handoff/test` results
- a feature list: display, touch, storage, USB, Wi-Fi, Bluetooth, audio,
  charging, battery, sensors, thermal, suspend and anything particular to the
  device, each marked works, partly works or does not work, with a line on
  what is missing where it partly works
- whether the device charges on this kernel

The pull request holds the kernel only. Userspace (Phonewave packages, boot
image packing, Droidloom) lives in its own repositories and comes as separate
pull requests once the kernel is in.
