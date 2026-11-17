# Survey: POCO F6 Pro (vermeer)

Status: step 0, read-only. This pull request adds no code, no device tree and
no configuration. It records what the device is and what a port would have to
import, so that step 1 can be reviewed before it is written.

## Device

| Field | Value |
| --- | --- |
| Codename | `vermeer` |
| Product | POCO F6 Pro / Redmi K70, model `23113RKC6G` |
| SoC | Qualcomm SM8550, upstream `arch/arm64/boot/dts/qcom/sm8550.dtsi` |
| Vendor fragment | `qcom`, `arch/arm64/configs/qcom.config` |
| SoC fragment | `sm8550`, `arch/arm64/configs/sm8550.config` |
| Boot chain | `android-gki` |
| Stock firmware | HyperOS `3.0.304.0.WNKMIXM`, Android 16 |
| Stock kernel | `5.15.194-android13-8-00019-gf4321180a397-ab15212794` |
| Active slot | `_a` |

## Recovery

- Bootloader: `adb reboot bootloader`, or Volume Down + Power while off.
- A/B device; slot `_b` stays stock and untouched.
- Xiaomi's bootloader rejects `fastboot boot` with "Invalid Boot image Header",
  so a test boot is a write to `boot_a`; the stock image stays ready to write
  back.
- The stock `vbmeta` is unsigned (algorithm NONE, 0-byte authentication block),
  so an unsigned `boot` is accepted. Verified by booting a custom GKI kernel
  and restoring stock afterwards.
- Stock images `boot`, `init_boot`, `vendor_boot`, `dtbo` and `vbmeta` from the
  ROM of record (MD5 `ca1b62a8f5737affc201ae6d4a3c90e9`); SHA-256 recorded in
  the device workspace.

## Boot chain

| Image | Header | Contents |
| --- | --- | --- |
| `boot` | `ANDROID!` v4, header 1584 B | kernel 47,577,600 B at offset 4096, ramdisk 0, empty cmdline |
| `init_boot` | `ANDROID!` v4 | generic ramdisk 2,242,446 B |
| `vendor_boot` | `VNDRBOOT` v4, page 4096 | vendor ramdisk 30,936,699 B, DTB blob 3,885,131 B (8 trees), bootconfig 113 B |

The bootloader selects base tree index 2 (`ro.boot.dtb_idx=2`) and overlays
0, 2, 4, 5 (`ro.boot.dtbo_idx=53`).

## Bootloader tree

Captured from `/sys/firmware/fdt` on a stock boot:

- model `Qualcomm Technologies, Inc. 8550 vermeer`
- root compatible `qcom,kalama-mtp`, `qcom,kalama`, `qcom,mtp`
- boot arguments name the device: `swinfo.fingerprint=vermeer:…`,
  `hwid.project=7`, `hwid.hwid_value=1638400`
- kernel physical range `0xa8010000-0xaab5ffff` (code),
  `0xaae00000-0xab38ffff` (data)

The root compatible is the Qualcomm MTP set, not a Xiaomi one, so the handoff
description has to match the MTP variant and name the board through the
`swinfo`/`hwid` words.

## Reference kernel

- `MiCode/Xiaomi_Kernel_OpenSource`, branch `bsp-vermeer-t-oss` (5.15, Android
  T). This is the reference: the device's own working kernel.
- The vendor tree publishes `arch/arm64/configs/vendor/vermeer_GKI.config`,
  `vermeer_consolidate.config` and `modules.list.msm.vermeer`, but **not**
  `sm8550-vermeer.dts` or `sm8550.dtsi`. The board tree has to be written from
  the captured one, not copied.
- Stock config: 2,045 options, `CONFIG_DRM_MSM`, `CONFIG_CFG80211`,
  `CONFIG_TOUCHSCREEN_GOODIX` and every in-kernel WLAN option absent; display,
  Wi-Fi, touch, audio, charger, sensors and camera are vendor modules.

## What the hardware is

Measured on a stock boot with root, from the live tree, `dmesg` and debugfs:

| Subsystem | Parts and wiring |
| --- | --- |
| Display | panel selected by `msm_drm.dsi_display0=qcom,mdss_dsi_n11_42_02_0a_dsc_cmd`, panel id `N11_PANEL_PA`, 1440×3200 command mode DSC, DSI PHYs `ae95000`/`ae97000` |
| Touch | Goodix GT9916 at i2c `0x5d`, 1440×3200, firmware `goodix_firmware_i2c.bin` |
| Audio | WCD938x codec plus **AW88261** smart amps (`awinic,aw882xx_smartpa`) at i2c `0x34`/`0x35`; WSA884x nodes are disabled; LPASS CDC rx/tx/va/wsa macros and SoundWire |
| Charging | `pmic_glink` client with protection domain `msm/adsp/charger_pd`; `pm8550b` charger and `smb1394` charge pump; fuel gauge on the ADSP; battery `N11_5000mah_120w` |
| Wi-Fi | WCN7850 at PCI `17cb:1107`, vendor driver `kiwi_v2`, firmware `kiwi/amss20.bin`, board data `bd_n11.elf` |
| Modem | `modem.mdt` on `remoteproc-mss`, IPA `ipa_fws.mdt`, `rmnet_ipa3` |
| Sensors | ADSP `sensor_pd`; LSM6DSV, AK7323, QMC6308, TCS3720, SX937X in the registry |
| Camera | OVX8000 wide (slot 0), OV16A1Q front (1), IMX355 ultra (2), GC02M1 macro (3) |
| NFC | NXP NCI over i2c plus an eSE |
| Fingerprint | Goodix under-display over SPI (`xiaomi,vermeer-spi`) |
| Storage / USB | UFS `1d84000.ufshc`; DWC3 `a600000.dwc3` |

## Import plan for step 1

1. **Device tree.** Write `sm8550-xiaomi-vermeer.dts` against this tree's
   `sm8550.dtsi`, from the captured tree, with the panel, amps, codec,
   charger and camera nodes described properly. Leave out what this tree has
   no driver for and list it as a step-3 item.
2. **Drivers.** Two are device-specific: the N11 panel and the AW88261 ASoC
   amplifier. The panel exists in the XEC mainline tree as evidence; the amp
   has no upstream driver and is its own commit with a binding.
3. **Handoff.** `/chosen` carries the `swinfo`/`hwid` boot arguments and a
   `fdt-handoff` node with the MTP compatibles; the fixture is base tree 2
   plus overlays 0/2/4/5.
4. **Registration.** `devices/vermeer/{device,kernel.config}` with
   `CONFIG_BUILTIN_DTB_NAME`, the panel and amp built in as far as they exist.
5. **Boot chain.** Reuse `devices/boot/android-gki`: v4 header, unsigned
   vbmeta, no ramdisk in `boot`.

## Charging

Charging is the first step-3 item, and the reason this survey exists. The
charger is not an apps-kernel driver on this device: policy and fuel gauge run
on the ADSP (`qcom,pmic-glink` → `msm/adsp/charger_pd`), driving `pm8550b`
and `smb1394` over SPMI. Two routes:

- native drivers for `pm8550b` and `smb1394`, in this tree's shape, with the
  vendor source used only as a register reference;
- `pmic_glink` + battery manager + UCSI, which needs the vendor ADSP image and
  therefore cannot be shipped.

The first is the route this tree can take; it is a rework project with its own
design, not an import.

## Feature matrix (reference kernel)

| Feature | Reference |
| --- | --- |
| Display, touch, storage, USB, GPU, Bluetooth, haptics | works |
| Charging, battery | works (vendor stack) |
| Wi-Fi | works (vendor `kiwi_v2`) |
| Audio | works (vendor WCD938x + AW88261) |
| Modem, camera, NFC, GPS, sensors, FDE | works (vendor stacks) |
| Suspend | works |

## Open questions

- The board DTS is not published by the vendor tree; step 1 writes it from the
  captured tree. A reviewer may prefer to wait for the XEC mainline port to
  publish its device package.
- Charging: native `pm8550b`/`smb1394` drivers are a project of their own; the
  port should not block on them, but the device cannot be used daily without
  them.
- Which party holds the hardware for steps 1–3.

## Evidence

The captured tree, boot arguments, dmesg excerpts, power-supply inventory and
the decompiled live DTS are kept with the device record, out of this tree as
the add-device skill requires. They will be attached to the device pull
request, together with the `devices/handoff/test` fixtures.
