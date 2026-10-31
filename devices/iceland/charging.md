# Pad 4 (iceland) fast charging

Design of 2026-10-18, accepted by the user ("2 then 3": test 7.3 without the
7.2 stack first, then rework it into proper drivers). The 7.2 stack is the
snapshot commit d856c24735f7 on `pad4/working`; Codex brought it up on
2026-10-13 and 2026-10-15.

## Status on 2026-10-31

What the 7.2 stack proved on the tablet:

| Path | Result |
|---|---|
| PD 9 V, buck | 8.9 V x 1.39 A, about 12.4 W. ICL 500 mA, `OPLUS_SET_PDO` (71), then ICL 1.4 A. `OPLUS_CHECK_PD_COMPLETED` (131) always reads 0 |
| PPS | Never run: the adapter offered no PD profiles |
| SuperVOOC, dual SC8547D | Adapter model 0x4B (80 W). 34.4 W for 601 s; 53.9 W for 445 s, then firmware "Overheat" with cells at 30 C and pumps at 59 C; 54.2 W for 4 min then 36.7 W on the 4.40 V taper, 10 min without fault, 53 to 64 % |

Known failure modes: after a failed session the firmware stays at its
500 mA fallback until a replug or reboot; a boot with the charger attached
sometimes hit the GMU startup timeout (seen at 5 V too, cause unknown).

The user asked for about 55 W with 70 C hardware limits.

Perpetua since the design was written:

- Aston's SuperVOOC landed as `sc8517_charger.c` (28170e56fa44): one driver
  with the VOOC protocol inline, buck handoff through the battmgr usb
  supply's `input_current_limit` (0 suspends, -1 restores). Iceland's
  `oplus-svooc` protocol code should be shared with it rather than written
  twice; moving aston onto the shared code is its own step after iceland
  works.
- qcom_battmgr: the OnePlus SM8550 variant (6863c8889702) and the writable
  SM8550 input limit (3c1fe3cc4f6d) cover part of 3.4.2 for SM8550 only;
  BAT/USB/WLS SET results are checked (ac4fb177caca). Section A below
  (request matching through the input limit policy) landed on 2026-10-31.
- The Kaanapali firmware holds every source at 500 mA and has no automatic
  limit, so battmgr ramps the input limit by charger type as stock does
  (A8, A9): a wall charger reporting as an Apple brick (type 9) draws
  3 A at 4.93 V, 14.7 W, where stock stops at 10 W.
- Sections B to F landed on 2026-10-31, built as devices-22: the gen3
  ADC thermistors, `sc8547_charger.c` for both pumps, the WCD939x D+/D-
  mux, the `oplus_svooc.c` protocol library with KUnit tests, and
  `sc8547_svooc.c` (`CONFIG_CHARGER_SC8547_SVOOC`), the auxiliary driver
  on `sc8547_charger.vooc-phy` that runs the session. It reports through
  the MAINS supply `sc8547-svooc` and debugfs `<device>/status`.
- devices-23 on 2026-10-31: SuperVOOC from the 80 W adapter (model 0x4B)
  at 51 to 55 W, both pumps near 3 A, battery 11.2 A, 35 to 38 % in two
  minutes; battery connector 48 C, pump MOSFETs 44 C. Getting there took
  three fixes: the PHY reply no longer waits for a pump ADC sample, the
  factory's tolerance for handshake noise and late replies, and the
  WCD939x keeping D+/D- on the PHY when the firmware repeats the Type-C
  state as VBUS rises (the glitch made the adapter cut VBUS).

The commit split in section 4 still applies.


**Sources read:**
- `pad4/working` (d856c24735f7): one snapshot commit on top of
  `map220v/opd2514-typec-dp`.
- `devices/v7.3` and `v7.3-rc7`.
- The factory DT `out/gpu-touch-bringup-20261013/live-stock.dts`, to check
  wiring.

Line numbers refer to `pad4/working` unless stated otherwise.
`M` = `drivers/power/supply/qcom_battmgr_main.c` (the stack renamed
`qcom_battmgr.c` to this).

## 0. Baseline in devices/v7.3

- **Battmgr variant.** `qcom_battmgr.c` already has the
  `OPLUS_BATTMGR_KAANAPALI` variant (623d1cea1ff9). It is selected by the
  pmic-glink compatible `oplus,kaanapali-pmic-glink`, which is a DT match,
  not a machine check. It provides:
  - OTG through Oplus properties;
  - `OPLUS_CHG_EN` on PLUGIN_IRQ;
  - a negated `BATT_CURR_NOW` (FG1+FG2).
- **Writable ICL.** 1c4b0f51f5eb makes `input_current_limit` writable, but
  for SM8550 only.
- **Missing bindings.** `oplus,kaanapali-pmic-glink` is not in
  `qcom,pmic-glink.yaml`, and there is no `oplus` vendor prefix (only
  `oneplus`).
- **Type-C.** `drivers/usb/typec` is unchanged and `wcd939x-usbss.c` is
  stock. The iceland `wcd_usbss` node (i2c3 @0xe) and the QMP PHY
  `mode-switch` are already present.
- **ADC.** `qcom-spmi-adc5-gen3.c` and its binding are upstream.
  `pmk8850.dtsi` has no ADC node.
- **Charge pumps.** `bq25980_charger.c` is the mainline precedent for a 2:1
  switched-capacitor charger. There is no SC8547 driver and no `southchip`
  vendor prefix.

## 1. What the stack does

### 1.1 Hardware (pad4-charging.dtsi, confirmed against the factory DT)

| Chip / block | Bus / address | Role |
|---|---|---|
| SOCCP battmgr FW (Oplus) | GLINK | Owns the PMIC buck charger, Type-C/PD (UCSI), dual gauge, APSD, ICL votes, PDO/PPS requests and OTG |
| PMK8850 ADC5 **Gen4**, SDAM0 @0x9000 | SPMI0 SID0 | Four 100k-pull-up thermistors on PMIH0108 (SPMI1 SID7): 0x2744 battery connector, 0x2747 USB connector, 0x274b SVOOC MOS, 0x274d subboard. Factory compatible is `qcom,spmi-adc5-gen4` |
| SC8547D main | i2c_hub_0 (0x980000) @0x6a, IRQ tlmm 0 | 2:1 charge pump **plus the built-in AP VOOC PHY** (factory `oplus,use_vooc_phy`, `oplus,use_ufcs_phy`). ID register 0x36 = 0x49 |
| SC8547D slave | i2c12 (0x890000) @0x6a, IRQ tlmm 3 | 2:1 pump only, 350 kHz (`pad4_svooc_power_dual.inc:33`) |
| WCD9395 USBSS | i2c3 @0x0e | Type-C analog switch. Reg 0x404 DNL/DPR bits 5:2 route connector D+/D- to the AP USB PHY (DN/DP, 0x14) or to DN2/DP2 (0x28), which goes to the SC8547D PHY (`typec/mux/pad4_usbss_route.c:10-30`) |

There is no separate SVOOC MCU. The SC8547D PHY carries VOOC frames over
D+/D-. Its registers:
- 0x2e: RX word (low byte data, high byte flags);
- 0x2c: TX;
- 0x31: "predata" auto-reply;
- 0x30: IRQ mask;
- 0x2b: PHY enable.

The AP runs the protocol inside the main pump's threaded IRQ.

### 1.2 Components

1. **ADC driver** (`pad4_adc.c`, `pad4_adc_bus.c`, `pad4_therm*.h`).
   - This is a private copy of the factory Gen4 SDAM0 conversion
     (`pad4_adc_bus.c:19-69`). It hard-codes:
     - SID byte 0x27, meaning "SPMI bus 1, SID 7";
     - ratiometric mode, decimation 340, 200 us settling.
   - Readings are converted with upstream's `adcmap7_100k` table, using
     full scale **65535** (`pad4_therm.h:14-23`).
   - It exposes four IIO temperature channels.
2. **SC8547D driver** (`pad4_sc8547d.c`), linked into qcom_battmgr.ko.
   - State is the globals `pumps[2]` plus `cp_lock`.
   - `pad4_cp_prepare()` (137-191) writes the factory table: 2:1 mode,
     5 s watchdog, OCP/OVP, ADC, `always_otg_en`.
   - `pad4_cp_start()` (193-203) writes 0x07=0x80.
   - `stop_locked()` (95-117) disables the slave, then the main, and checks
     that switching stopped.
   - `faults()` (213-233) decodes the D-variant fault bits.
   - It samples IBUS/VBUS/VSYS/VBAT/die (235-268).
   - The threaded IRQ (282-303) handles faults and dispatches to the VOOC
     bridges.
   - It also consumes the four thermistors over IIO (351-359) and hands
     them to the other components (`pad4_cp_temperatures`, 51-77).
3. **USBSS route** (`typec/mux/pad4_usbss.inc`, `pad4_usbss_route.c`),
   compiled into `wcd939x-usbss.c`. It provides a lease API
   (`pad4_usbss_acquire/valid/release`).
   - Acquire requires all of the following (`pad4_usbss.inc:38-69`,
     `_route.c:39-49`):
     - USB2/USB3/safe mode;
     - known orientation;
     - no SVID;
     - no audio FSM;
     - not suspended.
   - Any Type-C change, suspend, shutdown or remove revokes the lease and
     restores DN/DP first.
4. **VOOC protocol, pure C** (also host-tested).
   - `pad4_svooc.c`:
     - framing and 10-bit reversed encoding (12-31);
     - adapter authentication by nonce challenge (35-47) and a 4-nibble
       check (87-97);
     - model query;
     - the 6-byte battery "curve", taken from factory
       `svooc_batt_sys_curve` (55).
   - `pad4_svooc_power.c` builds the reply words:
     - current, in 100 mA units;
     - compensated battery voltage, in 10 mV steps from 3404 mV;
     - voltage status (low/normal/high, around 2×VBAT +100..200 mV).
5. **PHY transport** (`pad4_svooc_phy.c`, `pad4_svooc_run_phy.c`).
   - Setup: registers 0x21/0x22/0x33/0x30/0x34, then 0x2b=0x81
     (phy.c:46-81).
   - IRQ: RX, then TX (phy.c:96-124).
   - The power variant (run_phy.c:65-161) adds three steps:
     - installs predata 0x2002 before the first grant (113-119);
     - enables the pump before the "voltage normal" reply;
     - confirms current replies via TXDATA_DONE.
6. **Controller** (`pad4_svooc_control.c`).
   - States: IDENTIFY → VOLTAGE → FACTORY_START → CURRENT_ACK →
     START_READY → RUNNING.
   - Hard limits:
     - 60 W hard envelope (88-101);
     - 55 W regulation (105-134);
     - source maximum 6.2 A;
     - 3.3 A per pump;
     - 13 A battery.
   - Thermal derate/cutoff (47-69):

     | Sensor | Derate | Cutoff |
     |---|---|---|
     | Connectors, MOS | 60 C | 70 C |
     | Board | 45 C | 55 C |
     | Die | 60 C | 70 C |
     | Cell | 38 C | 44 C |

   - SOC entry curve (70-87):

     | SOC | Current |
     |---|---|
     | ≤55 % | 6.2 A |
     | <85 % | 3 A |
     | <90 % | 2 A |

   - Voltage taper at 4.40/4.42/4.44/4.46 V.
   - Timers: 7 s discharge debounce (166-171), 30 s startup, 4 h total.
   - Faults latch until a confirmed detach.
7. **CP bridges** (`.inc` files included at `pad4_sc8547d.c:279-280`).
   - `pad4_svooc_bridge.inc`: diagnostic handshake, no permission granted.
   - `pad4_svooc_power_bridge.inc`:
     - session ownership;
     - 50 ms watchdog (259-274);
     - 3.6 A start OCP 0x05=0x28 (6);
     - pump enable (160-206);
     - IRQ path (408-430).
   - `_dual.inc`: slave setup (5-39). The slave is enabled above 1.4 A
     main current, with sharing qualified within 300-500 ms (45-91).
   - `_sample.inc`: per-pump ADC under freeze (0x11 bit 5) and VBUS
     agreement (84-148).
8. **Buck handoff** (`pad4_svooc_handoff.c`, `pad4_battmgr_svooc_power.inc`).
   - Begin: guard → pumps → route → PHY.
   - Buck suspend happens only after authentication:
     - GLINK write `OPLUS_USB_INPUT_CURRENT_LIMIT = 0`;
     - then three samples ≥50 ms apart with `USB_CURR_NOW ≤ 25 mA`
       (svooc_power.inc:80-123);
     - the ICL readback stays at 2 A, so it proves nothing.
   - Stop: pumps → PHY → route → ICL = U32_MAX, only once VBUS is back at
     5 V (handoff.c:8-45, svooc_power.inc:149-160).
9. **PPS policy** (`pad4_pps_policy.c`, `pad4_battmgr_pps.inc`).
   - Limits: main pump only, ≤2.5 A, ≤9.5 V, 15-35 C, SOC < 70 %
     (`pad4_pps.h:13-16`).
   - The APDO is read from the firmware read-buffer opcode 0x10004
     (`M:1856-1872`).
   - Requests use `OPLUS_USB_SET_PPS_VOLT/CURR`, properties 29/30
     (pps.inc:120-135).
   - The buck is suspended with ICL 0.
10. **Fixed 9 V PD** (`qcom_battmgr_pad4_pd.h`, `M:2060-2220`). It requests
    9 V with `OPLUS_SET_PDO` and caps ICL at 1.4 A. It uses the buck only,
    no pumps.
11. **battmgr core changes** (`M`).
    - Request/response matching:
      - by opcode and property under `rx_lock` (`M:475-511`, callback
        `M:1817-1886`);
      - firmware result codes are not passed back as errno (`M:509`);
      - bat/usb/wls SET results are now checked (`M:1803-1806`).
    - Locking and service loss:
      - `enable_worker` now takes the lock (`M:2282`);
      - PDR-down completes a pending request (`M:2349-2375`).
    - Lifecycle: suspend, resume, remove and shutdown handlers
      (`M:2377-2460`).
    - Sessions: a `pd_generation` counter, bumped by Oplus notifications
      (`M:1366-1412`).
    - Startup:
      - `OPLUS_CHG_EN` for a charger already attached (`M:2222-2270`);
      - a 30 s "GPU bring-up" deferral (`pad4_battmgr_startup_delay.inc`).
    - One `pd_worker` drives every mode, at 100 ms, 1 s or 10 s
      (`M:2060-2220`).

### 1.3 Flow of one SVOOC session

```
SOCCP FW --GLINK--> battmgr pd_worker (100 ms, battmgr->lock): OPLUS USB_ONLINE/
  POWER_ROLE/USB_TYPE, BATT PRESENT/HEALTH/VOLT/CURR, OPLUS CAPACITY/TEMP, USB
  VOLT/CURR, ICL  + 4 thermistors via IIO  -> svooc_observation
 -> handoff_begin: CP prepare (both, 3.6 A OCP) -> USBSS lease (DN2/DP2) -> PHY on
SC8547D IRQ (cp_lock): RX 0x2e -> control_receive -> TX 0x2c (target <=1.5 ms)
  A8 bootstrap, offer(4), auth(12), model(5), curve(13)
  worker after auth: ICL=0 + 3 quiet IBUS samples (buck input suspended)
  offer with grant bit -> adapter raises VBUS -> voltage(2) "normal": enable
  main CP -> current(3) -> TXDATA_DONE -> RUNNING; 50 ms watchdog samples both
  CPs, slave enabled > 1.4 A; battery(1) replies, code 127 asks new current
teardown (fault/detach/Type-C/suspend/PDR): pumps off verified -> PHY off ->
  route released -> ICL=U32_MAX only at 5 V VBUS
```

**Modes.** Module parameters (`M:145-175`) select one mode at a time:

| Mode | Parameters |
|---|---|
| PD 9 V | `pad4_pd_arm` + `pad4_pd9v` |
| PPS | `pad4_pps_enable` + `pad4_pps_limit_ma` |
| VOOC diagnostic | `pad4_svooc_diag`, `pad4_svooc_diag_tail` |
| VOOC power | `pad4_svooc_power_arm`, or `pad4_svooc_auto` + `pad4_svooc_limit_ma` |

`pad4-supervooc.service` sets `pad4_svooc_auto` from userspace.

**What has been measured** (KERNEL_BUILD.md, `out/charging-55w-20261015`):
- 26.5 W automatic, dual pump;
- 34 W for 601 s;
- 53.9 W for 445 s. This run then stopped on firmware "Overheat" health
  while the cells read 27-30 C and the pump dies 59 C.

Sustained 55 W has **not** been validated.

## 2. Hooks into shared code

| Kind | Where | What |
|---|---|---|
| machine check | `wcd939x-usbss.c:740` | `of_machine_is_compatible("oneplus,iceland") && client->addr == 0x0e && of_device_is_compatible(..., "qcom,wcd9395-usbss")` sets the global lease provider |
| machine check | `M:2239` | startup `OPLUS_CHG_EN` only on iceland |
| machine check | `M:2515-2517` | `pd_enabled = variant == OPLUS_BATTMGR_KAANAPALI && of_machine_is_compatible("oneplus,iceland")` |
| machine check | `pad4_battmgr_startup_delay.inc:10` | 30 s deferral |
| machine + address checks | `pad4_sc8547d.c:339-343` | iceland, `addr == 0x6a`, adapter base `0x890000`/`0x980000` |
| machine check | `pad4_adc.c:91-94` | iceland, parent `qcom,pmk8850`, `reg == 0x9000` |
| global DT lookup | `pad4_svooc_bridge.inc:137` | `of_find_compatible_node(NULL, NULL, "qcom,wcd9395-usbss")` |
| .c/.inc in shared driver | `wcd939x-usbss.c:11-12,247` | `#include "pad4_usbss_route.c"`, `#include "pad4_usbss.inc"` |
| .inc in shared driver | `M:22-25,2047-2058` | `pad4_battmgr_pps.inc`, `_svooc.inc`, `_svooc_power.inc`, `_startup_delay.inc` |
| .inc in CP driver | `pad4_sc8547d.c:279-280` | power/diag bridges, which pull in `_dual.inc` and `_sample.inc` |
| module params | `M:145-175` | the 9 `pad4_*` parameters |
| exported symbols | `pad4_usbss.inc:70,87,107` | `EXPORT_SYMBOL_GPL(pad4_usbss_acquire/valid/release)`: qcom_battmgr.ko hard-depends on wcd939x-usbss.ko |
| composite module | `power/supply/Makefile:137-139` | `qcom_battmgr-y := qcom_battmgr_main.o pad4_*.o`; the ADC is built under `CONFIG_BATTERY_QCOM_BATTMGR`; no Kconfig symbols |
| CP registered by battmgr | `M:2608-2625` | `pad4_battmgr_init()` calls `pad4_cp_register()` |
| unconditional change | `wcd939x-usbss.c:577-578` | `mux_set` also re-applies on an SVID change. Likely a real fix |
| all-variant change | `M:475-511,1817-1886,2377-2460` | request matching, errno mapping, PM/remove/shutdown |
| private sysfs | `M:2582-2587`, `pad4_sc8547d.c:305-331` | `pad4_svooc_power`, `pad4_svooc_status`, `pad4_pps_status`, `pad4_pd_source_caps`, `pad4_charge_pump_state` |
| build artifact | `power/supply/qcom_battmgr.mod.c` | committed by mistake |
| wire-enum hazard | `pad4_battmgr_pps.inc:43`, `M:1958` | `OPLUS_USB_TYPE` (a firmware enum, see `pad4_svooc_factory_usb.h`) is compared with mainline `POWER_SUPPLY_USB_TYPE_PD/PD_PPS`. This only works because values ≤8 happen to match |

The same snapshot also carries non-charging changes: `pad4-cpu-cooling.dtsi`,
`capacity-dmips-mhz` and `cpufreq.h max_table_freq`. These belong in separate
topics.

## 3. Proposed architecture

```
 pmic-glink (oplus,kaanapali-pmic-glink)        pmk8850_vadc (qcom,spmi-adc5-gen4)
   qcom_battmgr: bat psy, usb psy                  4 thermistors + thermal zones
        ^ psy props / external_power_changed            | io-channels
        |                                               v
   oplus-svooc (auxiliary drv on SC8547D PHY) <-- southchip,sc8547d main (psy, owns IRQ)
        | mux_state_try_select/deselect        \- southchip,sc8547d secondary (psy)
        v
   wcd939x-usbss: typec mux/switch + mux-state provider for D+/D- (USB | DN2/DP2)
```

### 3.1 ADC: extend upstream, delete pad4_adc

**Driver and binding.** Add `qcom,spmi-adc5-gen4` to
`qcom,spmi-adc5-gen3.yaml`, and a Gen4 data variant to
`qcom-spmi-adc5-gen3.c`. The deltas known so far:
- Ratiometric full scale is 65535, not `RATIO_MAX_ADC7` (16384). This needs
  a per-variant scale in `qcom_vadc7_scale_hw_calib_therm`.
- The SID register carries the SPMI bus bit. Upstream `ADC5_GEN3_SID_MASK`
  is `GENMASK(3,0)`, which would truncate 0x27 to 7.

Before writing the patch, diff the factory Gen4 driver for any further
deltas.

**pmk8850.dtsi.** Add `adc@9000`:
- `reg = <0x9000>, <0x9100>`;
- interrupts 0x90/0x91;
- `#io-channel-cells` and `#thermal-sensor-cells`.

**iceland dts.**
- The four channels, with `qcom,ratiometric` and
  `qcom,hw-settle-time = <200>`.
- The pmih0108 gpio12/gpio15 high-Z pinctrl.
- Four thermal zones.

**Open check.** The firmware may use ADC_TM on the same SDAMs (the factory
sets `qcom,adc-tm-type = <1>` on batt/MOS). HLOS must use SDAM0 only, as the
stack does today.

### 3.2 SC8547D: standalone charge-pump driver

**Files.**
- `drivers/power/supply/sc8547_charger.c`, i2c, `"southchip,sc8547d"`.
- Binding `power/supply/southchip,sc8547.yaml`.
- `southchip` vendor prefix.
- Modelled on `bq25980_charger.c`, using regmap and devm.

**Probe.** Check ID 0x36 == 0x49 and use standard `interrupts`. No machine,
address or adapter checks, and no charging writes.

**Power supply.**
- `STATUS`, `ONLINE` and `HEALTH` (the fault bits).
- VBAT `VOLTAGE_NOW`, VBUS, IBUS `CURRENT_NOW` and die `TEMP`.
- An IIO provider for the raw ADCs is optional.
- The driver owns watchdog servicing and the freeze/read/thaw ADC
  transaction (from `_sample.inc:7-32`).

**DT properties.**
- Protection limits, with units, in the style of
  `ti,sc-ovp-limit-microvolt`;
- `watchdog-timeout-ms`;
- switching frequency;
- `southchip,always-otg-en`.

Main versus secondary is described by these properties, not by
`oneplus,pump-index`.

**Consumer API.** The consumer needs two things:
- a verified stop: disable, then read back switching off, as
  `stop_locked` does;
- atomic ADC snapshots.

That is easier to provide as a few functions exported by this driver than
as plain psy writes. The exports then come from the chip driver, not from a
shared driver.

**VOOC/UFCS PHY.** If the node has a `vooc-phy` subnode (main pump only),
the driver creates an auxiliary device `sc8547_charger.vooc-phy`:
- it shares the regmap and the chip lock;
- it dispatches PHY flag events from the same threaded IRQ.

This keeps the reply path to one context, as `cp_lock` does today.

### 3.3 WCD939x: the D+/D- path as a mux

**Provider.** In `wcd939x-usbss.c`, register a drivers/mux chip with
`#mux-state-cells = <1>`:
- state 0 = USB (DN/DP), the idle state;
- state 1 = DN2/DP2.

**`ops->set(1)`.**
- Enforce today's preconditions (USB2/USB3/safe, orientation, no
  SVID/audio, not suspended); otherwise return `-EBUSY`.
- Keep the verify-and-fallback sequence from `_route.c:10-58`.

**Restore.** Restore state 0 in hardware first on any typec
`switch_set`/`mux_set`, and on suspend, shutdown and remove.

**Separate fix.** Upstream the SVID-compare change (`wcd939x-usbss.c:577-578`)
as its own patch.

**Consumer.** The PHY node uses `mux-states = <&wcd_usbss 1>` with
`mux_state_try_select()` and `mux_state_deselect()`. This replaces the lease
API and its exports.

**Revocation.** The mux core cannot revoke a selection, so the consumer must
learn about Type-C changes another way:
- **(a)** Make the PHY an extra `mode-switch`/`orientation-switch` on the
  connector graph, so pmic_glink_altmode delivers the events to it too. This
  is the third device, which is `TYPEC_MUX_MAX_DEVS`.
- **(b)** Treat any `power_supply_changed` on the usb psy as a session stop.

Either way, stopping the pumps through the 50 ms watchdog is unchanged.

### 3.4 qcom_battmgr: cooperation without board checks

Scope everything to the existing DT-selected `OPLUS_BATTMGR_KAANAPALI`
variant, or keep it generic. Battmgr learns nothing about pumps, VOOC or PPS.

**1. Generic fixes (all variants, upstreamable on their own).**
- Match responses to the pending opcode/property and drop unsolicited ones.
- Map firmware result codes to `-EREMOTEIO`.
- Check bat/usb/wls SET results.
- Take the lock in `enable_worker`.
- Complete the pending request on PDR down.
- Cancel work on remove and shutdown.

**2. OPLUS variant mapped onto standard psy properties.** Consumers then stop
issuing raw property reads.
- `USB ONLINE` = `OPLUS_USB_ONLINE && POWER_ROLE == sink && !otg`.
- `USB_TYPE` goes through an explicit firmware-to-mainline table; unknown
  values report `UNKNOWN`.
- `INPUT_CURRENT_LIMIT` read/write maps to `OPLUS_USB_INPUT_CURRENT_LIMIT`,
  extending 1c4b0f51:
  - 0 = input-suspend vote;
  - -1 = remove the AP vote (factory `oplus_chg_8350_input_suspend`).
- `VOLTAGE_NOW` and `CURRENT_NOW` use the standard USB opcodes.
- Battery `CAPACITY` and `TEMP` come from `OPLUS_CAPACITY` and
  `OPLUS_BATTERY_TEMP`, but only if the standard ones turn out wrong on
  this firmware.

**3. Events.**
- Call `power_supply_changed(usb_psy)` on TYPEC_STATE_CHANGE, PLUGIN_IRQ,
  APSD_DONE, PD_SOURCECAP_DONE, OTG, and service up/down.
- Consumers keep their own session generation in `external_power_changed`.
  This replaces `pd_generation`.
- The rule "0x55 is not a detach" (`M:1382-1391`) moves into the consumer.

**4. Startup.** Keep the `OPLUS_CHG_EN` reconcile (`M:2222-2270`) for this
variant, without the machine check. Drop the 30 s deferral.

**5. Later: PPS.** Mirror tcpm's PPS psy ABI:
- `ONLINE` = 2 (`TCPM_PSY_PPS_ONLINE`);
- writable `VOLTAGE_NOW`/`CURRENT_NOW` mapped to `OPLUS_USB_SET_PPS_*`.

For source caps, prefer the typec PD class: check whether UCSI GET_PDOS
works here before keeping the 0x10004 opcode, and keep it inside battmgr.
Fixed 9 V has no ABI precedent and gives only 13.5 W. Leave it out.

**6. How consumers find battmgr's psys.** All three share the pmic-glink
fwnode, so `power_supply_get_by_reference()` is ambiguous.
- **(a)** `power-supplies = <&pmic_glink>`, then `power_supply_for_each_psy()`
  matching fwnode + `desc->type`. This needs no battmgr change.
- **(b)** Battmgr registers the usb psy on `connector@0`. This needs
  maintainer agreement.

Never look psys up by name.

### 3.5 The proprietary part: oplus-svooc

**Driver.** As built: the protocol in `oplus_svooc.c` (`CONFIG_OPLUS_SVOOC`,
KUnit in `oplus_svooc_test.c`) and the session in `sc8547_svooc.c`
(`CONFIG_CHARGER_SC8547_SVOOC`), an auxiliary driver on
`sc8547_charger.vooc-phy`. The file split below was the plan.

**File split.**

| File | Contents | Today |
|---|---|---|
| `oplus_svooc_proto.c` | wire format and authentication, with KUnit tests in place of `PAD4_HOST_TEST` | `pad4_svooc.c`, `pad4_svooc_power.c` |
| `oplus_svooc_control.c` | session state machine and limits | `pad4_svooc_control.c` |
| `oplus_svooc_charger.c` | PHY transport, run IRQ, buck handoff, 50 ms watchdog, dual-pump qualification, debugfs | replaces all `.inc` files |

**PHY DT subnode properties.**
- `oplus,battery-curve` (today the constant at `pad4_svooc.c:55`);
- `oplus,circuit-resistance` (factory `svooc_circuit_r_l/h`);
- `mux-states`;
- `io-channels`/`io-channel-names` for the thermistors;
- `power-supplies = <&pmic_glink>`;
- `oplus,secondary-charge-pump`;
- `monitored-battery` (simple-battery: max voltage, max current,
  temperature ranges).

**Limits.**
- Board limits that simple-battery cannot express go in DT, with units:
  connector/MOS/board temperatures, source maximum, power target.
- Fixed protocol-safety rules stay in code:
  - TX ACK semantics, freshness and generation checks;
  - pump-off is verified before route/buck release;
  - faults latch until a detach of at least 3 s.

**Opt-in without module params.** The driver registers a psy with
`charge_types` Standard (default) / Fast. This replaces `pad4_svooc_auto`,
`power_arm` and `pad4-supervooc.service`. Diagnostic mode and the status
dumps move to debugfs.

**Other notes.**
- The code derives from GPL-2.0 Oplus sources (`oplus_ap_voocphy.c`,
  `oplus_hal_sc8547d.c`). Keep the attribution.
- Out of scope: UFCS (on the same PHY), VOOC2/3 and reverse charging.
- A PPS direct-charge policy, if wanted later, should be separate. It would
  reuse the CP driver and the ABI from 3.4.5.

### 3.6 iceland DT

- `&pmk8850_vadc` channels and thermal zones, plus the pmih0108 pinctrl.
- `&i2c_hub_0` (400 kHz), main pump:
  - `charge-pump@6a`, `"southchip,sc8547d"`;
  - `interrupts-extended = <&tlmm 0 IRQ_TYPE_EDGE_FALLING>` with pull-up
    pinctrl;
  - factory limits;
  - a `vooc-phy` subnode.
- `&i2c12` (400 kHz), secondary pump: `charge-pump@6a`,
  `<&tlmm 3 IRQ_TYPE_EDGE_FALLING>`, secondary frequency.
- `&wcd_usbss`: `#mux-state-cells = <1>`. With 3.3(a), also a port to the
  connector.
- `pmic-glink`: bind `oplus,kaanapali-pmic-glink`, adding the vendor prefix,
  or rename it to `oneplus,` and update battmgr's match.
- A `battery` simple-battery node.

## 4. Commit split

**Rule:** standard charging must be unchanged after every commit until the
final DT and opt-in steps.

**A. battmgr**
1. dt-bindings: add the `oplus` prefix and `oplus,kaanapali-pmic-glink`.
2. qcom_battmgr: match responses to the outstanding request (rx_lock,
   enable_worker lock).
3. qcom_battmgr: report firmware SET result codes.
4. qcom_battmgr: fail the pending request when the service goes down.
5. qcom_battmgr: oplus: enable charging for an already-attached charger at
   startup.
6. qcom_battmgr: oplus: translate USB type/online; add a writable input
   limit.
7. qcom_battmgr: oplus: notify the usb supply on Type-C and plug events.
8. qcom_battmgr: oplus: set the input limit by charger type (stock
   steps and VBUS checks; 0 and -1 stay the host's suspend vote).
9. qcom_battmgr: oplus: let wall chargers ramp to 3 A.

**B. ADC**
8. dt-bindings: adc5-gen3: add Gen4.
9. iio: adc5-gen3: Gen4 full scale and bus SID.
10. dts: pmk8850: add the ADC.
11. dts: iceland: thermistors and thermal zones.

**C. SC8547D**
12. vendor-prefixes: `southchip`.
13. dt-bindings: SC8547D.
14. power: supply: SC8547D driver, monitor-only (ID, faults, ADC, IRQ, psy;
    never enables switching).
15. sc8547: verified enable/disable and protection setup.
16. sc8547: VOOC PHY auxiliary device.

**D. USBSS**
17. wcd939x: re-apply when only the SVID changes.
18. dt-bindings: wcd939x-usbss: D+/D- mux states.
19. wcd939x: D+/D- mux provider, restoring USB on Type-C/PM/remove.

**E. SuperVOOC**
20. dt-bindings: Oplus SuperVOOC PHY.
21. oplus-svooc: protocol library + KUnit.
22. oplus-svooc: identification-only handshake (debugfs).
23. oplus-svooc: single-pump session with buck handoff (≤3.3 A from DT).
24. oplus-svooc: dual-pump sharing, 55 W regulation, thermal/voltage taper.
25. oplus-svooc: Standard/Fast `charge_types` opt-in.

**F. DT enablement**
26. dts: iceland: SC8547D pumps and SuperVOOC PHY. The default stays
    Standard.

**G. Later, optional:** battmgr PPS ABI, then a PPS policy. Fixed 9 V only
if it can be justified.

**Dropped:**
- `pad4_adc*` and every `.inc` file;
- all module params and `pad4_*` sysfs;
- the 30 s deferral;
- `qcom_battmgr.mod.c` and the composite `qcom_battmgr-y`;
- the `pad4_usbss_*` exports.

## 5. Risks and hardware validation

**Ground rule.** Until step 26 *and* an explicit write of `Fast`, the Pad 4
must charge exactly as `devices/v7.3` does now. Gate every series on a
standard-charging regression run:
- SDP, DCP and PD;
- OTG;
- connected boot;
- the 500 mA fallback;
- suspend while on charge.

The A-series touches every battmgr variant, and Moto shares `devices/v7.3`
(KERNEL_BUILD.md:254), so run that regression on the Moto as well.

**Thermal and current limits.** 55 W sustained is unproven: the 53.9 W run
stopped on firmware Overheat at 445 s, with dies at 59 C. Ramp in stages:
1. single pump at 3 A;
2. dual pump at 4 A;
3. 6.2 A, with a 30 min soak at low SOC.

Log connector, MOS, board and die temperatures against the derate/cutoff
points. Keep the per-pump hardware OCP at 3.6 A (factory: 5.4/5.7 A) and the
60 W fault envelope. Never bypass the firmware health guard.

**ADC rework.** The new Gen4 scale and SID must reproduce the old driver's
readings on all four channels, at ambient and warm. A wrong full scale
produces plausible but wrong temperatures. Keep `-ERANGE` on open or shorted
sensors.

**Reply latency.** The factory target is about 1.5 ms. Regmap, the auxiliary
dispatch and the mux all add overhead. Compare `worst_reply_us`,
`worst_irq_wait_us` and `worst_sample_us` with the stack's figures before
allowing any power grant.

**Buck vote.** Writing ICL=0 is a vote, and the readback stays at 2 A. Keep
the 3-sample physical confirmation. Releasing the vote (-1) must never
override a lower thermal or APSD vote.

**Route ownership.**
- DP alt mode, an audio accessory or an orientation flip mid-session must
  restore D+/D- and stop the pumps within 50 ms.
- USB data must re-enumerate after every session.
- Pull the cable in every session state.

**Firmware interplay.**
- PLUGIN_IRQ during a PDO change, and 0x55 events that are not detaches.
- A SOCCP/PDR restart mid-session must still stop the pumps.
- A connected boot still needs one reconnect.
- Check that a charging current reads positive through the psy, given the
  OPLUS variant's negation.

**PM and teardown.**
- Suspend with an active session must stop cleanly or refuse (the stack
  returns `-EBUSY`).
- Test shutdown and reboot while charging.
- Unbind order must follow the device links: VOOC → CP, mux, battmgr.

**Fault handling.** Faults must clear only after a confirmed detach of at
least 3 s. Ambiguous I2C ACKs must still count as ownership taken before the
I/O.

**Upstream review.** Expect pushback on four points:
- policy limits in DT: keep them to board facts and simple-battery;
- a kernel-side charging policy: VOOC's millisecond reply protocol justifies
  it, PPS less so;
- the battmgr fwnode change (3.4.6b);
- a third Type-C mux device (3.3a).
