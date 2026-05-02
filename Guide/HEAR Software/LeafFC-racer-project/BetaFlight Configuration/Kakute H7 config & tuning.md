# Kakute H7 — Configuration & First-Flight Checklist

> **Board:** HBRO/KAKUTEH7 (STM32H743)  
> **Firmware Build:** 2026-04-06 @15:04:14  
> **Configurator:** 2025.12.2 (a2d0f50) — [app.betaflight.com](https://app.betaflight.com/)

---

## 1. Connect

- [ ] Plug USB-C (data cable) into FC
- [ ] Open Betaflight Configurator
- [ ] Select COM port → **Connect**
- [ ] Confirm Setup tab loads and 3D model renders

---

## 2. Firmware Check

- [ ] Go to **CLI** tab → type `version` → Enter
- [ ] Board must read: `HBRO/KAKUTEH7 (STM32H743)`
- [ ] Build date must match: `2026-04-06 @15:04:14`

> **MISMATCH → STOP. Do not proceed. Flash correct firmware first.**

---

## 3. Upload Config

1. Go to **CLI** tab
2. Click **Load from file**
3. Select `kakute_h7_config.txt`
4. Press **Enter** to execute
5. Wait for `save` → board reboots automatically
6. Reconnect

[INSERT SCREENSHOT: CLI — Load from file button]

**Verify:** Run `diff all` in CLI — no unexpected value differences.

---

## 4. Verification

### Receiver

- [ ] **LiPo battery connected** (USB alone won't power Rx)
- [ ] ELRS receiver bound (solid LED)
- [ ] Roll → Channel 1 responds
- [ ] Pitch → Channel 2 responds
- [ ] Throttle → Channel 3 responds
- [ ] Yaw → Channel 4 responds
- [ ] Centers ~1500, full range 1000–2000
- [ ] No jitter at rest

[INSERT SCREENSHOT: Receiver tab — all channels responding]

### Accelerometer

- [ ] Drone on flat, level surface
- [ ] Click **Calibrate Accelerometer** (Setup tab)
- [ ] 3D model shows level (0° roll, 0° pitch)

### Motors

> ⚠️ **PROPS OFF.**

- [ ] Go to **Motors** tab → acknowledge safety checkbox
- [ ] LiPo connected
- [ ] Spin each motor individually (~1050 throttle):

```
      FRONT
  M4 (CCW)  M2 (CW)
       ╲      ╱
        ╲    ╱
         ╲  ╱
         ╱  ╲
        ╱    ╲
       ╱      ╲
  M3 (CW)   M1 (CCW)
      REAR
```

- [ ] M1 rear-right → CCW
- [ ] M2 front-right → CW
- [ ] M3 rear-left → CW
- [ ] M4 front-left → CCW

Wrong direction → use motor direction toggle in Motors tab, or CLI: `dshotprog <motor_number> reverse` then `save`.

[INSERT SCREENSHOT: Motors tab — motor sliders]

### OSD

- [ ] Go to **OSD** tab
- [ ] Layout matches internal standard

### Blackbox

- [ ] SD card inserted and detected
- [ ] SD card formatted / empty
- [ ] Blackbox logging **enabled**
- [ ] Logging device set to `SDCARD`

### Electrical & System

- [ ] Wiring verified against wiring diagram
- [ ] RPi ↔ FC serial communication working
- [ ] Camera feed visible in analog goggles
- [ ] VTX not overheating (touch check after 30s power-on)

### Hardware Integrity & Identification

- [ ] All components secured (screws, mounts tight)
- [ ] No loose wires or connectors
- [ ] Frame intact — no cracks or looseness
- [ ] All components labeled (IP / asset ID)
- [ ] Labels match system records

**Ping / Identification:**

- [ ] Power drone on
- [ ] Verify correct unit responds (LED / telemetry match expected ID)
- [ ] Confirm this is the intended drone before proceeding

---

## ✅ READY FOR FIRST FLIGHT

- [ ] Motors & props correctly installed
- [ ] Electrical connections verified (against diagram)
- [ ] Receiver working (all channels correct)
- [ ] Accelerometer calibrated (level)
- [ ] Motor directions correct
- [ ] OSD verified
- [ ] RPi communication working
- [ ] Camera stream verified (goggles)
- [ ] VTX temperature normal (not overheating)
- [ ] Blackbox storage cleared and ready

---

```
Drone ID: _______________
Date:     _______________
Tech:     _______________  Signature: _______________
```
