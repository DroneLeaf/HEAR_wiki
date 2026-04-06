# Guide for stable serial port naming on Bench / SITL setups

## Introduction

When working with Bench or SITL setups, it is common to have multiple serial devices connected at the same time, such as:

- USB-to-serial adapters (`/dev/ttyUSB*`)
- flight controllers or radios exposing ACM serial ports (`/dev/ttyACM*`)

The default Linux names such as `/dev/ttyUSB0`, `/dev/ttyUSB1`, and `/dev/ttyACM0` are **not stable**. They can change after reboot, after reconnecting devices, or when plugging devices in a different order.

The recommended solution is to create **stable custom aliases** using `udev`, such as:

- `/dev/leaf_usb_a`
- `/dev/leaf_usb_b`
- `/dev/leaf_tx16s`

Your software should then use these stable names instead of the kernel-assigned `ttyUSB*` / `ttyACM*` names.

---

## Important recommendation

Do **not** try to force a device to become the real `/dev/ttyUSB0`.

That is brittle and depends on kernel enumeration order.

Instead, create your own stable symlinks and use those in / Bench / SITL configs.

!!!warning "Warning"
    Do **not** apply such rules on Raspberry Pi CC meant for actual flying drones. Betaflight/Raspberry Pi CC setups should use the built-in stable serial port naming based on physical USB port paths or unique serial numbers, without custom aliases.

---

## Recommended matching strategy

Use one of these methods, in this order of preference:

1. **Unique USB serial number**
   - best when the device has a real unique serial number
2. **Stable USB physical port path**
   - best for identical USB-UART adapters with no unique serial number
3. **Built-in `/dev/serial/by-path/...` or `/dev/serial/by-id/...`**
   - often usable directly without custom aliases

In our tested setup:

- the TX16S ACM serial device worked reliably with:
  - `KERNELS=="1-3.4:1.0"`
- the two CH340 USB serial adapters worked reliably with:
  - `KERNELS=="1-3.3:1.0"`
  - `KERNELS=="1-1:1.0"`

---

## Step 1: Identify current serial devices

List current serial devices:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

Also inspect Linux’s built-in stable serial links:

```bash
ls -l /dev/serial/by-id/
ls -l /dev/serial/by-path/
```

These are very useful because:

- `by-id` helps when a device has a unique serial number
- `by-path` helps distinguish identical devices by physical USB socket

---

## Step 2: Find the parent USB path for each device

For each serial device you want to name, inspect its `KERNELS==` path:

```bash
udevadm info -a -n /dev/ttyUSB1 | grep 'KERNELS=='
udevadm info -a -n /dev/ttyUSB2 | grep 'KERNELS=='
udevadm info -a -n /dev/ttyACM0 | grep 'KERNELS=='
```

Example output from the tested setup:

### CH340 adapter #1

```bash
KERNELS=="ttyUSB1"
KERNELS=="1-3.3:1.0"
KERNELS=="1-3.3"
KERNELS=="1-3"
KERNELS=="usb1"
...
```

### CH340 adapter #2

```bash
KERNELS=="ttyUSB2"
KERNELS=="1-1:1.0"
KERNELS=="1-1"
KERNELS=="usb1"
...
```

### TX16S ACM serial device

```bash
KERNELS=="1-3.4:1.0"
KERNELS=="1-3.4"
KERNELS=="1-3"
KERNELS=="usb1"
...
```

The important values are the USB interface paths:

- `1-3.3:1.0`
- `1-1:1.0`
- `1-3.4:1.0`

These are what we will use in the udev rules.

---

## Step 3: Create the udev rules file

Create a rules file:

```bash
sudo tee /etc/udev/rules.d/99-leaf-serial.rules > /dev/null <<'EOF'
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", KERNELS=="1-3.3:1.0", SYMLINK+="leaf_usb_a"
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", KERNELS=="1-1:1.0",   SYMLINK+="leaf_usb_b"
SUBSYSTEM=="tty", KERNEL=="ttyACM*", KERNELS=="1-3.4:1.0", SYMLINK+="leaf_tx16s"
EOF
```

This creates:

- `/dev/leaf_usb_a` for the CH340 device on USB path `1-3.3:1.0`
- `/dev/leaf_usb_b` for the CH340 device on USB path `1-1:1.0`
- `/dev/leaf_tx16s` for the ACM device on USB path `1-3.4:1.0`

---

## Step 4: Reload udev rules

Run:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

---

## Step 5: Verify the new stable names

Check that the aliases exist:

```bash
ls -l /dev/leaf_*
```

Expected result:

```bash
/dev/leaf_usb_a -> ttyUSB1
/dev/leaf_usb_b -> ttyUSB2
/dev/leaf_tx16s -> ttyACM0
```

The right-hand side may change later, and that is fine.

For example, after reboot it may become:

```bash
/dev/leaf_usb_a -> ttyUSB0
/dev/leaf_usb_b -> ttyUSB3
/dev/leaf_tx16s -> ttyACM1
```

That is not a problem. Your software should always use the stable aliases:

- `/dev/leaf_usb_a`
- `/dev/leaf_usb_b`
- `/dev/leaf_tx16s`

---

## Step 6: Use the stable aliases in HEAR / Bench / SITL

Update configs, launch scripts, or services to use:

```bash
/dev/leaf_usb_a
/dev/leaf_usb_b
/dev/leaf_tx16s
```

Do **not** use raw paths like:

```bash
/dev/ttyUSB0
/dev/ttyUSB1
/dev/ttyUSB2
/dev/ttyACM0
```

---

## Why this worked

An earlier attempt failed because it mixed attributes from different parent levels in the same udev rule.

Udev matching rules must use attributes from the device itself plus at most **one single parent device**.

Using the exact `KERNELS=="..."` USB interface path solved the issue cleanly.

---

## What to do when adding more devices

When you connect additional devices or start using different USB ports, repeat the same process.

### Case A: New device has a unique serial number

If the new device has a unique serial number, you may prefer matching by serial instead of USB path.

Check with:

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial'
```

If the device has a real unique serial, create a rule like:

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{serial}=="00000000001B", SYMLINK+="my_device_name"
```

This is usually better than matching by USB socket.

---

### Case B: New device is identical to an existing USB-UART dongle

If the new adapter is identical to another one and has no unique serial number:

1. plug it into the USB port where it will normally stay
2. inspect its `KERNELS==` path:

```bash
udevadm info -a -n /dev/ttyUSBX | grep 'KERNELS=='
```

3. identify the interface path, for example:

```bash
KERNELS=="1-2:1.0"
```

4. add a new rule:

```udev
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", KERNELS=="1-2:1.0", SYMLINK+="leaf_usb_c"
```

5. reload rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

6. verify:

```bash
ls -l /dev/leaf_*
```

---

### Case C: You move a device to another USB port

If you unplug a device and reconnect it to a different physical USB port, the `KERNELS=="..."` path will change.

That means the alias rule based on the old socket will no longer match.

To fix that:

1. plug the device into the new port
2. check the new path:

```bash
udevadm info -a -n /dev/ttyUSBX | grep 'KERNELS=='
```

3. update the rule file with the new `KERNELS=="..."` value
4. reload rules

So these aliases are **stable by socket location**, not by dongle identity, unless the device has a unique serial.

---

## Recommended naming convention

Use clear names based on the device function, not just the port type.

Examples:

- `/dev/leaf_tx16s`
- `/dev/leaf_fc`
- `/dev/leaf_elrs_rx`
- `/dev/leaf_elrs_tx`
- `/dev/leaf_usb_a`
- `/dev/leaf_usb_b`
- `/dev/leaf_usb_c`

Function-based names are usually better than raw names like `ttyUSB0`.

---

## Troubleshooting

### 1. Rule file exists but no alias appears

Check the file contents:

```bash
sudo cat /etc/udev/rules.d/99-leaf-serial.rules
```

Reload rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

Test a device against udev:

```bash
sudo udevadm test /sys/class/tty/ttyUSB1 2>&1 | grep -Ei '99-leaf|leaf_|symlink'
```

This helps verify whether the rule matches.

---

### 2. Device name changed from `ttyUSB1` to `ttyUSB0`

That is normal.

The whole point of this setup is that your stable alias remains the same even if the kernel number changes.

Use `/dev/leaf_usb_a`, not `/dev/ttyUSB1`.

---

### 3. Two identical devices keep swapping

That usually means:

- they are connected to different sockets than before, or
- you matched by vendor/product only, which is not enough for identical adapters

Fix by matching each device using its exact `KERNELS=="..."` path.

---

### 4. Built-in stable names may already be enough

Linux already created these stable names in the tested setup:

```bash
/dev/serial/by-path/pci-0000:00:14.0-usb-0:3.3:1.0-port0
/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0-port0
/dev/serial/by-path/pci-0000:00:14.0-usb-0:3.4:1.0
```

You can use those directly if you do not want custom aliases.

However, custom names like `/dev/leaf_usb_a` are much easier to read and maintain.

---

## Final tested rule set

For the tested machine, the working rules were:

```udev
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", KERNELS=="1-3.3:1.0", SYMLINK+="leaf_usb_a"
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", KERNELS=="1-1:1.0",   SYMLINK+="leaf_usb_b"
SUBSYSTEM=="tty", KERNEL=="ttyACM*", KERNELS=="1-3.4:1.0", SYMLINK+="leaf_tx16s"
```

Installed with:

```bash
sudo tee /etc/udev/rules.d/99-leaf-serial.rules > /dev/null <<'EOF'
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", KERNELS=="1-3.3:1.0", SYMLINK+="leaf_usb_a"
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", KERNELS=="1-1:1.0",   SYMLINK+="leaf_usb_b"
SUBSYSTEM=="tty", KERNEL=="ttyACM*", KERNELS=="1-3.4:1.0", SYMLINK+="leaf_tx16s"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

Verification:

```bash
ls -l /dev/leaf_*
```

Expected:

```bash
/dev/leaf_usb_a -> ttyUSB1
/dev/leaf_usb_b -> ttyUSB2
/dev/leaf_tx16s -> ttyACM0
```