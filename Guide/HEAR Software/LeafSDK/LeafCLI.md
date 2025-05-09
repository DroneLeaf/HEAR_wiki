# 🛰️ LeafCLI Full Operator Session

## 📋 Requirements

- `leafcli` installed and available in terminal
- Mission Computer connected to the same network as your computer
- MAVLink Router running and forwarding packets properly
- LeafFC drone powered on, ready, and sending heartbeats

---

# 🛫 Step 1: Build a New Mission

Create a new mission interactively using the wizard:

```bash
leafcli wizard saved_missions/inspection_mission.json
```

✅ Wizard asks you for:
- Mission name
- Takeoff altitude
- Waypoints (latitude, longitude, altitude, speed)
- Repeat until you finish

Result: a properly formatted `inspection_mission.json` file.

---

# 🛡️ Step 2: Validate the Mission

Always validate the mission before uploading:

```bash
leafcli validate saved_missions/inspection_mission.json
```

✅ Checks for:
- At least one waypoint
- Positive altitudes
- Correct format

If validation fails, fix the file and repeat.

---

# 📤 Step 3: Upload the Mission to Drone

Upload the validated mission to the LeafFC via MAVLink:

```bash
leafcli upload saved_missions/inspection_mission.json --conn udp:192.168.1.10:14550
```

✅ Uploads all waypoints into LeafFC memory  
✅ Verifies successful transfer

---

# 🔎 Step 4: Start the Mission with Pre-Flight Checks

Before the mission starts, **LeafSDK** automatically checks:

- Battery percentage (> 30%)
- GPS Fix type (>= 3D Fix)

Start the mission safely:

```bash
leafcli start --conn udp:192.168.1.10:14550
```

✅ Only starts if pre-flight checks pass  
✅ Logs results clearly (colorized)

---

# 📡 Step 5: Monitor the Mission Progress

View live mission telemetry (waypoints and battery percentage):

```bash
leafcli monitor --conn udp:192.168.1.10:14550
```

✅ Updates:
- Current mission waypoint
- Battery remaining
- Flight status heartbeat

To stop monitoring, press `Ctrl+C`.

---

# 🛬 Step 6: Abort Mission (If Needed)

If the mission must be aborted due to emergency or change of plans:

```bash
leafcli abort --conn udp:192.168.1.10:14550
```

✅ Drone safely returns to launch/home point  
✅ Abort command sent over MAVLink immediately

---

# ✅ Full Command Summary

| Purpose | Command |
|:--------|:--------|
| Create new mission | `leafcli wizard saved_missions/mission.json` |
| Validate mission | `leafcli validate saved_missions/mission.json` |
| Upload mission | `leafcli upload saved_missions/mission.json --conn udp:<drone-ip>:14550` |
| Start mission (with checks) | `leafcli start --conn udp:<drone-ip>:14550` |
| Monitor mission | `leafcli monitor --conn udp:<drone-ip>:14550` |
| Abort mission | `leafcli abort --conn udp:<drone-ip>:14550` |

---

# 📦 Bonus: How to Make it Even Nicer (Optional Later)

- Save all `leafcli` logs automatically into timestamped files
- Add colored progress bars for upload/start
- Add mission playback simulator after flight (for data analysis)

---
