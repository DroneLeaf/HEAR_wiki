# 🍃 LeafScript: Containerized Mission Editor for LeafSDK

---

## 🎯 Purpose of LeafScript

| Feature | Purpose |
|:--------|:--------|
| **Mission Scripting** | Allow users to write Python-based high-level mission scripts |
| **Mission Validation** | Validate missions before sending to LeafFC |
| **Mission Simulation (Optional)** | Simulate mission path/conditions before flight |
| **Mission Uploading** | Push missions to LeafSDK running on the Mission Computer |
| **User Friendly** | Use built-in templates, code snippets for common patterns |

---
  
## 🏗️ High-Level Design

### What LeafScript will be:
- A **VSCode Server** or **JupyterLab**-style development environment, wrapped in a Docker container.
- Pre-loaded with:
  - LeafSDK Python package
  - Mission templates
  - Validation tools
  - Simple push-to-mission-computer tools
- Accessible through web browser at `http://localhost:xxxx`

---

## 🐳 Docker Container Structure

```
LeafScript/
│
├── Dockerfile
├── docker-compose.yml
├── scripts/
│   ├── validate_mission.py    # Mission validator
│   ├── upload_mission.py      # Upload script
│   └── examples/
│       ├── sample_mission_1.py
│       ├── sample_mission_2.py
│
├── README.md
└── requirements.txt
```

---

## 📄 Dockerfile Example

Here’s a realistic `Dockerfile` for LeafScript:

```Dockerfile
FROM python:3.10-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install VSCode Server (code-server) for Web IDE
RUN curl -fsSL https://code-server.dev/install.sh | sh

# Set working directory
WORKDIR /workspace

# Copy SDK and scripts
COPY requirements.txt .
COPY scripts/ ./scripts/

# Install LeafSDK and other dependencies
RUN pip install -r requirements.txt

# Expose port
EXPOSE 8080

# Launch VSCode Server
CMD ["code-server", "--bind-addr", "0.0.0.0:8080", "--auth", "none", "/workspace"]
```

---

## 📄 docker-compose.yml Example

If you want simpler launching:

```yaml
version: "3.8"
services:
  leafscript:
    build: .
    ports:
      - "8080:8080"
    volumes:
      - ./workspace:/workspace
    container_name: leafscript_editor
    restart: always
```

---

## 🧩 requirements.txt Example

```text
pymavlink
opencv-python
leafsdk  # Your internally hosted SDK
```

(You can point `leafsdk` to a private PyPI server or install it manually.)

---

## 🧠 Workspace Tools

Inside `/workspace/scripts/` you can have:

### `validate_mission.py`
```python
import json
from leafsdk.mission.mission_planner import MissionPlanner

def validate_mission(file_path):
    with open(file_path, 'r') as f:
        mission_data = json.load(f)
    
    # Example validation checks
    if not mission_data.get('waypoints'):
        print("Mission Error: No waypoints defined!")
    else:
        print("Mission looks OK!")

if __name__ == "__main__":
    import sys
    validate_mission(sys.argv[1])
```

---

### `upload_mission.py`
```python
from leafsdk.connection.mavlink_interface import MAVLinkInterface
from leafsdk.mission.mission_planner import MissionPlanner
import json

def upload_mission(mission_path, connection_str):
    mav = MAVLinkInterface(connection_str)
    mission = MissionPlanner(mav)
    
    with open(mission_path, 'r') as f:
        mission_json = json.load(f)
    
    for wp in mission_json['waypoints']:
        mission.add_waypoint(wp['latitude'], wp['longitude'], wp['altitude'], wp.get('speed'))
    
    mission.upload_mission()
    print("Mission uploaded successfully.")

if __name__ == "__main__":
    import sys
    upload_mission(sys.argv[1], sys.argv[2])
```

---

# 🖥️ User Experience

When a user runs:

```bash
docker-compose up
```

- Go to `http://localhost:8080`
- Open `workspace/scripts/`
- Pick an example mission
- Modify it or create a new mission
- Validate it locally
- Upload it to a running LeafSDK instance!

---

# 🎁 Extra Features (Optional)

| Feature | Tool |
|:--------|:-----|
| **Auto-complete / IntelliSense** | Pre-load VSCode extensions for Python |
| **Mission Simulator** | Fake MAVLink telemetry stream playback (simulating missions) |
| **Telemetry Viewer** | See mission states live in a panel |
| **Template Generator** | CLI to scaffold new missions easily (`leafscript gen mymission`) |

---

# 🛠️ Full Dev Loop (with LeafScript + LeafSDK)

```text
  [User writes LeafScript Mission Code]
            ↓
    [Validate using scripts/]
            ↓
   [Upload to LeafSDK Service]
            ↓
    [LeafSDK uploads mission to LeafFC]
            ↓
         [Fly 🚁]
```

---

# ✅ Final Deliverables for LeafScript

| Component | Status |
|:----------|:-------|
| Docker Container | Yes (Python + VSCode Server) |
| Local Workspace | Yes (`/workspace/scripts/`) |
| Validation Scripts | Yes |
| Upload Scripts | Yes |
| Web Access | Yes (localhost:8080) |

---

# 🌟 Summary

- **LeafSDK** = Flight control SDK + Mission logic
- **LeafScript** = Beautiful containerized IDE to **write**, **validate**, and **upload** missions
- Together, they provide **a full professional-grade ground segment** for your drone!

---

# 🖥️ LeafScript Web IDE: Professional UI Layout

---

## 🌐 General Layout

```
+-------------------------------------------------------------+
| Top Menu Bar: [Open Mission] [New Mission] [Validate] [Upload] [Simulate] |
+-------------------------------------------------------------+
| Sidebar (Explorer)              | Main Editor Area        |
| -------------------------------- | ----------------------- |
| - /scripts/                      | [Open Tabs: mission1.py] |
|   - sample_mission_1.py           |                         |
|   - sample_mission_2.py           | [Code Editor]           |
|   - validate_mission.py           |                         |
|   - upload_mission.py             |                         |
| - /workspace/                    |                         |
|   - saved_missions/               |                         |
|   - templates/                    |                         |
+-------------------------------------------------------------+
| Terminal Area (bottom)           | OUTPUT LOGS             |
| > docker logs, mavlink logs      | Mission Validate Results |
| > mission upload success/fail    | Uploader messages        |
+-------------------------------------------------------------+
```

---

# 📋 Key Panels and What They Do

| Panel | Function |
|:------|:---------|
| **Explorer Sidebar** | Browse mission scripts, templates, validation scripts |
| **Editor Area** | Write mission scripts with syntax highlighting, autocomplete |
| **Terminal Area** | See outputs from validation, upload scripts, docker logs |
| **Top Toolbar Buttons** | Shortcut buttons for New Mission, Validate, Upload, Simulate |

---

# ✨ Toolbar Buttons (Top Menu)

| Button | Action |
|:-------|:-------|
| **New Mission** | Create a new blank mission file from template |
| **Open Mission** | Browse local workspace and open `.py` or `.json` mission files |
| **Validate Mission** | Run `validate_mission.py` against current file |
| **Upload Mission** | Run `upload_mission.py` to push mission to LeafSDK |
| **Simulate Mission (Optional)** | Simulate waypoints and conditions in software |

---

# 📂 Folder Structure in Workspace (inside the container)

```
/workspace/
│
├── scripts/
│   ├── validate_mission.py
│   ├── upload_mission.py
│   └── examples/
│       ├── sample_mission_1.py
│       ├── sample_mission_2.py
│
├── templates/
│   ├── blank_mission.py        # New mission template
│   ├── patrol_mission.py       # Loop example
│   ├── inspection_mission.py   # Normal waypoint mission
│
├── saved_missions/
│   ├── mission_april28.json
│   ├── inspection_building5.json
│
└── README.md
```

---

# 🛠️ UX Example — Developer Flow

Imagine this:

1. Open `http://localhost:8080`
2. See all available missions in sidebar (`examples/`, `saved_missions/`)
3. Click **New Mission** → choose from template → opens in Editor
4. Write waypoints, checkpoints, conditions
5. Click **Validate Mission** → see "Mission OK" in Terminal
6. Click **Upload Mission** → instant push to LeafSDK running on the drone!
7. (Optional) Click **Simulate Mission** → run local simulation of paths

**Everything without leaving the browser!**

---

# 🔥 Add-ons You Can Include Easily

| Feature | Tool | Difficulty |
|:--------|:-----|:-----------|
| Syntax Highlighting | Pre-installed Python extension | Easy |
| JSON/YAML Support | Built-in in VSCode | Easy |
| LeafSDK IntelliSense | Write basic `.pyi` stub files for autocomplete | Medium |
| Auto format code (black, isort) | Pre-install formatter | Easy |
| Run telemetry monitor inside terminal | Small extra script | Easy |
| Simulate Missions (in 2D map) | (future) add lightweight leaflet.js + simple backend | Medium |

---

# 📷 Quick Sketch

If you'd like, I can **draw a simple mockup diagram** of the LeafScript UI too (like a whiteboard wireframe).  
It would look something like this:

```
+-----------------------------------------------------+
|    [ New | Open | Validate | Upload | Simulate ]    |
+-----------------------------------------------------+
|  Explorer Sidebar |          Editor (code)          |
|                   |                                 |
| - scripts/        |  [mission1.py] tab open         |
| - templates/      |                                 |
| - saved_missions/ |                                 |
+-----------------------------------------------------+
| Terminal/Logs Below: Upload Logs, Validation Logs   |
+-----------------------------------------------------+
```

---

# 🚀 LeafScript + LeafSDK Full System View

| Component | Description |
|:----------|:------------|
| **LeafScript Container** | Development, editing, uploading |
| **LeafSDK Service** | Running inside the mission computer (near/inside drone) |
| **LeafFC** | Flight Controller executing missions |

---

# ✅ What You Will Have Ready

| Item | Status |
|:-----|:-------|
| LeafScript container | ✔️ |
| Mission editing UI | ✔️ |
| Local validation + upload scripts | ✔️ |
| Browser based, no install | ✔️ |
| Simulate later if needed | 🚀 |

---

# 📣 Final Thought

👉 **LeafScript** makes LeafSDK **accessible even to junior mission designers**,  
not just Python experts.  
👉 It separates **mission creation** from **low-level SDK internals** beautifully.  
👉 You can even let "non-coders" create basic missions from templates easily!

---
  
# 1️⃣ Simple UI Wireframe (LeafScript Browser View)

Here’s a **quick whiteboard-style wireframe** of what **LeafScript** will look like when accessed at `localhost:8080`:

```
┌────────────────────────────────────────────────────────────────────────────┐
│ Toolbar:  [New Mission] [Open Mission] [Validate] [Upload] [Simulate]       │
├────────────────────────────────────────────────────────────────────────────┤
│ Sidebar (Explorer)          │ Main Editor (Open Tabs)                      │
│                              │                                              │
│ - scripts/                   │ [ sample_mission.py ]                       │
│    - validate_mission.py      │ ┌──────────────────────────────────────┐   │
│    - upload_mission.py        │ │ import leafsdk                        │   │
│    - examples/                │ │                                        │   │
│       - sample_mission.py     │ │ mission.add_waypoint(...               │   │
│ - templates/                  │ │ mission.upload_mission()               │   │
│ - saved_missions/             │ │                                        │   │
│                              │ └──────────────────────────────────────┘   │
├────────────────────────────────────────────────────────────────────────────┤
│ Bottom Panel (Terminal / Output Logs)                                       │
│ > Validation results, upload status, telemetry logs                         │
└────────────────────────────────────────────────────────────────────────────┘
```

✅ Easy navigation  
✅ Quick validation + upload  
✅ Local simulation  
✅ Terminal view for realtime feedback

---

# 2️⃣ Simulate Button Feature: Mission Playback Simulator

### Goal:  
When a user clicks **Simulate**, a **local simulator** replays the mission trajectory on a map **without needing the real drone**.

---

### How It Works

- Launch a **local 2D map** (use `folium` or `leaflet.js`) inside a browser.
- Plot the **mission waypoints**.
- Animate a "drone icon" moving across the waypoints.
- Simulate conditions (like battery falling, conditions triggering).
- Output mission status live (e.g., "Reached Waypoint 2", "Triggering Precision Landing").

---

### Tech Stack

| Component | Choice |
|:----------|:-------|
| Map Engine | `folium` (Python) or `leaflet.js` (HTML/JS) |
| Backend | Simple Flask server OR inline Python script |
| Animation | JavaScript intervals if using leaflet.js |

---

### Quick Simulation Sketch

```
┌────────────────────────────────────────────┐
│               2D MAP View                   │
│ +-----------------------------------------+ │
│ | O (start)    -->  (waypoint 1)           | │
│ |             --> (waypoint 2)             | │
│ |            landing target (flag)         | │
│ +-----------------------------------------+ │
│ Mission Progress: 75%                      │
│ Current Battery: 35%                       │
│ Last Action: Gimbal set pitch -90           │
└────────────────────────────────────────────┘
```

---
  
### Bonus: Simulate Conditional Branching
- If you have missions with **battery failsafes** or **loops**, the simulator could show them too:
  - Ex: "Battery dropped below threshold → returning to checkpoint!"

---
  
# 3️⃣ New Mission Wizard: Step-by-Step Mission Builder

This will allow **even non-programmers** to build missions through **simple questions**, automatically generating the Python file.

---

### Example Wizard Flow:

| Step | User Action | Result |
|:-----|:------------|:-------|
| 1 | Select Mission Type: Patrol / Inspection / Custom | |
| 2 | Set Takeoff Altitude | |
| 3 | Add Waypoints (lat, lon, alt) | |
| 4 | Add Checkpoints (optional) | |
| 5 | Add Actions at Waypoints (e.g., gimbal control, take photo) | |
| 6 | Add Conditions (e.g., battery failsafe) | |
| 7 | Enable Precision Landing? (yes/no) | |
| 8 | Confirm and Generate Mission | Outputs ready-to-run `.py` or `.json` |

---

### Command-line Example (LeafScript CLI Wizard)

```bash
$ leafscript wizard
Welcome to LeafScript Mission Wizard 🚀

> Choose mission type:
[1] Simple Inspection
[2] Area Patrol (Loop)
[3] Custom

> Set takeoff altitude (meters):
30

> Add a waypoint (lat, lon, alt):
25.276987, 55.296249, 30

> Add another waypoint? (y/n):
y

> Add a waypoint (lat, lon, alt):
25.277500, 55.297000, 30

> Add a checkpoint here? (y/n):
y
Checkpoint Name: "inspection_point"

> Add gimbal action at this point? (y/n):
n

> Add a failsafe condition? (y/n):
y
Type: battery_below
Threshold: 20%
Action: return_to_checkpoint ("inspection_point")

> Enable precision landing? (y/n):
y

Generating your mission script... ✅
Saved to /workspace/saved_missions/inspection_mission.py
```

---
  
### Bonus Future Upgrade
- Add a **Web UI Wizard** later (drag and drop waypoints on a map!)
- Generate both **Python code** and **JSON** mission definition

---
  
# 📦 Deliverables You Would Have After This

| Component | Status |
|:----------|:-------|
| LeafScript Web UI | ✅ |
| New Mission Wizard CLI | ✅ |
| Validate / Upload Buttons | ✅ |
| 2D Local Simulator | ✅ (easy to add) |
| Auto-generate Missions | ✅ |

---

# 🎯 Full System Evolution View

```
User --> LeafScript IDE (New Mission Wizard, Editor, Validator, Simulator) --> LeafSDK Python Service --> MAVLink Router --> LeafFC Flight Controller
```

---
  
# 🏆 Final Vision

✅ An **SDK** (LeafSDK) that **controls missions professionally**.  
✅ A **Development Platform** (LeafScript) that **makes building, validating, and uploading missions extremely easy**.  
✅ A **Simulation/Preview Environment** so users **test missions safely** before real flight.  
✅ An **Extendable Ecosystem** ready for *Web UI*, *APIs*, *Telemetry Monitoring*.

---


## ✨ Future Extensions
- Upload missions from CLI (`leafcli upload mymission.json`)
- Mission "record and replay" feature
- Gimbal "tracking a target" mode using vision
- Return-to-checkpoint if battery low
- Live telemetry stream API (`FastAPI` REST server or gRPC)

## 🔮 Future Upgrade Options

If you later want LeafSDK to be even more powerful, here are natural extensions:
  
| Feature | Description |
|:--------|:------------|
| **FastAPI/gRPC Server** | Host a lightweight API on Mission Computer to send/upload new missions remotely |
| **Web UI** | Web Dashboard to define missions through drag-and-drop (send JSON to SDK) |
| **Auto Sync Logs** | After mission ends, automatically sync logs to a ground station |
| **Health Monitoring** | Pre-flight checks and automated reports (GPS fix status, battery health, sensors ready?) |
| **OTA Updates** | Send new firmware or mission software updates remotely |