# SDK Mission Mode Pause / Resume / Cancel / Manual Control (Mermaid)

<!-- Note: This is early draft that is not correct. Correction would follow in the upcoming commits. -->


_Source diagram: `Guide/HEAR Software/Flight Modes/SDK Mission Mode Pause Resume and Cancel Workflow and System Level Interractions.drawio`_

Each section below mirrors a lane in the original Draw.io diagram. The "Breakpoints" list is written so you can quickly re-create or tweak the Mermaid drawing without re-reading the XML.

---

## Pause & Resume Flow

**Breakpoints**
1. QGC issues `CMD_PAUSE`; SDK verifies whether the current step is pausable or schedules a pause at the next waypoint end.
2. SDK tells the FC to pause the mission; the FC marks the mission state as **PAUSED**.
3. FC computes a smooth-stop trajectory (zero yaw) for waypoint steps, aborts the current trajectory, and clears any queued setpoints (non-waypoint steps may simply halt).
4. When QGC sends `CMD_RESUME`, the SDK grabs the last setpoint from the MAV subscriber, re-computes a trajectory, and resumes the mission at the FC, switching the state back to **RUNNING**.

```mermaid
sequenceDiagram
    autonumber
    participant QGC as QGC / API
    participant SDK as SDK Manager
    participant FC as Flight Controller

    QGC->>SDK: CMD_PAUSE
    Note right of SDK: Validate step is pausable
    SDK->>FC: Pause mission request
    FC-->>FC: Mission state = PAUSED
    FC-->>FC: Generate smooth-stop trajectory (zero yaw)
    Note right of FC: Only waypoint steps get smooth_stop
    FC-->>FC: Abort current trajectory + clear queue

    QGC->>SDK: CMD_RESUME
    SDK-->>SDK: Fetch last setpoint from MAV subscriber
    SDK->>SDK: Re-compute trajectory
    SDK->>FC: Resume mission request
    FC-->>FC: Mission state = RUNNING
```

---

## Cancel Flow

**Breakpoints**
1. Operator sends `CMD_CANCEL`; SDK switches the mission into cancel mode.
2. SDK forwards cancellation parameters (hover, RTL, land-in-place, etc.) plus the desired pausing motion behavior to the FC.
3. FC transitions to **CANCELED**, generates a stopping trajectory, aborts any active trajectory/queue, and executes the requested cancellation action.

```mermaid
sequenceDiagram
    autonumber
    participant QGC as QGC / API
    participant SDK as SDK Manager
    participant FC as Flight Controller

    QGC->>SDK: CMD_CANCEL
    SDK-->>SDK: Cancel mission logic
    SDK->>FC: Cancel request (behaviour = hover / RTL / LIP)
    FC-->>FC: Mission state = CANCELED
    FC-->>FC: Generate stopping trajectory (zero yaw)
    FC-->>FC: Abort current trajectory + clear queue
    FC-->>FC: Execute cancellation action
    Note over SDK,FC: Cancellation behaviour carries the pausing motion parameter
```

---

## Abort Flow

**Breakpoints**
1. `CMD_ABORT` from QGC goes _directly_ to the flight controller; SDK may also raise `CMD_ABORT` toward the FC (for example, due to safety logic).
2. Regardless of the origin, the FC marks the mission state as **ABORTED**, generates a stopping trajectory, aborts any remaining queue, and terminates the mission.

```mermaid
sequenceDiagram
    autonumber
    participant QGC as QGC / API
    participant SDK as SDK Manager
    participant FC as Flight Controller

    QGC->>FC: CMD_ABORT (direct)
    SDK->>FC: CMD_ABORT (SDK-triggered)
    FC-->>FC: Mission state = ABORTED
    FC-->>FC: Generate stopping trajectory (zero yaw)
    FC-->>FC: Abort current trajectory + clear queue
    Note right of FC: Mission is terminated immediately
```

---

## Manual Joystick Control Flow

**Breakpoints**
1. Precondition: mission state is `PAUSED`, `CANCELED`, or `ABORTED`.
2. `CMD_ACTIVATE_JOYSTICK` can be initiated either by QGC or by the SDK; in both cases the command reaches the FC, which switches to **JOYSTICK_CTRL**, verifies stick positions, and starts consuming joystick inputs.
3. `CMD_DEACTIVATE_JOYSTICK` and `CMD_RESUME` can likewise originate from QGC or SDK. After the FC resets the mission state (back to paused/canceled/aborted or running), it explicitly executes a "Deactivate joystick commands" step so no manual inputs linger.

```mermaid
sequenceDiagram
    autonumber
    participant QGC as QGC / API
    participant SDK as SDK Manager
    participant FC as Flight Controller

    Note over FC: Preconditions: state ∈ {PAUSED, CANCELED, ABORTED}
    QGC->>FC: CMD_ACTIVATE_JOYSTICK
    SDK->>FC: CMD_ACTIVATE_JOYSTICK
    FC-->>FC: Mission state = JOYSTICK_CTRL
    FC-->>FC: Verify joystick reference position
    FC-->>FC: Apply joystick commands

    QGC->>FC: CMD_DEACTIVATE_JOYSTICK
    SDK->>FC: CMD_DEACTIVATE_JOYSTICK
    FC-->>FC: Mission state returns to paused/canceled/aborted
    FC-->>FC: Deactivate joystick commands

    QGC->>FC: CMD_RESUME
    SDK->>FC: CMD_RESUME
    FC-->>FC: Mission state = RUNNING
    FC-->>FC: Deactivate joystick commands
    Note over FC: Any mission-state change disables joystick control automatically
```

---

## Mission Reception & Execution Flowchart

This flowchart illustrates the complete lifecycle from mission reception through execution, including all decision points for LeafSDK mode, forced execution, pause/resume, and cancellation behaviors.

```mermaid
flowchart TD
    subgraph MissionReception["📥 Mission Reception"]
        A[Mission Received] --> B{Is LeafSDK Mode?}
        B -->|No| Z[Mission Rejected / Not Handled]
        B -->|Yes| C[Mission Loaded]
        C --> D{Forced Start?}
        
        D -->|No| E[Wait for Idling]
        E --> F[User clicks Start Mission in QGC]
        F --> G[Start Mission]
        
        D -->|Yes| H[Enter Idle State]
        H --> I[Start Mission Automatically]
        I --> G
    end

    subgraph MissionExecution["🚀 Mission Execution"]
        G --> J[Executing Mission Step]
        
        J --> K{Mission State?}
        
        K -->|Running| L[Continue Executing Steps]
        L --> M{More Steps?}
        M -->|Yes| J
        M -->|No| N[✅ Mission Complete]
        
        K -->|Paused| O[⏸️ Wait for Resume]
        O --> P{Resume Received?}
        P -->|Yes| J
        P -->|No| O
        
        K -->|Cancelled| Q[🛑 Stop Mission]
    end

    subgraph CancellationHandling["🔄 Cancellation Mode Handling"]
        Q --> R{Cancellation Mode?}
        
        R -->|Default| S[Execute Default Behavior]
        S --> T[Hover / Hold Position]
        
        R -->|RTL| U["🏠 Return To Launch"]
        U --> V[Ascend to Safety Height]
        V --> W[Go To Home Point]
        W --> X[Land]
        
        R -->|LIP| Y["📍 Land In Place"]
        Y --> AA[Land at Current Position]
    end

    T --> BB[🔚 Mission Terminated]
    X --> BB
    AA --> BB
    N --> CC[🎉 Mission Success]

    style A fill:#4CAF50,stroke:#2E7D32,color:#fff
    style Z fill:#f44336,stroke:#c62828,color:#fff
    style G fill:#2196F3,stroke:#1565C0,color:#fff
    style N fill:#8BC34A,stroke:#558B2F,color:#fff
    style CC fill:#8BC34A,stroke:#558B2F,color:#fff
    style BB fill:#FF9800,stroke:#EF6C00,color:#fff
    style O fill:#FFC107,stroke:#F57F17,color:#000
    style Q fill:#f44336,stroke:#c62828,color:#fff
    style U fill:#9C27B0,stroke:#6A1B9A,color:#fff
    style Y fill:#FF5722,stroke:#D84315,color:#fff
```

### Flowchart Legend

| Symbol | Meaning |
|--------|---------|
| 📥 | Mission Reception Phase |
| 🚀 | Mission Execution Phase |
| ⏸️ | Paused State |
| 🛑 | Cancelled/Stopped |
| 🏠 | Return To Launch (RTL) |
| 📍 | Land In Place (LIP) |
| ✅ | Mission Complete |
| 🔚 | Mission Terminated |

### Key Decision Points

1. **LeafSDK Mode Check**: Only LeafSDK missions are processed through this flow
2. **Forced Start**: 
   - **Forced = Yes**: Mission starts automatically after entering idle
   - **Forced = No**: User must manually trigger start via QGC button
3. **Cancellation Modes**:
   - **Default**: Hover in place / hold position
   - **RTL**: Ascend → Navigate home → Land
   - **LIP (Land In Place)**: Immediate landing at current position

---

These Mermaid snippets track one-to-one with the Draw.io swim lanes, so you can re-run or tweak them whenever the workflow evolves without needing to manually edit the original diagram.
