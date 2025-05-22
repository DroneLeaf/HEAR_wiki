# Vision-based precision landing
The drone is required to takeoff, go to a specific GPS location, detect a visual marker to land at, and if not, RTL.

## Suggested solution
### Pre-condition: 
Vision-based landing LeafService active

### Flow:
Defined by LeafScript:
- Arm()
- ExecutePrecisionLandingPlan()
- WaitForVisionBasedLandingDetectorConfirmation(x) % x: timeout
- 


### Post-condition

### Non-functional Specifications
#### Must-have
QGC Support

#### Nice-to-have
.plan file support for QGC missions

### Comments
- Design is hard-coded (no UI).


# Entities
## Services (To be defined)
### Vision-based landing LeafService
#### Definition
A standalone service that is responsible for reading camera input, processing it for landing marker detection, and output relative landing location.

#### Roles
- Enable/Disable marker detection.
- Inform once marker detection is validated.
- Send relative marker location in real-time.

#### Interface
Mavlink: Define system IDs and messages.