# Land Mode Settings
## Landing Descent Rate
Payload: PARAM_SET (23)
param_id (char): MPC_LAND_SPEED
param_value (float): 0.8 // land speed of 0.8 m/s
param_type (MAV_PARAM_TYPE): MAV_PARAM_TYPE_REAL32 (9)


## Disarm after

Payload: PARAM_SET (23)
param_id (char): COM_DISARM_LAND
param_type (MAV_PARAM_TYPE): MAV_PARAM_TYPE_REAL32 (9) // 32-bit little endian

**Checked with value of 2 seconds:**
param_value (float): 2

**Unchecked**
param_value (float): 0