# RC Loss Failsafe Trigger
## Failsafe Action
Payload: PARAM_SET (23)
param_id (char): NAV_RCL_ACT
param_type (MAV_PARAM_TYPE): MAV_PARAM_TYPE_INT32 (6) // 32 bit little endian

**Hold Mode**
param_value (float): 1.4013e-45 // Corresponds to 0x01000000 -> int value is 1
**Return Mode**
param_value (float): 2.8026e-45 // Corresponds to 0x02000000 -> int value is 2

**Land Mode**
int value is 3
**Terminate**
int value is 5
**Disarm**
int value is 6

## RC Loss Timeout
Payload: PARAM_SET (23)
param_id (char): COM_RC_LOSS_T
param_value (float): 0.9 // Example of 0.9 seconds
param_type (MAV_PARAM_TYPE): MAV_PARAM_TYPE_REAL32 (9)


