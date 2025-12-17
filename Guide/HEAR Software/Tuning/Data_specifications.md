# Data Specification for Real-Time Autotuning

This document outlines the data specifications, messaging format, and MQTT topics for the real-time autotuning feature.

## Architecture Overview

1.  **Request**: Flight Controller (FC) -> Redis -> Controller Dashboard (CD) -> MQTT -> Lambda
2.  **Response**: Lambda -> MQTT -> Controller Dashboard (CD) -> Redis -> Flight Controller (FC)

## Topics and Channels

### Redis Channels
*   **Request Channel**: `controller-dashboard/autotuning/tuning-request`
    *   Source: Flight Controller (FC)
    *   Destination: Controller Dashboard (CD)
*   **Response Channel**: `controller-dashboard/autotuning/tuning-response`
    *   Source: Controller Dashboard (CD)
    *   Destination: Flight Controller (FC)

### MQTT Topics
*   **Request Topic**: `org/{orgId}/device/{deviceId}/command/autotuning/tuning-request`
    *   Source: Controller Dashboard (CD)
    *   Destination: Cloud Lambda
*   **Response Topic**: `org/{orgId}/device/{deviceId}/command/autotuning/response`
    *   Source: Cloud Lambda
    *   Destination: Controller Dashboard (CD)

## JSON Message Formats

### 1 Flight Controller (FC) — Redis request/response (FC <-> Controller Dashboard)
*Note: FC will not send a request if behavior is E1. Values are limited to 5 significant figures.*



This section describes the exact Redis payloads the Flight Controller (FC) should publish and subscribe to when interacting with the Controller Dashboard (CD).

* Redis Request Channel (FC -> CD): `controller-dashboard/autotuning/tuning-request`

    Payload example (FC publishes this to the request channel):

```json
{
    "message": {
        "u": [0.0, 0.0, ..., 1.0],
        "pv": [0.00100, 0.00200, ..., -0.00100],
        "channel": "Roll",
        "dt": 0.00500
    }
}
```

Fields (FC request):
* `message`: Object containing tuning data.
        * `u`: Array of floats (Control input samples, 5 significant figures).
        * `pv`: Array of floats (Process variable samples, 5 significant figures).
        * `channel`: String (e.g., "Roll", "Pitch", "Z", "X", "Y").
        * `dt`: Float (Time step, 5 significant figures).



Example redis message:
```
redis-cli PUBLISH controller-dashboard/autotuning/tuning-request '{"message": {"u":[0.0,0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0],"pv":[0.001000005,0.002,0.003,0.005,0.008,0.012,0.018,0.024,0.028,0.031,0.032,0.030,0.025,0.019,0.012,0.006,-0.002,-0.009,-0.015,-0.019,-0.021,-0.022,-0.021,-0.018,-0.014,-0.009,-0.003,0.003,0.009,0.014,0.018,0.02100002,0.023,0.024,0.023,0.020,0.016,0.011,0.005,-0.001,-0.008,-0.014,-0.019,-0.022,-0.024,-0.024,-0.023,-0.020,-0.016,-0.011,-0.005,0.001,0.007,0.013,0.018,0.021,0.023,0.024,0.023,0.021,0.017,0.012,0.006,0.000,-0.007,-0.013,-0.018,-0.021,-0.023,-0.024,-0.023,-0.021,-0.017,-0.012,-0.006,-0.000,0.006,0.012,0.017,0.021,0.023,0.024,0.023,0.021,0.018,0.013,0.007,0.001,-0.005,-0.011,-0.016,-0.020,-0.023,-0.024,-0.024,-0.022,-0.019,-0.014,-0.008,-0.001],"channel":"roll","dt":0.005}}'
```


* Redis Response Channel (CD -> FC): `controller-dashboard/autotuning/tuning-response`

    The FC should subscribe to this channel to receive tuning responses. The Controller Dashboard will place the Lambda result inside a top-level `tuning_result` object so the FC can easily find the relevant data.

    Example response payload (FC will receive this on `controller-dashboard/autotuning/tuning-response`):

```json
{
    "tuning_result": {
        "tuning_request_id": "497ec08c-413e-4686-8ebc-19065f43bc9b",
        "channel": "Roll",
        "class_id": 22,
        "saved_to_cloud": true,
        "pid_parameters": {
            "dt": 0.00500,
            "en_pv_derivation": 1,
            "kd": 0.87168,
            "kp": 9.39380
        }
    }
        "metadata": {
        "timestamp": "2025-12-09T06:04:15.36845"
    },
    "mqtt_published": true,
    "original_topic": "org/936da01f-9abd-4d9d-80c7-02af85c822a8/device/312f3fe3-5fa4-4ab3-b1dd-1d9208697f36/command/autotuning/tuning-request",
    "response_topic": "org/936da01f-9abd-4d9d-80c7-02af85c822a8/device/312f3fe3-5fa4-4ab3-b1dd-1d9208697f36/command/autotuning/response",
    "success": true,
}
```

**Notes for FC developers:**
* The FC's relevant data is the `tuning_result` object — specifically `tuning_result.pid_parameters`, `tuning_result.channel`, `tuning_result.saved_to_cloud`, `tuning_result.tuning_request_id`, and optionally `tuning_result.class_id`
* Values are limited to 5 significant figures as with other messages

**Example Redis Subscribe Command for Local Testing:**

```bash
redis-cli SUBSCRIBE controller-dashboard/autotuning/tuning-response
```

**Actual Response Format (Formatted for readability):**

When you subscribe, you'll receive:
```
1) "message"
2) "controller-dashboard/autotuning/tuning-response"
3) <JSON payload below>
```

The JSON payload (item 3) will look like this when formatted:

```json
{
  "success": true,
  "message": "IoT data received and processed",
  "metadata": {
    "timestamp": "2025-12-15T10:43:14.462257"
  },
  "tuning_result": {
    "class_id": 22,
    "channel": "roll",
    "pid_parameters": {
      "kp": 9.3938,
      "kd": 0.87168,
      "dt": 0.005,
      "en_pv_derivation": 1
    },
    "tuning_request_id": "0a6a9967-eec2-4ae1-b131-3fb66ccafb52",
    "saved_to_cloud": true
  },
  "original_topic": "org/e8fc2cd9-f040-4229-84c0-62ea693b99f6/device/Instance-312f3fe3-5fa4-4ab3-b1dd-1d9208697f36/command/autotuning/tuning-request",
  "response_topic": "org/e8fc2cd9-f040-4229-84c0-62ea693b99f6/device/Instance-312f3fe3-5fa4-4ab3-b1dd-1d9208697f36/command/autotuning/response"
}
```

**Key Fields to Parse:**
* `tuning_result.pid_parameters.kp` - Proportional gain
* `tuning_result.pid_parameters.kd` - Derivative gain
* `tuning_result.pid_parameters.dt` - Time step
* `tuning_result.pid_parameters.en_pv_derivation` - Enable PV derivation (1=enabled, 0=disabled)
* `tuning_result.tuning_request_id` - Match this with your request ID
* `tuning_result.saved_to_cloud` - Indicates if values were persisted to DynamoDB

### 1.a Autotuning Request (CD -> Lambda via MQTT)

*Note: The Controller Dashboard injects `robot_type_id`, `tuning_request_id` and `development_mode` before sending to Lambda.*

**Payload Example:**

```json
{
  "orgId": "936da01f-9abd-4d9d-80c7-02af85c822a8",
  "deviceId": "312f3fe3-5fa4-4ab3-b1dd-1d9208697f36",
  "fullTopic": "org/936da01f-9abd-4d9d-80c7-02af85c822a8/device/312f3fe3-5fa4-4ab3-b1dd-1d9208697f36/command/autotuning/tuning-request",
  "message": {
    "robot_type_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
    "tuning_request_id": "497ec08c-413e-4686-8ebc-19065f43bc9b",
    "development_mode": false,
    "u": [0.0, 0.0, ..., 1.0],
    "pv": [0.00100, 0.00200, ..., -0.00100],
    "channel": "Roll",
    "dt": 0.00500
  }
}
```

**Fields:**
*   `orgId`: String (Organization ID).
*   `deviceId`: String (Device ID).
*   `fullTopic`: String (The intended MQTT topic).
*   `message`: Object containing tuning data.
    *   `robot_type_id`: String (Robot Type ID).
    *   `tuning_request_id`: String (Unique identifier for the job, generated by Controller Dashboard).
    *   `development_mode`: Boolean (Example: false. Injected by CD for testing or defaults).
    *   `u`: Array of floats (Control Input, 5 sig figs).
    *   `pv`: Array of floats (Process Variable, 5 sig figs).
    *   `channel`: String (e.g., "Roll", "Pitch", "Z", "X", "Y").
    *   `dt`: Float (Time step, 5 sig figs).

### 2. Autotuning Response (Lambda -> CD -> FC)

*Note: Lambda is behavior agnostic and will always return new PID values. Values are limited to 5 significant figures.*

**Payload Example:**

```json
{
    "metadata": {
        "timestamp": "2025-12-09T06:04:15.36845"
    },
    "mqtt_published": true,
    "original_topic": "org/936da01f-9abd-4d9d-80c7-02af85c822a8/device/312f3fe3-5fa4-4ab3-b1dd-1d9208697f36/command/autotuning/tuning-request",
    "response_topic": "org/936da01f-9abd-4d9d-80c7-02af85c822a8/device/312f3fe3-5fa4-4ab3-b1dd-1d9208697f36/command/autotuning/response",
    "success": true,
    "tuning_result": {
        "tuning_request_id": "497ec08c-413e-4686-8ebc-19065f43bc9b",
        "channel": "Roll",
        "class_id": 22,
        "saved_to_cloud": true,
        "pid_parameters": {
            "dt": 0.00500,
            "en_pv_derivation": 1,
            "kd": 0.87168,
            "kp": 9.39380
        }
    }
}
```

**Fields:**
*   `message`: Status message string.
*   `metadata`: Object containing timestamp.
*   `mqtt_published`: Boolean indicating if MQTT response was sent.
*   `original_topic`: The topic the request came from.
*   `response_topic`: The topic the response was sent to.
*   `success`: Boolean indicating overall success.
*   `tuning_result`: Object containing the actual tuning data.
    *   `tuning_request_id`: String (Matching the request id).
    *   `channel`: String (e.g. "Roll").
    *   `class_id`: Integer.
    *   `saved_to_cloud`: Boolean.
    *   `pid_parameters`: Object (New PID values).
        *   `dt`, `kp`, `kd`: Float (5 sig figs).
