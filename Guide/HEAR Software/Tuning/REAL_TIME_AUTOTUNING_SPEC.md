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

### 1. Autotuning Request (FC -> CD -> Lambda)

*Note: FC will not send a request if behavior is E1. Values are limited to 5 significant figures.*

**Payload Example:**

```json
{
  "message": {
    "job_id": "job-12345",
    "u": [0.0, 0.0, ..., 1.0],
    "pv": [0.00100, 0.00200, ..., -0.00100],
    "channel": "Roll",
    "dt": 0.00500
  },
  "orgId": "936da01f-9abd-4d9d-80c7-02af85c822a8",
  "deviceId": "312f3fe3-5fa4-4ab3-b1dd-1d9208697f36",
  "fullTopic": "org/936da01f-9abd-4d9d-80c7-02af85c822a8/device/312f3fe3-5fa4-4ab3-b1dd-1d9208697f36/command/autotuning/tuning-request"
}
```

**Fields:**
*   `message`: Object containing tuning data.
    *   `job_id`: String (Unique identifier for the job).
    *   `u`: Array of floats (Control Input, 5 sig figs).
    *   `pv`: Array of floats (Process Variable, 5 sig figs).
    *   `channel`: String (e.g., "Roll", "Pitch", "Z", "X", "Y").
    *   `dt`: Float (Time step, 5 sig figs).
*   `orgId`: String (Organization ID).
*   `deviceId`: String (Device ID).
*   `fullTopic`: String (The intended MQTT topic).

### 2. Autotuning Response (Lambda -> CD -> FC)

*Note: Lambda is behavior agnostic and will always return new PID values. Values are limited to 5 significant figures.*

**Payload Example:**

```json
{
    "message": {
        "job_id": "job-12345",
        "class_id": 22,
        "pid_parameters": {
            "dt": 0.00500,
            "en_pv_derivation": 1,
            "kd": 0.87168,
            "kp": 9.39380
        }
    },
    "metadata": {
        "timestamp": "2025-12-09T06:04:15.36845"
    },
    "mqtt_published": true,
    "original_topic": "org/936da01f-9abd-4d9d-80c7-02af85c822a8/device/312f3fe3-5fa4-4ab3-b1dd-1d9208697f36/command/autotuning/tuning-request",
    "response_topic": "org/936da01f-9abd-4d9d-80c7-02af85c822a8/device/312f3fe3-5fa4-4ab3-b1dd-1d9208697f36/command/autotuning/response",
    "success": true
}
```

**Fields:**
*   `message`: Object containing the result.
    *   `job_id`: String (Matching the request job_id).
    *   `pid_parameters`: Object (New PID values).
        *   `dt`, `kp`, `kd`: Float (5 sig figs).
    *   `class_id`: Integer.
