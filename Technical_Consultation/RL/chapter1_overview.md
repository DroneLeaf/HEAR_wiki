# Reinforcement Learning training and deployment infrastructure

## Overview
In today's increasingly complex requirements for UAV missions, we propose to offer a unified approach for RL training and deployment.

DroneLeaf would offer the following:
1) Training infrastructure:
   1) A dynamics model for simulation wrapped around a standardized interface.
   2) The dynamics model is bespoke to the client UAV.
   3) The dynamics model is augmented with randomization for direct sim-to-real transfer.

2) Deployment infrastructure:
   1) ONNX model format for deployment.
   2) ONNX model inference time is verified for real-time capability.
   3) ONNX model is deployed on the client UAV.
   4) The switching from Normal control to RL and vice-versa is controlled by the user through SDK.