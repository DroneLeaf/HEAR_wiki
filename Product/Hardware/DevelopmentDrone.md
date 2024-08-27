# Development Drone
## Operation
- Payload Support (100g)
- Flight-time: 10 min
- CTW>2

## Hardware: 
- Cinewhoop 3in prop holybro

## Avionics
- RPi CM4 + Kakute 1.3v2 from holybro
- Serial connection RPi<-->Kakute
- Battery SOC
- Closed RPM ESC mini 4in1 voxl
- MoCap markers

## Software Stack

### High fidelity model for RL training
- Pre-identified model that can be used by clients. 
- Fine-tuning procedure at client premises.
- Model interfacable for RL training, i.e. actions results in states.
- Model interfacable for testing of RL controllers.
  
### Flexible and performant software for RL deployment
- RL controller deployable at target with self computational performance check.
- RL checking program for hovering with RL to check stability.

## Mission

- Polynomial trajectory following
- Switch from position control to RL, and from RL to position control.
- Takeoff, Land, pre-defined disarm
- Dynamic stop trajectory
- Smooth bias elimination
- Persistent memory for biases