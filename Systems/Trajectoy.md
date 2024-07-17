# Base Trajectory System

- Transformation
  - Contains: Position Translation
- TrajectoryQueue
  - Contains: TrajectoryBuffer
  - Methods: clear(), abortAndClear(), queueTrajectory(id,optional scale+axis-angle+time_scale),getCurrentQueue()
- TrajectoryPrimitive
  - Contains: axis-angle + scale, ramp_up and ramp_down scale time, scale time, time offset
  - Methods: sample() pure, string=getType() pure, loadParameters(json), getTotalTime(), getPercentageComplete()
  - Child: PolynomialTrajectory, Sine, Circle, lemniscate
- TrajectoryBuffer
  - Methods: getTrajectoryPrimitive(id), Add(TrajectoryPrimitive,id)

## Folder Structure

Systems/TrajectoryExecuter/[primitive_type]/[primitive_instance]

# Goto Point Trajectory Generation

# GPS/Local coordinate frame bi-translator

# 