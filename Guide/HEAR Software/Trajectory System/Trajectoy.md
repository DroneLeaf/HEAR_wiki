# Overview

## Concept of Trajectories
1. Primitive: We have multiple types of primitives, e.g. Polynomial, Circle, etc. These are extendable.
2. Transformation: transformation applies to the primitive.

## Transformation convention
Sequence of transformation:
1. Sample the primitive
2. Apply spatial offset
3. Apply time scale
4. Apply spatial scale
5. Apply rotation

## Extending the Trajectory System

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

Systems/Trajectory/[primitive_type]/[primitive_instance]