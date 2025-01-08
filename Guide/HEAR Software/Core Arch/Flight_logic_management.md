# Overview
Flight logic management is a very delicate task within flight system design. It can easily grow into a mess. Proper abstraction should prevent that. 

## Components of Logic
In LeafFC, we initiate at the executable level these entities:
1. Flight Systems (FS): Flight systems host trajectory, control, state-estimation, and actuation algorithms. The FSs are stateful but they should not hold the Single Source of Truth (SSoT). i.e. a FS cannot report that it is Armed/Disarmed though it should be one of these.
2. Flight Logic Systems (FLS): This is the concern of this document. Within FLS we have two types of entities:
    1. Logic Managers (LM): These are Blocks that hold the status of a specific component of the system. For example, ArmManager holds the status of Arm/Disarm of the flight controller. This is the SSoT of logic.
    2. Logic Pipelines (LP): LPs hold mission elements that interact with other LMs and other pipelines. LP were made to organize sequential and sequential-conditional logic in a clear manner. LP allow reusability of sequential logic.

## Usage guidelines
1. LMs must be created to hold the right amount of coherent logic. Avoid for example PX4Manager since that would so vague and large. Do not over-specialize too, e.g. PX4ConnectionManager is not advised. Use, for the same example, instead ConnectivityManager.
2. LPs must be preferred over LMs whenever possible since they can be easily reused.
3. Managing the state of FS must be the responsibility of the LMs.
4. Avoid heavy computations in all FLS.

## Known Limitations
- Current connections in HEAR architecture do allow async and sync connections. Yet it does not directly support async-query operations. This is now substituted with sync connections with SSoTs emitting current status.