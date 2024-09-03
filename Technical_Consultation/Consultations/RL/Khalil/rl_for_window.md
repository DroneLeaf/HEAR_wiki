## Background

Refer to document "Technical_Consultation/external_consultation_guide.md" for general guideline on consultation for DroneLeaf.

Refer to document "Product/RL/chapter1_overview.md" for technical background information.

## Definitions

Definitions are taken from "Product/RL/chapter2_demo_skill_window_crossing.md".

Additional definitions:

- AI model test: testing the AI model on the compiled model.
- SITL Functional test: testing the AI model in addition to the flight logic with HEAR_SITL, HEAR_FC, HEAR_MC, and DroneLeaf AWS services.

## Deliverables

1) A UAV crossing the moving window in SITL with 100% success rate - Stage 1. Randomization range is the same as the training range. 

2) A UAV crossing the moving window with angle constraints in SITL with 100% success rate - Stage 2.

Demonstrations and proof of delivery are consultant scope.

## Out of consultant scope
The following will be provided by DroneLeaf:

1) The UAV model compiled from MATLAB/Simulink. Models describing multiple UAVs could be provided.
2) HEAR_SITL code infrastructure to run the UAV simulation.
3) HEAR_FC and HEAR_MC for integration and testing.
4) Server access or deployment of the inference model in AWS Lambda.

## Explicitly in consultant scope
Some items are worth stressing to be in consultant scope:

1) Refining of training of AI models based on experimental feedback.

## Timeline

1) Successful AI model test for Stage 1 and 2: 2024-09-16
2) Successful AI model test for Stage 1 in the SITL: 2024-09-23
3) Retrain and test or Stage 1 in the SITL and in hardware with DroneLeaf Development UAV: 2024-09-30
4) Retrain and test or Stage 2 in the SITL and in hardware with DroneLeaf Development UAV: 2024-10-07

## Compensation

Upon the full completion of deliverables, the consultant will be paid 100% of the agreed amount.

Agreed amount: 5,000 USD.