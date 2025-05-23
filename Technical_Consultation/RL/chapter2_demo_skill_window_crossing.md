# Window crossing as a RL training and deployment infrastructure demo
## Deliverables

Deliver on two stages:
1) RL model that can make a UAV cross a swinging window.
2) RL model that can make a UAV cross a swinging window with angle constraints.

## Definitions
Environment observable states: These are states with limited range, and are observable in the deployment stage.

Environment unobservable states: These are states with limited range, and are not observable in the deployment stage.


## Environment
Window moving periodically with a repeatable amplitude and frequency.
    - Window 0.2-0.6 Hz, 15-30 degrees. Window center distance from rotation pivot 0.5-1.0 m.

UAV
    - Initial condition of window away from the UAV: 1.5-2.0 m x-axis, -1.0-1.0 m y-axis, -0.5-0.5 m z-axis.
    - Initial velocity and acceleration of UAV: 0.
    - Velocity across x-axis is specified by the user. Range 1.2-2.0 m/s.
    - UAV used is the development UAV of DroneLeaf. The model will be provided based on real-world identification.
    
## Action space
Stage1:
    - Two sets of PD parameters one for y-axis and one for z-axis.
    - PD parameters: Kp, Kd.

Stage2:
    - Two sets of PD parameters one for y-axis and one for z-axis.
    - Switching distance norm $d_{sw}$. After switching, thrust command goes to zero and angle setpoint comes from the window angle. Direction of the set angle is based on the initial angle of the window.

## Loss
Stage1:
    - Loss function: $L = \sqrt{(y_c^2 + z_c^2)} $

Stage2:
    - Loss function: $L = \sqrt{(y_c^2 + z_c^2)} + \lambda \sqrt{((d_{w_x}-d_{d_x})^2 + (d_{w_y}-d_{d_y})^2 + (d_{w_z}-d_{d_z})^2)} $. $\bm{d_w}$ is the direction axis of the window and $\bm{d_d}$ is the direction axis of the UAV. $\lambda$ is a hyperparameter.


## Uncertainties
Uncertainties accounted for are mainly for the model parameters.

Three parameters would be randomized for now:
    1) $T_p$ propulsion time constant: covering the range of motor identification results, i.e. min to max.
    2) $\tau_p$ propulsion time delay: -10% to 80% of the nominal value.
    3) $k_p$ propulsion gain: -10% to 10% of the nominal value.

The cost with respect to these uncertainties is believed to be monotonic. Therefore the extremum of the ranges would be used to train the RL model.

## Deployment
1) Inference models deployed on AWS lambda.
2) Actor parametric actions are coded into HEAR_FC. 
3) Switching happens from normal flight mode -> skill execution mode -> back to normal flight mode. Beginning of skill execution mode is triggered by the user.

## Testing
testing stages:

1) SITL
2) HITL
   1) based on window model
   2) based on real captured window data
3) Full testing