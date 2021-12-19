# MRAC-Control-of-3-wheeled-Omni-robot

The project implementation details are briefly summarized below.

## Background

Omni-directional robots have more flexibility in movement as compared to their fixed-wheeled counterparts; however this is achieved at the cost of increased complexity. Modeling the dynamics of omni-directional robots/ vehicles is not a straightforward process and some non-linearities are bound to be involved. This can be countered by using an MRAC controller which is more forgiving of design inaccuracies.

Here, the simulation of a direct MRAC controller for a 3-wheeled omni-directional robot is presented. For Model Reference Control, a known refernce model is chosen, which is known to have the desired I/O behavior.

## System Model

The system model used for simulation is based on the one derived in the research paper, "Modeling and Assessing of Omni-directional Robots with Three and Four Wheels" by Helder P. Oliveira et. al.

### Kinematic Model
The Kinematic model basically relates the robot front and normal velocities to the individual wheel velocities:
![image](https://user-images.githubusercontent.com/73758224/146683911-e28c5496-591e-4e55-9f1e-b324709f8624.png)

### Dynamics Model
The system dynamics can be given as:

![image](https://user-images.githubusercontent.com/73758224/146684001-8aa2826a-3c10-4c19-b09b-a744399feb2c.png)

where forces with subscript B are the viscous friction forces and those with subscript C are the Coulomb friction forces.

Combining the Kinematics and Dynamics models would give us a final State-Space representation of the system:

![image](https://user-images.githubusercontent.com/73758224/146683408-e0ac7234-eaf5-4930-b966-5bc98d3cf40e.png)

where,

![image](https://user-images.githubusercontent.com/73758224/146683419-45d90fa1-c3f6-4c8a-8666-92b2972f41e3.png)

## Reference System

We need a reference system (the system whose behavior will be mimicked by the original system) for the DMRAC controller. The reference system is of the following form:

![image](https://user-images.githubusercontent.com/73758224/146683825-9e742147-6810-4edc-8313-b5c790d97d3f.png)


## Control Law

The control law for the direct MRAC Controller is given as:

![image](https://user-images.githubusercontent.com/73758224/146683490-8d40bed0-b2d3-4f1f-a673-86a8efc00eaa.png)

where Kx(t) and Kr(t) are the 3x3 Adaptation Gain Matrices. The adaptation gain dynamics are governed by:

![image](https://user-images.githubusercontent.com/73758224/146684228-f0dab9db-9396-4254-8a7c-5a93a645fdaa.png)

## Simulation Results

**Comparison of Reference and Actual Trajectories**

![image](https://user-images.githubusercontent.com/73758224/146684312-7e6a453e-6ecf-494e-a0d1-a9c996914f9e.png)

