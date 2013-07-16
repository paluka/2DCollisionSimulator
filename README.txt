Erik Paluka. Fall 2011.

Simulation of Multiple Rigid Bodies Colliding in 2D

Implemented in Java using the Open Source Physics framework (osp_core_ode_csm0.zip)

The most difficult aspect of rigid body simulation is modelling collisions. Modelling collisions requires the detection of contact points and computing the collision forces. Each body has a mass, position, velocity, orientation and angular velocity associated with it. Using Newton's Law of Restitution for Instantaneous Collisions, rigid body collisions can be accurately modelled.

Features
---------

-> Uses ordinary differential equations (ODE) to model body dynamics

-> Uses the fourth order Runge-Kutta method (RK4) for the ODE solver

-> When a collision is detected, the post-collision velocities and angles are resolved by calculating the impulse of the collision, and applying the impulse to the respective bodies at the time of collision

-> The time of the collision is found through a binary search. The ODE solver 'steps back time' to find the point of the collision

-> Calculates and graphs the total energy and the total momentum of the system