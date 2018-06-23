# tests_sim1
Learning project done to practice using Cygwin, Gnu Make, Vim, GDB, C++, SDL2, and IPOPT, as well as to learn the basics of GitHub.

Besides learning, the project was made to allow me to run a few different experiments involving continuous collision detection and simultaneous collision resolution of rigid bodies in two dimensions. The rigid bodies consist of frictionless discs or the set complement of a disc without rotation. The physics engine is impulse based. Collisions are resolved using generalized reflections (see http://www.cs.columbia.edu/cg/rosi/rosi.pdf). The timestep is fixed at .008 seconds, i.e. each physics loop simulates .008 seconds.

## What the project does
The project consists of four different tests together with a few settings that can be adjusted using the keyboard: method, tolerance and resting_tolerance, and coefficient of restitution (COR). A short video of the tests can be found here https://youtu.be/tMoFXv5tBHs.

### Method
The method setting determines whether or not collisions are resolved at the time they are predicted to occur. It is the setting I was originally interested in creating this project to test.

Method 1 resolves collisions at the time they are predicted to occur. More specifically the earliest time of collision within the timestep is determined, the rigid bodies are moved to their positions at that time, and all collisions at that time are resolved (giving updated positions and velocities to the rigid bodies). This is repeated using the remaining time in the time step until no more collisions are detected. The main issue with this is that it can freeze up when there are a very high number of collisions occurring in a timestep (can be an infinite number of collisions in some cases, especially when dealing with inelastic collisions).

Method 0 temporarily treats collisions as inelastic and sticking until the end of the timestep at which point all collisions are resolved. In other words for collisions occuring at time in the interior of the timestep, after moving the rigid bodies to the earliest time of collision, the colliding rigid bodies are clumped together and treated as a new rigid body (with velocity equal to the center of mass of all the original bodies involved in the collision and in contact with one another). Issues with this are due to realism (although in reality collisions are not instant either), in some cases visually unacceptable results, and as implemented here a distasteful variability in the time length of collisions.

### Tolerances
The tolerances affect what's considered "simultaneous" (tolerance) as well as what's considered "in contact" (resting_tolerance). 

The tolerance setting is a relative tolerance applied in calculating earliest time of collision in a timestep. If the earliest time of collision is min_time then any collision occuring between min_time and min_time*(1+tolerance) is treated as occuring at min_time. 

After moving objects to their positions at min_time, objects in contact but not colliding are calculated. At this step, essentially, if the two objects closest pair of points has distance less than or equal to resting_tolerance then the objects are treated as being in contact.

Tolerances were found to be needed as with 0 tolerance settings symmetry is not preserved (visually) for more than a few seconds. Even with generous tolerances symmetry only seems to be (visually) preserved for a few seconds in the fourth test, and around thirty seconds in the second test. As far as I can tell this is due to error accumulation from floating point arithmetic and error in the NLP solver IPOPT (although adjusting IPOPT error settings doesn't seem to have much effect), as well as error in initial conditions (e.g. sqrt(3) involved in setting up the triangle of balls in second test).

### COR
For single point collisions between two bodies, COR gives the coefficient of restitution. In multipoint collisions this setting shows up as a multiplier in the quadratic program that needs to be solved to resolve the collision.
