#ifndef PHYSICS_H 
#define PHYSICS_H 
#include "prim.h"
//reset_sap true indicates that prim_list has changed since the end of the last call and so sap box objects need to be recreated.

//method 0 treats collisions in time interval as inelastic and sticking until the end of the time interval at which point all collisions are resolved.
//method 1 (not zero) resolves collisions at time of impact.

//tolerance is a relative tolerance for the time of impact calculations. Given soonest collision time min_time, any collision time t such that
//min_time <= t <= tolerance*(1+min_time) 
//t is treated as occuring at min_time.

//resting_tolerance is an absolute tolerance used in calculating resting (touching, but not colliding) contacts at min_time.
//For all rigid bodies x and y, let d(x,y) denote the minimum distance between x and y (distance between a pair of closest points, one in x, one in y)
//at min_time. Then when calculating resting contacts at min_time if
//d(x,y) <= resting_tolerance
//then x and y are treated as being in resting contact (as if d(x,y) == 0).

//step_limit is for resolving contact_manifolds, too low of a step limit can result in artificially inelastic impact.

//time_step is the amount of time simulated in the call

//prim_list is list of rigid bodies (prim at this point) in simulation.
void step_physics(std::vector<prim> & prim_list, double time_step, int step_limit = 127, double tolerance = 0, double resting_tolerance = 0, int method = 0, bool reset_sap = true);
void step_physics_0(std::vector<prim> & prim_list, double time_step, int step_limit = 127, double tolerance = 0, double resting_tolerance = 0, bool reset_sap = true);
void step_physics_1(std::vector<prim> & prim_list, double time_step, int step_limit = 127, double tolerance = 0, double resting_tolerance = 0, bool reset_sap = true);
#endif
