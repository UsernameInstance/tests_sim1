#ifndef CONTACT_H
#define CONTACT_H

#include "prim.h"

struct contact
{
	prim* p_prim_0;
        prim* p_prim_1;
	double time;
	bool is_collision; 
	Eigen::Vector2d normal; //if nonzero then outward for p_prim_0, inward for p_prim_1; if zero then disregard contact. //impulse on 1 in normal direction impulse on 0 in -normal direction

	contact(prim* p_prim_0 = NULL, prim* p_prim_1 = NULL, double time = std::numeric_limits<double>::infinity(), bool is_collision = false, Eigen::Vector2d normal = Eigen::Vector2d(0,0));
	~contact();
};

//find contact(s) with contact time smallest nonnegative at most time_limit, if no such time exists time is set to infinity; if collision_only == true then non-colliding contacts treated as no-contact.
contact get_contact(prim & prim0, prim & prim1, double time_limit = std::numeric_limits<double>::infinity(), bool collision_only = false, double tolerance = 0);

#endif
