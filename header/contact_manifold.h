#ifndef CONTACT_MANIFOLD_H
#define CONTACT_MANIFOLD_H
#include "collection_contact.h"

struct contact_manifold
{
	std::vector<prim*> prim_pointers;
	std::vector<contact> contact_list;

	contact_manifold();
	contact_manifold(collection & input_collection);
	contact_manifold(std::vector<collection> & collection_list);

	~contact_manifold();
	
	//applies impact operator and updates *prim_pointers velocities.
	//returns false iff an error occured preventing solution; 
	bool impact_operator(bool ignore_resting_contacts = true);

	//calls impact_operator at most step_limit-1 times with ignore_resting_contacts == true, returning as soon as all collisions are resolved.
	//Last call to impact_operator (call # step_limit) uses ignore_resting_contacts == false.
	//after each call to impact_operator velocity vector is scaled to preserve kinetic energy and then
	//contacts in contact_list are re-checked to see if they are still a collision.
	//1 >> tolerance >= 0 is a constraint tolerance parameter. bigger tolerance, allows larger constraint violation to be considered feasible.
	//returns true iff all collisions resolved. 
	bool resolve(int step_limit, double tolerance = 0); 
};

#endif
