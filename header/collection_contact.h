#ifndef COLLECTION_CONTACT_H
#define COLLECTION_CONTACT_H
#include "collection.h"

struct collection_contact
{
	collection* p_collection_0;
	collection* p_collection_1;

	double time;
	bool has_collision;

	std::vector<contact> contact_list;

	collection_contact(collection* p_collection_0 = nullptr, collection* p_collection_1 = nullptr, std::vector<contact> contact_list = std::vector<contact>(), bool has_collision = false);
	~collection_contact();
};

//find all contacts between collections with contact time smallest nonnegative at most time_limit, if no such time exists time is set to infinity.
//if must_have_collision is true then all contacts between collections with contact time smallest nonnegative at most time_limit such that there is
//a collision between the collections with that contact time are returned, otherwise if no collisions occur then empty collection_contact with time 
//set to infinity is returned.
//can call with collection0 == collection1, will check for contacts between collection's prims assuming prim cannot self collide.
collection_contact get_contacts(collection & collection0, collection & collection1, double time_limit = std::numeric_limits<double>::infinity(), bool must_have_collision = false, double tolerance = 0);
#endif
