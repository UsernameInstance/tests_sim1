#include "collection_contact.h"

collection_contact::collection_contact(collection* p_collection_0, collection* p_collection_1, std::vector<contact> contact_list, bool has_collision)
	: p_collection_0(p_collection_0), p_collection_1(p_collection_1), contact_list(contact_list), has_collision(has_collision)
{
	if(!contact_list.empty())
		time = contact_list.back().time;
	else
		time = std::numeric_limits<double>::infinity();
}

collection_contact::~collection_contact() {}

#if 1
collection_contact get_contacts(collection & collection0, collection & collection1, double time_limit, bool must_have_collision, double tolerance)
{
	{ //if big bounding spheres don't contact, then contact wont happen.
		contact first_pass = get_contact(collection0.enclosing_circle, collection1.enclosing_circle, time_limit, false, tolerance);
		if(first_pass.time == std::numeric_limits<double>::infinity())
			return collection_contact();
	}

	std::vector<contact> super; //superset of valid contacts
	super.reserve((collection0.prim_pointers.size() > collection1.prim_pointers.size()) ? collection0.prim_pointers.size() : collection1.prim_pointers.size());
	double min_time = time_limit;

	if(must_have_collision)
	{
		bool has_collision = false;
		for(auto & element0 : collection0.prim_pointers)
		{
			for(auto & element1 : collection1.prim_pointers)
			{
				if(element0 == element1) //assumes that prim cannot self-collide
					continue;
			
				//get contact with time no worse than min_time
				contact loop_contact = get_contact(*element0, *element1, min_time, false); //default assignment operator fine.
				if(loop_contact.time <= min_time)
				{
					super.push_back(loop_contact);
					if(loop_contact.is_collision)
					{
						has_collision = true;
						if(loop_contact.time < min_time)
							min_time = loop_contact.time;
					}
				}
			}
		}
		if(!has_collision)
			return collection_contact();
		collection_contact output(&collection0, &collection1);
		output.time = min_time;
		output.has_collision = has_collision;
		for(auto & element : super)
			if(element.time == min_time)
				output.contact_list.push_back(element);
		return output;
	}
	else
	{
		size_t best_index = 0;

		for(auto & element0 : collection0.prim_pointers)
		{
			for(auto & element1 : collection1.prim_pointers)
			{
				if(element0 == element1) //assumes that prim cannot self-collide
					continue;
			
				//get contact with time no worse than min_time
				contact loop_contact = get_contact(*element0, *element1, min_time, false, tolerance); //default assignment operator fine.
				if(loop_contact.time <= min_time)
				{
					super.push_back(loop_contact);
					if(loop_contact.time < min_time)
					{
						min_time = loop_contact.time;
						best_index = super.size()-1;
					}
				}
			}
		}
		if(!super.size()) //if super empty
			return  collection_contact();
		
		collection_contact output(&collection0, &collection1);
		output.time = min_time;
		output.contact_list.reserve(super.size()-best_index);

		for(size_t i=best_index; i<super.size(); ++i)
		{
			output.contact_list.push_back(super.at(i));
			if(super.at(i).is_collision)
				output.has_collision = true;
		}

		return output;
	}
}
#else //identical to above, just verifying implementations
collection_contact get_contacts(collection & collection0, collection & collection1, double time_limit, bool must_have_collision, double tolerance)
{
	{ //if big bounding spheres don't contact, then contact wont happen.
		contact first_pass = get_contact(collection0.enclosing_circle, collection1.enclosing_circle, time_limit, false, tolerance);
		if(first_pass.time == std::numeric_limits<double>::infinity())
			return collection_contact();
	}

	std::vector<contact> super;
	double best_time = time_limit;
	double best_collision_time = std::numeric_limits<double>::infinity();

	for(auto & element0 : collection0.prim_pointers)
	{
		for(auto & element1 : collection1.prim_pointers)
		{
			if(element0 == element1) //assumes that prim cannot self-collide
				continue;

			contact loop_contact = get_contact(*element0, *element1, time_limit, false, tolerance); //default assignment operator fine.
			if(loop_contact.time <= time_limit)
			{
				super.push_back(loop_contact);
				if(loop_contact.time < best_time)
					best_time = loop_contact.time;
				if(loop_contact.is_collision && (loop_contact.time < best_collision_time))
					best_collision_time = loop_contact.time;
			}
		}
	}

	if(!super.size())
		return  collection_contact();
	if(must_have_collision && (best_collision_time == std::numeric_limits<double>::infinity()))
		return collection_contact();

	collection_contact output(&collection0, &collection1);
	if(must_have_collision)
	{
		output.time = best_collision_time;
		for(auto & element : super)
		{
			if(element.time == best_collision_time)
			{
				output.contact_list.push_back(element);
				if(element.is_collision)
					output.has_collision = true;
			}
		}
	}
	else
	{
		output.time = best_time;

		for(auto & element : super)
		{
			if(element.time == best_time)
			{
				output.contact_list.push_back(element);
				if(element.is_collision)
					output.has_collision = true;
			}
		}
	}

	return output;
}
#endif
