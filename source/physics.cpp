#include "physics.h"
#include "contact_manifold.h"
#include <cmath>
#include <algorithm>
#include <deque> //using std::vector for collection_list seems to run into bad_alloc issues due to apparently not being able to allocate a large enough block of contiguous memory

template <typename I, typename func>
void insertion_sort(I first, I last, func compare) {
  if (first == last) {
    return;
  }

  for (auto&& j = std::next(first); j != last; ++j) {
    auto key = *j;
    auto i = j;
    while (i != first && compare(key, *std::prev(i))) {
      *i = *std::prev(i);
      --i;
    }

    *i = key;
  }
}
//should probably make a .h and .cpp file for sweep and prune stuff, added last minute.
struct sapbox {
	collection* p_collection;
	double x0, x1, y0, y1; //x0 <= x1 && y0 <= y1 
	//bounds trajectory of object
	bool is_active;
	size_t index;

	sapbox(collection* p_collection = nullptr, double time_step = 0, size_t index = 0) 
		: p_collection(p_collection), index(index) {
			update(time_step); }

       void update(double time_step, double tolerance = 0) {
	       if(p_collection) {
		       if(p_collection->enclosing_circle.type == prim::type::exterior) {
			       x0 = -std::numeric_limits<double>::infinity();
			       x1 = -x0;
			       y0 = x0;
			       y1 = x1; }
		       else {
			       x0 = (p_collection->enclosing_circle.position)(0) - p_collection->enclosing_circle.radius;
			       x0 = std::min(x0, x0 + p_collection->enclosing_circle.velocity(0)*time_step) - tolerance;
			       x1 = (p_collection->enclosing_circle.position)(0) + p_collection->enclosing_circle.radius;
			       x1 = std::max(x1, x1 + p_collection->enclosing_circle.velocity(0)*time_step) + tolerance;
			       y0 = (p_collection->enclosing_circle.position)(1) - p_collection->enclosing_circle.radius;
			       y0 = std::min(y0, y0 + p_collection->enclosing_circle.velocity(1)*time_step) - tolerance;
			       y1 = (p_collection->enclosing_circle.position)(1) + p_collection->enclosing_circle.radius;
			       y1 = std::max(y1, y1 + p_collection->enclosing_circle.velocity(1)*time_step) + tolerance; }
		       is_active = true; }
	       else {
		       x0 = std::numeric_limits<double>::infinity();
		       x1 = -x0;
		       y0 = x0;
		       y1 = -x0;
		       is_active = false; } } };

std::vector<sapbox> sapx_list;
void step_physics(std::vector<prim> & prim_list, double time_step, int step_limit, double tolerance, double resting_tolerance, int method, bool reset_sap)
{
	if(method)
		step_physics_1(prim_list, time_step, step_limit, tolerance, resting_tolerance, reset_sap);
	else
		step_physics_0(prim_list, time_step, step_limit, tolerance, resting_tolerance, reset_sap);
	return;
}

//see physics.h for step_physics arguments' descriptions.
//should probably clean this file up. Lots of unintended stuff added over time (e.g. tolerances), just to try random stuff.
//experimenting with bound on number of collisions in timestep (no tunneling) by number of collidable objects
//collisions treated as inelastic and sticking until end of time step, at which point velocities from beginning of time step are restored and collisions resolved.
//All advice received seems to indicate that goto statements are evil and should rarely be used. Using them here for learning purposes.
/*
0. save original prim_list velocity vectors in backup, initialize collection_list with input prims, initialize time_left with time_step
1. get all min time collision containing collection_contacts
2. step all distinct collections forward to min(time_left, min time collision)
3. if no collisions found in step 1. then goto step 7. else continue to step 4.
4. merge all collections in contact with one another until there are no more distinct collections in contact
5. set time_left = time_left - min_time_collision
6. if time_left > 0 go to step 1. else if time_left <= 0 continue to step 7.
7. Get resting contacts, merge them appropriately, and restore all prims velocities in prim_list with backup created in step 0.
8. For each collection with is_own_proxy true, form contact manifold and resolve (collisions).
*/
void step_physics_0(std::vector<prim> & prim_list, double time_step, int step_limit, double tolerance, double resting_tolerance, bool reset_sap)
{
	//step 0. begin
	std::vector<Eigen::Vector2d> backup_velocities;
	backup_velocities.reserve(prim_list.size());

	std::deque<collection> collection_list;
	if(sapx_list.size() != prim_list.size() || reset_sap) //check if sap boxes need to be reset due to change in prim_list
	{
		sapx_list.clear();
		sapx_list.reserve(prim_list.size());
		for(size_t i = 0; i < prim_list.size(); ++i)
		{
			backup_velocities.push_back(Eigen::Vector2d(prim_list.at(i).velocity));
			collection_list.push_back(collection(prim_list.at(i)));
			sapx_list.push_back(sapbox(&collection_list.back(), time_step, i));
		}
	}
	else
	{
		for(size_t i = 0; i < prim_list.size(); ++i)
		{
			backup_velocities.push_back(Eigen::Vector2d(prim_list.at(i).velocity));
			collection_list.push_back(collection(prim_list.at(i)));
		}
		for(size_t i = 0; i < prim_list.size(); ++i)
			sapx_list.at(i).p_collection = &collection_list.at(sapx_list.at(i).index);
	}
	
	double time_left = std::fmax(0,time_step);
	std::vector<sapbox> active_sapx_list;
	active_sapx_list.reserve(sapx_list.size());
	//step 0. end

	//step 1. begin
step_1:
	active_sapx_list.clear();
	for(auto & ele0 : sapx_list)
	{
		if(!ele0.p_collection->is_own_proxy)
			continue;
		ele0.p_collection->update(); //necessary... collection.assimilate/adjoin is not updating collection parameters correctly 
		ele0.update(time_left);
	}
	insertion_sort(sapx_list.begin(), sapx_list.end(), [](sapbox & box0, sapbox & box1)->bool{ return box0.x0 < box1.x0; });
	
	std::vector<collection_contact> collisions;
	double min_time = time_left; //only equal to min_time_collision if a collision is actually found in the following
	double approx_min_time = min_time*(1+tolerance);
	std::vector<collection_contact> superset_collisions; //collision times will be decreasing in superset_collisions after loop
	size_t best_index = 0; //if superset_collisions nonempty after loop then smallest index of collision with min_time in superset_collisions.

	for(auto & ele0 : sapx_list)
	{
		if(!ele0.p_collection->is_own_proxy)
			continue;

		bool remove_something = false;
		for(auto & ele1 : active_sapx_list)
			if(ele0.x0 > ele1.x1)
			{
				ele1.is_active = false;
				remove_something = true;
			}
			else if(!(ele0.x1 < ele1.x0 || ele0.y0 > ele1.y1 || ele0.y1 < ele1.y0))
			{
				collection_contact temp = get_contacts(*ele0.p_collection, *ele1.p_collection, approx_min_time, true);
				if(std::isfinite(temp.time) && temp.time <= approx_min_time)
				{
					superset_collisions.push_back(temp); //a collision is found within time_left
					if(temp.time < min_time)
					{
						min_time = temp.time; //the collision found occurs strictly sooner than the previously best found collision (if any, else time_left)
						approx_min_time = min_time*(1+tolerance);
						best_index = superset_collisions.size()-1;
					}
				}
			}
		if(remove_something)
		{
			active_sapx_list.erase(std::remove_if(active_sapx_list.begin(), active_sapx_list.end(), [](sapbox & box)->bool{ return !box.is_active; }), active_sapx_list.end());
		}
		active_sapx_list.push_back(ele0);
	}
	collisions.reserve(superset_collisions.size() - best_index); //best index cannot exceed superset_collisions.size() so no worry about unsigned integer arithmetic
	if(tolerance>0)
		for(size_t j = 0; j < best_index; ++j)
		{
			collection_contact& temp = superset_collisions.at(j);	
			if(std::isfinite(temp.time) && temp.time <= approx_min_time)
				collisions.push_back(superset_collisions.at(j));
		}

	for(size_t j=best_index; j<superset_collisions.size(); ++j)
		collisions.push_back(superset_collisions.at(j));
	//step 1. end
	
	//step 2. begin
	for(size_t i=0; i<collection_list.size(); ++i)
	{
		if(collection_list.at(i).is_own_proxy)
		{
			collection_list.at(i).position += min_time*collection_list.at(i).velocity;
			for(auto & p_ele : collection_list.at(i).prim_pointers)
				p_ele->position += min_time*(p_ele->velocity);
		}
	}
	//step 2. end

	//step 3. begin
	if(collisions.size() == 0) //no collisions so wrap up
		goto step_7;
	//step 3. end

	//step 4. begin
	for(auto & ele : collisions)
	{
		ele.p_collection_0->get_proxy().assimilate(ele.p_collection_1->get_proxy()); //if collection_1 and collection_0 same proxy then nothing happens this line
		ele.p_collection_0->get_proxy().internal_contacts.insert(ele.p_collection_0->get_proxy().internal_contacts.end(), ele.contact_list.begin(), ele.contact_list.end()); 
		ele.p_collection_0->get_proxy().homogenize_velocities(); //experimenting with bound on number of collisions in timestep (no tunneling though);
	}
	//step 4. end
	
	//step 5. begin
	time_left = time_left - min_time;
	//step 5. end
	
	//step 6. begin
	if(time_left>0)
		goto step_1;
	//step 6. end

	//step 7. begin
step_7:
	//get resting (non-colliding) contacts
	collisions.clear();	
	for(auto & ele0 : sapx_list)
	{
		if(!ele0.p_collection->is_own_proxy)
			continue;
		else
		{
			ele0.p_collection->update();
			ele0.update(0, resting_tolerance);
		}
	}

	insertion_sort(sapx_list.begin(), sapx_list.end(), [](sapbox & box0, sapbox & box1)->bool{ return box0.x0 < box1.x0; });
	active_sapx_list.clear();
	for(auto & ele0 : sapx_list)
	{
		if(!ele0.p_collection->is_own_proxy)
			continue;

		bool remove_something = false;
		for(auto & ele1 : active_sapx_list)
			if(ele0.x0 > ele1.x1)
			{
				ele1.is_active = false;
				remove_something = true;
			}
			else if(!(ele0.x1 < ele1.x0 || ele0.y0 > ele1.y1 || ele0.y1 < ele1.y0))
			{
				collection_contact resting_contact = get_contacts(*ele0.p_collection, *ele1.p_collection, 0, false, resting_tolerance);
				if(!resting_contact.has_collision && resting_contact.time <= 0)
					collisions.push_back(resting_contact);
			}
		if(remove_something)
		{
			active_sapx_list.erase(std::remove_if(active_sapx_list.begin(), active_sapx_list.end(), [](sapbox & box)->bool{ return !box.is_active; }), active_sapx_list.end());
		}
		active_sapx_list.push_back(ele0);
	}
	for(auto & ele : collisions)
	{
		ele.p_collection_0->get_proxy().assimilate(ele.p_collection_1->get_proxy()); //if collection_1 and collection_0 same proxy then nothing happens this line
		ele.p_collection_0->get_proxy().internal_contacts.insert(ele.p_collection_0->get_proxy().internal_contacts.end(), ele.contact_list.begin(), ele.contact_list.end()); 
	}
	
	//restore velocities
	for(size_t i=0; i<prim_list.size(); ++i)
	{
		prim_list.at(i).velocity = backup_velocities.at(i);
	}
	//step 7. end

	//steps 8. begin
	contact_manifold the_manifold; //single manifold re-use
	for(size_t i=0; i<collection_list.size(); ++i)
	{
		if(collection_list.at(i).is_own_proxy)
		{
			the_manifold.prim_pointers = collection_list.at(i).prim_pointers;
			the_manifold.contact_list = collection_list.at(i).internal_contacts;
			the_manifold.resolve(step_limit, 0);
		}
	}
	//steps 8. end
	
	return;
}

//for non-performance (behavior) comparison purposes
//should probably redesign from scratch with intent to resolve all (as many as possible) collisions as soon as they occur instead of altering above code.
//grinds to a halt with low tolerances+inelastic collisions since successive collisions are so close together in time (step 2. gives 0 or tiny timestep)
/*
0. Initialize collection_list_0 and collection_list with input prims and time_left with time_step.
   while time_left > 0 do 
	1. Get all min time collision containing collection_contacts
	2. Step all distinct collections forward to min(time_left, min time collision) 
	3. If no collisions found in step 1. then return else get resting_contacts.
	4. Merge all collections in contact with one another until there are no more distinct collections in contact
	5. For each collection with is_own_proxy true and at least one internal contact, form contact manifold and resolve (collisions).
	6. Set time_left = time_left - min_time_collision and collection_list = collection_list_0
*/

void step_physics_1(std::vector<prim> & prim_list, double time_step, int step_limit, double tolerance, double resting_tolerance, bool reset_sap)
{
	//step 0. begin
	std::vector<collection> collection_list_0;
	collection_list_0.reserve(prim_list.size());
	std::vector<collection> collection_list;
	collection_list.reserve(prim_list.size());
	if(sapx_list.size() != prim_list.size() || reset_sap)
	{
		sapx_list.clear();
		sapx_list.reserve(prim_list.size());
		for(size_t i = 0; i < prim_list.size(); ++i)
		{
			collection_list_0.push_back(collection(prim_list.at(i)));
			collection_list.push_back(collection(prim_list.at(i)));
			sapx_list.push_back(sapbox(&collection_list.back(), time_step, i));
		}
	}
	else
	{
		for(size_t i = 0; i < prim_list.size(); ++i)
		{
			collection_list_0.push_back(collection(prim_list.at(i)));
			collection_list.push_back(collection(prim_list.at(i)));
		}
		for(size_t i = 0; i < prim_list.size(); ++i)
			sapx_list.at(i).p_collection = &collection_list.at(sapx_list.at(i).index);
	}
	
	size_t collection_list_size = collection_list.size();
	double time_left = time_step;
	//step 0. end	
	std::vector<sapbox> active_sapx_list;
	active_sapx_list.reserve(sapx_list.size());
	while(time_left > 0)
	{
		//step 1. begin
		active_sapx_list.clear();
		for(size_t i=0; i < sapx_list.size(); ++i)
		{
			collection_list.at(sapx_list.at(i).index).update();
			sapx_list.at(i).update(time_left, 0);
		}
		insertion_sort(sapx_list.begin(), sapx_list.end(), [](sapbox & box0, sapbox & box1)->bool{ return box0.x0 < box1.x0; });
		std::vector<collection_contact> collisions;
		double min_time = time_left; //only equal to min_time_collision if a collision is actually found in the following
		double approx_min_time = min_time*(1+tolerance); //min_time+tolerance;//min_time*(1+tolerance);
		std::vector<collection_contact> superset_collisions; //collision times will be decreasing in superset_collisions after loop
		size_t best_index = 0; //if superset_collisions nonempty after loop then smallest index of collision with min_time in superset_collisions.

		for(auto & ele0 : sapx_list)
		{
			bool remove_something = false;
			for(auto & ele1 : active_sapx_list)
				if(ele0.x0 > ele1.x1)
				{
					ele1.is_active = false;
					remove_something = true;
				}
				else if(!(ele0.x1 < ele1.x0 || ele0.y0 > ele1.y1 || ele0.y1 < ele1.y0))
				{
					collection_contact temp = get_contacts(*ele0.p_collection, *ele1.p_collection, approx_min_time, true);
					if(std::isfinite(temp.time) && temp.time <= approx_min_time)
					{
						superset_collisions.push_back(temp); //a collision is found within time_left
						if(temp.time < min_time)
						{
							min_time = temp.time; //the collision found occurs strictly sooner than the previously best found collision (if any, else time_left)
							approx_min_time = min_time*(1+tolerance); //min_time+tolerance; //min_time*(1+tolerance);
							best_index = superset_collisions.size()-1;
						}
					}
				}
			if(remove_something)
			{
				active_sapx_list.erase(std::remove_if(active_sapx_list.begin(), active_sapx_list.end(), [](sapbox & box)->bool{ return !box.is_active; }), active_sapx_list.end());
			}
			active_sapx_list.push_back(ele0);
		}
		collisions.reserve(superset_collisions.size() - best_index); //best index cannot exceed superset_collisions.size() so no worry about unsigned integer arithmetic
		if(tolerance>0)
			for(size_t j = 0; j < best_index; ++j)
			{
				collection_contact& temp = superset_collisions.at(j);	
				if(std::isfinite(temp.time) && temp.time <= approx_min_time)
					collisions.push_back(superset_collisions.at(j));
			}

		for(size_t j=best_index; j<superset_collisions.size(); ++j)
			collisions.push_back(superset_collisions.at(j));
		//step 1. end

		//step 2. begin
		for(size_t j=0; j<collection_list_size; ++j)
		{
			size_t i = sapx_list.at(j).index;
			collection_list.at(i).position += min_time*collection_list.at(i).velocity;
			for(auto & p_ele : collection_list.at(i).prim_pointers)
				p_ele->position += min_time*(p_ele->velocity);
			collection_list.at(i).update_enclosing_circle();
			sapx_list.at(j).update(0, resting_tolerance); //for resting contact determination
		}
		//step 2. end

		//step 3. begin
		if(collisions.size() == 0) //no collisions so done
			return;
		//get resting (non-colliding) contacts
		insertion_sort(sapx_list.begin(), sapx_list.end(), [](sapbox & box0, sapbox & box1)->bool{ return box0.x0 < box1.x0; });
		active_sapx_list.clear();
		for(auto & ele0 : sapx_list)
		{
			bool remove_something = false;
			for(auto & ele1 : active_sapx_list)
				if(ele0.x0 > ele1.x1)
				{
					ele1.is_active = false;
					remove_something = true;
				}
				else if(!(ele0.x1 < ele1.x0 || ele0.y0 > ele1.y1 || ele0.y1 < ele1.y0))
				{
					collection_contact resting_contact = get_contacts(*ele0.p_collection, *ele1.p_collection, 0, false, resting_tolerance);
					if(!resting_contact.has_collision && resting_contact.time <= 0)
						collisions.push_back(resting_contact);
				}
			if(remove_something)
			{
				active_sapx_list.erase(std::remove_if(active_sapx_list.begin(), active_sapx_list.end(), [](sapbox & box)->bool{ return !box.is_active; }), active_sapx_list.end());
			}
			active_sapx_list.push_back(ele0);
		}
		//step 3. end
		
		//step 4. begin
		for(auto & ele : collisions)
		{
			ele.p_collection_0->get_proxy().assimilate(ele.p_collection_1->get_proxy()); //contact_list; //if collection_1 and collection_0 same proxy then nothing happens this line
			ele.p_collection_0->get_proxy().internal_contacts.insert(ele.p_collection_0->get_proxy().internal_contacts.end(), ele.contact_list.begin(), ele.contact_list.end()); 
		}
		//step 4. end
		
		//step 5. begin
		contact_manifold the_manifold; //single manifold re-use
		for(size_t i=0; i<collection_list_size; ++i)
		{
			if(collection_list.at(i).is_own_proxy)
				if(collection_list.at(i).internal_contacts.size())
				{
					the_manifold.prim_pointers = collection_list.at(i).prim_pointers;
					the_manifold.contact_list = collection_list.at(i).internal_contacts;
					the_manifold.resolve(step_limit, 0);
				}
		}
		//step 5. end
			
		//step 6. begin
		time_left = time_left - min_time;
		collection_list = collection_list_0;
		//step 6. end
	}
	
	return;
}
