#include "collection.h"
#include "log.h"

collection::collection() : mass(0), position{Eigen::Vector2d(0,0)}, velocity{Eigen::Vector2d(0,0)}, is_own_proxy(true), p_proxy(this), enclosing_circle() {}
collection::collection(prim & reference_prim) : mass(0), position{Eigen::Vector2d(0,0)}, velocity{Eigen::Vector2d(0,0)}, is_own_proxy(true), p_proxy(this), enclosing_circle()
{
	adjoin(reference_prim);
	enclosing_circle = reference_prim;
}

collection::collection(const collection & reference_collection) 
	: mass(reference_collection.mass), position(reference_collection.position), velocity(reference_collection.velocity), is_own_proxy(true), p_proxy(this), enclosing_circle(reference_collection.enclosing_circle), prim_pointers(reference_collection.prim_pointers), internal_contacts(reference_collection.internal_contacts)
{
	if(!(reference_collection.is_own_proxy))
	{
		is_own_proxy = false;
		p_proxy = reference_collection.p_proxy;
	}
}

collection::~collection() {}

collection& collection::operator=(collection&& ref)
{
	if(&ref == this)
		return *this;
	mass = ref.mass;
	position = ref.position;
	velocity = ref.velocity;
	enclosing_circle = ref.enclosing_circle;
	prim_pointers = ref.prim_pointers;
	internal_contacts = ref.internal_contacts;
	if(ref.is_own_proxy)
	{
		is_own_proxy = true;
		p_proxy = this;
	}
	else
	{
		is_own_proxy = false;
		p_proxy = ref.p_proxy;
	}

	return *this;
}

collection& collection::operator=(const collection& ref)
{
	if(&ref == this)
		return *this;
	mass = ref.mass;
	position = ref.position;
	velocity = ref.velocity;
	enclosing_circle = ref.enclosing_circle;
	prim_pointers = ref.prim_pointers;
	internal_contacts = ref.internal_contacts;
	if(ref.is_own_proxy)
	{
		is_own_proxy = true;
		p_proxy = this;
	}
	else
	{
		is_own_proxy = false;
		p_proxy = ref.p_proxy;
	}

	return *this;
}

void collection::update_enclosing_circle()
{
	//gives enclosing circle that for at most 2 is minimal and which for more than 2 objects gives
	//best case approximately 28.9% bigger than the smallest enclosing circle
	//worst case approximately 36.6% bigger than the smallest enclosing circle
	
	if(prim_pointers.size() == 0)
		return;
	if(prim_pointers.size() == 1)
		enclosing_circle = *prim_pointers.at(0);

	double diameter = 0;
	Eigen::Vector2d position_1, position_2, position_3;

	for(size_t i=0; i<prim_pointers.size()-1; ++i)
	{
		Eigen::Vector2d slope;
		for(size_t j = i+1; j < prim_pointers.size(); ++j)
		{
			slope = (prim_pointers.at(j)->position - prim_pointers.at(i)->position);
			double temp = slope.norm() + prim_pointers.at(j)->radius + prim_pointers.at(i)->radius;

			if(temp > diameter)
			{
				diameter = temp;
				slope.normalize();

				position_1 = prim_pointers.at(i)->position - prim_pointers.at(i)->radius * slope;
				position_2 = prim_pointers.at(j)->position + prim_pointers.at(j)->radius * slope;
				position_3 = (position_1 + position_2)/2;
			}
		}
	}
	double radius = (std::sqrt(3)/2)*diameter;
	enclosing_circle.position = position_3;
	enclosing_circle.radius = radius;
	enclosing_circle.velocity = velocity;
}

void collection::update_enclosing_circle(const prim & prim_input)
{
	if(prim_pointers.size() == 0)
	{
		enclosing_circle = prim_input;
		return;
	}

	//quick
	if((prim_input.position - enclosing_circle.position).norm() + prim_input.radius <= enclosing_circle.radius)
		return;

	Eigen::Vector2d difference = prim_input.position - enclosing_circle.position;
	double new_radius = (difference.norm() + prim_input.radius + enclosing_circle.radius)/2;
	enclosing_circle.position = (prim_input.position + enclosing_circle.position + (prim_input.radius-enclosing_circle.radius)*difference.normalized())/2;
	enclosing_circle.radius = new_radius;
	enclosing_circle.velocity = velocity;
}

void collection::update_enclosing_circle(const collection & collection_input)
{
	update_enclosing_circle(collection_input.enclosing_circle);
}

void collection::free()
{
	mass = 0;
	position = Eigen::Vector2d(0,0);
	velocity = Eigen::Vector2d(0,0);

	std::vector<prim*>().swap(prim_pointers);
	std::vector<contact>().swap(internal_contacts);
}

void collection::adjoin(prim & prim_reference)
{
	//treating max() as +infinity as IPOPT has trouble with std::_numeric_limits<double>::infinity()
	if(mass >= std::numeric_limits<double>::max() && prim_reference.mass < std::numeric_limits<double>::max()) 
	{
	}
	else if(mass < std::numeric_limits<double>::max() && prim_reference.mass >= std::numeric_limits<double>::max())
	{
		mass = prim_reference.mass;
		position = prim_reference.position;
		velocity = prim_reference.velocity;
	}
	else if(mass >= std::numeric_limits<double>::max() && prim_reference.mass >= std::numeric_limits<double>::max())
	{
		position = (.5)*(position + prim_reference.position);
		velocity = (.5)*(velocity + prim_reference.velocity);
	}
	else
	{
		double m1 = std::fmin(mass, prim_reference.mass);
		double m2 = std::fmax(mass, prim_reference.mass);
		Eigen::Vector2d p1, p2, v1, v2;
		if(m1 == mass && m2 == prim_reference.mass)
		{
			p1 = position;
			p2 = prim_reference.position;
			v1 = velocity;
			v2 = prim_reference.velocity;
		}
		else if(m1 == prim_reference.mass && m2 == mass)
		{
			p2 = position;
			p1 = prim_reference.position;
			v2 = velocity;
			v1 = prim_reference.velocity;
		}
		else
		{
			LOG("Warning in void collection::adjoin(prim &): could not determine m1 and m2, exiting without updating parameters.");
			return;
		}
		double ratio = std::fmin(mass,prim_reference.mass)/std::fmax(mass,prim_reference.mass);
		double w1 = ratio/(1+ratio);
		double w2 = 1.0/(1+ratio);
		bool update_position = true;
		bool update_velocity = true;
		if(!(std::isfinite(ratio) && std::isfinite(1+ratio) && std::isfinite(w1) && std::isfinite(w2)))
		{
			LOG("Warning in void collection::adjoin(prim &): !(std::isfinite(ratio) && std::isfinite(1+ratio) && std::isfinite(w1) && std::isfinite(w2)), exiting without updating parameters.");
			return;
		}
		if(!(std::isfinite(w1*p1(0)) && std::isfinite(w1*p1(1)) && std::isfinite(w2*p2(0)) && std::isfinite(w2*p2(1))))
		{
			LOG("Warning in void collection::adjoin(prim &): !(std::isfinite(w1*p1(0)) && std::isfinite(w1*p1(1)) && std::isfinite(w2*p2(0)) && std::isfinite(w2*p2(1))), could not update position.");
			update_position = false;
		}
		if(!(std::isfinite(w1*v1(0)) && std::isfinite(w1*v1(1)) && std::isfinite(w2*v2(0)) && std::isfinite(w2*v2(1))))
		{
			LOG("Warning in void collection::adjoin(prim &): !(std::isfinite(w1*v1(0)) && std::isfinite(w1*v1(1)) && std::isfinite(w2*v2(0)) && std::isfinite(w2*v2(1))), could not update velocity.");
			update_velocity = false;
		}
		double new_mass = mass + prim_reference.mass;
		if(!std::isfinite(new_mass))
		{
			mass = std::numeric_limits<double>::max();
		}
		else
		{
			mass = new_mass;
		}
		if(update_position)
		{
			position = w1*p1+w2*p2;
		}
		if(update_velocity)
		{
			velocity = w1*v1+w2*v2;
		}
	}

	prim_pointers.push_back(&prim_reference);
	update_enclosing_circle(prim_reference);
}

void collection::adjoin(collection & collection_reference)
{
	if(&collection_reference == this)
		return;
	
	//treating max() as +infinity as IPOPT has trouble with std::_numeric_limits<double>::infinity()
	if(mass >= std::numeric_limits<double>::max() && collection_reference.mass < std::numeric_limits<double>::max()) 
	{
	}
	else if(mass < std::numeric_limits<double>::max() && collection_reference.mass >= std::numeric_limits<double>::max())
	{
		mass = collection_reference.mass;
		position = collection_reference.position;
		velocity = collection_reference.velocity;
	}
	else if(mass >= std::numeric_limits<double>::max() && collection_reference.mass >= std::numeric_limits<double>::max())
	{
		position = (.5)*(position + collection_reference.position);
		velocity = (.5)*(velocity + collection_reference.velocity);
	}
	else
	{
		double m1 = std::fmin(mass, collection_reference.mass);
		double m2 = std::fmax(mass, collection_reference.mass);
		Eigen::Vector2d p1, p2, v1, v2;
		if(m1 == mass && m2 == collection_reference.mass)
		{
			p1 = position;
			p2 = collection_reference.position;
			v1 = velocity;
			v2 = collection_reference.velocity;
		}
		else if(m1 == collection_reference.mass && m2 == mass)
		{
			p2 = position;
			p1 = collection_reference.position;
			v2 = velocity;
			v1 = collection_reference.velocity;
		}
		else
		{
			LOG("Warning in void collection::adjoin(collection &): could not determine m1 and m2, exiting without updating parameters.");
			return;
		}
		double ratio = std::fmin(mass,collection_reference.mass)/std::fmax(mass,collection_reference.mass);
		double w1 = ratio/(1+ratio);
		double w2 = 1.0/(1+ratio);
		bool update_position = true;
		bool update_velocity = true;
		if(!(std::isfinite(ratio) && std::isfinite(1+ratio) && std::isfinite(w1) && std::isfinite(w2)))
		{
			LOG("Warning in void collection::adjoin(collection &): !(std::isfinite(ratio) && std::isfinite(1+ratio) && std::isfinite(w1) && std::isfinite(w2)), exiting without updating parameters.");
			return;
		}
		if(!(std::isfinite(w1*p1(0)) && std::isfinite(w1*p1(1)) && std::isfinite(w2*p2(0)) && std::isfinite(w2*p2(1))))
		{
			LOG("Warning in void collection::adjoin(collection &): !(std::isfinite(w1*p1(0)) && std::isfinite(w1*p1(1)) && std::isfinite(w2*p2(0)) && std::isfinite(w2*p2(1))), could not update position.");
			update_position = false;
		}
		if(!(std::isfinite(w1*v1(0)) && std::isfinite(w1*v1(1)) && std::isfinite(w2*v2(0)) && std::isfinite(w2*v2(1))))
		{
			LOG("Warning in void collection::adjoin(collection &): !(std::isfinite(w1*v1(0)) && std::isfinite(w1*v1(1)) && std::isfinite(w2*v2(0)) && std::isfinite(w2*v2(1))), could not update velocity.");
			update_velocity = false;
		}
		double new_mass = mass + collection_reference.mass;
		if(!std::isfinite(new_mass))
		{
			mass = std::numeric_limits<double>::max();
		}
		else
		{
			mass = new_mass;
		}
		if(update_position)
		{
			position = w1*p1+w2*p2;
		}
		if(update_velocity)
		{
			velocity = w1*v1+w2*v2;
		}
	}
	prim_pointers.insert(prim_pointers.end(), collection_reference.prim_pointers.begin(), collection_reference.prim_pointers.end());
	internal_contacts.insert(internal_contacts.end(), collection_reference.internal_contacts.begin(), collection_reference.internal_contacts.end());
	update_enclosing_circle(collection_reference);
}

//for use in merging collections to avoid deleting or rearranging std::vector<collection> entries.
//just dump all of one collection into another and have the now empty collection hold a pointer p_proxy to the new collection
void collection::assimilate(collection & collection_reference)
{
	if(&collection_reference == this)
		return;

	adjoin(collection_reference);
	collection_reference.p_proxy = p_proxy;
	collection_reference.is_own_proxy = false;
}

//pointer to collection with is_own_proxy true, containing this collection as a subcollection
collection& collection::get_proxy() 
{
	if(p_proxy == this)
	{
		return *p_proxy;
	}
	else if(p_proxy == nullptr)
	{
		p_proxy= this;
		return *p_proxy;
	}
	
	while(true)
	{
		if(p_proxy != p_proxy->p_proxy)
			p_proxy= p_proxy->p_proxy;
		else
			return *p_proxy;
	}
}

void collection::homogenize_velocities() //for merging collections
{
	for(auto & element : prim_pointers)
		element->velocity = velocity;
}
//update physics parameters 
void collection::update()
{
	if(prim_pointers.size() == 0)
	{
		mass = 0;
		position = Eigen::Vector2d(0,0);
		velocity = position;
		enclosing_circle = prim();
	}
	if(prim_pointers.size() == 1)
	{
		mass = prim_pointers.at(0)->mass;
		position = prim_pointers.at(0)->position;
		velocity = prim_pointers.at(0)->velocity;
		enclosing_circle = *prim_pointers.at(0);
	}

	long double new_mass = 0;
	long double new_x_0 = 0;
	long double new_x_1 = 0;
	long double new_v_0 = 0;
	long double new_v_1 = 0;
	bool has_max_mass_prim = false;
	for(auto & ele : prim_pointers)
	{
		if(ele->mass >= std::numeric_limits<double>::max())
		{
			has_max_mass_prim = true;
			break;
		}
		
		new_mass += ele->mass;
		new_x_0 += (ele->mass)*(ele->position(0));
		new_x_1 += (ele->mass)*(ele->position(1));
		new_v_0 += (ele->mass)*(ele->velocity(0));
		new_v_1 += (ele->mass)*(ele->velocity(1));
		
		if(!(std::isfinite(new_mass) && std::isfinite(new_x_0) && std::isfinite(new_x_1) && std::isfinite(new_v_0) && std::isfinite(new_v_1)))
		{
			LOG("Warning collection::update() computes nonfinite_sum. Terminating without update.");
			return;
		}
	}

	if(has_max_mass_prim)
	{
		new_mass = std::numeric_limits<double>::max();
		new_x_0 = 0;
		new_x_1 = 0;
		new_v_0 = 0;
		new_v_1 = 0;
		size_t max_count = 0;
		for(auto & ele : prim_pointers)
		{
			if(ele->mass >= mass)
			{
				++max_count;
				new_x_0 += (ele->position(0));
				new_x_1 += (ele->position(1));
				new_v_0 += (ele->velocity(0));
				new_v_1 += (ele->velocity(1));
			}
		}
		new_x_0 = new_x_0/static_cast<long>(max_count);
		new_x_1 = new_x_1/static_cast<long>(max_count);
		new_v_1 = new_v_0/static_cast<long>(max_count);
		new_v_1 = new_v_1/static_cast<long>(max_count);
		if(!(std::isfinite(new_x_0) && std::isfinite(new_x_1) && std::isfinite(new_v_0) && std::isfinite(new_v_1)))
		{
			LOG("Warning collection::update() computes nonfinite_sum. Terminating without update.");
			return;
		}
		mass = new_mass;
		position(0) = new_x_0;
		position(1) = new_x_1;
		velocity(0) = new_v_0;
		velocity(1) = new_v_1;
		update_enclosing_circle();
		return;
	}
	
	new_x_0 /= new_mass;
	new_x_1 /= new_mass;
	new_v_0 /= new_mass;
	new_v_1 /= new_mass;

	if(!(std::isfinite(new_x_0) && std::isfinite(new_x_1) && std::isfinite(new_v_0) && std::isfinite(new_v_1)))
	{
		LOG("Warning collection::update() computes nonfinite_sum. Terminating without update.");
		return;
	}
	
	mass = new_mass;
	position(0) = new_x_0;
	position(1) = new_x_1;
	velocity(0) = new_v_0;
	velocity(1) = new_v_1;

	update_enclosing_circle();
}
