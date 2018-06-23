#ifndef COLLECTION_H
#define COLLECTION_H
#include "contact.h"

struct collection//collision collection 
{
	double mass;//, angular_mass;
	Eigen::Vector2d position, velocity; //center of mass

	std::vector<prim*> prim_pointers;
	std::vector<contact> internal_contacts; //contacts between *prim_pointers

	prim enclosing_circle; //To speed up contact queries in method 0

	collection* p_proxy;

	bool is_own_proxy; 

	void free(); //clears and frees the memory used by the vectors prim_pointers and internal_contacts. Unused and unneeded.

	void adjoin(prim & prim_reference); //Add prim to collection, updates mass, position, velocity.
	//Does not alter internal_contacts, prim_reference.position, or prim_reference.velocity;
        //Assumes prim_reference isn't already in collection (does no checking to prevent duplicates).

	void adjoin(collection & collection_reference); //add all prims in collection_reference to collection
	//Does alter internal_contacts via adjoining collection_reference.internal_contacts but does not check for new contacts between current prims and prims in collection_reference being adjoined.
	//Does not alter prim positions or velocitys
	
	void homogenize_velocities(); //sets all prim_pointers velocities to velocity.
	
	void assimilate(collection & collection_reference); //if(&collection_reference == this) then does nothing else
	//calls adjoin(collection_reference), sets collection_reference.p_proxy = p_proxy, and collection_reference.is_own_proxy = false
	//make sure only called on this object if this object has is_own_proxy true.

	collection& get_proxy(); //updates p_proxy then returns its dereferenced updated value.

	void update_enclosing_circle();
	void update_enclosing_circle(const prim & prim_input);
	void update_enclosing_circle(const collection & collection_input);
	void update();//update physics parameters 

	collection();
	collection(prim & reference_prim);
	collection(const collection & reference_collection); //copy constructor
	~collection();

	collection& operator=(collection&& ref);
	collection& operator=(const collection& ref);
};

#endif
