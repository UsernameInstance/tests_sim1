#include "contact_manifold.h"
#include "quadratic_program.h"
#include "log.h"

contact_manifold::contact_manifold() {}
contact_manifold::~contact_manifold() {}

contact_manifold::contact_manifold(collection & input_collection) 
	: prim_pointers(input_collection.prim_pointers), contact_list(input_collection.internal_contacts)
{
}

contact_manifold::contact_manifold(std::vector<collection> & collection_list) 
{
	for(auto & ele : collection_list)
	{
		if(ele.is_own_proxy)
		{
			prim_pointers.insert(prim_pointers.end(), ele.prim_pointers.begin(), ele.prim_pointers.end());
			contact_list.insert(contact_list.end(), ele.internal_contacts.begin(), ele.internal_contacts.end());
		}
	}
}


/*
 *	std::vector<prim*> prim_pointers;
	std::vector<contact> contact_list;
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
contact get_contact(prim & prim0, prim & prim1, double time_limit = std::numeric_limits<double>::infinity(), bool collision_only = false);
*/

/* Planar frictionless motion .
 * note semicolon used to denote new column of matrix in this comment, and space new row.
 * let O = {o_1, ..., o_{|O|}} be the set of the center of masses of the rigid bodies in the contact manifold 
 * and C = {c_1, ..., c_{|C|}} be the contact points in the contact manifold.
 * let m_i be the mass of body i, 
 * I_i the moment of inertia of body i, 
 * y_{i,j} contact point i on body j whenever c_i on body j and 0 otherwise,
 * c_{i,j} = c_i - o_j, and 
 * n_{i,j} the outward unit normal for body j at y_{i,j} whenever c_i on body j and 0 otherwise.
 * let p_i denote the magnitude of the impulse applied at c_i to resolve the collision, and 
 * let p = [p_1 ... p_{|C|}]^{T}.
 * Let M_i = [m_i  0   0 ;
 * 	      0   m_i  0 ;
 * 	      0    0  I_i], 
 * (M_i)^{-1] = [(m_i)^{-1} 	0 		0 	  ;
 * 		 0 		(m_i)^{-1} 	0	  ; 
 * 		 0 		0 		(I_i)^{-1}]. 
 * Let N_i = 		     [ -n_{1,i}.dot((1,0,0)) ... -n_{|C|,i}.dot((1,0,0));
 * 	      		       -n_{1,i}.dot((0,1,0)) ... -n_{|C|,i}.dot((0,1,0));
 * 	       -(c_{1,i}.cross(n_{1,i}).dot((0,0,1)) ... -(c_{|C|,i}.cross(n_{|C|,i}).dot((0,0,1))],
 * let M be the block diagonal matrix with block M_i the ith diagonal block for i in {1,...,|O|}, and
 * let M^{-1} the block diagonal matrix with block (M_i)^{i} the ith diagonal block for i in {1,...,|O|}, and
 * let N be the block matrix
 * N = [N_1    ;
 * 	N_2    ;
 * 	 :
 * 	N_{|O|}].
 * let q denote the velocities of the bodys after collisions resolved (ignoring resting contacts iff ignore_resting_contacts) and q(0) their initial velocities
 * where for each i v_i = [v_i.dot((1,0,0)); v_i.dot((0,1,0))] denotes the velocity of the center of mass of of body i 
 * and w_i is s.t the angular velocity of body i is w_i*[0; 0; 1] then q = [[v_1 ; w_1] ; [v_2 ; w_2;] ; ... ; [v_{|O|} ; w_{|O|}]]
 * (for q(0) replace the components of q by the corresponding initial velocities). 
 *
 * With these definitions,
 *
 * 	q = q(0) + M^{-1}*N*p
 *
 * and p is an argmin of the quadratic program
 *
 * 	min_{x}( x^{T}*(I+E)*N^{T}*q(0) + x^{T}*N^{T}*M^{-1}*N*x )
 * 	s.t. 	(I+E)*N^{T}*q(0) + N^{T}*M^{-1}*N*x >= 0
 * 		x >= 0 
 *
 * where E is some matrix with coefficients in the closed unit interval [0,1] depending on the bodies in the contact manifold
 * and I is the identity matrix with dimensions equal to the dimensions of E and
 * E is either a 1 by 1 denoting a scalar (in which case I = 1), or E has number of rows and columns equal to |C|.
 * E corresponds to coefficient of restitution in case of single contact point.
 
 * For no rotation remove all w_i and I_i columns/rows/etc.. from the preceeding.
 */
//assuming all prim in any contact in contact_list have a pointer in prim_pointers
//and all elements of prim_pointers are distinct
bool contact_manifold::impact_operator(bool ignore_resting_contacts)
{
	struct  aux
	{
		aux(contact* pc = nullptr) : pc(pc) {};
		contact* pc; //pointer to contact
		size_t in0; //row index corresponding to pc->p_prim_0->velocity(0) in N
		size_t in1; //row index corresponding to pc->p_prim_1->velocity(0) in N
	};
	std::vector<aux> auxvec;
	auxvec.reserve(contact_list.size());
	if(ignore_resting_contacts)
	{
		for(auto & ele : contact_list)
			if(ele.is_collision)
				auxvec.push_back(&ele);
	}
	else
	{
		for(auto & ele: contact_list)
			auxvec.push_back(&ele);
	}

	std::vector<prim*> ppvec; //prim pointer vector to prims not ignored (due to ignore_resting_contacts)
	ppvec.reserve(auxvec.size()+1);
	for(size_t i=0; i< prim_pointers.size(); ++i) //assumes no duplicates in prim_pointers
	{
		bool push = false;
		for(size_t j = 0; j<auxvec.size(); ++j)
		{
			if(prim_pointers.at(i) == auxvec.at(j).pc->p_prim_0 ||
					prim_pointers.at(i) == auxvec.at(j).pc->p_prim_1)
			{
				if(prim_pointers.at(i) != auxvec.at(j).pc->p_prim_1)
					auxvec.at(j).in0 = 2*ppvec.size();
				else if(prim_pointers.at(i) != auxvec.at(j).pc->p_prim_0)
					auxvec.at(j).in1 = 2*ppvec.size();
				else //if self-contact is possible
				{
					auxvec.at(j).in0 = 2*ppvec.size();
					auxvec.at(j).in1 = 2*ppvec.size();
				}
				push = true;
			}
		}
		if(push)
			ppvec.push_back(prim_pointers.at(i));
	}
	

	//init initial velocities
	Eigen::VectorXd q0(2*ppvec.size()); //no rotation yet so only 2*
	for(int k=0; k<ppvec.size(); ++k)
	{
		q0(2*k) = ppvec.at(k)->velocity(0);
		q0(2*k+1) = ppvec.at(k)->velocity(1);
	}

	//B = I+E
	Eigen::SparseMatrix<double> B(auxvec.size(), auxvec.size());
	B.reserve(Eigen::VectorXi::Constant(auxvec.size(),1));
	for(size_t i=0; i<auxvec.size(); ++i)
		B.insert(i,i) = 1+(auxvec.at(i).pc->p_prim_0->bounciness + auxvec.at(i).pc->p_prim_1->bounciness)/2.0;
	Eigen::SparseMatrix<double> M_inv(2*ppvec.size(), 2*ppvec.size());
	M_inv.reserve(Eigen::VectorXi::Constant(2*ppvec.size(),1));

	for(size_t i=0; i<ppvec.size(); ++i)
	{
		double temp;
		if(ppvec.at(i)->mass != 0.0)
			temp = 1.0/ppvec.at(i)->mass;
		else
		{
			temp = 0; //0 mass objects assumed to be uncollidable so excluded 
		}

		M_inv.insert(2*i,2*i) = temp;
		M_inv.insert(2*i+1,2*i+1) = temp;
	}

	Eigen::SparseMatrix<double> N(2*ppvec.size(), auxvec.size());
	std::vector<Eigen::Triplet<double>> triplet_list;
	triplet_list.reserve(4*auxvec.size());
	for(size_t i=0; i<auxvec.size(); ++i)
	{
		triplet_list.push_back(Eigen::Triplet<double>(auxvec.at(i).in0, i, -auxvec.at(i).pc->normal(0)));
		triplet_list.push_back(Eigen::Triplet<double>(auxvec.at(i).in0 + 1, i, -auxvec.at(i).pc->normal(1)));
		triplet_list.push_back(Eigen::Triplet<double>(auxvec.at(i).in1, i, auxvec.at(i).pc->normal(0)));
		triplet_list.push_back(Eigen::Triplet<double>(auxvec.at(i).in1 + 1, i, auxvec.at(i).pc->normal(1)));
	}
	N.setFromTriplets(triplet_list.begin(), triplet_list.end());

	//for input into quadratic_program
	Eigen::SparseMatrix<double> A = N.transpose()*M_inv*N;
	Eigen::VectorXd a = B*N.transpose()*q0;
	//G == A
	//g_l == -a
	Eigen::VectorXd g_u = Eigen::VectorXd::Constant(a.size(), std::numeric_limits<double>::infinity()); //g_u == +infinity vec
	Eigen::VectorXd x_l = Eigen::VectorXd::Constant(auxvec.size(), 0); //x_l == 0
	Eigen::VectorXd x_u = Eigen::VectorXd::Constant(auxvec.size(), std::numeric_limits<double>::infinity()); //x_u == +infinity vec

	quadratic_program the_QP(A, a, A, -a, g_u, x_l, x_u);
	the_QP.optimize();

	Eigen::VectorXd p = the_QP.get_solution(); 
	if(p.size() != N.cols())
	{
		LOG("Error in contact_manifold::impact_operator: p.size() != N.cols()");
		return false;
	}
	Eigen::VectorXd q = q0 + M_inv*N*p;

	for(size_t i=0; i<ppvec.size(); ++i)
	{
		ppvec.at(i)->velocity(0) = q(2*i);
		ppvec.at(i)->velocity(1) = q(2*i+1);
	}

	return true;
}

bool contact_manifold::resolve(int step_limit, double tolerance)
{
	double K0 = 0;	//K0 == 2*(initial kinetic energy)
	for(size_t j=0; j<prim_pointers.size(); ++j)
	{
		double delta_K = (prim_pointers.at(j)->mass) * prim_pointers.at(j)->velocity.squaredNorm();
		if(delta_K != delta_K) //if delta_K is NaN ignore.
			continue;

		K0 += delta_K;
	}

	double collision_bound; //colliding iff normal velocity at point < collision_bound
	bool has_collision; //true iff some normal velocity at a contact point < collision_bound
	double K = K0; //2*(current kinetic energy)

	for(size_t i=0; i<step_limit-1; ++i)
	{
		has_collision = false;
		collision_bound = -tolerance*K; //check for collisions according to tolerance
		for(auto & ele : contact_list)
		{
			if((ele.p_prim_1->velocity - ele.p_prim_0->velocity).dot(ele.normal) < collision_bound)
			{
				ele.is_collision = true;
				has_collision = true;
			}
			else
			{
				ele.is_collision = false;
			}
		}
		if(!has_collision)
			return true; //if no collisions then done

		impact_operator();
		
		K = 0; //update K
		for(size_t j=0; j<prim_pointers.size(); ++j)
		{
			double delta_K = (prim_pointers.at(j)->mass) * prim_pointers.at(j)->velocity.squaredNorm();
			if(delta_K != delta_K) //if delta_K is NaN ignore.
				continue;

			K += delta_K;
		}

		double K_ratio = K0/K;

		if(K0 > 0 && K > K0 && !(K_ratio != K_ratio) ) //make sure safe
		{
			for(size_t j=0; j<prim_pointers.size(); ++j) //scale velocity to prevent kinetic energy gain
			{
				prim_pointers.at(j)->velocity = (prim_pointers.at(j)->velocity)*K_ratio;
			}
		}
		
	}

	//for final iteration in impact_operator call set ignore_resting_contacts = false as failsafe.
	has_collision = false;
	collision_bound = -tolerance*K; //check for collisions according to tolerance
	for(auto & ele : contact_list)
	{
		if((ele.p_prim_1->velocity - ele.p_prim_0->velocity).dot(ele.normal) < collision_bound)
		{
			ele.is_collision = true;
			has_collision = true;
		}
		else
		{
			ele.is_collision = false;
		}
	}
	if(!has_collision)
		return true; //if no collisions then done

	impact_operator(false);
	
	K = 0; //update K
	for(size_t j=0; j<prim_pointers.size(); ++j)
	{
		double delta_K = (prim_pointers.at(j)->mass) * prim_pointers.at(j)->velocity.squaredNorm();
		if(delta_K != delta_K) //if delta_K is NaN ignore.
			continue;

		K += delta_K;
	}

	double K_ratio = K0/K;

	if(K0 > 0 && K > K0 && !(K_ratio != K_ratio) ) //make sure safe
	{
		for(size_t j=0; j<prim_pointers.size(); ++j) //scale velocity to prevent kinetic energy gain
		{
			prim_pointers.at(j)->velocity = (prim_pointers.at(j)->velocity)*K_ratio;
		}
	}

	//one last check to see if all collisions resolved within tolerance
	has_collision = false;	
	collision_bound = -tolerance*K; //check for collisions according to tolerance
	for(auto & ele : contact_list)
	{
		if((ele.p_prim_1->velocity - ele.p_prim_0->velocity).dot(ele.normal) < collision_bound)
		{
			ele.is_collision = true;
			has_collision = true;
		}
		else
		{
			ele.is_collision = false;
		}
	}
	if(!has_collision)
		return true; //if no collisions then success 
	
	//still collisions so failure
	return false;
}

