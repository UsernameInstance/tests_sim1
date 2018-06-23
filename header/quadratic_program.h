#ifndef QUADRATIC_PROGRAM_H
#define QUADRATIC_PROGRAM_H

#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"
#include <Eigen/SparseCore>
#include <Eigen/Dense>

using namespace Ipopt; //kill this

class quadratic_program 
{
	/* quadratic_program 
	 * This presents the problem form:
	 *
	 * 	min_{x} x^{T}*A*x + a^{T}*x
	 * 	s.t. g_l <= G*x <= g_u
	 * 	     x_l <= x <= x_u 
	 */

	// TNLP interface implementation
	  /** This interface presents the problem form:
	   *  
	   *     min f(x)
	   *
	   *     s.t. g_l <= g(x) <= g_u
	   *
	   *          x_l <=  x   <= x_u
	   *
	   *  In order to specify an equality constraint, set g_l_i = g_u_i =
	   *  rhs.  The value that indicates "infinity" for the bounds
	   *  (i.e. the variable or constraint has no lower bound (-infinity)
	   *  or upper bound (+infinity)) is set through the option
	   *  nlp_lower_bound_inf and nlp_upper_bound_inf.  To indicate that a
	   *  variable has no upper or lower bound, set the bound to
	   *  -ipopt_inf or +ipopt_inf respectively
	   */
	class TNLP_implementation : public TNLP
	{
		virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, 
				Index& nnz_h_lag, IndexStyleEnum& index_style);

		virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u, 
				Index m, Number* g_l, Number* g_u);

		virtual bool get_starting_point(Index n, bool init_x, Number* x, 
				bool init_z, Number* z_L, Number* z_U, 
				Index m, bool init_lambda, 
				Number* lambda);

		virtual bool eval_f(Index n, const Number* x, bool new_x, 
				Number& obj_value);
		
		virtual bool eval_grad_f(Index n, const Number* x, bool new_x, 
				Number* grad_f);
		
		virtual bool eval_g(Index n, const Number* x, bool new_x, 
				Index m, Number* g);
		
		virtual bool eval_jac_g(Index n, const Number* x, bool new_x, 
				Index m, Index nele_jac, Index* iRow, 
				Index *jCol, Number* values);
		
		virtual bool eval_h(Index n, const Number* x, bool new_x, 
				Number obj_factor, Index m, const Number* lambda, 
				bool new_lambda, Index nele_hess, 
				Index* iRow, Index* jCol, Number* values);
		
		virtual void finalize_solution(SolverReturn status, 
				Index n, const Number* x, const Number* z_L, const Number* z_U, 
				Index m, const Number* g, const Number* lambda, 
				Number obj_value,
				const IpoptData* ip_data,
				IpoptCalculatedQuantities* ip_cq);

		quadratic_program & associated_quadratic_program;

		TNLP_implementation() = delete;

		public:
		TNLP_implementation(quadratic_program & associated_quadratic_program);
		~TNLP_implementation();
	}; 

	//must create new TNLP_implementation using SmartPtr for Windows dll. 
	SmartPtr<TNLP> p_TNLP_implementation;

	//only need A and G in sparse format, vectors can be dense.
	Eigen::SparseMatrix<double> A;
	Eigen::VectorXd a;
	Eigen::SparseMatrix<double> A_t;//A transpose

	Eigen::SparseMatrix<double> G;
	Eigen::VectorXd g_l;
	Eigen::VectorXd g_u;

	Eigen::VectorXd x_l;
	Eigen::VectorXd x_u;

	Eigen::SparseMatrix<double> H; //For the Hessian, H = A+A^{T}. Since symmetric only upper right triangular part is stored.
	//the strict lower left triangular part will be zero. To use H correctly in matrix/vector multiplications use 
	//H.selfAdjointView<Eigen::Upper>() in place of H.

	Eigen::VectorXd x; //last optimal solution computed by an optimize call; 
	double obj_value; //last optimal value computed by an optimize call; 
	
	bool is_set_problem;

	public:
	quadratic_program();
	~quadratic_program();

	quadratic_program(const Eigen::SparseMatrix<double> & A, const Eigen::VectorXd & a, 
			const Eigen::SparseMatrix<double> & G, const Eigen::VectorXd & g_l, const Eigen::VectorXd & g_u, 
			const Eigen::VectorXd & x_l, const Eigen::VectorXd & x_u);

	//set up a new quadratic programming problem replacing current A, a, G, g_l, etc...
	bool set_problem(const Eigen::SparseMatrix<double> & A, const Eigen::VectorXd & a, 
			const Eigen::SparseMatrix<double> & G, const Eigen::VectorXd & g_l, const Eigen::VectorXd & g_u, 
			const Eigen::VectorXd & x_l, const Eigen::VectorXd & x_u); //return false iff error

	//returns IPOPT ApplicationReturnStatus typecasted to int. See IPOPT documentation.
	//for optimization to succeed it's necessary that scaled error becomes less than tolerance value within max_iter iterations. See IPOPT documentation.
	int optimize(); 

	Eigen::VectorXd get_solution(); //make sure optimize() is called first
	double get_value(); //make sure optimize() is called first
};

#endif
