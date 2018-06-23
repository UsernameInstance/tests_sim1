#include "quadratic_program.h"
#include "log.h"
#include <Eigen/SparseCholesky>

//since Hessian is assumed symmetric by TNLP nnz_h_lag refers to the number of nonzero entries in 
//the upper right triangular part of the hessian of the lagrangian.

bool quadratic_program::TNLP_implementation::get_nlp_info(
Index& n, //(out) number of variables in the problem (dimension of x).
Index& m, //(out) number of constraints in the problem (dimension of g(x)).
Index& nnz_jac_g, //(out) number of nonzero entries in the Jacobian of g. 
Index& nnz_h_lag, //(out) number of nonzero entries in the upper right triangular part of the Hessian of the lagrangian.
IndexStyleEnum& index_style) //(out), numbering style for row/col entries in sparse matrix format
{
	n = associated_quadratic_program.x.size();
	m = associated_quadratic_program.G.rows();

	//since g(x) = G*x, the Jacobian of g(x) is G.
	nnz_jac_g = associated_quadratic_program.G.nonZeros();
	
	nnz_h_lag = associated_quadratic_program.H.nonZeros();

	index_style = C_STYLE; //0-based, as opposed to 1-based FORTRAN_STYLE

	return true;
}

bool quadratic_program::TNLP_implementation::get_bounds_info(
Index n, //(in), number of variables in the problem (the output n from get_nlp_info).
Number* x_l, //(out) lower bounds for x.
Number* x_u, //(out) upper bounds for x.
Index m, //(in) number of constraints in the problem (the output m from get_nlp_info).
Number* g_l, //(out) the lower bounds for g(x).
Number* g_u) //(out) the upper bounds for g(x).
{
	if(n != associated_quadratic_program.x.size()) //this should be impossible.
		LOG("Error in quadratic_program::TNLP_implementation::get_bounds_info: "
				"n != associated_quadratic_program.x.size()");
	if(m != associated_quadratic_program.G.rows()) //this should be impossible.
		LOG("Error in quadratic_program::TNLP_implementation::get_bounds_info: "
				"m != associated_quadratic_program.G.rows()");

	for(Index i = 0; i<n; ++i)
	{
		x_l[i] = associated_quadratic_program.x_l(i);
		x_u[i] = associated_quadratic_program.x_u(i);
	}

	for(Index i = 0; i<m; ++i)
	{
		g_l[i] = associated_quadratic_program.g_l(i);
		g_u[i] = associated_quadratic_program.g_u(i);
	}

	return true;
}

bool quadratic_program::TNLP_implementation::get_starting_point(
Index n, //(in) dim x
bool init_x, //(in) if true this method msut provide an initial value for x.
Number* x, //(out) initial values for the primal variables.
bool init_z, //(in) if true this method must provide an initial value for the bound multipliers. 
Number* z_L, //(out) initial values for the bound multipliers.
Number* z_U, //(out) initial values for the bound multipliers.
Index m,  //(in) dim g(x)
bool init_lambda, //(in) if true this method must provide an initial value for the constraint multipliers.
Number* lambda) //(out) the initial values for the constraint multipliers.
{
	if(!init_x || init_z || init_lambda)
		LOG("Error in quadratic_program::TNLP_implementation::get_starting_point: " 
				"!init_x || init_z || init_lambda");

	//(Eigen::RowVectorXd::Ones(A.rows()) * A.cwiseAbs()).maxCoeff(); //colwise 1-norm
	//(A.cwiseAbs() * Eigen::VectorXd::Ones(A.cols())).maxCoeff(); //rowwise 1-norm
	
	/* 1-st order iterative method for pseudoinverse of real matrix M:
	 * Y(0) = c*(M^{T}),
	 * Y(k+1) = (I-c*(M^{T})*M)*Y(k) + c*(M^{T}) <- k = 0, 1, 2, ...,
	 * where c is a real scalar.
	 * Sequence converges to Moore-Penrose psuedoinverse of M whenever
	 * 0 < c < 2/L ,
	 * where L is a largest eigenvalue of (M^{T})*M. note L <= rowwise 1-norm of M.
	 */
	//too slow
/*	
	//Assume A is symmetric positive definite then global minimum of f(x) = x^T A x + a^T x is at x = -(1/2)*A^{-1}a; Replace Y(K) by Y(K) * (-0.5) * a in the preceeding.
	Eigen::SparseMatrix<double> B = associated_quadratic_program.A_t * associated_quadratic_program.A;	
	Eigen::VectorXd y0; 
	{
		double c = (B.cwiseAbs() * Eigen::VectorXd::Ones(B.cols())).maxCoeff(); //rowwise 1-norm >= L
		if(c > 0) c = 1.7/c;
		else c = 0;
		
		y0 = (-0.5)*c*associated_quadratic_program.A_t*associated_quadratic_program.a;

		Eigen::SparseMatrix<double> temp(B.rows(), B.cols());
		temp.setIdentity();
		B = temp - c*B; //this is symmetric, can replace with its upper right triangular part (converted to sparse format) and use selfadjointView<Eigen::Upper>() instead.
	}
	//y(0) == y0
	//y(k+1) == B*y(k) + y(0)

	int num_iter = 1;//11*B.nonZeros(); //can replace with error tolerance if less lazy.
	Eigen::VectorXd y_prev = y0;
	Eigen::VectorXd y = y0;
	Eigen::VectorXd y_trans = y0;

	for(int k=0; k<num_iter; ++k)
	{
		y_trans = y;
		y = B*y_prev + y0;
		y_prev = y_trans;
	}

	if(n != y.size())
	{
		LOG("Error in quadratic_program::TNLP_implementation::get_starting_point: n != y.size()");
		return false;
	}

	for(Index k=0; k<n; ++k)
		x[k] = y(k);
*/
	//Az = (-.5)*a
	
	Eigen::VectorXd z;
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
	z = solver.compute(associated_quadratic_program.A).solve((-.5)*associated_quadratic_program.a);
	if(solver.info() != Eigen::Success)
	{
		LOG("Warning solver.compute(A).solve(-.5*a) failed in quadratic_program::TNLP_implementation::get_starting_point");

		for(Index k=0; k<n; ++k)
			x[k] = 0;
	}
	else
	{
		for(Index k=0; k<n; ++k)
			x[k] = z(k);
	}

	return true;
}

bool quadratic_program::TNLP_implementation::eval_f(
Index n, //(in) number of variables (dimension of x)
const Number* x, //(in) values of components of x, at which to evaluate f(x).
bool new_x, //(in) false iff any evaluation method was previously called with the same values in x
Number& obj_value) //(out) computer value of objective function f(x)
{
	Eigen::VectorXd vec_x(associated_quadratic_program.x.size());
	
	if(n != vec_x.size())
	{
		LOG("Error in quadratic_program::TNLP_implementation::eval_f: n != vec_x.size()");
		return false;
	}

	for(Index k = 0; k < n; ++k)
		vec_x(k) = x[k];

	obj_value = vec_x.dot(associated_quadratic_program.A*vec_x) + 
		vec_x.dot(associated_quadratic_program.a);

	return true;
}

bool quadratic_program::TNLP_implementation::eval_grad_f(
Index n, //(in) number of variables (dimension of x)
const Number* x, //(in) values of components of x, at which to evaluate gradient of f at x.
bool new_x, //(in) false iff any evaluation method was previously called with the same values in x
Number* grad_f) //(out) computed array of values for the gradient of objective function (f(x))
{
	//since f(x) = x^{T}*A*x + a*x,
	//grad_f(x) = (A+A^{T})*x + a = H*x + a;
	
	Eigen::VectorXd vec_x(associated_quadratic_program.x.size());
	if(n != vec_x.size())
	{
		LOG("Error in quadratic_program::TNLP_implementation::eval_grad_f: n != vec_x.size()");
		return false;
	}

	for(Index k = 0; k < n; ++k)
		vec_x(k) = x[k];

	Eigen::VectorXd vec_grad_f = 
		associated_quadratic_program.H.selfadjointView<Eigen::Upper>()*vec_x + 
		associated_quadratic_program.a;
	for(Index k=0; k<n; ++k)
		grad_f[k] = vec_grad_f(k);

	return true;
}

bool quadratic_program::TNLP_implementation::eval_g(
Index n, //(in) number of variables (dimension of x)
const Number* x, //(in) values of components of x, at which to evaluate g(x).
bool new_x, //(in) false iff any evaluation method was previously called with the same values in x
Index m, //(in) number of constraints in the problem (dimension of g(x)).
Number* g) //(out) computed array of values for the constraint function (g(x))
{
	//g(x) == G*x
	Eigen::VectorXd vec_x(associated_quadratic_program.x.size());
	if(n != vec_x.size())
	{
		LOG("Error in quadratic_program::TNLP_implementation::eval_g: n != vec_x.size()");
		return false;
	}

	for(Index k = 0; k < n; ++k)
		vec_x(k) = x[k];

	Eigen::VectorXd vec_g = associated_quadratic_program.G * vec_x;
	if(m != vec_g.size())
	{
		LOG("Error in quadratic_program::TNLP_implementation::eval_g: m != vec_g.size()");
		return false;
	}

	for(Index k=0; k<m; ++k)
		g[k] = vec_g(k);

	return true;
}

//return either the sparsity structure of the Jacobian of g, or the values for the Jacobian at x
bool quadratic_program::TNLP_implementation::eval_jac_g(
Index n, //(in) number of variables (dimension of x)
const Number* x, //(in) values of components of x, at which to evaluate g(x).
bool new_x, //(in) false iff any evaluation method was previously called with the same values in x
Index m, //(in) number of constraints in the problem (dimension of g(x)).
Index nele_jac, //(in) number of nonzero elements in the Jacobian (== dims of iRow, jCol, & values)
Index* iRow, //(out) row indices of entries in the Jacobian
Index *jCol, //(out) col indices of entries in the Jacobian
Number* values) //(out) values of the entries in the Jacobian
{
	//triplet format (iRow[k], jCol[k], values[k]) gives entry
	//in row iRow[k] and column jCol[k] Jacobian as values[k]
	//fill in only nonzero values
	
	Eigen::SparseMatrix<double> & G = associated_quadratic_program.G;
	if(values == nullptr) //return sparsity structure of Jacobian (iRow and jCol only)
	{
		Index ind = 0;
		//iterate over nonzeros
		for(int k=0; k<G.outerSize(); ++k)
			for(Eigen::SparseMatrix<double>::InnerIterator it(G,k); it; ++it)
			{
				iRow[ind] = it.row();
				jCol[ind] = it.col();
				++ind;
			}
	}
	else
	{
		//return values x doesn't matter here since jacobian is constant matrix G.
		Index ind = 0;
		//iterate over nonzeros
		for(int k=0; k<G.outerSize(); ++k)
			for(Eigen::SparseMatrix<double>::InnerIterator it(G,k); it; ++it)
			{
				values[ind] = it.value();
				++ind;
			}
	}

	return true;
}

bool quadratic_program::TNLP_implementation::eval_h(
Index n, //(in) number of variables (dimension of x)
const Number* x, //(in) values of components of x, at which to evaluate g(x).
bool new_x, //(in) false iff any evaluation method was previously called with the same values in x
Number obj_factor, //(in) factor in front of the objective term in the Hessian sigma_f
Index m, //(in) number of constraints in the problem (dimension of g(x)).
const Number* lambda, //(in) values for the constraint multipliers at which Hessian to be evaluated
bool new_lambda, //(in) false iff any evaluation method was previously called with same values in lambda 
Index nele_hess, //(in) number of nonzero elements in the Hessian (== dims of iRow, jCol, & values)
Index* iRow, //(out) row indices of entries in the Hessian
Index *jCol, //(out) col indices of entries in the Hessian
Number* values) //(out) values of the entries in the Hessian
{
	Eigen::SparseMatrix<double> & H = associated_quadratic_program.H;
	if(values == nullptr) //return sparsity structure of Hessian (iRow and jCol only)
	{
		Index ind = 0;
		//iterate over nonzeros
		for(int k=0; k<H.outerSize(); ++k)
			for(Eigen::SparseMatrix<double>::InnerIterator it(H,k); it; ++it)
			{
				iRow[ind] = it.row();
				jCol[ind] = it.col();
				++ind;
			}
	}
	else
	{
		//return values x doesn't matter here since hessian is sigma_f*H
		//since gradient of g is constant matrix G, Hessian value doesn't depend on lambda
		Index ind = 0;
		//iterate over nonzeros
		for(int k=0; k<H.outerSize(); ++k)
			for(Eigen::SparseMatrix<double>::InnerIterator it(H,k); it; ++it)
			{
				values[ind] = obj_factor*it.value();
				++ind;
			}
	}

	return true;
}

void quadratic_program::TNLP_implementation::finalize_solution(
SolverReturn status, //(in) status of algorithm as specified in IpAlgTypes.hpp
Index n, //(in) dimension of x
const Number* x, //(in) final values for the primal variables x_opt
const Number* z_L, //(in) final values for the lower bound multipliers
const Number* z_U, //(in) final values for the upper bound multipliers
Index m, //(in) dimension of g(x)
const Number* g, //(in) final values for the constraint function values g(x_opt)
const Number* lambda, //(in) final values of the constraint multipliers
Number obj_value, //(in) final value of the objective function f(x_opt)
const IpoptData* ip_data, //for expert users
IpoptCalculatedQuantities* ip_cq) //for expert users
{
	for(Index k=0; k<n; ++k)
		associated_quadratic_program.x(k) = x[k];

	associated_quadratic_program.obj_value = obj_value;

	return;
}

quadratic_program::TNLP_implementation::TNLP_implementation(quadratic_program & associated_quadratic_program) 
	: associated_quadratic_program(associated_quadratic_program) {}

quadratic_program::TNLP_implementation::~TNLP_implementation() {}

quadratic_program::quadratic_program() 
	: p_TNLP_implementation(new TNLP_implementation(*this)) {}

quadratic_program::~quadratic_program() {}

quadratic_program::quadratic_program(const Eigen::SparseMatrix<double> & A, const Eigen::VectorXd & a, 
		const Eigen::SparseMatrix<double> & G, const Eigen::VectorXd & g_l, const Eigen::VectorXd & g_u, 
		const Eigen::VectorXd & x_l, const Eigen::VectorXd & x_u) 
	: p_TNLP_implementation(new TNLP_implementation(*this)) 
{
	is_set_problem = set_problem(A, a, G, g_l, g_u, x_l, x_u);
}

//return false iff error
bool quadratic_program::set_problem(const Eigen::SparseMatrix<double> & A, const Eigen::VectorXd & a, const Eigen::SparseMatrix<double> & G, const Eigen::VectorXd & g_l, const Eigen::VectorXd & g_u, const Eigen::VectorXd & x_l, const Eigen::VectorXd & x_u)
{
	//make sure dimensions compatible
	size_t num_rows_A, num_cols_A, size_a, num_rows_G, num_cols_G, size_g_l, size_g_u, size_x_l, size_x_u;
	num_rows_A = A.rows();
	num_cols_A = A.cols();
	size_a = a.size();
	if(num_rows_A != num_cols_A)
	{
		LOG("Warning in quadratic_program::set_problem failed: A.rows() != A.cols().");
		return false;
	}

	if(num_cols_A != size_a)
	{
		LOG("Warning in quadratic_program::set_problem failed: A.cols() != a.size().");
		return false;
	}


	num_cols_G = G.cols();
	if(num_cols_G != size_a)
	{
		LOG("Warning in quadratic_program::set_problem failed: G.cols() != a.size().");
		return false;
	}
	num_rows_G = G.rows();
	size_g_l = g_l.size();
	if(num_rows_G != size_g_l)
	{
		LOG("Warning in quadratic_program::set_problem failed: G.rows() != g_l.size().");
		return false;
	}

	size_g_u = g_u.size();
	if(size_g_u != size_g_l)
	{
		LOG("Warning in quadratic_program::set_problem failed: g_u.size() != g_l.size().");
		return false;
	}

	size_x_l = x_l.size();
	if(size_x_l != size_a)
	{
		LOG("Warning in quadratic_program::set_problem failed: x_l.size() != a.size().");
		return false;
	}

	size_x_u = x_u.size();
	if(size_x_u != size_x_l)
	{
		LOG("Warning in quadratic_program::set_problem failed: x_u.size() != x_l.size().");
		return false;
	}
	
	this->A = A;
	this->a = a;
	this->G = G;
	this->g_l = g_l;
	this->g_u = g_u;
	this->x_l = x_l;
	this->x_u = x_u;
	this->A_t = A.transpose(); 
	this->H = (Eigen::SparseMatrix<double>(A_t) + A).triangularView<Eigen::Upper>();
	//since the constraints are linear the Hessian is the Hessian of f(x) and,
	//since f(x) is the quadratic x^{T}*A*x+a^{T}*x the hessian is A + A^{T}.

	this->x.resize(x_l.size());

	return true;
}

int quadratic_program::optimize()
{
	if(!is_set_problem)
	{
		LOG("Error in quadratic_program::optimize: !is_set_problem");
		return 2; //Infeasible_Problem_Detected == 2
	}

	SmartPtr<IpoptApplication> p_application(IpoptApplicationFactory());
	p_application->Options()->SetStringValue("jac_c_constant", "yes"); //Are all equality constraints linear?
	p_application->Options()->SetStringValue("jac_d_constant", "yes"); //Are all inequality constraints linear?
	p_application->Options()->SetStringValue("hessian_constant", "yes"); //Is the problem quadratic?
	double mult = 1e-0;
	p_application->Options()->SetNumericValue("tol", mult*1e-8);
	p_application->Options()->SetNumericValue("dual_inf_tol", mult*1);
	p_application->Options()->SetNumericValue("acceptable_tol", mult*1e-6);
	p_application->Options()->SetIntegerValue("acceptable_iter", 15);
	p_application->Options()->SetNumericValue("acceptable_dual_inf_tol", mult*1e10);
	p_application->Options()->SetNumericValue("compl_inf_tol", mult*1e-4);
	p_application->Options()->SetNumericValue("acceptable_compl_inf_tol", mult*1e-2);

//	p_application->Options()->SetStringValue("linear_system_scaling", "mc19");
//	p_application->Options()->SetNumericValue("obj_scaling_factor", 1);
//	p_application->Options()->SetStringValue("nlp_scaling_method", "equilibration-based"); //Use mehrotra's predictor-corrector algorithm? Should work well for linear and convex quadratic programs.

//	p_application->Options()->SetStringValue("mehrotra_algorithm", "yes"); //Use mehrotra's predictor-corrector algorithm? Should work well for linear and convex quadratic programs.
//	p_application->Options()->SetStringValue("linear_solver", "mumps");

//	p_application->Options()->SetIntegerValue("print_level", 0); //make sure use SetIntegerValue not SetNumericValue
//	p_application->Options()->SetStringValue("print_user_options", "yes");

//	List of termination options set to their default values.
//	See documentation for details.
/*
	p_application->Options()->SetNumericValue("tol", 1e-8);
	p_application->Options()->SetIntegerValue("max_iter", 3000);
	p_application->Options()->SetNumericValue("max_cpu_time", 1e6);
	p_application->Options()->SetNumericValue("dual_inf_tol", 1);
	p_application->Options()->SetNumericValue("constr_viol_tol", 1e-4);
	p_application->Options()->SetNumericValue("compl_inf_tol", 1e-4);

	p_application->Options()->SetNumericValue("acceptable_tol", 1e-6);
	p_application->Options()->SetIntegerValue("acceptable_iter", 15);
	p_application->Options()->SetNumericValue("acceptable_constr_viol_tol", 1e-2);
	p_application->Options()->SetNumericValue("acceptable_dual_inf_tol", 1e10);
	p_application->Options()->SetNumericValue("acceptable_compl_inf_tol", 1e-2);
	p_application->Options()->SetNumericValue("acceptable_obj_change_tol", 1e20);
	p_application->Options()->SetNumericValue("diverging_iterates_tol", 1e20);
*/
	ApplicationReturnStatus status = p_application->Initialize();
	if(status != Solve_Succeeded)
	{
		LOG("Warning in quadratic_program::optimize: p_application->Initialize() != Solve_Succeeded.");
		return (int) status;
	}
	
	status = p_application->OptimizeTNLP(p_TNLP_implementation);
	if(status != Solve_Succeeded)
		LOG("Warning in quadratic_program::optimize: p_application->OptimizeTNLP(p_TNLP_implementation) != Solve_Succeeded.");

	return status;
}

Eigen::VectorXd quadratic_program::get_solution()
{
	return x;
}

double quadratic_program::get_value()
{
	return obj_value;
}
