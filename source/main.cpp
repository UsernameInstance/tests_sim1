

#define TEST_EIGEN_SPARSE_MATRIX 0

#if TEST_EIGEN_SPARSE_MATRIX
#include <Eigen/SparseCore>
#include <iostream>
#include "quadratic_program.h"
#include <limits>

//returns number of cols for colMajor with a nonzero entry 
//returns number of rows for rowMajor with a nonzero entry
template <typename T>
int nnz_outer(Eigen::SparseMatrix<T> input)
{
	int nnz = 0;
	if(input.isCompressed())
	{
		for(int k=0; k<input.outerSize(); ++k)
			nnz += (input.outerIndexPtr()[k+1] > input.outerIndexPtr()[k]);
	}
	else
	{
		for(int k=0; k<input.outerSize(); ++k)
			nnz += (input.innerNonZeroPtr()[k]>0);
	}

	return nnz;
}

void test()
{
	bool test_1 = false;
	bool test_2 = false;
	bool test_3 = false;
	bool test_4 = true;

	if(test_1)
	{
		Eigen::SparseMatrix<double> M1(3,3);
		M1.coeffRef(0,0) = 1;
		M1.coeffRef(1,0) = 9;
		M1.coeffRef(0,1) = 6;
		M1.coeffRef(2,0) = 1;
		M1.coeffRef(0,2) = 1;
		M1.makeCompressed();

		Eigen::SparseMatrix<double> M2(3,3);
		M2.coeffRef(0,0) = 1;
		M2.coeffRef(2,2) = 9;

		Eigen::SparseMatrix<double> M3(3,3);
		M3.coeffRef(0,0) = -1;
		M3.coeffRef(2,0) = -1;

		Eigen::SparseMatrix<double> M4(3,3);
		M4.coeffRef(0,0) = -7;
		M4.coeffRef(0,2) = 5;
		M4.makeCompressed();

		std::cout << "M1 == \n" << M1 << std::endl;	
		std::cout << "M1.rows() == " << M1.rows() << "\n";
		std::cout << "M1.cols() == " << M1.cols() << "\n";
		std::cout << "M1.innerSize() == " << M1.innerSize() << "\n";
		std::cout << "M1.outerSize() == " << M1.outerSize() << "\n";
		std::cout << "M1.nonZeros() == " << M1.nonZeros() << "\n";
		std::cout << "nnz_outer(M1) == " << nnz_outer(M1) << "\n";
		std::cout << std::endl;

		std::cout << "M2 == \n" << M2 << std::endl;	
		std::cout << "M2.rows() == " << M2.rows() << "\n";
		std::cout << "M2.cols() == " << M2.cols() << "\n";
		std::cout << "M2.innerSize() == " << M2.innerSize() << "\n";
		std::cout << "M2.outerSize() == " << M2.outerSize() << "\n";
		std::cout << "M2.nonZeros() == " << M2.nonZeros() << "\n";
		std::cout << "nnz_outer(M2) == " << nnz_outer(M2) << "\n";
		std::cout << std::endl;

		std::cout << "M3 == \n" << M3 << std::endl;	
		std::cout << "M3.rows() == " << M3.rows() << "\n";
		std::cout << "M3.cols() == " << M3.cols() << "\n";
		std::cout << "M3.innerSize() == " << M3.innerSize() << "\n";
		std::cout << "M3.outerSize() == " << M3.outerSize() << "\n";
		std::cout << "M3.nonZeros() == " << M3.nonZeros() << "\n";
		std::cout << "nnz_outer(M3) == " << nnz_outer(M3) << "\n";
		std::cout << std::endl;

		std::cout << "M4 == \n" << M4 << std::endl;	
		std::cout << "M4.rows() == " << M4.rows() << "\n";
		std::cout << "M4.cols() == " << M4.cols() << "\n";
		std::cout << "M4.innerSize() == " << M4.innerSize() << "\n";
		std::cout << "M4.outerSize() == " << M4.outerSize() << "\n";
		std::cout << "M4.nonZeros() == " << M4.nonZeros() << "\n";
		std::cout << "nnz_outer(M4) == " << nnz_outer(M4) << "\n";
		std::cout << std::endl;
	}
	
	if(test_2)
	{
		int n = 5;
		Eigen::SparseMatrix<double> mat(n, n);
		Eigen::VectorXd vec(n);
		for(int i=0; i<n; ++i)
		{
			vec(i) = i;
			for(int j=0; j<n; ++j)
				mat.coeffRef(i, j) = (2*i+5*j*j+9*i*j)%10;
		}

		std::cout << "mat == \n" << mat << "\n\n";
		std::cout << "vec == \n" << vec << "\n\n";
		std::cout << "mat.triangularView<Eigen::Upper>() == \n" << mat.triangularView<Eigen::Upper>() << "\n\n";
		std::cout << "mat.selfadjointView<Eigen::Upper>() == \n" << mat.selfadjointView<Eigen::Upper>() << "\n\n";
		
		Eigen::SparseMatrix<double> up_mat = mat.triangularView<Eigen::Upper>();

		std::cout << "up_mat == \n" << up_mat << "\n\n";
		std::cout << "up_mat.selfadjointView<Eigen::Upper>() == \n" << up_mat.selfadjointView<Eigen::Upper>() << "\n\n";

		std::cout << "mat.selfadjointView<Eigen::Upper>()*vec == \n" << mat.selfadjointView<Eigen::Upper>()*vec << "\n\n";
		std::cout << "up_mat.selfadjointView<Eigen::Upper>()*vec == \n" << up_mat.selfadjointView<Eigen::Upper>()*vec << "\n\n";
	}

	if(test_3)
	{
		Eigen::SparseMatrix<double> A(2,2);
		A.coeffRef(0,0) = 3;
		A.coeffRef(1,0) = 1;
		A.coeffRef(0,1) = 1;
		A.coeffRef(1,1) = 1;

		Eigen::VectorXd a(2);
		a(0) = 1;
		a(1) = 6;

		Eigen::SparseMatrix<double> G(1,2);
		G.coeffRef(0,0) = 2; G.coeffRef(0,1) = 3;

		Eigen::VectorXd g_l(1);
		g_l(0) = 4;
		Eigen::VectorXd g_u(1);
		g_u(0) = std::numeric_limits<double>::infinity();

		Eigen::VectorXd x_l(2);
		x_l(0) = 0; x_l(1) = 0;
		Eigen::VectorXd x_u(2);
		x_u(0) = std::numeric_limits<double>::infinity();
		x_u(1) = std::numeric_limits<double>::infinity();

		//solution should be approximately x =(.5, 1), obj_value = 9.25;

		quadratic_program test_qp(A,a,G,g_l,g_u,x_l,x_u);

		test_qp.optimize();

		std::cout << "test_qp.get_solution() == \n" << test_qp.get_solution() << "\n";
		std::cout << "test_qp.get_value() == \n" << test_qp.get_value() << "\n";

	}

	if(test_4)
	{
		Eigen::SparseMatrix<double> A(3,3);
		A.coeffRef(0,0) = 2.5;
		A.coeffRef(1,0) = -1;
		A.coeffRef(2,0) = -.5;
		A.coeffRef(0,1) = -1;
		A.coeffRef(1,1) = 2;
		A.coeffRef(2,1) = 1.5;
		A.coeffRef(0,2) = -.5;
		A.coeffRef(1,2) = 1.5;
		A.coeffRef(2,2) = 2.5;

		Eigen::VectorXd a(3);
		a(0) = 2;
		a(1) = -35;
		a(2) = -47;

		Eigen::SparseMatrix<double> G(0,3);

		Eigen::VectorXd g_l(0);
		Eigen::VectorXd g_u(0);

		Eigen::VectorXd x_l(3);
		x_l(0) = -std::numeric_limits<double>::infinity();
		x_l(1) = -std::numeric_limits<double>::infinity();
		x_l(2) = -std::numeric_limits<double>::infinity();

		Eigen::VectorXd x_u(3);
		x_u(0) = std::numeric_limits<double>::infinity();
		x_u(1) = std::numeric_limits<double>::infinity();
		x_u(2) = std::numeric_limits<double>::infinity();

		//solution should be approximately x =(3, 5, 7);

		quadratic_program test_qp(A,a,G,g_l,g_u,x_l,x_u);

		test_qp.optimize();

		std::cout << "test_qp.get_solution() == \n" << test_qp.get_solution() << "\n";
		std::cout << "test_qp.get_value() == \n" << test_qp.get_value() << "\n";

	}

}

#else
#include "app.h"

#endif

int main(int argc, char* argv[])
{
#if TEST_EIGEN_SPARSE_MATRIX
	test();
	return 0;
#else
	return app::instance.execute(argc, argv);
#endif
}
