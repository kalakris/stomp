#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/QR>
#include <Eigen/OrderingMethods>
#include <vector>
#include <cstdio>
#include <iostream>
#include <ros/time.h>
#include <boost/progress.hpp>

static const double diff_2[2] = {-1.0, 1.0};
static const double diff_3[3] = {-1.0, 2.0, -1.0};

class EigenPerfTest
{
public:

  void createDiffMatrix(int bandwidth=2)
  {
    // allocate memory
    dense_A = Eigen::MatrixXd::Zero(size+bandwidth-1, size);
    sparse_A = Eigen::SparseMatrix<double>(size+bandwidth-1, size);

    const double* diff_rule;
    if (bandwidth==2)
      diff_rule = diff_2;
    else
      diff_rule = diff_3;

    // create triplets
    std::vector<Eigen::Triplet<double> > triplets;
    triplets.reserve(bandwidth*(size+bandwidth-1));
    for (int i=0; i<size+bandwidth-1; ++i)
    {
      for (int j=0; j<bandwidth; ++j)
      {
        int k = i + j - (bandwidth-1);
        if (k>=0 && k<size)
        {
          triplets.push_back(Eigen::Triplet<double>(i, k, diff_rule[j]));
        }
      }

    }

    // fill the dense matrix
    for (size_t i=0; i<triplets.size(); ++i)
    {
      Eigen::Triplet<double>& t = triplets[i];
      //printf("[%3d, %3d] = %f\n", t.row(), t.col(), t.value());
      dense_A(t.row(), t.col()) = t.value();
    }

    // fill the sparse matrix
    sparse_A.setFromTriplets(triplets.begin(), triplets.end());
  }

  /////// all comments about performance are for bandwidth = 2 / 3
  /////// all tests on ULV core i7 laptop @ 2Ghz, for matrix size = 700

  void createCostMatrix()
  {
    int reps = 1; // 10 = 1.71s / 1.8s  (0.17 / 0.18 seconds for dense matrix mult)

    //ros::WallTime start = ros::WallTime::now();
    {
      boost::progress_timer t;
      for (int i=0; i<reps; ++i)
        dense_R = dense_A.transpose() * dense_A;

      printf("%d reps of dense matrix multiplication: ", reps);
    }

    reps = 1; // 10000 = 0.5s / 0.83s     (50us / 83us for sparse matrix mult)
    {
      boost::progress_timer t;
      for (int i=0; i<reps; ++i)
        sparse_R = sparse_A.transpose() * sparse_A;

      printf("%d reps of sparse matrix multiplication: ", reps);
    }

    Eigen::MatrixXd temp = sparse_R;
    Eigen::MatrixXd diff = dense_R - temp;
    printf("Error norm = %f\n", diff.norm());
  }

  void computeCholeskyFactor()
  {
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double> >& sparse_solver = sparse_cholesky_solver;
    Eigen::LLT<Eigen::MatrixXd>& dense_solver = dense_cholesky_solver;

    int reps = 1; // 10000 = 1.44s / 1.35s   (0.144ms / 0.135ms for sparse matrix analysis)

    {
      boost::progress_timer t;
      for (int i=0; i<reps; ++i)
        sparse_solver.analyzePattern(sparse_R);
      printf("%d reps of sparse LLT pattern analysis: ", reps);
    }

    reps = 1;  // 100000 = 3.39s / 4.37s  (33us / 43us for sparse cholesky factorization)
    {
      boost::progress_timer t;
      for (int i=0; i<reps; ++i)
        sparse_solver.factorize(sparse_R);
      printf("%d reps of sparse LLT factorization: ", reps);
    }
    sparse_chol_R = sparse_solver.permutationPinv() * sparse_solver.matrixL();

    reps = 1;  // 100 = 1.21s / 1.23s (12ms / 13ms for dense cholesky factorization)
    {
      boost::progress_timer t;
      for (int i=0; i<reps; ++i)
        dense_solver.compute(dense_R);
      printf("%d reps of dense LLT factorization: ", reps);
    }
    dense_chol_R = dense_solver.matrixL();



//    Eigen::SparseMatrix<double> sparse_prod = (sparse_solver.permutationPinv() * (sparse_chol_R * sparse_chol_R.transpose()))
//        * sparse_solver.permutationP();
    Eigen::SparseMatrix<double> sparse_prod = sparse_chol_R * sparse_chol_R.transpose();
    Eigen::SparseMatrix<double> sparse_error = sparse_prod - sparse_R;
    Eigen::MatrixXd sparse_error_dense = sparse_error;
    printf("Sparse cholesky error = %f\n", sparse_error_dense.norm());

    double dense_error = ((dense_chol_R * dense_chol_R.transpose()) - dense_R).norm();
    printf("Dense cholesky error = %f\n", dense_error);

//    Eigen::MatrixXd temp = sparse_prod;
//    Eigen::MatrixXd diff = (dense_chol_R * dense_chol_R.transpose()) - temp;
//    printf("Error norm = %f\n", diff.norm());
//
//    std::cout << diff;
  }

  void createConstraintMatrix()
  {
    int num_constraints = num_time_steps-1;

    // allocate memory
    dense_C = Eigen::MatrixXd::Zero(size, num_constraints);
    sparse_C = Eigen::SparseMatrix<double, Eigen::ColMajor>(size, num_constraints);

    // create triplets
    std::vector<Eigen::Triplet<double> > triplets;
    triplets.reserve(num_joints*2*num_constraints);

    for (int t=0; t<num_constraints; ++t)
    {
      for (int j=0; j<num_joints; ++j)
      {
        triplets.push_back(Eigen::Triplet<double>(t*num_joints + j, t, -1.0 * Eigen::internal::random<double>(0.0,1.0)));
        triplets.push_back(Eigen::Triplet<double>((t+1)*num_joints + j, t, +1.0 * Eigen::internal::random<double>(0.0,1.0)));
      }
    }

    // fill the dense matrix
    for (size_t i=0; i<triplets.size(); ++i)
    {
      Eigen::Triplet<double>& t = triplets[i];
      //printf("[%3d, %3d] = %f\n", t.row(), t.col(), t.value());
      dense_C(t.row(), t.col()) = t.value();
    }

    // fill the sparse matrix
    sparse_C.setFromTriplets(triplets.begin(), triplets.end());

  }

  void computeConstraintQR()
  {
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> > sparse_solver;
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dense_solver;

    //sparse_solver.compute(sparse_C.transpose());
    int reps = 1; // 500 = 0.01s
    {
      printf("%d reps of sparse QR reordering: ", reps);
      boost::progress_timer t;
      for (int i=0; i<reps; ++i)
        sparse_solver.analyzePattern(sparse_C);
    }

    reps = 1000; // 500 = 1.49 s (3ms for sparse QR decomposition)
    {
      printf("%d reps of sparse QR factorization: ", reps);
      boost::progress_timer t;
      for (int i=0; i<reps; ++i)
        sparse_solver.factorize(sparse_C);
    }
    //sparse_C_Q = sparse_solver.matrixQ();
    sparse_C_R = sparse_solver.matrixR();

    reps = 1000; // 500 = 1.65s (3.3ms for dense QR decomposition)
    {
      printf("%d reps of dense QR decomposition: ", reps);
      boost::progress_timer t;
      for (int i=0; i<reps; ++i)
        dense_solver.compute(dense_C);
    }
    dense_C_Q = dense_solver.matrixQ();
    dense_C_R = dense_solver.matrixR();
    printf("dense QR rank = %ld\n", dense_solver.rank());
    printf("sparse QR rank = %d\n", sparse_solver.rank());

  }

  void computeConstraintProjector()
  {
    // first the dense version
    int reps = 100; // 100 = 1.83s (18ms for dense constraint projector computation)
    {
      printf("%d reps of dense projector computation: ", reps);
      boost::progress_timer t;
      for (int i=0; i<reps; ++i)
      {
        Eigen::MatrixXd dense_V = dense_cholesky_solver.solve(dense_C);  // V => n x m
        //std::cout << V << std::endl;

        // A => m x n
        // AV => m x m

        Eigen::MatrixXd dense_AV = dense_C.transpose() * dense_V;

        // AV is symmetric
        dense_projector.transpose() = dense_AV.llt().solve(dense_V.transpose());
      }
    }


    // now the sparse version
    reps = 100;
    {
      printf("%d reps of sparse projector computation: ", reps);
      boost::progress_timer t;
      for (int i=0; i<reps; ++i)
      {
        Eigen::SparseMatrix<double> sparse_V = sparse_cholesky_solver.solve(sparse_C); // 2.71ms
        //Eigen::MatrixXd dense_V = sparse_V; // slow
        Eigen::MatrixXd dense_V = sparse_cholesky_solver.solve(dense_C); // 1.5 ms

        Eigen::MatrixXd dense_AV = dense_C.transpose() * dense_V;
        //dense_projector.transpose() = dense_AV.ldlt().solve(dense_V.transpose());
        dense_projector.transpose() = dense_AV.ldlt().solve(dense_V.transpose());



//        Eigen::SparseMatrix<double> sparse_AV = sparse_C.transpose() * sparse_V;
//
//
//        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > sparse_chol_solver;
//        sparse_chol_solver.compute(sparse_AV);
//        Eigen::SparseMatrix<double> sparse_V_trans = sparse_V.transpose();
//        Eigen::SparseMatrix<double> sparse_W_trans = sparse_chol_solver.solve(sparse_V_trans);
//        sparse_projector = sparse_W_trans.transpose();
      }
    }

    //Eigen::MatrixXd dense_sparse_proj = sparse_projector;
    //printf("projector error = %f\n", (dense_sparse_proj - dense_projector).norm());

  }

private:
  // A = num diff matrix
  // R = A^T A  (quadratic cost matrix)
  // R = L L^T  (L -> chol_R)
  // C = constraint matrix
  Eigen::MatrixXd dense_A, dense_R, dense_chol_R, dense_C, dense_C_Q, dense_C_R, dense_projector;
  Eigen::SparseMatrix<double> sparse_A, sparse_R, sparse_chol_R, sparse_C, sparse_C_Q, sparse_C_R, sparse_projector;

  Eigen::SimplicialLLT<Eigen::SparseMatrix<double> > sparse_cholesky_solver;
  Eigen::LLT<Eigen::MatrixXd> dense_cholesky_solver;

  static const int num_time_steps = 100;
  static const int num_joints = 7;
  static const int size = num_time_steps * num_joints;
};

int main(int argc, char** argv)
{
  // create the differentiation matrix

  EigenPerfTest ept;
  ept.createDiffMatrix(3);
  ept.createCostMatrix();
  ept.computeCholeskyFactor();
  ept.createConstraintMatrix();
  //ept.computeConstraintQR();

  ept.computeConstraintProjector();
  return 0;
}
