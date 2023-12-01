#include "global_planner/path_smoother/osqp_smoother.h"

using namespace Eigen;
using namespace std;

//路径优化 平滑代价 几何相似代价 长度代价
nav_msgs::Path OSQPSmoother::smootherPath(nav_msgs::Path path, double cost_smoother, double cost_length, double cost_ref)
{
    nav_msgs::Path path_r;
    N_ = path.poses.size();
    MatrixXd H = MatrixXd::Zero(2*N_, 2*N_);
    MatrixXd HSmoother = MatrixXd::Zero(2*N_, 2*N_);
    MatrixXd HLength = MatrixXd::Zero(2*N_, 2*N_);
    MatrixXd HRef = MatrixXd::Zero(2*N_, 2*N_);
    
    //H
    set_smootherH(HSmoother);
    set_lengthH(HLength);
    set_refH(HRef);
    H = cost_smoother * HSmoother + cost_length * HLength + cost_ref * HRef;
    
    //F
    MatrixXd F = MatrixXd::Zero(2*N_, 2*N_);
    set_f(path, F);

    // lb ub
    MatrixXd lb = MatrixXd::Zero(2*N_, 1);
    MatrixXd ub = MatrixXd::Zero(2*N_, 1);
    set_lb(path, lb);
    set_ub(path, ub);

    //A
    MatrixXd A = MatrixXd::Identity(2*N_, 2*N_);

    OsqpEigen::Solver solver;
    Eigen::SparseMatrix<double> H_os;
    H_os.resize(2*N_, 2*N_);
    Eigen::VectorXd f_os;
    f_os.resize(2*N_);
    Eigen::SparseMatrix<double> A_os;
    A_os.resize(2*N_, 2*N_);
    Eigen::VectorXd lb_os;
    lb_os.resize(2*N_);
    Eigen::VectorXd ub_os;
    ub_os.resize(2*N_);

    for(int i = 0; i < H.rows(); i++)
    {
        for(int j = 0; j < H.cols(); j++)
        {
            H_os.insert(i,j) = H(i,j);
        }
    }

    for(int i = 0; i < A.rows(); i++)
    {
        for(int j = 0; j < A.cols(); j++)
        {
            A_os.insert(i,j) = A(i,j);
        }
    }

    for(int i = 0; i < F.rows(); i++)
    {
        f_os(i) = F(i,0);
    }

    for(int i = 0; i < lb.rows(); i++)
    {
        lb_os(i) = lb(i,0);
    }

    for(int i = 0; i < ub.rows(); i++)
    {
        ub_os(i) = ub(i, 0);
    }

    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(2*N_);
    solver.data()->setNumberOfConstraints(2*N_);
    if(!solver.data()->setHessianMatrix(H_os))
    {
        return path_r;
    }
    if(!solver.data()->setGradient(f_os))
    {
        return path_r;
    }
    if(!solver.data()->setLinearConstraintsMatrix(A_os))
    {
        return path_r;
    }
    if(!solver.data()->setLowerBound(lb_os))
    {
        return path_r;
    }
    if(!solver.data()->setUpperBound(ub_os))
    {
        return path_r;
    }
    if(!solver.initSolver())
    {
        return path_r;
    }

    Eigen::VectorXd qpsolution;
    if(!solver.solve())
    {
        return path_r;
    }

    qpsolution = solver.getSolution();
    geometry_msgs::PoseStamped p;
    for(int i = 0; i < qpsolution.rows(); i+=2)
    {
        p.pose.position.x = qpsolution(i);
        p.pose.position.y = qpsolution(i+1);
        // printf("%ff ",qpsolution(i));
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = 0.0;
        p.pose.orientation.w = 1.0;
        path_r.poses.push_back(p);
    }
    return path_r;
}

void OSQPSmoother::set_smootherH(MatrixXd &HSmoother)
{
    MatrixXd base = MatrixXd::Zero(6,2);
    base << 1, 0,
            0, 1,
           -2, 0,
            0,-2,
            1, 0,
            0, 1;
    MatrixXd A1T = MatrixXd::Zero(2*N_, 2*N_-4);
    for(int i = 0; i < N_ - 2; i++)
    {
        A1T.block(i*2, i*2, 6, 2) = base;
    }
    HSmoother = A1T * A1T.transpose();
}

void OSQPSmoother::set_lengthH(MatrixXd &HLength)
{
    MatrixXd base = MatrixXd::Zero(4, 2);
    base << 1, 0,
            0, 1,
           -1, 0,
            0,-1;
    MatrixXd A2T = MatrixXd::Zero(2*N_, 2*N_ - 2);
    for(int i = 0; i < N_-1; i++)
    {
        A2T.block(i*2, i*2, 4, 2) = base;
    }
    HLength = A2T * A2T.transpose();
}

void OSQPSmoother::set_refH(MatrixXd & HRef)
{
    HRef.setIdentity(2*N_, 2*N_);
}

void OSQPSmoother::set_f(nav_msgs::Path path, MatrixXd &f)
{
    for(int i = 0; i < N_; i++)
    {
        f(2*i,0) = -2 * path.poses[i].pose.position.x;
        f(2*i+1,0) = -2 * path.poses[i].pose.position.y;
    }
}

void OSQPSmoother::set_lb(nav_msgs::Path path, MatrixXd &lb)
{
    for(int i = 0; i < N_; i++)
    {
        if(i == 0 || i == N_ -1)
        {
            lb(2*i,0) = path.poses[i].pose.position.x;
            lb(2*i+1,0) = path.poses[i].pose.position.y;
        }
        else
        {
            lb(i*2,0) = path.poses[i].pose.position.x - 0.1;
            lb(i*2+1,0) = path.poses[i].pose.position.y - 0.1;
        }
    }
}

void OSQPSmoother::set_ub(nav_msgs::Path path, MatrixXd &ub)
{
    for(int i = 0; i < N_; i++)
    {
        if(i == 0 || i == N_ -1)
        {
            ub(2*i,0) = path.poses[i].pose.position.x;
            ub(2*i+1,0) = path.poses[i].pose.position.y;
        }
        else
        {
            ub(i*2,0) = path.poses[i].pose.position.x + 0.1;
            ub(i*2+1,0) = path.poses[i].pose.position.y + 0.1;
        }
    }
}