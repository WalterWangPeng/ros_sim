#ifndef OSQPSMOOTHER_H
#define OSQPSMOOTHER_H

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "OsqpEigen/OsqpEigen.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "path_smoother.h"
 
using namespace Eigen;
class OSQPSmoother : public PathSmoother{
    private:
        int N_ = 0;
        void set_smootherH(MatrixXd &HSmoother);
        void set_lengthH(MatrixXd &HLength);
        void set_refH(MatrixXd &HRef);
        void set_f(nav_msgs::Path, MatrixXd &f);
        void set_lb(nav_msgs::Path, MatrixXd &lb);
        void set_ub(nav_msgs::Path, MatrixXd &up);

    public:
        nav_msgs::Path smootherPath(nav_msgs::Path path, double cost_smoother, double cost_length, double cost_ref);
        
};

#endif