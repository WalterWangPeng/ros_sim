#include "global_planner/path_smoother/path_smoother.h"
#include "global_planner/path_smoother/osqp_smoother.h"
#include <memory>

std::unique_ptr<PathSmoother> PathSmoother::create(const std::string &type)
{
    if(type == "OSQP")
    {
        return std::unique_ptr<PathSmoother>(new OSQPSmoother());
    }
    else
    {
        std::cout << "No such smoother" << std::endl;
        return nullptr;
    }

    return nullptr;
}