#ifndef PATHSMOOTHER_H
#define PATHSMOOTHER_H

#include <iostream>
#include <string>
#include <memory>
#include "nav_msgs/Path.h"


class PathSmoother{
    public:
        static std::unique_ptr<PathSmoother> create(const std::string &type);
        virtual nav_msgs::Path smootherPath(nav_msgs::Path path, double cost_smoother, double cost_length, double cost_ref) = 0;
};

#endif