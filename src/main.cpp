#include <iostream>
#include <Eigen/Dense>
#include "simulation.hpp"

int main(int, char**)
{
    Simulation s;
    s.addObj(10,1,Eigen::Vector3d(0,0,0));
    s.run();
}
