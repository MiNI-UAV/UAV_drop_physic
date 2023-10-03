#include <iostream>
#include <Eigen/Dense>
#include "simulation.hpp"
#include "common.hpp"

int main(int, char**)
{
    Logger::setLogDirectory("drop_physic");
    Simulation s;
    s.run();
}
