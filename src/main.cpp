#include <iostream>
#include <Eigen/Dense>
#include <cxxopts.hpp>
#include "simulation.hpp"
#include "common.hpp"
#include "params.hpp"

Params parseArgs(int argc, char** argv)
{
    Params p;
    cxxopts::Options options("drop", "Physic engine for non-propelled objects");
    options.add_options()
        ("dt", "Step time of simulation in ms. Default: 3 ms", cxxopts::value<int>())
        ("o,ode", "ODE solver. Defaulf: RK4", cxxopts::value<std::string>())
        ("h,help", "Print usage");
    auto result = options.parse(argc, argv);
    if(result.count("help"))
    {
        std::cout << options.help() << std::endl;
        exit(0);
    }
    if(result.count("dt"))
    {
        p.STEP_TIME = result["dt"].as<int>()/1000.0;
        std::cout << "Step time changed to " << p.STEP_TIME << "s" << std::endl;
    }
    if(result.count("ode"))
    {
        p.ODE_METHOD = result["ode"].as<std::string>();
        std::cout << "ODE method changed to " << p.ODE_METHOD  << std::endl;
    }
    return p;
}

int main(int argc, char** argv)
{
    auto params = parseArgs(argc,argv);
    Logger::setLogDirectory("drop_physic");
    Simulation s(params);
    s.run();
}
