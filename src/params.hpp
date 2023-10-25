#pragma once
#include <string>

/// @brief Simulation parameters
struct Params
{
    /// @brief Step time of simulation. Step of ODE solving methods
    double STEP_TIME = 0.003;

    /// @brief ODE solving method used in simulation
    std::string ODE_METHOD = "RK4";
};
