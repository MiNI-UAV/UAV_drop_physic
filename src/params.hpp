#pragma once
#include <string>

/// @brief Simulation parameters
class Params
{
public:
    /// @brief Constructor
    Params();

    /// @brief Deconstructor
    ~Params();

    /// @brief Step time of simulation. Step of ODE solving methods
    double STEP_TIME;

    /// @brief ODE solving method used in simulation
    std::string ODE_METHOD;

    /// @brief Get singleton of Params.
    /// @return const pointer to Params instance. Return nullptr if not initialized
    static const Params* getSingleton();

private:
    static Params* _singleton;
};
