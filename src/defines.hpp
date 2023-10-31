#pragma once


/// @brief Simulation constants
namespace def 
{
    /// @brief Gravity constant on Earth in m/s2
    const double GRAVITY_CONST = 9.81;

    /// @brief minimal friction that is calculated (numerical float eps)
    const double FRICTION_EPS = 0.001;

    /// @brief artificial force cofficient. Protect again diving objects in horizontal wall
    const double GENTLY_PUSH = 0.15;

    /// Dry air density in normal conditions in kg/m3
    const double DEFAULT_AIR_DENSITY = 1.224;

    /// @brief how many steps outer force should be valid
    const static int VALIDITY_OF_FORCE = 1;
} // namespace def