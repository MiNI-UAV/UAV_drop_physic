#include "params.hpp"
#include <iostream>

Params* Params::_singleton = nullptr;

Params::Params() 
{
    if(_singleton != nullptr)
    {
        std::cerr << "Only one instance of Params should exist";
        return;
    }
    _singleton = this;

    STEP_TIME = 0.003;
    ODE_METHOD = "RK4";
}

Params::~Params() 
{
    _singleton = nullptr;
}

const Params *Params::getSingleton() { return _singleton; }