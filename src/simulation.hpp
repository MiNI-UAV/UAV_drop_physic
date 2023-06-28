#pragma once
#include <zmq.hpp>
#include <thread>

class Simulation
{
    public:
        Simulation();
        ~Simulation();
        void run();

    private:
        zmq::context_t _ctx;
};
