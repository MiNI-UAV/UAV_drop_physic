#pragma once
#include <zmq.hpp>
#include <thread>
#include "state.hpp"
#include <Eigen/Dense>
#include <functional>

class Simulation
{
    public:
        Simulation();
        ~Simulation();
        void run();

        void addObj(double mass, double diameter,
            Eigen::Vector3d pos, Eigen::Vector3d vel = Eigen::Vector3d());
        void removeObj(int index);



    private:
        const double step_time = 0.01;
        const Eigen::Vector3d gravity = Eigen::Vector3d(0.0,0.0,9.81);

        zmq::context_t _ctx;
        State state;
        std::function<Eigen::VectorXd(double,Eigen::VectorXd)> RHS;

        void sendState(double time, Eigen::VectorXd state);
        void calcRHS();
};
