#include <Eigen/Dense>
#include <zmq.hpp>
#include <iostream>
#include <cstdio>
#include <thread>
#include <mutex>
#include <Eigen/Dense>
#include <functional>

#include "simulation.hpp"
#include "timed_loop.hpp"
#include "state.hpp"
#include "RK4.hpp"


Simulation::Simulation()
{
    RHS = [] (double, Eigen::VectorXd) {return Eigen::VectorXd();};
}

void Simulation::run()
{
    TimedLoop loop(std::round(step_time*1000.0), [this](){
        std::unique_lock<std::mutex> lock(state.stateMutex);
        Eigen::VectorXd next = RK4_step(state.real_time,state.getState(),RHS,step_time);
        state.updateState(next);
        double next_time = state.real_time + step_time;
        state.real_time = next_time;
        lock.unlock();
        sendState(next_time,next);
    }, state.status);

    loop.go();
}

void Simulation::addObj(double mass, double diameter,
    Eigen::Vector3d pos, Eigen::Vector3d vel) 
{
    const std::lock_guard<std::mutex> lock(state.stateMutex);
    state.addObj(mass,diameter,pos,vel);
    calcRHS();
}

void Simulation::removeObj(int index)
{
    const std::lock_guard<std::mutex> lock(state.stateMutex);
    state.removeObj(index);
    calcRHS();
}

void Simulation::calcRHS()
{
    RHS = [this] (double, Eigen::VectorXd local_state) 
        {
            int no = state.getNoObj();
            VectorXd res(6*no);
            res.segment(0,6*no - 3) = local_state.segment(3, 6*no);
            for (int i = 0; i < no; i++)
            {
                ObjParams p = state.getParams(i);
                res.segment<3>(3+6*i) = (p.mass*gravity)/p.mass;
            }
            
            return res;
        };
}

void Simulation::sendState(double time, Eigen::VectorXd state)
{
    std::cout << time << std::endl << state << std::endl;  
}

Simulation::~Simulation()
{

}
