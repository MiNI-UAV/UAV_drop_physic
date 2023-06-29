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
    calcRHS();
    controlListener = std::thread([this]()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        for (size_t i = 1; i < 5; i++)
        {
            Eigen::Vector3d v;
            v.setConstant(i);
            addObj(i,i,v,v);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        removeObj(2);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        removeObj(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        removeObj(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    });
}

void Simulation::run()
{
    TimedLoop loop(std::round(step_time*1000.0), [this](){
        std::unique_lock<std::mutex> lock(state.stateMutex);
        Eigen::VectorXd next = RK4_step(state.real_time,state.getState(),RHS,step_time);
        state.updateState(next);
        state.real_time += step_time;
        auto msg = state.to_string();
        lock.unlock();
        sendState(std::move(msg));
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

void Simulation::removeObj(int id)
{
    const std::lock_guard<std::mutex> lock(state.stateMutex);
    state.removeObj(id);
    calcRHS();
}

void Simulation::calcRHS()
{
    if(state.getNoObj() == 0)
    {
        RHS = [] (double, Eigen::VectorXd) {return VectorXd();};
        return;
    }
    RHS = [this] (double, Eigen::VectorXd local_state) 
        {
            int no = state.getNoObj();
            VectorXd res(6*no);
            res.segment(0,6*no - 3) = local_state.segment(3, 6*no-3);
            for (int i = 0; i < no; i++)
            {
                ObjParams& p = state.getParams(i);
                res.segment<3>(3+6*i) = (p.mass*gravity)/p.mass;
            }
            return res;
        };
    std::cout << "setted" << std::endl;
}

void Simulation::sendState(std::string&& msg)
{
    std::cout << msg << std::endl;
}

Simulation::~Simulation()
{
    controlListener.join();
}
