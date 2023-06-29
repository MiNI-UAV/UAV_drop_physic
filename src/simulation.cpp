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
            Eigen::Vector3d v;
            v.setConstant(3);
            addObj(3,3,v,v);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    });
    statePublishSocket = zmq::socket_t(_ctx, zmq::socket_type::pub);
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

Eigen::Vector3d Simulation::calcAerodynamicForce(Vector3d vel, ObjParams& params)
{
    Eigen::Vector3d diff = vel-params.wind;
    double dynamic_pressure = 0.5*air_density*diff.dot(diff);
    if(dynamic_pressure == 0.0)
    {
        return Eigen::Vector3d(0.0,0.0,0.0);
    }
    return -params.CS_coff*dynamic_pressure*diff.normalized();
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
            Eigen::VectorXd res(6*no);
            res.segment(0,6*no - 3) = local_state.segment(3, 6*no-3);
            for (int i = 0; i < no; i++)
            {
                ObjParams& p = state.getParams(i);
                Eigen::Vector3d vel = local_state.segment<3>(3+6*i);
                res.segment<3>(3+6*i) = (p.mass*gravity + calcAerodynamicForce(vel,p))/p.mass;
            }
            return res;
        };
}

void Simulation::sendState(std::string&& msg)
{
    std::cout << msg << std::endl;
    zmq::message_t message(msg.data(), msg.size());
    statePublishSocket.send(message,zmq::send_flags::none);
}

Simulation::~Simulation()
{
    controlListener.join();
}
