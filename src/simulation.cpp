#include <Eigen/Dense>
#include <zmq.hpp>
#include <iostream>
#include <cstdio>
#include <thread>
#include <mutex>
#include <Eigen/Dense>
#include <functional>
#include <map>
#include <filesystem>
namespace fs = std::filesystem;
#include "simulation.hpp"
#include "common.hpp"
#include "state.hpp"


Simulation::Simulation(const Params& params)
    : _params{params}
{

    if (!std::filesystem::exists(path.substr(6)) && !fs::create_directory(path.substr(6)))
        std::cerr <<  "Can not create comunication folder" <<std::endl;
    calcRHS();
    statePublishSocket = zmq::socket_t(_ctx, zmq::socket_type::pub);
    statePublishSocket.bind(path + "/state");
    std::cout << "Drop&shot state: " << path + "/state" << std::endl;
    controlListener = std::thread([this]()
    {
        std::cout << "Drop&shot control: " << path + "/control"  << std::endl;
        zmq::socket_t controlInSock = zmq::socket_t(_ctx, zmq::socket_type::rep);
        controlInSock.bind(path + "/control");
        bool run = true;
        while(run)
        {
            zmq::message_t msg;
            const auto res = controlInSock.recv(msg, zmq::recv_flags::none);
            if(!res)
            {
                std::cerr << "Listener recv error" << std::endl;
                return;
            } 
            std::string msg_str =  std::string(static_cast<char*>(msg.data()), msg.size());
            zmq::message_t response("ok",2);
            switch(msg_str[0])
            {
                case 'a':
                    addCommand(msg_str,controlInSock);
                break;
                case 'r':
                    removeObj(std::stoi(msg_str.substr(2)));
                    controlInSock.send(response,zmq::send_flags::none);
                break;
                case 'w':
                    updateWind(msg_str,controlInSock);
                break;
                case 'f':
                    updateForce(msg_str,controlInSock);
                break;
                case 'j':
                    solidSurfColision(msg_str,controlInSock);
                break;
                case 's':
                    run = false;
                    state.status = Status::exiting;
                    controlInSock.send(response,zmq::send_flags::none);
                break;
                default:
                    response.rebuild("error",5);
                    controlInSock.send(response,zmq::send_flags::none);
                    std::cerr << "Unknown msg: " << msg_str << std::endl;
                    state.status = Status::exiting;
                    run = false;
            }
        }
        controlInSock.close();
    });

}

void Simulation::run()
{
    TimedLoop loop(std::round(_params.STEP_TIME*1000.0), [this](){
        std::unique_lock<std::mutex> lock(state.stateMutex);
        Eigen::VectorXd next = RK4_step(state.real_time,state.getState(),RHS,_params.STEP_TIME);
        state.updateState(next);
        state.real_time += _params.STEP_TIME;
        auto msg = state.to_string();
        lock.unlock();
        sendState(std::move(msg));
    }, state.status);

    loop.go();
}

int Simulation::addObj(double mass, double CS,
    Eigen::Vector3d pos, Eigen::Vector3d vel) 
{
    const std::scoped_lock lock(state.stateMutex);
    int id = state.addObj(mass,CS,pos,vel);
    calcRHS();
    return id;
}

void Simulation::removeObj(int id)
{
    const std::scoped_lock lock(state.stateMutex);
    state.removeObj(id);
    calcRHS();
}

void Simulation::addCommand(std::string msg, zmq::socket_t& sock) 
{
    std::istringstream f(msg.substr(2));
    std::string res;
    int i;
    double m = 0, CS = 0;
    Eigen::Vector3d pos,vel;
    for (i = 0; i < 8; i++)
    {
        if(!getline(f, res, ',')) break;
        switch (i)
        {
            case 0:
                m = std::stod(res);
                break;
            case 1:
                CS = std::stod(res);
                break;
            case 2:
            case 3:
            case 4:
                pos(i-2) = std::stod(res);
                break;
            case 5:
            case 6:
            case 7:
                vel(i-5) = std::stod(res);
                break;
        }
    }
    zmq::message_t response("error",5);
    if(i == 5 || i == 8)
    {
        int id = addObj(m,CS,pos,vel);
        auto msg = "ok;" + std::to_string(id);
        response.rebuild(msg.data(),msg.size());
    }
    else
    {
        std::cerr << "Invalid add command: " << msg << std::endl;
    }
    sock.send(response,zmq::send_flags::none);
}

void Simulation::updateWind(std::string msg, zmq::socket_t& sock)
{
    zmq::message_t response("error",5);
    std::istringstream f(msg.substr(2));
    std::string s, res;
    int i;
    std::map<int,Eigen::Vector3d> wind;
    for (i = 0; i < state.getNoObj() + 3; i++)
    {
        if(!getline(f, res, ';')) break;
        std::istringstream iss(res);
        int id, j;
        Eigen::Vector3d wind_vec;
        for(j = 0; j < 4; j++)
        {
            if(!getline(iss, s, ',')) break;
            switch (j)
            {
                case 0:
                    id = std::stoi(s);
                    break;
                case 1:
                case 2:
                case 3:
                    wind_vec(j-1) = std::stod(s);
                    break;
            }
        }
        if(j != 4)
        {
            std::cerr << "Invalid add command: " << msg << std::endl;
            sock.send(response,zmq::send_flags::none);
            return;
        }
        wind.insert({id,wind_vec});
    }
    for(auto &i: wind)
    {
        state.updateWind(i.first,i.second);
    }
    response.rebuild("ok",2);
    sock.send(response,zmq::send_flags::none);
}

void Simulation::updateForce(std::string msg, zmq::socket_t &sock)
{
    zmq::message_t response("error",5);
    std::istringstream f(msg.substr(2));
    std::string s;
    int id = -1,j;
    Eigen::Vector3d force;
    for(j = 0; j < 4; j++)
    {
        if(!getline(f, s, ',')) break;
        switch (j)
        {
            case 0:
                id = std::stoi(s);
                break;
            case 1:
            case 2:
            case 3:
                force(j-1) = std::stod(s);
                break;
        }
    }
    if(j != 4 || id < 0)
    {
        std::cerr << "Invalid add command: " << msg << std::endl;
        sock.send(response,zmq::send_flags::none);
        return;
    }
    state.updateForce(id,force);
    response.rebuild("ok",2);
    sock.send(response,zmq::send_flags::none);
}

Eigen::Vector3d Simulation::calcAerodynamicForce(Eigen::Vector3d vel, ObjParams* params)
{
    Eigen::Vector3d diff = vel-params->getWind();
    double dynamic_pressure = 0.5*def::DEFAULT_AIR_DENSITY*diff.dot(diff);
    if(dynamic_pressure == 0.0)
    {
        return Eigen::Vector3d(0.0,0.0,0.0);
    }
    return -params->CS_coff*dynamic_pressure*diff.normalized();
}

void Simulation::calcImpulseForce(int id,double COR, double mi_static, double mi_dynamic, Eigen::Vector3d surfaceNormal)
{
    const std::lock_guard<std::mutex> lock(state.stateMutex);
    int index = state.findIndex(id);
    std::cout << "Index " << index << std::endl; 
    if(index < 0) return;

    Eigen::Vector3d v = state.getVel(index);
    Eigen::Vector3d X_g = v;
    double vn = v.dot(surfaceNormal);
    if(vn >= 0.0)
    {
        return;
    }
    double mass = state.getParams(index)->mass;
    std::cout << "Energy before collision: " << 0.5*state.getParams(index)->mass* X_g.squaredNorm() << std::endl;
    if(vn > -def::GENTLY_PUSH) vn = -def::GENTLY_PUSH;
    double jr = (-(1+COR)*vn)*mass;
    X_g = X_g + (jr/mass)*surfaceNormal;
    Eigen::Vector3d vt = v - (v.dot(surfaceNormal))*surfaceNormal;
    if(vt.squaredNorm() > def::FRICTION_EPS)
    {
        Eigen::Vector3d tangent = vt.normalized();
        double js = mi_static*jr;
        double jd = mi_dynamic*jr;
        double jf = -vt.norm()*mass;
        if(jf > js) jf = jd;
        X_g = X_g + (jf/mass) *tangent;
    }
    std::cout << "Energy after collision: " << 0.5*state.getParams(index)->mass* v.squaredNorm()  << std::endl;
    state.setVel(index,X_g);
}

bool isNormal(double factor)
{
    return factor >= 0.0 && factor <= 1.0;
}

void Simulation::solidSurfColision(std::string& msg_str, zmq::socket_t& sock)
{
    std::istringstream f(msg_str.substr(2));
    zmq::message_t response("error",5);
    int i, id = -1;
    double COR = 0.0, mi_static = 0.0, mi_dynamic = 0.0;
    Eigen::Vector3d surfaceNormal(0.0,0.0,0.0);
    std::string res;
    for (i = 0; i < 7; i++)
    {
        if(!getline(f, res, ',')) break;
        switch (i)
        {
        case 0:
            id = std::stoi(res);
        break;
        case 1:
            COR = std::stod(res);
            break;
        case 2:
            mi_static = std::stod(res);
            break;
        case 3:
            mi_dynamic = std::stod(res);
            break;
        case 4:
        case 5:
        case 6:
            surfaceNormal(i-4) = std::stod(res);
            break;
        }
    }
    if (i == 7
        && id >= 0
        && isNormal(COR)
        && isNormal(mi_static)
        && isNormal(mi_dynamic)
        && mi_static >= mi_dynamic)
    {
        calcImpulseForce(id,COR, mi_static, mi_dynamic, surfaceNormal);
        response.rebuild("ok",2);
    }
    sock.send(response,zmq::send_flags::none);
    return; 
}

void Simulation::calcRHS()
{
    static const Eigen::Vector3d gravity = Eigen::Vector3d(0.0,0.0,def::GRAVITY_CONST);
    if(state.getNoObj() == 0)
    {
        RHS = [] (double, Eigen::VectorXd) {return Eigen::VectorXd();};
        return;
    }
    RHS = [this] (double, Eigen::VectorXd local_state) 
        {
            int no = state.getNoObj();
            Eigen::VectorXd res(6*no);
            res.segment(0,6*no - 3) = local_state.segment(3, 6*no-3);
            for (int i = 0; i < no; i++)
            {
                ObjParams* p = state.getParams(i);
                Eigen::Vector3d vel = local_state.segment<3>(3+6*i);
                res.segment<3>(3+6*i) = (p->mass*gravity + calcAerodynamicForce(vel,p) + p->getForce())/p->mass;
            }
            return res;
        };
}

void Simulation::sendState(std::string&& msg)
{
    //std::cout << msg << std::endl;
    zmq::message_t message(msg.data(), msg.size());
    statePublishSocket.send(message,zmq::send_flags::none);
}

Simulation::~Simulation()
{
    controlListener.join();
}
