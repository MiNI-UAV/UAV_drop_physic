#pragma once
#include <zmq.hpp>
#include <thread>
#include "state.hpp"
#include <Eigen/Dense>
#include <functional>
#include "common.hpp"
#include "defines.hpp"




class Simulation
{
    public:
        Simulation();
        ~Simulation();
        void run();

        int addObj(double mass, double CS,
            Eigen::Vector3d pos, Eigen::Vector3d vel = Eigen::Vector3d());
        void removeObj(int id);
        void addCommand(std::string msg, zmq::socket_t& sock);
        void updateWind(std::string msg, zmq::socket_t& sock);
        void updateForce(std::string msg, zmq::socket_t& sock);
        void solidSurfColision(std::string& msg_str, zmq::socket_t& sock);
        void calcImpulseForce(int id, double COR,
            double mi_static, double mi_dynamic, Eigen::Vector3d surfaceNormal);

    private:
        const std::string path = "ipc:///tmp/drop_shot";

        zmq::context_t _ctx;
        State state;
        std::function<Eigen::VectorXd(double,Eigen::VectorXd)> RHS;
        std::thread controlListener;
        zmq::socket_t statePublishSocket;

        void sendState(std::string&& msg);
        void calcRHS();
        Eigen::Vector3d calcAerodynamicForce(Eigen::Vector3d vel, ObjParams* params);
};
