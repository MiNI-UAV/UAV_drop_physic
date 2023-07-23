#pragma once
#include <zmq.hpp>
#include <thread>
#include "state.hpp"
#include <Eigen/Dense>
#include <functional>

#define FRICTION_EPS 0.001
#define GENTLY_PUSH 0.15

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
        const double step_time = 0.005;
        const Eigen::Vector3d gravity = Eigen::Vector3d(0.0,0.0,9.81);
        const double air_density = 1.204;
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
