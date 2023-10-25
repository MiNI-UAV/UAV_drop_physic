#pragma once
#include <zmq.hpp>
#include <thread>
#include "state.hpp"
#include <Eigen/Dense>
#include <functional>
#include "common.hpp"
#include "defines.hpp"
#include "params.hpp"




class Simulation
{
    public:
        /// @brief Constructor
        /// @param params simulation params
        Simulation(const Params& params);
        /// @brief Deconstructor
        ~Simulation();
        /// @brief Run simulation
        void run();

        /// @brief Add new object to simulation
        /// @param mass obj mass
        /// @param CS aerodynamic drag force cofficent multipled by aerodynamic field
        /// @param pos start position of object
        /// @param vel start velocity of object
        /// @return id of added object
        int addObj(double mass, double CS,
            Eigen::Vector3d pos, Eigen::Vector3d vel = Eigen::Vector3d());
            
        /// @brief Remove object from simulation
        /// @param id object id
        void removeObj(int id);

        /// @brief Handle add new object command
        /// @param msg message content
        /// @param sock zmq socket reply is send by
        void addCommand(std::string msg, zmq::socket_t& sock);

        /// @brief Handle update wind command
        /// @param msg message content
        /// @param sock zmq socket reply is send by
        void updateWind(std::string msg, zmq::socket_t& sock);

        /// @brief Handle update force command
        /// @param msg message content
        /// @param sock zmq socket reply is send by
        void updateForce(std::string msg, zmq::socket_t& sock);

        /// @brief Handle solid surface collision command
        /// @param msg message content
        /// @param sock zmq socket reply is send by
        void solidSurfColision(std::string& msg_str, zmq::socket_t& sock);

        /// @brief Calculates object state after collision with given surface
        /// @param id object id
        /// @param COR coefficient of restitution. e = 0 is perfect inelastic collision, e = 1 is perfect elastic collision.
        /// 0 < e < 1 is a real-world inelastic collision, in which some kinetic energy is dissipated.
        /// @param mi_static static friction cofficient
        /// @param mi_dynamic dynamic friction cofficient
        /// @param surfaceNormal surface normal vector
        void calcImpulseForce(int id, double COR,
            double mi_static, double mi_dynamic, Eigen::Vector3d surfaceNormal);

    private:
        const std::string path = "ipc:///tmp/drop_shot";

        zmq::context_t _ctx;
        State state;
        std::function<Eigen::VectorXd(double,Eigen::VectorXd)> RHS;
        std::thread controlListener;
        zmq::socket_t statePublishSocket;
        const Params& _params;
        std::unique_ptr<ODE> ode;

        void sendState(std::string&& msg);
        void calcRHS();
        Eigen::Vector3d calcAerodynamicForce(Eigen::Vector3d vel, ObjParams* params);
};
