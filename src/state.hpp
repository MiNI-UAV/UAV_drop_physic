#pragma once
#include <Eigen/Dense>
#include <zmq.hpp>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include "status.hpp"

class ObjParams
{
    static int counter;
    constexpr static int validityOfForce = 3;

    public: 
        const int id;
        const double mass;
        const double CS_coff;
       


        ObjParams(double mass, double CS_coff):
        id{counter++}, mass{mass}, CS_coff{CS_coff}, wind{Eigen::Vector3d()} , force{Eigen::Vector3d()}, forceValidityCounter{-1}
        {   
        }

        ObjParams(ObjParams&& rhs)
        : id{rhs.id}, mass{rhs.mass}, CS_coff{CS_coff}, wind{rhs.wind}
        {
        }

        void setWind(Eigen::Vector3d newWind);
        Eigen::Vector3d getWind();
        void setForce(Eigen::Vector3d newForce);
        Eigen::Vector3d getForce();

    private:
        Eigen::Vector3d wind;
        std::mutex mtxWind;
        Eigen::Vector3d force;
        std::atomic_int forceValidityCounter;
        std::mutex mtxForce;

};

class State
{
    public:
        State();
        Eigen::VectorXd getState();
        void updateState(Eigen::VectorXd newState);
        void updateWind(int id, Eigen::Vector3d newWind);
        void updateForce(int id, Eigen::Vector3d newForce);
        std::mutex stateMutex;

        void addObj(double mass, double CS_coff, Eigen::Vector3d pos, Eigen::Vector3d vel = Eigen::Vector3d());
        void removeObj(int id);
        std::string to_string();

        int findIndex(int id);
        inline int getNoObj() {return noObj;}
        inline ObjParams* getParams(int index) {return obj_params[index].get();}
        inline Eigen::Vector3d getPos(int index) {return state.segment<3>(6*index);}
        inline Eigen::Vector3d getVel(int index) {return state.segment<3>(3+6*index);}
        inline void setVel(int index, Eigen::Vector3d newVel) {state.segment<3>(3+6*index) = newVel;}

        double real_time;
        Status status;


    private:
        int noObj;
        Eigen::VectorXd state;
        std::vector<std::unique_ptr<ObjParams>> obj_params;
       
};