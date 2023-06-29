#pragma once
#include <Eigen/Dense>
#include <zmq.hpp>
#include <thread>
#include <vector>
#include "status.hpp"

struct ObjParams
{
    const int id;
    const double mass;
    const double diameter;
    Eigen::Vector3d wind;

    static int counter;

    ObjParams(double mass, double diameter):
    id{counter++}, mass{mass}, diameter{diameter}, wind{Eigen::Vector3d()}
    {   
    }

    ObjParams(ObjParams&& rhs)
     : id{rhs.id}, mass{rhs.mass}, diameter{diameter}, wind{rhs.wind}
    {
    }
};

class State
{
    public:
        State();
        Eigen::VectorXd getState();
        void updateState(Eigen::VectorXd newState);
        std::mutex stateMutex;

        void addObj(double mass, double diameter, Eigen::Vector3d pos, Eigen::Vector3d vel = Eigen::Vector3d());
        void removeObj(int id);
        std::string to_string();

        inline int getNoObj() {return noObj;}
        inline ObjParams& getParams(int index) {return *obj_params[index];}
        inline Eigen::Vector3d getPos(int index) {return state.segment<3>(6*index);}
        inline Eigen::Vector3d getVel(int index) {return state.segment<3>(3+6*index);}

        double real_time;
        Status status;


    private:
        int noObj;
        Eigen::VectorXd state;
        std::vector<std::unique_ptr<ObjParams>> obj_params;
       
};