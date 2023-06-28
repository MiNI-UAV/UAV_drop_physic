#pragma once
#include <Eigen/Dense>
#include <zmq.hpp>
#include <thread>
#include <vector>
#include "status.hpp"

struct ObjParams
{
    double mass;
    double diameter;
};

class State
{
    public:
        State();
        Eigen::VectorXd getState();
        void updateState(Eigen::VectorXd newState);
        std::mutex stateMutex;

        void addObj(double mass, double diameter, Eigen::Vector3d pos, Eigen::Vector3d vel = Eigen::Vector3d());
        void removeObj(int index);

        inline int getNoObj() {return noObj;}
        inline ObjParams getParams(int index) {return obj_params[index];}
        inline Eigen::Vector3d getPos(int index) {return state.segment<3>(6*index);}
        inline Eigen::Vector3d getVel(int index) {return state.segment<3>(3+6*index);}

        double real_time;
        Status status;


    private:
        int noObj;
        Eigen::VectorXd state;
        std::vector<ObjParams> obj_params;
       
};