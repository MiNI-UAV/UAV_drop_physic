#include <Eigen/Dense>
#include <mutex>
#include <iostream>
#include "state.hpp"
#include "common.hpp"

int ObjParams::counter = 0;


void ObjParams::setWind(Eigen::Vector3d newWind) 
{
    std::scoped_lock lock(mtxWind);
    wind = newWind;
}

Eigen::Vector3d ObjParams::getWind()
{
    std::scoped_lock lock(mtxWind);
    return wind;
}

void ObjParams::setForce(Eigen::Vector3d newForce)
{
    std::scoped_lock lock(mtxForce);
    force = newForce;
    forceValidityCounter = validityOfForce * 4; //< 4 times bcs RK4 call function 4 times.
}

Eigen::Vector3d ObjParams::getForce()
{
    std::scoped_lock lock(mtxForce);
    if(forceValidityCounter > 0)
    {
        forceValidityCounter--;
        return force;
    }
    return Eigen::Vector3d(0.0,0.0,0.0);
}


State::State(): logger("state.csv","time,id,PosX,PosY,PosZ,VelX,VelY,VelZ")
{
    status = Status::running;
    real_time = 0.0;
    noObj = 0;
    state = Eigen::VectorXd();
}

Eigen::VectorXd State::getState()
{
    return state;
}

void State::updateState(Eigen::VectorXd newState) {
    if(state.size() == newState.size())
    {
        state = newState;
    }
}

void State::updateWind(int id, Eigen::Vector3d newWind) {
    auto iter = std::find_if(obj_params.begin(),obj_params.end(),[id](std::unique_ptr<ObjParams>& o) {return o->id == id;});
    if(iter == obj_params.end()) return;
    iter->get()->setWind(newWind);
}

void State::updateForce(int id, Eigen::Vector3d newForce) 
{
    auto iter = std::find_if(obj_params.begin(),obj_params.end(),[id](std::unique_ptr<ObjParams>& o) {return o->id == id;});
    if(iter == obj_params.end()) return;
    iter->get()->setForce(newForce);
}

int State::addObj(double mass, double CS, Eigen::Vector3d pos,
                   Eigen::Vector3d vel) 
{
    static Logger obj_params_logger("params.csv", "time,id,CS");

    auto new_obj_param = std::make_unique<ObjParams>(mass,CS);
    int id = new_obj_param->id;
    obj_params.push_back(std::move(new_obj_param));
    noObj++;
    Eigen::VectorXd newState(state.size() + 6);
    newState << state, pos, vel;
    state = newState;
    obj_params_logger.log(real_time,{static_cast<double>(id), CS});
    return id;
}

void State::removeObj(int id) {
    auto iter = std::find_if(obj_params.begin(),obj_params.end(),[id](std::unique_ptr<ObjParams>& o) {return o->id == id;});
    int index = iter - obj_params.begin();
    if(index == noObj) return;
    obj_params.erase(iter);
    noObj--;
    Eigen::VectorXd newState(state.size() - 6);
    newState << state.head(6*index), state.tail(6*(noObj-index));
    state = newState;
}

std::string State::to_string()
{
    static Eigen::IOFormat commaFormat(6, Eigen::DontAlignCols," ",",","","",",",";");
    std::string msg;
    msg.reserve((noObj*60 + 100));
    msg += std::to_string(real_time);
    msg.push_back(';');
    for (int i = 0; i < noObj; i++)
    {
        msg += std::to_string(obj_params[i]->id);
        std::stringstream ss;
        ss << state.segment<6>(6*i).format(commaFormat);
        msg += ss.str();
        logger.log(real_time,{ Eigen::Vector<double,1>(obj_params[i]->id),state.segment<6>(6*i)});
    }
    return msg;
}

int State::findIndex(int id)
{
    auto iter = std::find_if(obj_params.begin(),obj_params.end(),[id](std::unique_ptr<ObjParams>& o) {return o->id == id;});
    int index = iter - obj_params.begin();
    return index == noObj ? -1 : index;
}
