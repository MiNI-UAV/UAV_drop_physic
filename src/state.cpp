#include <Eigen/Dense>
#include "state.hpp"
#include "status.hpp"

int ObjParams::counter = 0;

State::State()
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
    iter->get()->wind = newWind;
}

void State::addObj(double mass, double diameter, Eigen::Vector3d pos,
                   Eigen::Vector3d vel) 
{
    obj_params.push_back(std::make_unique<ObjParams>(mass,diameter));
    noObj++;
    Eigen::VectorXd newState(state.size() + 6);
    newState << state, pos, vel;
    state = newState;
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
    }
    return msg;
}
