#include <Eigen/Dense>
#include "state.hpp"
#include "status.hpp"

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

void State::addObj(double mass, double diameter, Eigen::Vector3d pos,
                   Eigen::Vector3d vel) 
{
    //const std::lock_guard<std::mutex> lock(stateMutex);
    obj_params.push_back(ObjParams{mass,diameter});
    noObj++;
    Eigen::VectorXd newState(state.size() + 6);
    newState << state, pos, vel;
    state = newState;
}

void State::removeObj(int index) {
    if(index < 0 || index >= noObj) return;
    //const std::lock_guard<std::mutex> lock(stateMutex);
    obj_params.erase(obj_params.begin() + index);
    noObj--;
    Eigen::VectorXd newState(state.size() - 6);
    newState << state.head(6*index), state.tail(6*(noObj-index));
    state = newState;
}