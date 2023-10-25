#pragma once
#include <Eigen/Dense>
#include <zmq.hpp>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include "common.hpp"

/// @brief Single obj parameters
class ObjParams
{
    public: 
        /// @brief object id
        const int id;
        /// @brief object mass
        const double mass;
        /// @brief aerodynamic drag force cofficent multipled by aerodynamic field 
        const double CS_coff;
       
        /// @brief Constructor
        /// @param mass object mass
        /// @param CS_coff aerodynamic drag force cofficent multipled by aerodynamic field
        ObjParams(double mass, double CS_coff):
        id{counter++}, mass{mass}, CS_coff{CS_coff}, wind{Eigen::Vector3d()} , force{Eigen::Vector3d()}, forceValidityCounter{-1}
        {   
        }

        /// @brief Moving constructor
        /// @param rhs other instant that should be consumed
        ObjParams(ObjParams&& rhs)
        : id{rhs.id}, mass{rhs.mass}, CS_coff{CS_coff}, wind{rhs.wind}
        {
        }

        /// @brief Set wind vector affecting on object
        /// @param newWind new wind speed vector in m/s
        void setWind(Eigen::Vector3d newWind);
        
        /// @brief Get wind vector
        /// @return wind speed vector in m/s
        Eigen::Vector3d getWind();

        /// @brief Set outer force applied to object
        /// @param newForce new force vector in N
        void setForce(Eigen::Vector3d newForce);

        /// @brief Get outer force
        /// @return outer force vector in N
        Eigen::Vector3d getForce();

    private:
        Eigen::Vector3d wind;
        std::mutex mtxWind;
        Eigen::Vector3d force;
        std::atomic_int forceValidityCounter;
        std::mutex mtxForce;
    
    /// @brief static counter of instances. Used to get next ID
    static int counter;
    /// @brief how many steps outer force should be valid
    constexpr static int validityOfForce = 1;
};

class State
{
    public:
        /// @brief Constructor
        State();

        /// @brief Get full state as vector
        /// @return state vector
        Eigen::VectorXd getState();

        /// @brief Update state
        /// @param newState new state vector
        void updateState(Eigen::VectorXd newState);

        /// @brief update wind speed for obj specified by id
        /// @param id id of updated obj
        /// @param newWind new wind speed vector
        void updateWind(int id, Eigen::Vector3d newWind);

        /// @brief update outer force applied to object specified by id
        /// @param id id of updated obj
        /// @param newForce new force value
        void updateForce(int id, Eigen::Vector3d newForce);

        /// @brief mutex to manipule on state responses
        std::mutex stateMutex;

        /// @brief Add new object to simulation
        /// @param mass mass of object
        /// @param CS_coff aerodynamic drag force cofficent multipled by aerodynamic field 
        /// @param pos start position
        /// @param vel start velocity
        /// @return id of added object
        int addObj(double mass, double CS_coff, Eigen::Vector3d pos, Eigen::Vector3d vel = Eigen::Vector3d());

        /// @brief remove object specified by id
        /// @param id id of removing object
        void removeObj(int id);

        /// @brief Serialize state to string
        /// @return serialized state
        std::string to_string();

        /// @brief Find index of object specified by id
        /// @param id object id
        /// @return object index
        int findIndex(int id);

        /// @brief Get number of active object in simulation
        /// @return number of object
        inline int getNoObj() {return noObj;}

        /// @brief get params of object specified by index
        /// @param index index of object
        /// @return pointer to object params
        inline ObjParams* getParams(int index) {return obj_params[index].get();}

        /// @brief Get position of object specified by index
        /// @param index index of object
        /// @return position vector
        inline Eigen::Vector3d getPos(int index) {return state.segment<3>(6*index);}

        /// @brief Get velocity of object specified by index
        /// @param index index of object
        /// @return velocity of object
        inline Eigen::Vector3d getVel(int index) {return state.segment<3>(3+6*index);}

        /// @brief Override velocity of object, for example after collision
        /// @param index index of object
        /// @param newVel new velocity vector
        inline void setVel(int index, Eigen::Vector3d newVel) {state.segment<3>(3+6*index) = newVel;}


        /// @brief time of simulation
        double real_time;

        /// @brief status for timed loop
        Status status;


    private:
        int noObj;
        Eigen::VectorXd state;
        std::vector<std::unique_ptr<ObjParams>> obj_params;
        Logger logger;
       
};