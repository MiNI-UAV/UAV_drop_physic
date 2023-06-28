#include <iostream>
#include <Eigen/Dense>
#include "state.hpp"

int main(int, char**)
{
    State s;
    for (size_t i = 0; i < 4; i++)
    {
        Eigen::Vector3d v;
        v.setConstant(i);
        s.addObj(i,i,v,v);
        std::cout << "i: " << i << "\n" << s.getState() << std::endl;
    }

    s.removeObj(2);
    std::cout << "remove 2\n" << s.getState() << std::endl;
    s.removeObj(0);
    std::cout << "remove 0\n" << s.getState() << std::endl;
    s.removeObj(1);
    std::cout << "remove 1\n" << s.getState() << std::endl;
}