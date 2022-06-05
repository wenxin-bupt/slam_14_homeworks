#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char** argv) {
    Eigen::Quaterniond q_1(0.55, 0.3, 0.2, 0.2);
    Eigen::Vector3d t_1(0.7, 1.1, 0.2);

    q_1.normalize();

    Eigen::Quaterniond q_2(-0.1,  0.3, -0.7, 0.2);
    Eigen::Vector3d t_2(-0.1, 0.4, 0.8);

    q_2.normalize();

    Eigen::Vector3d p_1(0.5, -0.1, 0.2);


    // 1. trans back to world axis
    Eigen::Vector3d p_w = p_1 - t_1;
    p_w = q_1.inverse() * p_w; // q-1 * p_w * q

    // 2. trans to robot2 axis 
    Eigen::Vector3d p_2 = q_2 * p_w;
    p_2 = p_2 + t_2;

    std::cout << "the point on robot 2 axis is\n" << p_2<< std::endl;

    return 0;
}