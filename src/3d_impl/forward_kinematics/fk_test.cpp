#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "forward_kinematics_roarm_double.h"


int main(){

    double curr_pos[NU] = {2.13747, -0.331807, 2.09324, 0.937579, 2.58646};

    double qcos[NU] = {
        cos(curr_pos[0]), cos(curr_pos[1]), cos(curr_pos[2]), cos(curr_pos[3]), cos(curr_pos[4])
    };
    double qsin[NU] = {
        sin(curr_pos[0]), sin(curr_pos[1]), sin(curr_pos[2]), sin(curr_pos[3]), sin(curr_pos[4])
    };

    double v[NU] = {0};
    double a[NU] = {0};

    auto fk = ForwardKinematics(
        qcos[0], qcos[1], qcos[2], qcos[3], qcos[4],
        qsin[0], qsin[1], qsin[2], qsin[3], qsin[4],
        v[0], v[1], v[2], v[3], v[4],
        a[0], a[1], a[2], a[3], a[4]
    );

    std::cout << "Forward Kinematics Result:" << std::endl;

    std::cout << "Joint 0:" << std::endl;
    std::cout << "Translation: " << std::endl;
    std::cout << fk.SE3[0].translation[0] << " " << fk.SE3[0].translation[1] << " " << fk.SE3[0].translation[2] << std::endl;

    std::cout << "Rotation: " << std::endl;
    std::cout << fk.SE3[0].rotation[0][0] << " " << fk.SE3[0].rotation[0][1] << " " << fk.SE3[0].rotation[0][2] << std::endl;
    std::cout << fk.SE3[0].rotation[1][0] << " " << fk.SE3[0].rotation[1][1] << " " << fk.SE3[0].rotation[1][2] << std::endl;
    std::cout << fk.SE3[0].rotation[2][0] << " " << fk.SE3[0].rotation[2][1] << " " << fk.SE3[0].rotation[2][2] << std::endl;

    
    std::cout << "Joint 1:" << std::endl;
    std::cout << "Translation: " << std::endl;
    std::cout << fk.SE3[1].translation[0] << " " << fk.SE3[1].translation[1] << " " << fk.SE3[1].translation[2] << std::endl;
    
    std::cout << "Rotation: " << std::endl;
    std::cout << fk.SE3[1].rotation[0][0] << " " << fk.SE3[1].rotation[0][1] << " " << fk.SE3[1].rotation[0][2] << std::endl;
    std::cout << fk.SE3[1].rotation[1][0] << " " << fk.SE3[1].rotation[1][1] << " " << fk.SE3[1].rotation[1][2] << std::endl;
    std::cout << fk.SE3[1].rotation[2][0] << " " << fk.SE3[1].rotation[2][1] << " " << fk.SE3[1].rotation[2][2] << std::endl;


    std::cout << "Joint 2:" << std::endl;
    std::cout << "Translation: " << std::endl;
    std::cout << fk.SE3[2].translation[0] << " " << fk.SE3[2].translation[1] << " " << fk.SE3[2].translation[2] << std::endl;

    std::cout << "Rotation: " << std::endl;
    std::cout << fk.SE3[2].rotation[0][0] << " " << fk.SE3[2].rotation[0][1] << " " << fk.SE3[2].rotation[0][2] << std::endl;
    std::cout << fk.SE3[2].rotation[1][0] << " " << fk.SE3[2].rotation[1][1] << " " << fk.SE3[2].rotation[1][2] << std::endl;
    std::cout << fk.SE3[2].rotation[2][0] << " " << fk.SE3[2].rotation[2][1] << " " << fk.SE3[2].rotation[2][2] << std::endl;


    std::cout << "Joint 3:" << std::endl;
    std::cout << "Translation: " << std::endl;
    std::cout << fk.SE3[3].translation[0] << " " << fk.SE3[3].translation[1] << " " << fk.SE3[3].translation[2] << std::endl;
    
    std::cout << "Rotation: " << std::endl;
    std::cout << fk.SE3[3].rotation[0][0] << " " << fk.SE3[3].rotation[0][1] << " " << fk.SE3[3].rotation[0][2] << std::endl;
    std::cout << fk.SE3[3].rotation[1][0] << " " << fk.SE3[3].rotation[1][1] << " " << fk.SE3[3].rotation[1][2] << std::endl;
    std::cout << fk.SE3[3].rotation[2][0] << " " << fk.SE3[3].rotation[2][1] << " " << fk.SE3[3].rotation[2][2] << std::endl;


    std::cout << "Joint 4:" << std::endl;
    std::cout << "Translation: " << std::endl;
    std::cout << fk.SE3[4].translation[0] << " " << fk.SE3[4].translation[1] << " " << fk.SE3[4].translation[2] << std::endl;

    std::cout << "Rotation: " << std::endl;
    std::cout << fk.SE3[4].rotation[0][0] << " " << fk.SE3[4].rotation[0][1] << " " << fk.SE3[4].rotation[0][2] << std::endl;
    std::cout << fk.SE3[4].rotation[1][0] << " " << fk.SE3[4].rotation[1][1] << " " << fk.SE3[4].rotation[1][2] << std::endl;
    std::cout << fk.SE3[4].rotation[2][0] << " " << fk.SE3[4].rotation[2][1] << " " << fk.SE3[4].rotation[2][2] << std::endl;


}