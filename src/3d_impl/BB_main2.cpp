#include "rrt_3d.h" 
#include "forward_kinematics_roarm_double.h"
#include <vector>
#include <cmath>
#include <random>   
#include <stdexcept> 
#include <limits>    
#include <iostream>
#include <algorithm>
#include <sys/stat.h> 
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string>
#include <dirent.h>
#include <cstring>
#include <cerrno>

// --- Main Application Logic ---
int main() {
    std::cout << "reached main" << std::endl;

    // --- Configuration ---
    const Configuration c_init = {0.0, 0.0, 0.0, -1.0, 0.0};
    const Configuration c_goal = {0.5, 1.0, 1.0, -0.5, 1.0};
    const double p = 0.5; // Goal bias probability
    const int k = 20;      // Number of neighbors for RRT*
    const double step_size = 0.1;
    const int NUM_NODES = 1000; // Number of iterations/nodes to add
    const int SEED = 42;
    //const double XYZ_MIN = -2.0;
    //const double XYZ_MAX = 2.0;

    std::vector<SphereObstacle> obstacles;
    obstacles.emplace_back(SphereObstacle{0.14 , 0.05, 0.4, 0.05});
    obstacles.emplace_back(SphereObstacle{0.24 , 0.05, 0.4, 0.05});
    obstacles.emplace_back(SphereObstacle{0.14 , 0.0025, 0.4, 0.05});
    obstacles.emplace_back(SphereObstacle{0.14 , 0.0025, 0.3, 0.05});
    obstacles.emplace_back(SphereObstacle{0.0 , 0.05, 0.3, 0.05});
    obstacles.emplace_back(SphereObstacle{0.0 , 0.05, 0.2, 0.05});
    obstacles.emplace_back(SphereObstacle{0.1 , 0.04, 0.3, 0.05});
    obstacles.emplace_back(SphereObstacle{0.1 , 0.04, 0.2, 0.05});

    // Joint angle restrictions in radians
    std::vector<std::pair<double, double>> joint_limits = { {-2.8973, 2.8973}, {-1.7628, 1.7628}, {-2.8973, 2.8973}, {-3.0718, -0.0698}, {-2.8973, 2.8973} };

    // --- RRT Initialization ---
    RRTStar3D rrt(SEED, obstacles, joint_limits);
    rrt.init_rrt(c_init, c_goal);

    // --- RRT Execution ---
    std::cout << "Running RRT* for " << NUM_NODES << " iterations..." << std::endl;
    for (int i = 0; i < NUM_NODES; ++i) {
        rrt.add_node(p, k, step_size);
        if ((i + 1) % 100 == 0) {
            std::cout << "Iteration: " << (i + 1) << "/" << NUM_NODES << std::endl;
        }
    }
    std::cout << "RRT* calculation finished." << std::endl;

    // --- Data Collection ---
    std::cout << "Collecting data for output..." << std::endl;
    auto goal_points = rrt.get_path_to_goal(); 
    if (goal_points.empty()) {
        std::cerr << "No valid path to goal found." << std::endl;
        //close(serial_fd);
        return 1;
    }

    // Set up joint positions robot will execute to reach goal
    std::vector<std::vector <double>> joint_positions;
    for (int i =0; i < goal_points.size(); i++){
        joint_positions.push_back(goal_points[i].first);
    }
    joint_positions.push_back(goal_points[goal_points.size() - 1].second);

    std::ofstream outfile("output.txt");
    // --- Send path to outfile ---
    for(const auto& point : joint_positions){
        if(point.size() != 5){
            std::cerr << "Invalid joint config size" << std::endl;
            continue;
        }

        char command[256];
        snprintf(command, sizeof(command),
            "{\"T\":102,\"base\":%.6f,\"shoulder\":%.6f,\"elbow\":%.6f,\"wrist\":%.6f,\"roll\":%.6f,\"hand\":%.4f,\"spd\":%d,\"acc\":%d}",
            point[0], point[1], point[2], point[3], point[4],
            3.13, // Default hand/gripper value
            1,      // Default speed
            1);    // Default acceleration
        
        outfile << command << std::endl;
    }

    std::cout << "All commands sent" << std::endl;
    outfile.close();
    sleep(5);
    system("python process_commands.py /dev/ttyUSB0");
    return 0;
}