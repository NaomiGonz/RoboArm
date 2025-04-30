#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <cmath>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdexcept>

#include "rrt_3d.h"
#include "forward_kinematics/forward_kinematics_roarm_double.h"

using Configuration = std::vector<double>;

// --- Helper to call ForwardKinematics using a vector ---
inline auto call_forward_kinematics(const Configuration& joint_angles) { 
    if (joint_angles.size() != 5) {
        throw std::runtime_error("call_forward_kinematics error: Expected 5 joint angles.");
    }

    double qcos[5];
    double qsin[5];
    double v[5] = {0};
    double a[5] = {0};

    for (size_t i = 0; i < 5; ++i) {
        qcos[i] = std::cos(joint_angles[i]);
        qsin[i] = std::sin(joint_angles[i]);
    }

    return ForwardKinematics(
        qcos[0], qcos[1], qcos[2], qcos[3], qcos[4],
        qsin[0], qsin[1], qsin[2], qsin[3], qsin[4],
        v[0], v[1], v[2], v[3], v[4],
        a[0], a[1], a[2], a[3], a[4]
    );
}


int main() {
    std::cout << "reached main" << std::endl;

    // --- Configuration ---
    const Configuration c_init = {-1.5, 1.0, 1.0, -0.5, 1.0};
    const Configuration c_goal = {0.5, 1.0, 1.0, -0.5, 1.0};
    const double p = 0.5; // Goal bias probability
    const int k = 20;      // Number of neighbors for RRT*
    const double step_size = 0.1;
    const int NUM_NODES = 1000; // Number of iterations/nodes to add
    const int SEED = 42;

    std::vector<SphereObstacle> obstacles;
    obstacles.emplace_back(SphereObstacle{0.30, -0.17, 0.09, 0.20});

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
    auto all_edges = rrt.get_all_edges();
    auto goal_path = rrt.get_path_to_goal(); 
    std::vector<std::pair<Configuration, Configuration>> simplified_path;
    if (!goal_path.empty()) {
        simplified_path = rrt.simplify_path(goal_path, step_size);
    }
    double final_goal_cost = rrt.get_goal_cost();

    // --- Output File Creation ---
    std::string output_dir = "results";
    std::string filename;

    // Create results directory
    struct stat st = {0};
    if (stat(output_dir.c_str(), &st) == -1) {
        if (mkdir(output_dir.c_str(), 0755) == 0) {
             std::cout << "Created directory: " << output_dir << std::endl;
        } else {
            std::cerr << "Error: Could not create directory " << output_dir << std::endl;
            perror("mkdir error");
            return 1;
        }
    }

    std::cout << "Enter the output filename (e.g., rrt_output.csv): ";
    std::cin >> filename;
    std::string full_path = output_dir + "/" + filename;

    std::ofstream outfile(full_path);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open file " << full_path << " for writing." << std::endl;
        return 1;
    }
    std::cout << "Writing data to " << full_path << std::endl;

    outfile << std::fixed << std::setprecision(6);

    // --- Write Data Sections ---
    outfile << "# Section: Parameters" << std::endl;
    outfile << "Param,Value" << std::endl;
    outfile << "p," << p << std::endl;
    outfile << "k," << k << std::endl;
    outfile << "step_size," << step_size << std::endl;
    outfile << "num_nodes," << NUM_NODES << std::endl;
    outfile << "goal_cost,";
    if (std::isinf(final_goal_cost)) {
        outfile << "inf" << std::endl;
    } else {
        outfile << final_goal_cost << std::endl;
    }
    outfile << std::endl;

    // Joint Limits
    outfile << "# Section: Joint_Limits\njoint,q_min,q_max\n";
     for (std::size_t j = 0; j < joint_limits.size(); ++j) {
        outfile << (j + 1)               << ','
                << joint_limits[j].first  << ','
                << joint_limits[j].second << '\n';
    }
    outfile << std::endl;

    // Start Configuration
    outfile << "# Section: Start\nq1,q2,q3,q4,q5\n";
    for(size_t ii = 0; ii < c_init.size(); ++ii){
        outfile << c_init[ii] << (ii + 1 == c_init.size() ? "" : ",");
    }
    outfile << std::endl << std::endl;

    // Goal Configuration
    outfile << "# Section: Goal\nq1,q2,q3,q4,q5\n";
    for(size_t ii = 0; ii < c_goal.size(); ++ii){
        outfile << c_goal[ii] << (ii + 1 == c_goal.size() ? "" : ",");
    }
    outfile << std::endl << std::endl;

    // Obstacles
    outfile << "# Section: Obstacles\ncenter_x,center_y,center_z,radius\n";
    for (const auto& o : obstacles)
        outfile << o.x << ',' << o.y << ',' << o.z << ',' << o.radius << '\n';
    outfile << std::endl;

    // All Edges
    outfile << "# Section: All_Edges\n"
               "q1_src,q2_src,q3_src,q4_src,q5_src,"
               "q1_dst,q2_dst,q3_dst,q4_dst,q5_dst\n";
     for (const auto& e : all_edges) {
        for (size_t i = 0; i < e.first.size(); ++i)
             outfile << e.first[i] << ',';
        for (size_t i = 0; i < e.second.size(); ++i)
            outfile << e.second[i] << (i + 1 == e.second.size() ? '\n' : ',');
    }
    outfile << std::endl;

    // Goal Path (Joint Space Edges)
    outfile << "# Section: Goal_Path\n"
               "q1_src,q2_src,q3_src,q4_src,q5_src,"
               "q1_dst,q2_dst,q3_dst,q4_dst,q5_dst\n";
    for (const auto& e : goal_path) {
         for (size_t i = 0; i < e.first.size(); ++i)
             outfile << e.first[i] << ',';
        for (size_t i = 0; i < e.second.size(); ++i)
            outfile << e.second[i] << (i + 1 == e.second.size() ? '\n' : ',');
    }
    outfile << std::endl;

    // Simplified Path (Joint Space Edges)
    outfile << "# Section: Simplified_Path\n"
               "q1_src,q2_src,q3_src,q4_src,q5_src,"
               "q1_dst,q2_dst,q3_dst,q4_dst,q5_dst\n";
     for (const auto& e : simplified_path) {
         for (size_t i = 0; i < e.first.size(); ++i)
             outfile << e.first[i] << ',';
        for (size_t i = 0; i < e.second.size(); ++i)
            outfile << e.second[i] << (i + 1 == e.second.size() ? '\n' : ',');
    }
    outfile << std::endl;


    // --- Goal Path Cartesian (All Joints) ---
    outfile << "# Section: Goal_Path_Cartesian_All_Joints\n"
               "path_node_index,joint_frame_index,x,y,z\n"; // New header

    if (!goal_path.empty()) {
        std::cout << "Calculating Cartesian coordinates for all joint frames in goal path..." << std::endl;
        try {
            int path_node_index = 0;

            // Preform FK on the first node configuration 
            Configuration current_node = goal_path[0].first;
            auto fk_result = call_forward_kinematics(current_node);

            // Extract x y z for each joint
            for (int joint_frame_index = 0; joint_frame_index < 5; ++joint_frame_index) {
                const auto& frame_pose = fk_result.SE3[joint_frame_index]; 
                outfile << path_node_index << ","                            
                        << joint_frame_index << ","                         
                        << frame_pose.translation[0] << ","                
                        << frame_pose.translation[1] << ","                
                        << frame_pose.translation[2] << "\n";             
            }
            path_node_index++; 

            // Preform FK on the destination node of each edge
            for (const auto& edge : goal_path) {
                current_node = edge.second; 
                fk_result = call_forward_kinematics(current_node);

                // Extract x y z for each joint
                for (int joint_frame_index = 0; joint_frame_index < 5; ++joint_frame_index) {
                    const auto& frame_pose = fk_result.SE3[joint_frame_index]; 
                    outfile << path_node_index << ","                            
                            << joint_frame_index << ","                         
                            << frame_pose.translation[0] << ","                
                            << frame_pose.translation[1] << ","               
                            << frame_pose.translation[2] << "\n";               
                }
                path_node_index++; 
            }
             std::cout << "Cartesian coordinates calculation complete." << std::endl;

        } catch (const std::runtime_error& e) {
            std::cerr << "Error during Forward Kinematics calculation: " << e.what() << std::endl;
        }
    } else {
        std::cout << "Goal path is empty, skipping Cartesian coordinate calculation." << std::endl;
    }
    outfile << std::endl; 


    outfile.close();
    std::cout << "Data successfully written to " << full_path << std::endl;

    return 0;
}

