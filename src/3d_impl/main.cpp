#include <iostream>
#include <fstream>
#include "rrt_3d.h" 

int main() {
    std::cout << "reached main" << std::endl;

    // --- Configuration ---
    const Configuration c_init = {0.0, 0.0, 0.0, 0.0, 0.0};
    const Configuration c_goal = {0.5, 1.0, 1.0, -0.5, 1.0};
    const double p = 0.5; // Goal bias probability
    const int k = 20;      // Number of neighbors for RRT*
    const double step_size = 0.1;
    const int NUM_NODES = 1000; // Number of iterations/nodes to add
    const int SEED = 42;
    //const double XYZ_MIN = -2.0;
    //const double XYZ_MAX = 2.0;

    std::vector<SphereObstacle> obstacles;
    //obstacles.emplace_back(SphereObstacle{0.073214 , 0.0906615, 0.2217195, 0.0538023});
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
    auto all_edges = rrt.get_all_edges();
    auto goal_path = rrt.get_path_to_goal(); 
    std::vector<std::pair<Configuration, Configuration>> simplified_path;
    if (!goal_path.empty()) {
        simplified_path = rrt.simplify_path(goal_path, step_size);
    }
    double final_goal_cost = rrt.get_goal_cost(); // Returns infinity if no path

    // --- Output File Creation ---
    std::string output_dir = "results";
    std::string filename;

    // Create results directory if it doesn't exist
    struct stat st = {0};
    if (stat(output_dir.c_str(), &st) == -1) {
        mkdir(output_dir.c_str());
        std::cout << "Created directory: " << output_dir << std::endl;
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

    // Set precision for floating point numbers in the output file
    outfile << std::fixed << std::setprecision(6);

    // --- Write Data to CSV ---

    // Parameters
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
        outfile << (j + 1)             << ','   
            << joint_limits[j].first  << ','
            << joint_limits[j].second << '\n';
    }
    outfile << std::endl;

    // Start
    outfile << "# Section: Start\nq1,q2,q3,q4,q5\n";
    for(int ii; ii < c_init.size() -1; ii++){
        outfile << c_init[ii] << ",";
    }
    outfile << c_init[c_init.size()];
    outfile << std::endl;

    // Goal
    outfile << "# Section: Goal\nq1,q2,q3,q4,q5\n";
    for(int ii; ii < c_goal.size() -1; ii++){
        outfile << c_goal[ii] << ",";
    }
    outfile << c_init[c_goal.size()];
    outfile << std::endl;

    // Obstacles
    outfile << "# Section: Obstacles\ncenter_x,center_y,center_z,radius\n";
    for (auto const& o : obstacles)
        outfile << o.x << ',' << o.y << ',' << o.z << ',' << o.radius << '\n';
    outfile << std::endl;

    // All Edges
    outfile << "# Section: All_Edges\n"
           "q1_src,q2_src,q3_src,q4_src,q5_src,"
           "q1_dst,q2_dst,q3_dst,q4_dst,q5_dst\n";
    for (auto const& e : all_edges) {
        for (double q : e.first)  outfile << q << ',';
        for (size_t i = 0; i < e.second.size(); ++i)
            outfile << e.second[i] << (i + 1 == e.second.size() ? '\n' : ',');
    }
    outfile << std::endl;

    // Goal Path
    outfile << "# Section: Goal_Path\n"
           "q1_src,q2_src,q3_src,q4_src,q5_src,"
           "q1_dst,q2_dst,q3_dst,q4_dst,q5_dst\n";
    for (auto const& e : goal_path) {
        for (double q : e.first)  outfile << q << ',';
        for (size_t i = 0; i < e.second.size(); ++i)
            outfile << e.second[i] << (i + 1 == e.second.size() ? '\n' : ',');
    }
    outfile << std::endl;

    // Simplified Path
    outfile << "# Section: Simplified_Path\n"
           "q1_src,q2_src,q3_src,q4_src,q5_src,"
           "q1_dst,q2_dst,q3_dst,q4_dst,q5_dst\n";
    for (auto const& e : simplified_path) {
        for (double q : e.first)  outfile << q << ',';
        for (size_t i = 0; i < e.second.size(); ++i)
            outfile << e.second[i] << (i + 1 == e.second.size() ? '\n' : ',');
    }
    outfile << std::endl;


    outfile.close();
    std::cout << "Data successfully written." << std::endl;

    return 0;
}