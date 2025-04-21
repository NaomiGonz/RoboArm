#include "rrt_2d.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <iomanip> 
#include <sys/stat.h> 
#include <filesystem> 

// --- Constructor ---
RRTStar2D::RRTStar2D(int seed, const std::vector<CircleObstacle>& _obstacles, double _xy_min, double _xy_max)
    : GoalBiasedGreedySteerKNeighborhoodRRTStarBase(seed), 
      obstacles(_obstacles),
      xy_min(_xy_min),
      xy_max(_xy_max),
      dof(2)
{ }

// --- RRT 2D Class Functions ---

double RRTStar2D::distance(const Configuration& c1, const Configuration& c2) {
    return std::sqrt(std::pow(c1[0] - c2[0], 2) + std::pow(c1[1] - c2[1], 2));
}

Configuration RRTStar2D::steer(const Configuration& c0, const Configuration& c, double step_size) {
    // If points are the same or very close just return orginal point
    if (allclose(c0, c)) return c0;

    // Find distance and normalize each step to find step size 
    double total_dist = distance(c0, c);
    double dir_x = c[0] - c0[0];
    double dir_y = c[1] - c0[1];
    double step_dx = dir_x * (step_size / total_dist);
    double step_dy = dir_y * (step_size / total_dist);

    Configuration curr_point = c0;
    bool made_step = false;

    // Slowly step towards the target c
    int max_iterations = (int)(total_dist / step_size) + 2; // 2 is an added buffer
    for (int i = 0; i < max_iterations; ++i){
        double dist_left = distance(curr_point, c);

        // Check if we can go to point c if distance left is small
        if (dist_left <= step_size){
            // If target c is not a vaild point break loop 
            if (!valid(c)) break;

            // Make final step if collision free 
            if (collision_free(curr_point, c, step_size)){
                curr_point = c;
                made_step = true;
                break;
            }
        }

        // If not find next potential point and try going to that point
        else {
            Configuration next_point = {curr_point[0] + step_dx, curr_point[1] + step_dy};
            if (!collision_free(curr_point, next_point, step_size)) break;
            else {
                curr_point = next_point;
                made_step = true;
            }
        }
    }

    // Return closet point to target c or nothing if not able to make any steps
    if (made_step) return curr_point;
    else return {};
}


bool RRTStar2D::allclose(const Configuration& c1, const Configuration& c2) {
    // Based on numpy : https://numpy.org/doc/stable/reference/generated/numpy.allclose.html
    return (distance(c1, c2) < 1e-5);
}

Configuration RRTStar2D::sample(double p) {
    std::uniform_real_distribution<double> probability_dist(0.0, 1.0);

    // If goal not reachable or probability less then p
    if (!is_goal_reachable() && probability_dist(random_generator) < p){
        return goal_coordinates;
    }

    // Sample a random point 
    std::uniform_real_distribution<double> coordinate_dist(xy_min, xy_max);
    double rand_x = coordinate_dist(random_generator);
    double rand_y = coordinate_dist(random_generator);

    return {rand_x, rand_y};
}

bool RRTStar2D::valid(const Configuration& c) {
    // Check size of vector
    if (c.size() != dof) return false;

    // Check if point is valid
    for (const CircleObstacle& obs : obstacles){
        if (std::sqrt(std::pow(c[0]-obs.x, 2) + std::pow(c[1]-obs.y, 2)) < obs.radius){
            return false;
        }
    }
    return true; 
}

// Helper for collision_free
bool RRTStar2D::intersect(double Ax, double Ay, double Bx, double By, const CircleObstacle& obs) {
    // line segment P(t) = start + t * vec (t is within [0,1])
    // A = vec . vec
    // B = vec . (start - center)  
    // C = (start - center) . (start - center) - radius^2

    // vec = 
    double Px = Ax - obs.x; 
    double Py = Ay - obs.y;


    double A = (Bx * Bx) + (By * By);
    double B = (Bx * Px) + (By * Py);
    double C = (Px * Px) + (Py * Py) - ((obs.radius + 0.05) * (obs.radius + 0.05);

    // Check if A is close to zero
    if (std::fabs(A) < std::numeric_limits<double>::epsilon()){
        return (C <= 0);
    }

    double delta = B * B - A * C;

    // Does not intersect if delta is negative
    if (delta < 0) return false;
    
    // Calculate the two potential intersection places
    double sqrt_delta = std::sqrt(delta);
    double t1 = (-B + sqrt_delta) / A;
    double t2 = (-B - sqrt_delta) / A;

    // Check if either intersection point lies within the segment bounds [0, 1]
    bool t1_in_range = (t1 >= 0.0 && t1 <= 1.0);
    bool t2_in_range = (t2 >= 0.0 && t2 <= 1.0);
    if (t1_in_range || t2_in_range) return true;

    // Check if both points are inside an obstacle  
    if (C <= 0 && (A + 2.0 * B + C <= 0)) return true;

    return false;
}


bool RRTStar2D::collision_free(const Configuration& c1, const Configuration& c2, double step_size) {
    // Error check coordinate length
    if (c1.size() != dof || c2.size() != dof) return false;

    // If points are super close check if endpoint valid and just return that
    double epsilon = 1e-6;
    if (distance(c1, c2) < epsilon) return valid(c2);

    double vec_x = c2[0] - c1[0];
    double vec_y = c2[1] - c1[1];

    // Check it doesn't collide with every obstacle
    for (const CircleObstacle& obs : obstacles){
        if (intersect(c1[0], c1[1], vec_x, vec_y, obs)) return false;
    }

    return true;
}


// --- Main Application Logic ---

int main() {
    // --- Configuration ---
    const Configuration c_init = {0.0, 0.0};
    const Configuration c_goal = {0.0, -1.5};
    const double p = 0.5;  // Goal bias probability
    const int k = 20;      // Number of neighbors for RRT*
    const double step_size = 0.1;
    const int NUM_NODES = 1000; // Number of iterations/nodes to add
    const int SEED = 42;
    const double XY_MIN = -2.0;
    const double XY_MAX = 2.0;

    const std::vector<CircleObstacle> obstacles = {
        {-1.0, 0.5, 0.5}, {1.0, 1.0, 0.5}, {-0.8, -0.8, 1.0},
        {1.0, 0.0, 0.6}, {1.0, -1.0, 1.0}
    };

    // --- RRT Initialization ---
    RRTStar2D rrt(SEED, obstacles, XY_MIN, XY_MAX);
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
    try {
         if (!std::filesystem::exists(output_dir)) {
            std::filesystem::create_directory(output_dir);
            std::cout << "Created directory: " << output_dir << std::endl;
         }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error creating directory " << output_dir << ": " << e.what() << std::endl;
        return 1; 
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
    outfile << "xy_min," << XY_MIN << std::endl;
    outfile << "xy_max," << XY_MAX << std::endl;
    outfile << std::endl;

    // Start
    outfile << "# Section: Start" << std::endl;
    outfile << "x,y" << std::endl;
    outfile << c_init[0] << "," << c_init[1] << std::endl;
    outfile << std::endl;

    // Goal
    outfile << "# Section: Goal" << std::endl;
    outfile << "x,y" << std::endl;
    outfile << c_goal[0] << "," << c_goal[1] << std::endl;
    outfile << std::endl;

    // Obstacles
    outfile << "# Section: Obstacles" << std::endl;
    outfile << "center_x,center_y,radius" << std::endl;
    for(const auto& obs : obstacles) {
        outfile << obs.x << "," << obs.y << "," << obs.radius << std::endl;
    }
    outfile << std::endl;

    // All Edges
    outfile << "# Section: All_Edges" << std::endl;
    outfile << "x1,y1,x2,y2" << std::endl;
    for(const auto& edge : all_edges) {
        outfile << edge.first[0] << "," << edge.first[1] << ","
                << edge.second[0] << "," << edge.second[1] << std::endl;
    }
    outfile << std::endl;

    // Goal Path
    outfile << "# Section: Goal_Path" << std::endl;
    outfile << "x1,y1,x2,y2" << std::endl;
    for(const auto& edge : goal_path) { // Will be empty if no path found
        outfile << edge.first[0] << "," << edge.first[1] << ","
                << edge.second[0] << "," << edge.second[1] << std::endl;
    }
    outfile << std::endl;

    // Simplified Path
    outfile << "# Section: Simplified_Path" << std::endl;
    outfile << "x1,y1,x2,y2" << std::endl;
     for(const auto& edge : simplified_path) { // Will be empty if no path found
        outfile << edge.first[0] << "," << edge.first[1] << ","
                << edge.second[0] << "," << edge.second[1] << std::endl;
    }
    outfile << std::endl;


    outfile.close();
    std::cout << "Data successfully written." << std::endl;

    return 0;
}