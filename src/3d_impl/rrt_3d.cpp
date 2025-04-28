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


namespace fs = std::filesystem;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- Constructor ---
RRTStar3D::RRTStar3D(int seed, const std::vector<SphereObstacle>& _obstacles, const std::vector<std::pair<double, double>>& _rad_limits)
    : GoalBiasedGreedySteerKNeighborhoodRRTStarBase(seed), 
      obstacles(_obstacles),          
      rad_limits(_rad_limits),     
      dof(5)                         
{
    // Error check correct number of joint limits
    if (rad_limits.size() != dof) {
        throw std::invalid_argument("RRTStar3D Error: Constructor requires exactly " +
                                    std::to_string(dof) + " joint limits. Received " +
                                    std::to_string(rad_limits.size()) + ".");
    }
}


// --- RRT 3D Class Functions ---

Configuration RRTStar3D::sample(double p /* p = goal bias probability */){
	const int max_tries = 25; // avoid infiinte loops
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    for(int attempt = 0; attempt < max_tries; attempt++){
        Configuration q(dof);

        // goal biased
        if(unit(random_generator) < p && !is_goal_reachable()){
            q = goal_coordinates;
        } else {
            for(int ii = 0; ii < dof; ii++){
                double lo = rad_limits[ii].first;
                double hi = rad_limits[ii].second;

                std::uniform_real_distribution<double> jdist(lo,hi);
                q[ii] = jdist(random_generator);
            }
        }

        if(valid(q)) return q; // successful 
    }
    return Configuration(); // signal failure
}

// returns negative if error
double RRTStar3D::distance(const Configuration& c1, const Configuration& c2){
    if((c1.size() != dof) || (c2.size() != dof)){
        std::cerr << "Configurations must equal degrees of freedom";
        return -1;
    }
    
    double sum_sq = 0.0;

    for(int ii = 0; ii < dof; ii++){
        double diff = c1[ii] - c2[ii];
        sum_sq += diff*diff;
    }

    return std::sqrt(sum_sq);
}

Configuration RRTStar3D::steer(const Configuration& c0, const Configuration& c, double step_size){
	// If points are the same or very close just return orginal joint point
    if (allclose(c0, c)) return c0;

    // Find distance and normalize each step to find step size 
    double total_dist = distance(c0, c);
    Configuration step_vec(dof);
    for (int i = 0; i < dof; ++i) {
        double diff = c[i] - c0[i];

        // In case angle wraps around take the shorter one
        double normalized_diff = std::remainder(diff, 2.0 * M_PI);
        step_vec[i] = normalized_diff * (step_size / total_dist);
    }

    Configuration curr_point = c0;
    bool made_step = false;

    // Slowly step towards the target c
    int max_iterations = static_cast<int>(total_dist / step_size) + 2;
    for (int i = 0; i < max_iterations; ++i){
        double dist_left = distance(curr_point, c);
        if (dist_left < 0) break;

        // Check if we can go to c if distance left is small
        if (dist_left <= step_size){
            // If target c is not a vaild point break loop 
            if (!valid(c)) break;

            // Make final step if collision free 
            if (collision_free(curr_point, c, step_size)) {
                curr_point = c;
                made_step = true;
            }
            break;
        }

        // If not find next potential point and try going to that point
        else{
            Configuration next_point(dof);
            for (int j = 0; j < dof; ++j) {
                next_point[j] = curr_point[j] + step_vec[j];
            }

            if (!collision_free(curr_point, next_point, step_size)) break;
            else {
                curr_point = next_point;
                made_step = true;
            }

        }
    }

    // Return closet point to target c or nothing if not able to make any steps
    if (made_step) return curr_point;
    else return Configuration();
}


bool RRTStar3D::allclose(const Configuration& c1, const Configuration& c2){
	if((c1.size() != dof) || (c2.size() != dof)){
        std::cerr << "Configurations must equal degrees of freedom";
        return false;
    }
    
    // two tolerances are more adaptable when working in 6dof
    const double atol = 1e-5; // absolute tolerance
    const double rtol = 1e-3; // relative tolerance

    for(int ii = 0; ii < dof; ii++){
        if(std::fabs(c1[ii] - c2[ii]) > atol + rtol * std::fabs(c2[ii])){
            return false;
        }
    }
    return true;
}

bool RRTStar3D::valid(const Configuration& c) {
    // Check size of vector
    if (c.size() != dof) return false;

    // Check Joint Limits
    for (int i = 0; i < dof; ++i) {
        if (c[i] < rad_limits[i].first || c[i] > rad_limits[i].second) {
            return false;
        }
    }

    // Do FK to change to cartiesain and check if collsion found
    std::vector<Configuration> robot_points = forward_kinematics(c);
    if (robot_points.empty()) {
        std::cout << "ERROR: Forward Kinematics failed or returned no points. valid()\n";
        return false;
    }
    bool collision_detected = check_robot_collision(robot_points);

    for(const Configuration& p : robot_points){
        if(p.size() >= 3 && p[2] < 0.0) return false;
    }

    // Return true if no collision was detected
    return !collision_detected;
}

bool RRTStar3D::collision_free(const Configuration& c1, const Configuration& c2, double step_size /* Context only */) {
    // Check dimensions
    if (c1.size() != dof || c2.size() != dof) return false;

    // If points are super close check if endpoint valid and just return that
    if (allclose(c1, c2)) return valid(c2); 

    // Determine num of steps for checking validity along the path
    double total_dist = distance(c1, c2);
    if (total_dist >= std::numeric_limits<double>::infinity()) return false; 

    // Choose max distance in joint space between checks 
    // TODO: Tune this 
    double check_resolution = 0.1; 
    int num_steps = std::max(1, static_cast<int>(std::ceil(total_dist / check_resolution)));

    // Loop along the joint space path, check if valid 
    Configuration q_interp(dof);
    for (int i = 1; i <= num_steps; ++i) {
        // Calculate interpolation factor (fraction of path covered)
        double t = static_cast<double>(i) / num_steps;

        // Calculate intermediate joint configuration using shortest angle difference 
        for (int j = 0; j < dof; ++j) {
            double diff = c2[j] - c1[j];
            double normalized_diff = std::remainder(diff, 2.0 * M_PI); 
            q_interp[j] = c1[j] + normalized_diff * t;

        }

        // Check if the intermediate configuration is valid and collison free
        if (!valid(q_interp)) return false; 
    }

    // If the loop completes, all intermediate points were valid + collison free
    return true;
}

bool RRTStar3D::check_robot_collision(const std::vector<Configuration>& robot_cartesian_points) {
    const double TABLE_MARGIN = -0.05; // allow some margin for table collision
    const double SELF_COLLISION_RADIUS = 0.01;
    const int NUM_SPHERES_PER_LINK = 3; // number of spheres along each link

    // --- Table Collision ---
    if (!robot_cartesian_points.empty()) {
        const Configuration& end_effector = robot_cartesian_points.back();
        if (end_effector.size() >= 3 && end_effector[2] < TABLE_MARGIN) {
            return true; // end-effector dipped below table
        }
    }
    
    // --- Sphere Collision --- 
    // Check first if joint points are inside sphere obstacle
    for (const auto& point : robot_cartesian_points) {
        // Check dimension
        if (point.size() != 3) {
            std::cout << "WARNING: skipped because incorrect dimensions\n";
            continue;
        }

        // For each point check if inside sphere
        for (const auto& obs : this->obstacles) {
            double dx = point[0] - obs.x;
            double dy = point[1] - obs.y;
            double dz = point[2] - obs.z;
            double dist_sq = dx * dx + dy * dy + dz * dz;
            double radius_sq = obs.radius * obs.radius;
            if (dist_sq < radius_sq) {
                return true; 
            }
        }
    }

    // Check at least 2 points in the vector
    if (robot_cartesian_points.size() < 2){
        std::cout << "WARNING: can't check collsion\n";
        return false;
    }

    // Loop through each line segment to check for collision
    for (size_t i = 0; i < robot_cartesian_points.size() - 1; ++i){
        const Configuration& p1 = robot_cartesian_points[i];
        const Configuration& p2 = robot_cartesian_points[i+1];

        // Check dimesnsions
        if (p1.size() != 3 || p2.size() != 3) {
            std::cout << "WARNING: skipped because incorrect dimensions\n";
            continue;
        }

        // Loop through all obstacles, check intersection 
        for (const auto& obs : this->obstacles){
            if (segment_sphere_intersect(p1, p2, obs)) return true;
        }
    }

    // --- Self Collision --- 
    // models each segment as a series of spheres
    std::vector<Configuration> spheres;

    for(size_t ii = 0; ii + 1 < robot_cartesian_points.size(); ii++){
        const Configuration& p1 = robot_cartesian_points[ii];
        const Configuration& p2 = robot_cartesian_points[ii + 1];

        for(int ss = 0; ss <= NUM_SPHERES_PER_LINK; ss++){
            double alpha = static_cast<double>(ss) / NUM_SPHERES_PER_LINK;
            Configuration sphere_center = {
                (1 - alpha) * p1[0] + alpha * p2[0],
                (1 - alpha) * p1[1] + alpha * p2[1],
                (1 - alpha) * p1[2] + alpha * p2[2]
            };
            spheres.push_back(sphere_center);
        }
    }

    //check collisions between spheres, skip checking adjacent spheres
    for(size_t ii = 0; ii < spheres.size(); ii++){
        for(size_t jj = ii + 5; jj < spheres.size(); jj++){
            double dx = spheres[ii][0] - spheres[jj][0];
            double dy = spheres[ii][1] - spheres[jj][1];
            double dz = spheres[ii][2] - spheres[jj][2];

            double dist_sq = dx*dx + dy*dy + dz*dz;
            double min_allowed_dist = 2 * SELF_COLLISION_RADIUS;

            if(dist_sq < (min_allowed_dist * min_allowed_dist)){
                return true;
            }
        }
    }

    return false;
}

bool RRTStar3D::segment_sphere_intersect(const Configuration& p1, const Configuration& p2, const SphereObstacle& obs) {
    // Check Dimensions
    if (p1.size() != 3 || p2.size() != 3) {
        std::cout << "Segment-Sphere Intersect ERROR: Points must be 3D\n";
        return false; 
    }

    // Vector from sphere center to segment start (W = p1 - C)
    double Wx = p1[0] - obs.x;
    double Wy = p1[1] - obs.y;
    double Wz = p1[2] - obs.z;

    // Vector representing the segment (V = p2 - p1)
    double Vx = p2[0] - p1[0];
    double Vy = p2[1] - p1[1];
    double Vz = p2[2] - p1[2];

    // Calculate coefficients for At^2 + Bt + C = 0
    // || p1 + t*V - C ||^2 = r^2  -> || W + t*V ||^2 = r^2
    // -> (V.V)t^2 + 2(V.W)t + (W.W - r^2) = 0
    double A = (Vx * Vx) + (Vy * Vy) + (Vz * Vz); // A = V.V 
    double B = 2.0 * ((Vx * Wx) + (Vy * Wy) + (Vz * Wz)); // B = 2 * (V.W)
    double C = (Wx * Wx) + (Wy * Wy) + (Wz * Wz) - (obs.radius * obs.radius); // C = W.W - r^2

    // Check if A is close to zero
    if (std::fabs(A) < std::numeric_limits<double>::epsilon()) {
        return C <= 0;
    }

    // Calculate Delta = B^2 - 4AC
    double delta = B * B - 4.0 * A * C;

    // If delta is negative no collision 
    if (delta < 0.0) {
        return false;
    }

    // Calculate the two potential intersection places
    double sqrt_delta = std::sqrt(delta);
    double t1 = (-B + sqrt_delta) / (2.0 * A);
    double t2 = (-B - sqrt_delta) / (2.0 * A);

    // Check if either intersection point lies within the segment bounds [0, 1]
    if ((t1 >= 0.0 && t1 <= 1.0) || (t2 >= 0.0 && t2 <= 1.0)) return true; 


    // Check if segment inside sphere
    if (C <= 0 && (A + 2.0 * B + C <= 0)) {
        return true;
    }

    return false;
}

std::vector<Configuration> RRTStar3D::forward_kinematics(const Configuration& joint_angles) {
    if (joint_angles.size() != dof) {
        throw std::runtime_error("Forward Kinematics Error: Incorrect number of joint angles provided (" + std::to_string(joint_angles.size()) + ", expected " + std::to_string(dof) + ").");
    }

    // Prepare inputs for the specific FK function call 
    double qcos[NU]; 
    double qsin[NU];
    double v[NU] = {0.0}; // zero velocity cuz static 
    double a[NU] = {0.0}; // zero acceleration cuz static 

    // Compute sin + cos
    for(int i = 0; i < dof; ++i) { 
        qcos[i] = std::cos(joint_angles[i]);
        qsin[i] = std::sin(joint_angles[i]);
    }

    // Call the specific FK function 
    auto fk_result = ForwardKinematics(
        qcos[0], qcos[1], qcos[2], qcos[3], qcos[4], 
        qsin[0], qsin[1], qsin[2], qsin[3], qsin[4], 
        v[0], v[1], v[2], v[3], v[4],
        a[0], a[1], a[2], a[3], a[4]
    );

    // initialize varibles to hold results
    std::vector<Configuration> robot_points;
    //int num_fk_frames = fk_result.SE3.size(); 
    int points_to_extract = dof; //std::min(num_fk_frames, dof); 
    robot_points.reserve(points_to_extract); 

    // Extract cartesian points {x, y, z} from the result
    for (int i = 0; i < points_to_extract; ++i) {
         robot_points.push_back({fk_result.SE3[i].translation[0],
                                 fk_result.SE3[i].translation[1],
                                 fk_result.SE3[i].translation[2]});
    }

    // Check to see if points were found
    if (robot_points.empty() && points_to_extract > 0) {
         std::cout << "WARNING: FK result yielded no points, though did have SE3\n";
    }

    // Return results
    return robot_points;
}

// --- Helper function: find serial port ---
std::string find_serial_port(){
    const char* dir_path = "/dev/";
    DIR* dir = opendir(dir_path);

    if(!dir){
        std::cerr << "Error opening /dev/" << std::endl;
        return "";
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr){
        if(strncmp(entry->d_name, "ttyUSB", 6) == 0){
            closedir(dir);
            return std::string("/dev/") + entry->d_name;
        }
    }

    closedir(dir);
    return "";
}

// --- Helper function: open serial oprt ---
int open_serial_port(const char* portname){
    int serial_fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if(serial_fd < 0){
        std::cerr << "Error opening serial port" << std::endl;
        return -1;
    }

    struct termios tty{};

    if(tcgetattr(serial_fd, &tty) != 0){
        std::cerr << "Error getting termios" << std::endl;
        close(serial_fd);
        return -1;
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0; // no signalling chars, no echo, no canonical prcoessing
    tty.c_oflag = 0; // no remapping, no delays
    tty.c_cc[VMIN] = 1; // read blocks
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if(tcsetattr(serial_fd, TCSANOW, &tty) != 0){
        std::cerr << "Error setting termios" << std::endl;
        close(serial_fd);
        return -1;
    }

    return serial_fd;
}

// --- Helper: read one line until \n from serial oprt --- 
std::string read_line_from_serial(int serial_fd){
    std::string result;
    char c;

    while(true){
        ssize_t n = read(serial_fd, &c, 1);
        if(n > 0){
            result += c;
            if(c == '\n') break;
        } else if (n == 0){
            break; // end of file
        } else {
            std::cerr << "error reading serial" << std::endl;
            break;
        }
    }

    return result;
}

// --- Main Application Logic ---
int main() {
    std::cout << "reached main" << std::endl;

    // detect serial port
    std::string serial_port = find_serial_port();
    if(serial_port.empty()){
        std::cerr << "could not find USB device" << std::endl;
        return 1;
    }
    std::cout << "found serial port" << std::endl;

    // --- Serial Setup ---
    int serial_fd = open_serial_port(serial_port.c_str());
    if(serial_fd < 0) return 1;
    usleep(1000000);

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
    auto goal_points = rrt.get_points_to_goal(); 
    if (goal_points.empty()) {
        std::cerr << "No valid path to goal found." << std::endl;
        close(serial_fd);
        return 1;
    }

    // --- Send path to robot point by point ---
    for(const auto& point : goal_points){
        if(point.size() != 5){
            std::cerr << "Invalid joint config size" << std::endl;
            continue;
        }

        char command[256];
        snprintf(command,sizeof(command),
            "{\"T\":102,\"base\":%.6f,\"shoulder\":%.6f,\"elbow\":%.6f,\"wrist\":%.6f,\"roll\":%.6f,\"hand\":%.4f,\"spd\":%d,\"acc\":%d}\n",
            point[0], point[1], point[2], point[3], point[4],
            3.13, 1, 10);
        
        write(serial_fd, command, strlen(command));
        tcdrain(serial_fd); // flush output buffer
        std::cout << "sent: " << command << std::endl;

        // wait for robot arm confirmation
        sleep(10);
    }

    std::cout << "All commands sent" << std::endl;
    close(serial_fd);
    return 0;
}



