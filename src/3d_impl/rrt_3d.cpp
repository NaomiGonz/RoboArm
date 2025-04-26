#include "rrt_3d.h" 
#include <vector>
#include <cmath>
#include <random>   
#include <stdexcept> 
#include <limits>    
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

RRTStar3D::RRTStar3D(int seed, const std::vector<SphereObstacle>& _obstacles, const std::vector<std::pair<double, double>> _rad_limits)
    : GoalBiasedGreedySteerKNeighborhoodRRTStarBase(seed), 
      obstacles(_obstacles),          
      rad_limits(_rad_limits),     
      dof(6)                         
{
    // Error check correct number of joint limits
    if (rad_limits.size() != dof) {
        throw std::invalid_argument("RRTStar3D Error: Constructor requires exactly " +
                                    std::to_string(dof) + " joint limits. Received " +
                                    std::to_string(rad_limits.size()) + ".");
    }
}

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


