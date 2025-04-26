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

bool RRTStar3D::collision_free(const Configuration& c1, const Configuration& c2, double step_size){
	return 0;
}

bool RRTStar3D::check_robot_collision(const std::vector<Configuration>& robot_cartesian_points){
	return 0;
}