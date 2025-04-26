#include "rrt_3d.h" 
#include <vector>
#include <cmath>
#include <random>   
#include <stdexcept> 
#include <limits>    
#include <iostream>

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
	return {};
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

bool RRTStar3D::valid(const Configuration& c){
	return 0;
}

bool RRTStar3D::collision_free(const Configuration& c1, const Configuration& c2, double step_size){
	return 0;
}

bool RRTStar3D::check_robot_collision(const std::vector<Configuration>& robot_cartesian_points){
	return 0;
}