#ifndef RRT_STAR_2D_H
#define RRT_STAR_2D_H

#include "rrt_base.h" 
#include <vector>
#include <cmath> 

// Structure for circle obstacles
struct CircleObstacle {
    double x, y, radius;
};

class RRTStar2D : public GoalBiasedGreedySteerKNeighborhoodRRTStarBase {
private:
    std::vector<CircleObstacle> obstacles;
    double xy_min, xy_max;
    int dof; // Degrees of freedom 

    bool intersect(double Ax, double Ay, double Bx, double By, const CircleObstacle& obs);

public:
    RRTStar2D(int seed,
              const std::vector<CircleObstacle>& _obstacles,
              double _xy_min,
              double _xy_max);

    // --- Implementations of Virtual Functions from Base Class ---

    double distance(const Configuration& c1, const Configuration& c2) override;

    Configuration steer(const Configuration& c0, const Configuration& c, double step_size) override;

    bool allclose(const Configuration& c1, const Configuration& c2) override;

    Configuration sample(double p) override;

    bool valid(const Configuration& c) override;

    bool collision_free(const Configuration& c1, const Configuration& c2, double step_size) override;
};

#endif