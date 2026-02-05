#pragma once

#include "linalg.hpp"

#include <functional>
#include <string>
#include <vector>

// Trajectory point at time t
struct TrajectoryPoint {
    Vec3 position;
    Vec3 velocity;  // Desired velocity (feedforward)
};

// Function type for trajectory generators
// Takes time t, returns desired position and velocity
using TrajectoryFunc = std::function<TrajectoryPoint(double t)>;

// Trajectory generators
namespace trajectories {

// Hover at a fixed position
TrajectoryFunc hover(const Vec3& pos);

// Circular trajectory in the XZ plane (vertical circle)
// center: circle center
// radius: circle radius
// omega: angular velocity (rad/s)
TrajectoryFunc circle(const Vec3& center, double radius, double omega);

// Linear interpolation between waypoints
// pts: waypoint positions
// times: arrival times at each waypoint (must be sorted, same length as pts)
// Returns hover at final point after last time
TrajectoryFunc waypoints(const std::vector<Vec3>& pts, const std::vector<double>& times);

// Parse trajectory from config string
// Formats:
//   "hover 0.0 0.0 1.0"
//   "circle 0.0 0.0 1.0 0.5 1.0"  (center_x, center_y, center_z, radius, omega)
//   "waypoints 0,0,0@0.0 1,0,1@1.0 0,0,1@2.0"  (x,y,z@time ...)
TrajectoryFunc parse(const std::string& spec);

} // namespace trajectories
