#include "trajectory.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace trajectories {

TrajectoryFunc hover(const Vec3& pos) {
    return [pos](double /*t*/) -> TrajectoryPoint {
        return {pos, Vec3::Zero()};
    };
}

TrajectoryFunc circle(const Vec3& center, double radius, double omega) {
    return [center, radius, omega](double t) -> TrajectoryPoint {
        double theta = omega * t;
        double c = std::cos(theta);
        double s = std::sin(theta);

        // Circle in XZ plane (vertical circle, constant Y)
        Vec3 position(
            center.x() + radius * c,
            center.y(),
            center.z() + radius * s
        );

        // Velocity is tangent to circle in XZ plane
        Vec3 velocity(
            -radius * omega * s,
            0.0,
            radius * omega * c
        );

        return {position, velocity};
    };
}

TrajectoryFunc waypoints(const std::vector<Vec3>& pts, const std::vector<double>& times) {
    if (pts.empty()) {
        throw std::invalid_argument("waypoints: empty point list");
    }
    if (pts.size() != times.size()) {
        throw std::invalid_argument("waypoints: pts and times must have same length");
    }
    for (size_t i = 0; i + 1 < times.size(); ++i) {
        if (times[i + 1] <= times[i]) {
            throw std::invalid_argument("waypoints: times must be strictly increasing");
        }
    }

    // Capture by value for lambda
    return [pts, times](double t) -> TrajectoryPoint {
        // Before first waypoint
        if (t <= times[0]) {
            return {pts[0], Vec3::Zero()};
        }

        // After last waypoint - hover at final position
        if (t >= times.back()) {
            return {pts.back(), Vec3::Zero()};
        }

        // Find segment containing t
        size_t i = 0;
        while (i + 1 < times.size() && t > times[i + 1]) {
            ++i;
        }

        // Linear interpolation between pts[i] and pts[i+1]
        double t0 = times[i];
        double t1 = times[i + 1];
        double alpha = (t - t0) / (t1 - t0);

        Vec3 position = (1.0 - alpha) * pts[i] + alpha * pts[i + 1];
        Vec3 velocity = (pts[i + 1] - pts[i]) / (t1 - t0);

        return {position, velocity};
    };
}

namespace {

// Helper to parse a Vec3 from "x,y,z" format
Vec3 parseVec3(const std::string& s) {
    std::istringstream iss(s);
    double x, y, z;
    char comma1, comma2;
    if (!(iss >> x >> comma1 >> y >> comma2 >> z) || comma1 != ',' || comma2 != ',') {
        throw std::invalid_argument("Invalid Vec3 format: " + s);
    }
    return Vec3(x, y, z);
}

} // namespace

TrajectoryFunc parse(const std::string& spec) {
    std::istringstream iss(spec);
    std::string type;
    iss >> type;

    if (type == "hover") {
        double x, y, z;
        if (!(iss >> x >> y >> z)) {
            throw std::invalid_argument("hover requires: x y z");
        }
        return hover(Vec3(x, y, z));
    }

    if (type == "circle") {
        double cx, cy, cz, radius, omega;
        if (!(iss >> cx >> cy >> cz >> radius >> omega)) {
            throw std::invalid_argument("circle requires: center_x center_y center_z radius omega");
        }
        return circle(Vec3(cx, cy, cz), radius, omega);
    }

    if (type == "waypoints") {
        std::vector<Vec3> pts;
        std::vector<double> times;

        std::string token;
        while (iss >> token) {
            // Format: x,y,z@time
            size_t at = token.find('@');
            if (at == std::string::npos) {
                throw std::invalid_argument("waypoint format: x,y,z@time, got: " + token);
            }
            Vec3 pt = parseVec3(token.substr(0, at));
            double t = std::stod(token.substr(at + 1));
            pts.push_back(pt);
            times.push_back(t);
        }

        if (pts.empty()) {
            throw std::invalid_argument("waypoints requires at least one point");
        }

        return waypoints(pts, times);
    }

    throw std::invalid_argument("Unknown trajectory type: " + type);
}

} // namespace trajectories
