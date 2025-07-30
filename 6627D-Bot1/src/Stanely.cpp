#include "main.h"
#include "pid.h"
#include <algorithm>
#include <cmath>
#include <vector>

constexpr double kGain = 500;
constexpr double wheelTrack = 0.30; //copy from arc turns for 
constexpr double cg2axle = wheelTrack / 2.0;
constexpr double maxSpeed = 1200; 
constexpr double eps = 1e-3;

struct Pose {
    double x;
    double y; 
    double theta; 
};

struct waypoint {double x, y;};

Pose getPose() {
    static Pose p{0,0,0};

    return p;
}

double getForwardVelocity() {
return 0.5;
}

double normAngle(double a) {

    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

std::pair<Pose, size_t> nearestPoint(const Pose& p, const std::vector<waypoint>& path) {
    double bestDist2 = 1e9;
    Pose bestPt;
    size_t idx = 0;
    for(size_t i=0; i+1<path.size();++i){
        auto A = path[i]; auto B = path[i+1];
        double vx = B.x - A.x , vy  = B.y - A.y;
        double wx = p.x - A.x, wy = p.y - A.y;
        double c1 = vx*wx + vy*wy;
        double c2 = vx*vx + vy*vy;
        double t = std::clamp(c1/c2, 0.0, 1.0);
        Pose proj{A.x + t*vx , A.y + t*vy, 0};
        double d2 = std::pow(p.x-proj.x,2) +std::pow(p.y-proj.y, 2);
        if (d2 < bestDist2) { bestDist2 = d2; bestPt = proj; idx = i; }
    }

    double dx = path [idx+1].x - path[idx].x;
    double dy = path [idx+1].y - path[idx].y;
    bestPt.theta = std::atan2(dy,dx);
    return {bestPt, idx};
}

std::pair<double, double> stanelyStep(const Pose& robot, const std::vector<waypoint>& path) {
    Pose target; size_t seg;
    std::tie(target, seg) = nearestPoint(robot, path);

    double headingVecX = std::cos(target.theta);
    double headingVecY = std::sin(target.theta);
    double dx = robot.x - target.x;
    double dy = robot.y - target.y;
    double cross = headingVecX*dy - headingVecY*dx;
    double ec = cross;

    double theta_e = normAngle(target.theta - robot.theta);

    double v = std::clamp(getForwardVelocity(), 0.05, maxSpeed);
    double delta = theta_e + std::atan2(kGain * ec, v + eps);

    double kappa = std::tan (delta) / cg2axle;
    double vLeft = v * (1.0 - 0.5 * kappa * wheelTrack);
    double vRight = v * (1.0 + 0.5 * kappa * wheelTrack);

    double maxW = std::max(std::fabs(vLeft),  std::fabs(vRight));
    
if (maxW > maxSpeed) {
    vLeft  *= maxSpeed / maxW;
    vRight *= maxSpeed / maxW;
  }
  return {vLeft, vRight};
}

// ****  TASK LOOP  ****
void stanleyTask(void*) {
  // Example 8‑ft straight then 90° turn (in meters)
  std::vector<waypoint> path = {
    {0.0, 0.0}, {2.44, 0.0}, {2.44, 1.22}
  };

  while (true) {
    Pose pose = getPose();
    auto [vl, vr] = stanelyStep(pose, path);

    // Convert m/s to rpm  (rpm = (v / (2πR)) * 60)
    constexpr double wheelR = 0.0508;  // 2‑inch radius (4‑inch dia) in meters
    double rpmLeft  = vl / (2*M_PI*wheelR) * 60.0;
    double rpmRight = vr / (2*M_PI*wheelR) * 60.0;
    chasMove(rpmLeft, rpmLeft, rpmLeft, rpmRight, rpmRight, rpmRight);
    //use chasMove and make the motors move at rpm left and rpm right

    pros::delay(10); // 10 ms
  }
}
