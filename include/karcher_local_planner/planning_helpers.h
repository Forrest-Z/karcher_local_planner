#ifndef PLANNING_HELPERS_H_
#define PLANNING_HELPERS_H_

#include <vector>
#include <math.h>
#include <limits>

#define _USE_MATH_DEFINES
#define RAD2DEG 180.0 / M_PI
#define distance2points(from , to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2pointsSqr(from , to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x*v.x + v.y*v.y)

namespace local_planner {

typedef struct  
{
    double x;
    double y;
    double heading;
    double left_width;
    double right_width;
    double cost;
    double speed;
} Waypoint;

typedef struct  
{
    double x;
    double y;
    double yaw;
    double speed;
} VehicleState;

class PlanningHelpers
{
public:
    PlanningHelpers();
    virtual ~PlanningHelpers();

    // Local Planning functions
    static void extractGlobalPathSection(std::vector<Waypoint>& extracted_path);
    static int getClosestNextWaypointIndex(const std::vector<Waypoint>& path);
    static double angleBetweenTwoAnglesPositive(const double& a1, const double& a2);
    static double fixNegativeAngle(const double& a);
    static void fixPathDensity(std::vector<Waypoint>& path);
    static void smoothPath(std::vector<Waypoint>& path);
    static double calculateAngleAndCost(std::vector<Waypoint>& path);
    static void generateRollOuts(const std::vector<Waypoint>& path, std::vector<std::vector<Waypoint>>& roll_outs);
    static bool getRelativeInfo(const std::vector<Waypoint>& trajectory, int& back_index, double& perp_distance);
}; 

} // namespace local_planner

#endif // PLANNING_HELPERS_H