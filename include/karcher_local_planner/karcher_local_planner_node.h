#ifndef KARCHER_LOCAL_PLANNER_NODE_H_
#define KARCHER_LOCAL_PLANNER_NODE_H_

// namespace local_planner {

typedef struct  
{
    double x;
    double y;
    double heading;
    double left_width;
    double right_width;
    double speed;
    double cost;
    double time_cost;
} Waypoint;

// } // namespace local_planner

#endif  // KARCHER_LOCAL_PLANNER_NODE_H_