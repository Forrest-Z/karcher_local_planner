#include "karcher_local_planner/planning_helpers.h"

namespace local_planner
{

PlanningHelpers::PlanningHelpers()
{
}

PlanningHelpers::~PlanningHelpers()
{
}

void PlanningHelpers::extractGlobalPathSection(std::vector<Waypoint>& extracted_path)
{
    if(global_path.size() < 2) return;

    extracted_path.clear();

    int closest_index = getClosestNextWaypointIndex(global_path);

    if(closest_index + 1 >= global_path.size())
        closest_index = global_path.size() - 2;
    
    // prev_closest_index = closest_index;
    // prev_cost = global_path[prev_closest_index].cost;

    double d = 0;

    // // include points before the closest next point
    // for(int i = closest_index; i >= 0; i--)
    // {
    //     extracted_path.insert(extracted_path.begin(), global_path[i]);

    //     if(i < global_path.size())
    //         d += hypot(global_path[i].x - global_path[i+1].x, global_path[i].y - global_path[i+1].y);
    //     if(d > 10) 
    //         break;
    // }

    // d = 0;
    for(int i = closest_index; i < (int)global_path.size(); i++)
    {
        extracted_path.push_back(global_path[i]);
        if(i > 0)
            d += hypot(global_path[i].x - global_path[i-1].x, global_path[i].y - global_path[i-1].y);
        if(d > MAX_LOCAL_PLAN_DISTANCE_)
            break;
    }

    if(extracted_path.size() < 2)
    {
        ROS_WARN("Karcher Local Planner Node: Extracted Global Plan is too small, Size = %d", (int)extracted_path.size());
        return;
    }

    fixPathDensity(extracted_path);
    smoothPath(extracted_path);
    calculateAngleAndCost(extracted_path);
    visualiseExtractedPath(extracted_path);
}

int PlanningHelpers::getClosestNextWaypointIndex(const std::vector<Waypoint>& path)
{   
    double d = 0, min_d = DBL_MAX;
    int min_index = prev_closest_index;

    for(int i = prev_closest_index; i < path.size(); i++)
    {
        d = distance2pointsSqr(path[i], current_state);
        double angle_diff = angleBetweenTwoAnglesPositive(path[i].heading, current_state.yaw)*RAD2DEG;

        if(d < min_d && angle_diff < 45)
        {
            min_index = i;
            min_d = d;
        }
    }

    // ROS_INFO("Closest Global Waypoint Index = %d", min_index);

    // if(min_index < int(global_path.size()-2))
    // {
    //     Waypoint curr, next;
    //     curr.x = global_path[min_index].x;
    //     curr.y = global_path[min_index].y;
    //     next.x = global_path[min_index+1].x;
    //     next.y = global_path[min_index+1].y;
    //     double norm_curr = pointNorm(curr);
    //     double norm_next = pointNorm(next);
    //     double dot_pro = curr.x*next.x + curr.y*next.y;
    //     double a = fixNegativeAngle(acos(dot_pro/(norm_curr*norm_next)));
    //     if(a <= M_PI_2)
    //         min_index = min_index + 1;
    // }
    return min_index;
}

double PlanningHelpers::angleBetweenTwoAnglesPositive(const double& a1, const double& a2)
{
    double diff = a1 - a2;
    if(diff < 0)
        diff = a2 - a1;

    if(diff > M_PI)
        diff = 2.0 * M_PI - diff;

    return diff;
}

double PlanningHelpers::fixNegativeAngle(const double& a)
{
    double angle = 0;
    if (a < -2.0 * M_PI || a >= 2.0 * M_PI) 
    {
        angle = fmod(a, 2.0 * M_PI);
    } 
    else 
    {
        angle = a;
    }

    if(angle < 0) 
    {
        angle = 2.0 * M_PI + angle;
    }

    return angle;
}

void PlanningHelpers::fixPathDensity(std::vector<Waypoint> &path)
{
    if(path.size() == 0 || PATH_DENSITY_ == 0) return;

    double d = 0, a = 0;
    double margin = PATH_DENSITY_ * 0.01;
    double remaining = 0;
    int num_points = 0;
    std::vector<Waypoint> fixed_path;
    fixed_path.push_back(path[0]);
    for(int start_index = 0, end_index = 1; end_index < path.size(); )
    {
        d += hypot(path[end_index].x - path[end_index-1].x, path[end_index].y - path[end_index-1].y) + remaining;
        a = atan2(path[end_index].y - path[start_index].y, path[end_index].x - path[start_index].x);

        if(d < PATH_DENSITY_ - margin ) // downsample
        {
            end_index++;
            remaining = 0;
        }
        else if(d > (PATH_DENSITY_ +  margin)) // upsample
        {
            Waypoint new_wp = path[start_index];
            num_points = d / PATH_DENSITY_;
            for(int k = 0; k < num_points; k++)
            {
                new_wp.x = new_wp.x + PATH_DENSITY_ * cos(a);
                new_wp.y = new_wp.y + PATH_DENSITY_ * sin(a);
                fixed_path.push_back(new_wp);
            }
            remaining = d - num_points * PATH_DENSITY_;
            start_index++;
            path[start_index] = new_wp;
            d = 0;
            end_index++;
        }
        else
        {
            d = 0;
            remaining = 0;
            fixed_path.push_back(path[end_index]);
            end_index++;
            start_index = end_index - 1;
        }
    }

    path = fixed_path;
}

void PlanningHelpers::smoothPath(std::vector<Waypoint> &path)
{
  if (path.size() <= 2 ) return;

  std::vector<Waypoint> smoothed_path = path;

  double change = SMOOTH_TOLERANCE_;
  double xtemp, ytemp;
  int nIterations = 0;

  while (change >= SMOOTH_TOLERANCE_)
  {
    change = 0.0;
    for (int i = 1; i < path.size()-1; i++)
    {
        xtemp = smoothed_path[i].x;
        ytemp = smoothed_path[i].y;

        smoothed_path[i].x += SMOOTH_DATA_WEIGHT_ * (path[i].x - smoothed_path[i].x);
        smoothed_path[i].y += SMOOTH_DATA_WEIGHT_ * (path[i].y - smoothed_path[i].y);

        smoothed_path[i].x += SMOOTH_WEIGHT_ * (smoothed_path[i-1].x + smoothed_path[i+1].x - (2.0 * smoothed_path[i].x));
        smoothed_path[i].y += SMOOTH_WEIGHT_ * (smoothed_path[i-1].y + smoothed_path[i+1].y - (2.0 * smoothed_path[i].y));

        change += fabs(xtemp - smoothed_path[i].x);
        change += fabs(ytemp - smoothed_path[i].y);
    }
    nIterations++;
  }

//   ROS_INFO("Number of iterations: %d", nIterations);

  path = smoothed_path;
}

double PlanningHelpers::calculateAngleAndCost(std::vector<Waypoint> &path)
{
    if(path.size() < 2) return 0;

    if(path.size() == 2)
    {
        // path[0].heading = fixNegativeAngle(atan2(path[1].y - path[0].y, path[1].x - path[0].x));
        path[0].heading = atan2(path[1].y - path[0].y, path[1].x - path[0].x);
        path[0].cost = prev_cost;
        path[1].heading = path[0].heading;
        path[1].cost = path[0].cost + distance2points(path[0], path[1]);
        return path[1].cost;
    }

    // path[0].heading = fixNegativeAngle(atan2(path[1].y - path[0].y, path[1].x - path[0].x));
    path[0].heading = atan2(path[1].y - path[0].y, path[1].x - path[0].x);
    path[0].cost = prev_cost;

    for(int j = 1; j < path.size()-1; j++)
    {
        // path[j].heading = fixNegativeAngle(atan2(path[j+1].y - path[j].y, path[j+1].x - path[j].x));
        path[j].heading = atan2(path[j+1].y - path[j].y, path[j+1].x - path[j].x);
        path[j].cost = path[j-1].cost +  distance2points(path[j-1], path[j]);
    }

    int j = (int)path.size()-1;

    path[j].heading = path[j-1].heading;
    path[j].cost = path[j-1].cost + distance2points(path[j-1], path[j]);

    for(int j = 0; j < path.size()-1; j++)
    {
        if(path[j].x == path[j+1].x && path[j].y == path[j+1].y)
            path[j].heading = path[j+1].heading;
    }

    return path[j].cost;
}

void PlanningHelpers::generateRollOuts(const std::vector<Waypoint>& path, std::vector<std::vector<Waypoint>>& roll_outs)
{
    std::cout << "path size: " << path.size() << std::endl;
    if(path.size() == 0) return;
    if(MAX_LOCAL_PLAN_DISTANCE_ <= 0) return;
    roll_outs.clear();

    int i_limit_index = (SAMPLING_TIP_MARGIN_/0.3)/PATH_DENSITY_;
    if(i_limit_index >= path.size())
        i_limit_index = path.size() - 1;
    std::cout << "i_limit_index: " << i_limit_index << std::endl;

    int closest_index;
    double initial_roll_in_distance;
    
    getRelativeInfo(path, closest_index, initial_roll_in_distance);
    std::cout << "closest_index: " << closest_index << std::endl;
    std::cout << "initial_roll_in_distance: " << initial_roll_in_distance << std::endl;

    double remaining_distance = 0;
    for(int i = 0; i < path.size()-1; i++)
    {
        remaining_distance += distance2points(path[i], path[i+1]);
    }
    std::cout << "remaining_distance: " << remaining_distance << std::endl;

    // calculate the starting index
    double d_limit = 0;
    int start_index = 0; 
    int end_index = 0;
    // int far_index = closest_index;

    // calculate end index
    double start_distance = ROLL_IN_SPEED_FACTOR_ * current_state.speed + ROLL_IN_MARGIN_;
    if(start_distance > remaining_distance)
        start_distance = remaining_distance;
    std::cout << "start_distance: " << start_distance << std::endl;

    d_limit = 0;
    for(int i = 0; i < path.size()-1; i++)
    {
        d_limit += distance2points(path[i], path[i+1]);

        if(d_limit >= start_distance)
        {
            end_index = i;
            break;
        }
    }
    // std::cout << "far_index: " << far_index << std::endl;

    int central_trajectory_index = ROLL_OUTS_NUMBER_/2;
    std::vector<double> end_laterals;
    for(int i = 0; i< ROLL_OUTS_NUMBER_+1; i++)
    {
        double end_roll_in_distance = ROLL_OUT_DENSITY_ * (i - central_trajectory_index);
        end_laterals.push_back(end_roll_in_distance);
        // std::cout << "roll out num: " << i << ", end_roll_in_distance: " << end_roll_in_distance << std::endl;
    }

    // calculate the actual calculation starting index
    d_limit = 0;
    int smoothing_start_index = start_index;
    int smoothing_end_index = end_index;

    for(int i = smoothing_start_index; i < path.size(); i++)
    {
        if(i > 0)
            d_limit += distance2points(path[i], path[i-1]);
        if(d_limit > SAMPLING_TIP_MARGIN_)
            break;

        smoothing_start_index++;
    }

    d_limit = 0;
    for(int i = end_index; i < path.size(); i++)
    {
        if(i > 0)
            d_limit += distance2points(path[i], path[i-1]);
        if(d_limit > SAMPLING_TIP_MARGIN_)
            break;

        smoothing_end_index++;
    }
    std::cout << "start_index: " << start_index << ", end_index: " << end_index << ", smoothing_start_index: " 
                << smoothing_start_index << ", smoothing_end_index: " << smoothing_end_index << std::endl;

    int nSteps = end_index - smoothing_start_index;
    std::cout << "nSteps: " << nSteps << std::endl;

    std::vector<double> inc_list;
    std::vector<double> inc_list_inc;
    for(int i = 0; i< ROLL_OUTS_NUMBER_+1; i++)
    {
        double diff = end_laterals[i] - initial_roll_in_distance;
        // std::cout << "diff: " << diff << std::endl;
        inc_list.push_back(diff/(double)nSteps);
        roll_outs.push_back(std::vector<Waypoint>());
        inc_list_inc.push_back(0);
    }

    std::vector<std::vector<Waypoint>> excluded_from_smoothing;
    for(int i = 0; i < ROLL_OUTS_NUMBER_+1; i++)
        excluded_from_smoothing.push_back(std::vector<Waypoint>());

    Waypoint wp;
    // Insert first straight points within the tip of the car range
    for(int j = start_index; j < smoothing_start_index; j++)
    {
        wp = path[j];
        double original_speed = wp.speed;
        for(int i = 0; i < ROLL_OUTS_NUMBER_+1; i++)
        {
            wp.x = path[j].x - initial_roll_in_distance * cos(wp.heading + M_PI_2);
            wp.y = path[j].y - initial_roll_in_distance * sin(wp.heading + M_PI_2);
            if(i != central_trajectory_index)
                wp.speed = original_speed * LANE_CHANGE_SPEED_FACTOR_;
            else
                wp.speed = original_speed;

            if(j < i_limit_index)
                excluded_from_smoothing[i].push_back(wp);
            else
                roll_outs[i].push_back(wp);
        }
    }

    for(int j = smoothing_start_index; j < end_index; j++)
    {
        wp = path[j];
        double original_speed = wp.speed;
        for(int i = 0; i < ROLL_OUTS_NUMBER_+1; i++)
        {
            inc_list_inc[i] += inc_list[i];
            double d = inc_list_inc[i];
            wp.x = path[j].x - initial_roll_in_distance * cos(wp.heading + M_PI_2) - d * cos(wp.heading + M_PI_2);
            wp.y = path[j].y - initial_roll_in_distance * sin(wp.heading + M_PI_2) - d * sin(wp.heading + M_PI_2);

            if(i != central_trajectory_index)
                wp.speed = original_speed * LANE_CHANGE_SPEED_FACTOR_;
            else
                wp.speed = original_speed;

            roll_outs[i].push_back(wp);
        }
    }

    // Insert last straight points to make better smoothing
    for(int j = end_index; j < smoothing_end_index; j++)
    {
        wp = path[j];
        double original_speed = wp.speed;
        for(int i = 0; i < ROLL_OUTS_NUMBER_+1; i++)
        {
            double d = end_laterals[i];
            wp.x = path[j].x - d * cos(wp.heading + M_PI_2);
            wp.y = path[j].y - d * sin(wp.heading + M_PI_2);
            if(i != central_trajectory_index)
                wp.speed = original_speed * LANE_CHANGE_SPEED_FACTOR_;
            else
                wp.speed = original_speed;
            roll_outs[i].push_back(wp);
        }
    }

    for(int i = 0; i < ROLL_OUTS_NUMBER_+1; i++)
        roll_outs[i].insert(roll_outs[i].begin(), excluded_from_smoothing[i].begin(), excluded_from_smoothing[i].end());

    d_limit = 0;
    for(int j = smoothing_end_index; j < path.size(); j++)
    {
        if(j > 0)
            d_limit += distance2points(path[j], path[j-1]);

        if(d_limit > MAX_LOCAL_PLAN_DISTANCE_) //max_roll_distance)
            break;

        wp = path[j];
        double original_speed = wp.speed;
        for(int i = 0; i < roll_outs.size(); i++)
        {
            double d = end_laterals[i];
            wp.x = path[j].x - d * cos(wp.heading + M_PI_2);
            wp.y = path[j].y - d * sin(wp.heading + M_PI_2);

            if(i != central_trajectory_index)
                wp.speed = original_speed * LANE_CHANGE_SPEED_FACTOR_;
            else
                wp.speed = original_speed;

            roll_outs[i].push_back(wp);
        }
    }

    for(int i = 0; i < ROLL_OUTS_NUMBER_+1; i++)
    {
        smoothPath(roll_outs[i]);
        calculateAngleAndCost(roll_outs[i]);
    }

    visualiseRollOuts(roll_outs);
}

bool PlanningHelpers::getRelativeInfo(const std::vector<Waypoint>& trajectory, int& back_index, double& perp_distance)
{
    if(trajectory.size() < 2) return false;

    Waypoint p0, p1;
    int front_index;
    if(trajectory.size() == 2)
    {
        p0 = trajectory[0];
        p1.x = (trajectory[0].x+trajectory[1].x)/2.0;
        p1.y = (trajectory[0].y+trajectory[1].y)/2.0;
        p1.heading = trajectory[0].heading;
        front_index = 1;
        back_index = 0;
    }
    else
    {
        front_index = getClosestNextWaypointIndex(trajectory);

        if(front_index > 0)
            back_index = front_index - 1;
        else
            back_index = 0;

        if(front_index == 0)
        {
            p0 = trajectory[front_index];
            p1 = trajectory[front_index+1];
        }
        else if(front_index > 0 && front_index < trajectory.size()-1)
        {
            p0 = trajectory[front_index-1];
            p1 = trajectory[front_index];
        }
        else
        {
            p0 = trajectory[front_index-1];
            p1.x = (p0.x+trajectory[front_index].x)/2.0;
            p1.y = (p0.y+trajectory[front_index].y)/2.0;
            p1.heading = p0.heading;
        }
    }

    // Waypoint prevWP = p0;
    // Mat3 rotationMat(-p1.heading);
    // Mat3 translationMat(-current_state.x, -current_state.y);
    // Mat3 invRotationMat(p1.heading);
    // Mat3 invTranslationMat(current_state.x, current_state.y);

    p0.x = p0.x - current_state.x;
    p0.y = p0.y - current_state.y;
    // std::cout << "After translation: p0.x: " << p0.x << ", p0.y: " << p0.y << std::endl;
    double x = p0.x;
    p0.x = cos(-p1.heading)*p0.x - sin(-p1.heading)*p0.y;
    p0.y = sin(-p1.heading)*x + cos(-p1.heading)*p0.y;
    // std::cout << "After rotation: p0.x: " << p0.x << ", p0.y: " << p0.y << std::endl;
    // std::cout << "p1.heading: " << p1.heading << std::endl;
    
    p1.x = p1.x - current_state.x;
    p1.y = p1.y - current_state.y;
    x = p1.x;
    p1.x = cos(-p1.heading)*p1.x - sin(-p1.heading)*p1.y;
    p1.y = sin(-p1.heading)*x + cos(-p1.heading)*p1.y;

    double m = (p1.y-p0.y)/(p1.x-p0.x);
    perp_distance = p1.y - m*p1.x; // solve for x = 0
    // std::cout << "m: " << m << std::endl;

    if(std::isnan(perp_distance) || std::isinf(perp_distance)) perp_distance = 0;

    // info.to_front_distance = fabs(p1.pos.x); // distance on the x axes

    // info.perp_point = p1;
    // info.perp_point.pos.x = 0; // on the same y axis of the car
    // info.perp_point.pos.y = info.perp_distance; //perp distance between the car and the trajectory

    // info.perp_point.pos = invRotationMat  * info.perp_point.pos;
    // info.perp_point.pos = invTranslationMat  * info.perp_point.pos;

    // info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);

    // info.angle_diff = UtilityH::AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a)*RAD2DEG;

    return true;
}
}