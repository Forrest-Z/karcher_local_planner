#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

#include "karcher_local_planner/Waypoint.h"
#include "karcher_local_planner/WaypointArray.h"

#include <math.h>
#include <limits>
#include <vector>

#define _USE_MATH_DEFINES
#define RAD2DEG 180.0 / M_PI
#define distance2points(from , to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2pointsSqr(from , to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x*v.x + v.y*v.y)

typedef struct  
{
    double x;
    double y;
    double heading;
    double left_width;
    double right_width;
    double cost;
} Waypoint;

typedef struct  
{
    double x;
    double y;
    double yaw;
    double speed;
} VehicleState;

class KarcherLocalPlannerNode
{
public:
    // Constructor
    KarcherLocalPlannerNode();

    // Destructor
    virtual ~KarcherLocalPlannerNode(){};

private:
    ros::NodeHandle nh;

    // Hyperparameters
    double planning_frequency_;

    // parameters
    double MAX_SPEED_;                  // max speed that planner should not exceed
    double MIN_LOCAL_PLAN_DISTANCE_;    // length of local trajectory roll outs
    double PATH_DENSITY_;               // distance between waypoints of local trajectory
    int ROLL_OUTS_NUMBER_;              // number of roll outs not including the center tracjectory (this number should be even)
    double SAMPLING_TIP_MARGIN_;        // length of car tip margin
    double SAMPLING_OUT_MARGIN_;        // length of roll in margin (??)
    double ROLL_OUT_DENSITY_;           // distance between adjacent trajectories
    

    double MIN_FOLLOWING_DISTANCE_;     // distance threshold for exiting following behaviour
    double MAX_FOLLOWING_DISTANCE_;     // distance threshold for entering following behaviour
    double MIN_DISTANCE_TO_AVOID;       // distance threshold for obstacle avoidance behaviour

    double VEHICLE_WIDTH_;        
    double VEHICLE_LENGTH_;
    double WHEELBASE_LENGTH_;
    double TURNING_RADIUS_;
    
    // Smoothing Weights
    double SMOOTH_DATA_WEIGHT_; 
    double SMOOTH_WEIGHT_;
    double SMOOTH_TOLERANCE_;

    // Params
    std::vector<double> PARAMS_;

    // subscriber and publishers
    ros::Subscriber odom_sub;
    ros::Subscriber obstacle_sub;
    ros::Subscriber global_path_sub;

    ros::Publisher global_path_rviz_pub;
    ros::Publisher extracted_path_rviz_pub;
    ros::Publisher current_pose_rviz_pub;
    ros::Publisher cmd_vel_pub;

    // timer
    ros::Timer timer;

    // Variables and Functions for subscribe to odom topic
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // Main Function 
    void mainTimerCallback(const ros::TimerEvent& timer_event);

    // Functions for subscribing
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    // void obstacleCallback(obstacle_msg);
    void globalPathCallback(const karcher_local_planner::WaypointArray::ConstPtr& global_path_msg);

    // Functions for publishing results
    void visualizeGlobalPath();
    void visualizeExtractedPath(const std::vector<Waypoint>& extracted_path);
    void publishCmdVel();
    void visualizeCurrentPose();

    // Local Planning functions
    void extractGlobalPathSection(std::vector<Waypoint>& extracted_path);
    int getClosestNextWaypointIndex();
    double angleBetweenTwoAnglesPositive(const double& a1, const double& a2);
    double fixNegativeAngle(const double& a);
    void fixPathDensity(std::vector<Waypoint>& path);
    void smoothPath(std::vector<Waypoint>& path);
    double calculateAngleAndCost(std::vector<Waypoint>& path);
    void generateRollOuts(const std::vector<Waypoint>& path, std::vector<std::vector<Waypoint>> roll_outs);

    // Variables
    bool b_global_path;
    bool b_vehicle_state;
    bool b_obstacles;

    std::vector<Waypoint> global_path;
    VehicleState current_state;
    int prev_closest_index;
    double prev_cost;
};

// Constructor
KarcherLocalPlannerNode::KarcherLocalPlannerNode() : tf_listener(tf_buffer)
{
    ros::NodeHandle private_nh("~");

    // topics
    std::string odom_topic_;
    std::string obstacle_topic_;
    std::string global_path_topic_;

    std::string global_path_rviz_topic_;
    std::string extracted_path_rviz_topic_;
    std::string current_pose_rviz_topic_;
    std::string cmd_vel_topic_;

    // // Parameters from launch file: topic names
    ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
    ROS_ASSERT(private_nh.getParam("obstacle_topic", obstacle_topic_));
    ROS_ASSERT(private_nh.getParam("global_path_topic", global_path_topic_));

    ROS_ASSERT(private_nh.getParam("global_path_rviz_topic", global_path_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("extracted_path_rviz_topic", extracted_path_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("current_pose_rviz_topic", current_pose_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("cmd_vel_topic", cmd_vel_topic_));

    // Hyperparameters
    ROS_ASSERT(private_nh.getParam("planning_frequency", planning_frequency_));

    // Parameters from launch file: Planner Parameters
    ROS_ASSERT(private_nh.getParam("max_speed", MAX_SPEED_));    
    ROS_ASSERT(private_nh.getParam("min_local_plan_distance", MIN_LOCAL_PLAN_DISTANCE_));   
    ROS_ASSERT(private_nh.getParam("path_density", PATH_DENSITY_));        
    ROS_ASSERT(private_nh.getParam("roll_outs_number", ROLL_OUTS_NUMBER_));           
    ROS_ASSERT(private_nh.getParam("sampling_tip_margin", SAMPLING_TIP_MARGIN_));     
    ROS_ASSERT(private_nh.getParam("sampling_out_margin", SAMPLING_OUT_MARGIN_));     
    ROS_ASSERT(private_nh.getParam("roll_out_density", ROLL_OUT_DENSITY_)); 
    
    ROS_ASSERT(private_nh.getParam("min_following_distance", MIN_FOLLOWING_DISTANCE_));   
    ROS_ASSERT(private_nh.getParam("max_following_distance", MAX_FOLLOWING_DISTANCE_));       
    ROS_ASSERT(private_nh.getParam("min_distance_to_avoid", MIN_DISTANCE_TO_AVOID));         
    
    ROS_ASSERT(private_nh.getParam("vehicle_width", VEHICLE_WIDTH_));  
    ROS_ASSERT(private_nh.getParam("vehicle_length", VEHICLE_LENGTH_));       
    ROS_ASSERT(private_nh.getParam("wheelbase_length", WHEELBASE_LENGTH_));       
    ROS_ASSERT(private_nh.getParam("turning_radius", TURNING_RADIUS_));   

    // Smoothing weights
    ROS_ASSERT(private_nh.getParam("smooth_data_weight", SMOOTH_DATA_WEIGHT_));
    ROS_ASSERT(private_nh.getParam("smooth_weight", SMOOTH_WEIGHT_));
    ROS_ASSERT(private_nh.getParam("smooth_tolerance", SMOOTH_TOLERANCE_));

    // Subscribe & Advertise
    odom_sub = nh.subscribe(odom_topic_, 1, &KarcherLocalPlannerNode::odomCallback, this);
    // obstacle_sub = nh.subscribe(obstacle_topic_, 1, &KarcherLocalPlannerNode::obstacleCallback, this);
    global_path_sub = nh.subscribe(global_path_topic_, 1, &KarcherLocalPlannerNode::globalPathCallback, this);

    global_path_rviz_pub = nh.advertise<nav_msgs::Path>(global_path_rviz_topic_, 1, true);
    extracted_path_rviz_pub = nh.advertise<nav_msgs::Path>(extracted_path_rviz_topic_, 1, true);
    current_pose_rviz_pub = nh.advertise<geometry_msgs::PoseStamped>(current_pose_rviz_topic_, 1, true);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

    // timer
    timer = nh.createTimer(ros::Duration(1.0 / planning_frequency_), &KarcherLocalPlannerNode::mainTimerCallback, this);

    b_global_path = false;
    b_vehicle_state = false;
    b_obstacles = false;
    prev_closest_index = 0;
};

void KarcherLocalPlannerNode::mainTimerCallback(const ros::TimerEvent& timer_event)
{
    if(!b_global_path)
    {
        ROS_WARN("Karcher Local Planner Node: Global path not received!");
        return;
    }
    if(!b_vehicle_state)
    {
        ROS_WARN("Karcher Local Planner Node: Odom data not received!");
        return;
    }

    publishCmdVel();

    // if(!b_obstacles)
    // {
    //     ROS_WARN("Karcher Local Planner Node: Obstacles data not received!");
    //     return;
    // }

    std::vector<Waypoint> extracted_path;
    extractGlobalPathSection(extracted_path);

    std::vector<std::vector<Waypoint>> roll_outs;
    // generateRollOuts(extracted_path, roll_outs);


}

void KarcherLocalPlannerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    b_vehicle_state = true;

    current_state.speed = odom_msg->twist.twist.linear.x;

    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer.lookupTransform("map", "odom", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    geometry_msgs::PoseStamped pose_before_transform, pose_after_transform;
    pose_before_transform.header.frame_id = odom_msg->header.frame_id;
    pose_before_transform.header.stamp = odom_msg->header.stamp;
    pose_before_transform.pose = odom_msg->pose.pose;
    tf2::doTransform(pose_before_transform, pose_after_transform, transform_stamped);

    tf::Quaternion q(pose_after_transform.pose.orientation.x, pose_after_transform.pose.orientation.y,
                    pose_after_transform.pose.orientation.z, pose_after_transform.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_state.yaw);

    // Current XY of robot (map frame)
    current_state.x = pose_after_transform.pose.position.x;
    current_state.y = pose_after_transform.pose.position.y;

    // updateVehicleFrontlinkState();
    visualizeCurrentPose();
}

// void KarcherLocalPlannerNode::obstacleCallback(obstacle_msg)
// {
//     b_obstacles = true;
// }

void KarcherLocalPlannerNode::visualizeGlobalPath()
{
    nav_msgs::Path path;
    path.header.frame_id = "map";

    for(unsigned int i = 0; i < global_path.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = global_path[i].x;
        pose.pose.position.y = global_path[i].y;
        geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(global_path[i].heading);
        pose.pose.orientation = pose_quat;
        path.poses.push_back(pose);
    }

  global_path_rviz_pub.publish(path);
}

void KarcherLocalPlannerNode::visualizeExtractedPath(const std::vector<Waypoint>& extracted_path)
{
    nav_msgs::Path path;
    path.header.frame_id = "map";

    for(unsigned int i = 0; i < extracted_path.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = extracted_path[i].x;
        pose.pose.position.y = extracted_path[i].y;
        geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(extracted_path[i].heading);
        pose.pose.orientation = pose_quat;
        path.poses.emplace_back(pose);
    }

  extracted_path_rviz_pub.publish(path);
}

void KarcherLocalPlannerNode::visualizeCurrentPose()
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = current_state.x;
    pose.pose.position.y = current_state.y;
    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(current_state.yaw);
    pose.pose.orientation = pose_quat;

    current_pose_rviz_pub.publish(pose);
}

void KarcherLocalPlannerNode::publishCmdVel()
{
    geometry_msgs::Twist cmd_vel_msg;

    cmd_vel_msg.linear.x = 0.5;
    cmd_vel_msg.linear.y = 0;
    cmd_vel_msg.linear.z = 0;

    cmd_vel_msg.angular.x = 0;
    cmd_vel_msg.angular.y = 0;
    cmd_vel_msg.angular.z = 0;

    cmd_vel_pub.publish(cmd_vel_msg);
}

void KarcherLocalPlannerNode::globalPathCallback(const karcher_local_planner::WaypointArray::ConstPtr& global_path_msg)
{
    b_global_path = true;

    global_path.clear();

    Waypoint wp;
    for(unsigned int i = 0; i < global_path_msg->waypoints.size(); i++)
    {
        wp.x = global_path_msg->waypoints[i].x;
        wp.y = global_path_msg->waypoints[i].y;
        wp.heading = global_path_msg->waypoints[i].heading;
        wp.left_width = global_path_msg->waypoints[i].left_width;
        wp.right_width = global_path_msg->waypoints[i].right_width;
        wp.cost = 0;

        global_path.push_back(wp);
    }

    visualizeGlobalPath();
}

void KarcherLocalPlannerNode::extractGlobalPathSection(std::vector<Waypoint>& extracted_path)
{
    if(global_path.size() < 2) return;

    extracted_path.clear();

    int closest_index = getClosestNextWaypointIndex();

    if(closest_index + 1 >= global_path.size())
        closest_index = global_path.size() - 2;
    
    prev_closest_index = closest_index;
    prev_cost = global_path[prev_closest_index].cost;

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

    d = 0;
    for(int i = closest_index + 1; i < (int)global_path.size(); i++)
    {
        extracted_path.push_back(global_path[i]);
        if(i > 0)
            d += hypot(global_path[i].x - global_path[i-1].x, global_path[i].y - global_path[i-1].y);
        if(d > MIN_LOCAL_PLAN_DISTANCE_)
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
    visualizeExtractedPath(extracted_path);
}

int KarcherLocalPlannerNode::getClosestNextWaypointIndex()
{   
    double d = 0, minD = DBL_MAX;
    int min_index = prev_closest_index;

    for(int i = prev_closest_index; i < global_path.size(); i++)
    {
        d = distance2pointsSqr(global_path[i], current_state);
        double angle_diff = angleBetweenTwoAnglesPositive(global_path[i].heading, current_state.yaw)*RAD2DEG;

        if(d < minD && angle_diff < 45)
        {
            min_index = i;
            minD = d;
        }
    }

    ROS_INFO("Closest Global Waypoint Index = %d", min_index);

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

double KarcherLocalPlannerNode::angleBetweenTwoAnglesPositive(const double& a1, const double& a2)
{
    double diff = a1 - a2;
    if(diff < 0)
        diff = a2 - a1;

    if(diff > M_PI)
        diff = 2.0 * M_PI - diff;

    return diff;
}

double KarcherLocalPlannerNode::fixNegativeAngle(const double& a)
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

void KarcherLocalPlannerNode::fixPathDensity(std::vector<Waypoint> &path)
{
    if(path.size() == 0 || PATH_DENSITY_ == 0) return;

    double d = 0, a = 0;
    double margin = PATH_DENSITY_ * 0.01;
    double remaining = 0;
    int nPoints = 0;
    std::vector<Waypoint> fixed_path;
    fixed_path.push_back(path[0]);
    for(unsigned int si = 0, ei = 1; ei < path.size(); )
    {
        d += hypot(path[ei].x - path[ei-1].x, path[ei].y - path[ei-1].y) + remaining;
        a = atan2(path[ei].y - path[si].y, path[ei].x - path[si].x);

        if(d < PATH_DENSITY_ - margin ) // skip
        {
            ei++;
            remaining = 0;
        }
        else if(d > (PATH_DENSITY_ +  margin)) // skip
        {
            Waypoint pm = path[si];
            nPoints = d  / PATH_DENSITY_;
            for(int k = 0; k < nPoints; k++)
            {
                pm.x = pm.x + PATH_DENSITY_ * cos(a);
                pm.y = pm.y + PATH_DENSITY_ * sin(a);
                fixed_path.push_back(pm);
            }
            remaining = d - nPoints*PATH_DENSITY_;
            si++;
            path[si] = pm;
            d = 0;
            ei++;
        }
        else
        {
            d = 0;
            remaining = 0;
            fixed_path.push_back(path[ei]);
            ei++;
            si = ei - 1;
        }
    }

    path = fixed_path;
}

void KarcherLocalPlannerNode::smoothPath(std::vector<Waypoint> &path)
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

  ROS_INFO("Number of iterations: %d", nIterations);

  path = smoothed_path;
}

double KarcherLocalPlannerNode::calculateAngleAndCost(std::vector<Waypoint> &path)
{
    if(path.size() < 2) return 0;

    if(path.size() == 2)
    {
        path[0].heading = fixNegativeAngle(atan2(path[1].y - path[0].y, path[1].x - path[0].x));
        path[0].cost = prev_cost;
        path[1].heading = path[0].heading;
        path[1].cost = path[0].cost + distance2points(path[0], path[1]);
        return path[1].cost;
    }

    path[0].heading = fixNegativeAngle(atan2(path[1].y - path[0].y, path[1].x - path[0].x));
    path[0].cost = prev_cost;

    for(int j = 1; j < path.size()-1; j++)
    {
        path[j].heading = fixNegativeAngle(atan2(path[j+1].y - path[j].y, path[j+1].x - path[j].x));
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

void KarcherLocalPlannerNode::generateRollOuts(const std::vector<Waypoint>& path, std::vector<std::vector<Waypoint>> roll_outs)
{

}

// void PlannerH::GenerateRunoffTrajectory(const std::vector<std::vector<WayPoint> >& referencePaths,const WayPoint& carPos, const bool& bEnableLaneChange, const double& speed, const double& microPlanDistance,
//     const double& maxSpeed,const double& minSpeed, const double&  carTipMargin, const double& rollInMargin,
//     const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
//     const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
//     const double& SmoothTolerance, const double& speedProfileFactor, const bool& bHeadingSmooth,
//     const int& iCurrGlobalPath, const int& iCurrLocalTraj,
//     std::vector<std::vector<std::vector<WayPoint> > >& rollOutsPaths,
//     std::vector<WayPoint>& sampledPoints_debug)
// {

//   if(referencePaths.size()==0) return;
//   if(microPlanDistance <=0 ) return;
//   rollOutsPaths.clear();

//   sampledPoints_debug.clear(); //for visualization only

//   for(unsigned int i = 0; i < referencePaths.size(); i++)
//   {
//     std::vector<std::vector<WayPoint> > local_rollOutPaths;
//     int s_index = 0, e_index = 0;
//     vector<double> e_distances;
//     if(referencePaths.at(i).size()>0)
//     {
//       PlanningHelpers::CalculateRollInTrajectories(carPos, speed, referencePaths.at(i), s_index, e_index, e_distances,
//           local_rollOutPaths, microPlanDistance, maxSpeed, carTipMargin, rollInMargin,
//           rollInSpeedFactor, pathDensity, rollOutDensity,rollOutNumber,
//           SmoothDataWeight, SmoothWeight, SmoothTolerance, bHeadingSmooth, sampledPoints_debug);
//     }
//     else
//     {
//       for(int j=0; j< rollOutNumber+1; j++)
//       {
//         local_rollOutPaths.push_back(vector<WayPoint>());
//       }
//     }

//     rollOutsPaths.push_back(local_rollOutPaths);
//   }
// }

// void PlanningHelpers::CalculateRollInTrajectories(const WayPoint& carPos, const double& speed, const vector<WayPoint>& originalCenter, int& start_index,
//     int& end_index, vector<double>& end_laterals ,
//     vector<vector<WayPoint> >& rollInPaths, const double& max_roll_distance,
//     const double& maxSpeed, const double&  carTipMargin, const double& rollInMargin,
//     const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
//     const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
//     const double& SmoothTolerance, const bool& bHeadingSmooth,
//     std::vector<WayPoint>& sampledPoints)
// {
//   WayPoint p;
//   double dummyd = 0;

//   int iLimitIndex = (carTipMargin/0.3)/pathDensity;
//   if(iLimitIndex >= originalCenter.size())
//     iLimitIndex = originalCenter.size() - 1;

//   //Get Closest Index
//   RelativeInfo info;
//   GetRelativeInfo(originalCenter, carPos, info);
//   double remaining_distance = 0;
//   int close_index = info.iBack;
//   for(unsigned int i=close_index; i< originalCenter.size()-1; i++)
//     {
//     if(i>0)
//       remaining_distance += distance2points(originalCenter[i].pos, originalCenter[i+1].pos);
//     }

//   double initial_roll_in_distance = info.perp_distance ; //GetPerpDistanceToTrajectorySimple(originalCenter, carPos, close_index);


//   vector<WayPoint> RollOutStratPath;
//   ///***   Smoothing From Car Heading Section ***///
// //  if(bHeadingSmooth)
// //  {
// //    unsigned int num_of_strait_points = carTipMargin / pathDensity;
// //    int closest_for_each_iteration = 0;
// //    WayPoint np = GetPerpendicularOnTrajectory_obsolete(originalCenter, carPos, dummyd, closest_for_each_iteration);
// //    np.pos = carPos.pos;
// //
// //    RollOutStratPath.push_back(np);
// //    for(unsigned int i = 0; i < num_of_strait_points; i++)
// //    {
// //      p = RollOutStratPath.at(i);
// //      p.pos.x = p.pos.x +  pathDensity*cos(p.pos.a);
// //      p.pos.y = p.pos.y +  pathDensity*sin(p.pos.a);
// //      np = GetPerpendicularOnTrajectory_obsolete(originalCenter, p, dummyd, closest_for_each_iteration);
// //      np.pos = p.pos;
// //      RollOutStratPath.push_back(np);
// //    }
// //
// //    initial_roll_in_distance = GetPerpDistanceToTrajectorySimple_obsolete(originalCenter, RollOutStratPath.at(RollOutStratPath.size()-1), close_index);
// //  }
//   ///***   -------------------------------- ***///


//   //printf("\n Lateral Distance: %f" , initial_roll_in_distance);

//   //calculate the starting index
//   double d_limit = 0;
//   unsigned int far_index = close_index;

//   //calculate end index
//   double start_distance = rollInSpeedFactor*speed+rollInMargin;
//   if(start_distance > remaining_distance)
//     start_distance = remaining_distance;

//   d_limit = 0;
//   for(unsigned int i=close_index; i< originalCenter.size(); i++)
//     {
//       if(i>0)
//         d_limit += distance2points(originalCenter[i].pos, originalCenter[i-1].pos);

//       if(d_limit >= start_distance)
//       {
//         far_index = i;
//         break;
//       }
//     }

//   int centralTrajectoryIndex = rollOutNumber/2;
//   vector<double> end_distance_list;
//   for(int i=0; i< rollOutNumber+1; i++)
//     {
//       double end_roll_in_distance = rollOutDensity*(i - centralTrajectoryIndex);
//       end_distance_list.push_back(end_roll_in_distance);
//     }

//   start_index = close_index;
//   end_index = far_index;
//   end_laterals = end_distance_list;

//   //calculate the actual calculation starting index
//   d_limit = 0;
//   unsigned int smoothing_start_index = start_index;
//   unsigned int smoothing_end_index = end_index;

//   for(unsigned int i=smoothing_start_index; i< originalCenter.size(); i++)
//   {
//     if(i > 0)
//       d_limit += distance2points(originalCenter[i].pos, originalCenter[i-1].pos);
//     if(d_limit > carTipMargin)
//       break;

//     smoothing_start_index++;
//   }

//   d_limit = 0;
//   for(unsigned int i=end_index; i< originalCenter.size(); i++)
//   {
//     if(i > 0)
//       d_limit += distance2points(originalCenter[i].pos, originalCenter[i-1].pos);
//     if(d_limit > carTipMargin)
//       break;

//     smoothing_end_index++;
//   }

//   int nSteps = end_index - smoothing_start_index;


//   vector<double> inc_list;
//   rollInPaths.clear();
//   vector<double> inc_list_inc;
//   for(int i=0; i< rollOutNumber+1; i++)
//   {
//     double diff = end_laterals.at(i)-initial_roll_in_distance;
//     inc_list.push_back(diff/(double)nSteps);
//     rollInPaths.push_back(vector<WayPoint>());
//     inc_list_inc.push_back(0);
//   }



//   vector<vector<WayPoint> > execluded_from_smoothing;
//   for(unsigned int i=0; i< rollOutNumber+1 ; i++)
//     execluded_from_smoothing.push_back(vector<WayPoint>());



//   //Insert First strait points within the tip of the car range
//   for(unsigned int j = start_index; j < smoothing_start_index; j++)
//   {
//     p = originalCenter.at(j);
//     double original_speed = p.v;
//     for(unsigned int i=0; i< rollOutNumber+1 ; i++)
//     {
//       p.pos.x = originalCenter.at(j).pos.x -  initial_roll_in_distance*cos(p.pos.a + M_PI_2);
//       p.pos.y = originalCenter.at(j).pos.y -  initial_roll_in_distance*sin(p.pos.a + M_PI_2);
//       if(i!=centralTrajectoryIndex)
//         p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
//       else
//         p.v = original_speed ;

//       if(j < iLimitIndex)
//         execluded_from_smoothing.at(i).push_back(p);
//       else
//         rollInPaths.at(i).push_back(p);

//       sampledPoints.push_back(p);
//     }
//   }

//   for(unsigned int j = smoothing_start_index; j < end_index; j++)
//     {
//       p = originalCenter.at(j);
//       double original_speed = p.v;
//       for(unsigned int i=0; i< rollOutNumber+1 ; i++)
//       {
//         inc_list_inc[i] += inc_list[i];
//         double d = inc_list_inc[i];
//         p.pos.x = originalCenter.at(j).pos.x -  initial_roll_in_distance*cos(p.pos.a + M_PI_2) - d*cos(p.pos.a+ M_PI_2);
//         p.pos.y = originalCenter.at(j).pos.y -  initial_roll_in_distance*sin(p.pos.a + M_PI_2) - d*sin(p.pos.a+ M_PI_2);
//         if(i!=centralTrajectoryIndex)
//           p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
//         else
//           p.v = original_speed ;

//         rollInPaths.at(i).push_back(p);

//         sampledPoints.push_back(p);
//       }
//     }

//   //Insert last strait points to make better smoothing
//   for(unsigned int j = end_index; j < smoothing_end_index; j++)
//   {
//     p = originalCenter.at(j);
//     double original_speed = p.v;
//     for(unsigned int i=0; i< rollOutNumber+1 ; i++)
//     {
//       double d = end_laterals.at(i);
//       p.pos.x  = originalCenter.at(j).pos.x - d*cos(p.pos.a + M_PI_2);
//       p.pos.y  = originalCenter.at(j).pos.y - d*sin(p.pos.a + M_PI_2);
//       if(i!=centralTrajectoryIndex)
//         p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
//       else
//         p.v = original_speed ;
//       rollInPaths.at(i).push_back(p);

//       sampledPoints.push_back(p);
//     }
//   }

//   for(unsigned int i=0; i< rollOutNumber+1 ; i++)
//     rollInPaths.at(i).insert(rollInPaths.at(i).begin(), execluded_from_smoothing.at(i).begin(), execluded_from_smoothing.at(i).end());

//   ///***   Smoothing From Car Heading Section ***///
// //  if(bHeadingSmooth)
// //  {
// //    for(unsigned int i=0; i< rollOutNumber+1 ; i++)
// //    {
// //      unsigned int cut_index = GetClosestNextPointIndex(rollInPaths.at(i), RollOutStratPath.at(RollOutStratPath.size()-1));
// //      rollInPaths.at(i).erase(rollInPaths.at(i).begin(), rollInPaths.at(i).begin()+cut_index);
// //      rollInPaths.at(i).insert(rollInPaths.at(i).begin(), RollOutStratPath.begin(), RollOutStratPath.end());
// //    }
// //  }
//   ///***   -------------------------------- ***///



//   d_limit = 0;
//   for(unsigned int j = smoothing_end_index; j < originalCenter.size(); j++)
//     {
//     if(j > 0)
//       d_limit += distance2points(originalCenter.at(j).pos, originalCenter.at(j-1).pos);

//     if(d_limit > max_roll_distance)
//       break;

//       p = originalCenter.at(j);
//       double original_speed = p.v;
//       for(unsigned int i=0; i< rollInPaths.size() ; i++)
//       {
//         double d = end_laterals.at(i);
//         p.pos.x  = originalCenter.at(j).pos.x - d*cos(p.pos.a + M_PI_2);
//         p.pos.y  = originalCenter.at(j).pos.y - d*sin(p.pos.a + M_PI_2);

//         if(i!=centralTrajectoryIndex)
//           p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
//         else
//           p.v = original_speed ;

//         rollInPaths.at(i).push_back(p);

//         sampledPoints.push_back(p);
//       }
//     }

//   for(unsigned int i=0; i< rollOutNumber+1 ; i++)
//   {
//     SmoothPath(rollInPaths.at(i), SmoothDataWeight, SmoothWeight, SmoothTolerance);
//   }

// //  for(unsigned int i=0; i< rollInPaths.size(); i++)
// //    CalcAngleAndCost(rollInPaths.at(i));
// }

// void PlanningHelpers::PredictConstantTimeCostForTrajectory(std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& currPose, const double& minVelocity, const double& minDist)
// {
//   if(path.size() == 0) return;

//   for(unsigned int i = 0 ; i < path.size(); i++)
//     path.at(i).timeCost = -1;

//   if(currPose.v == 0 || currPose.v < minVelocity) return;

//   RelativeInfo info;
//   PlanningHelpers::GetRelativeInfo(path, currPose, info);

//   double total_distance = 0;
//   double accum_time = 0;

//   path.at(info.iFront).timeCost = 0;
//   if(info.iFront == 0 ) info.iFront++;

//   for(unsigned int i=info.iFront; i<path.size(); i++)
//   {
//     total_distance += hypot(path.at(i).pos.x- path.at(i-1).pos.x,path.at(i).pos.y- path.at(i-1).pos.y);
//     accum_time = total_distance/currPose.v;
//     path.at(i).timeCost = accum_time;
//   }
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "karcher_local_planner_node");
    KarcherLocalPlannerNode karcher_local_planner_obj;  
    ros::spin();
    return 0;
}