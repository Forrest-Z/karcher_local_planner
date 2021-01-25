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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "karcher_local_planner/Waypoint.h"
#include "karcher_local_planner/WaypointArray.h"

#include "karcher_local_planner/karcher_local_planner_node.h"
#include "karcher_local_planner/MatrixOperations.h"
#include "karcher_local_planner/Polygon.h"

#include "obstacle_detector/Obstacles.h"

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
    double yaw;
    double speed;
    double steer;
} VehicleState;

typedef struct 
{
    int front_index;
    int back_index;
    double perp_distance;
    double to_front_distance;
    double from_back_distance;
    double angle_diff;
    Waypoint perp_point;
} RelativeInfo;

typedef struct
{
    int index;
    int relative_index;
    double closest_obj_velocity;
    double distance_from_center;
    double priority_cost; //0 to 1
    double transition_cost; // 0 to 1
    double closest_obj_cost; // 0 to 1
    double cost;
    double closest_obj_distance;

    int lane_index;
    double lane_change_cost;
    double lateral_cost;
    double longitudinal_cost;
    bool bBlocked;
    std::vector<std::pair<int, double>> lateral_costs;
} TrajectoryCost;

typedef struct
{
    double x;
    double y;
    double vx;
    double vy;
    double radius;
    double true_radius; 
} CircleObstacle;

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
    double MAX_LOCAL_PLAN_DISTANCE_;    // length of local trajectory roll outs
    double PATH_DENSITY_;               // distance between waypoints of local trajectory
    int ROLL_OUTS_NUMBER_;              // number of roll outs not including the center tracjectory (this number should be even)
    double SAMPLING_TIP_MARGIN_;        // length of car tip margin
    double SAMPLING_OUT_MARGIN_;        // length of roll in margin (??)
    double ROLL_OUT_DENSITY_;           // distance between adjacent trajectories
    double ROLL_IN_SPEED_FACTOR_;
    double ROLL_IN_MARGIN_;
    double LANE_CHANGE_SPEED_FACTOR_;
    double HORIZON_DISTANCE_;

    double HORIZONTAL_SAFETY_DISTANCE_;
    double VERTICAL_SAFETY_DISTANCE_;
    double MAX_STEER_ANGLE_;
    double MIN_SPEED_;
    double LATERAL_SKIP_DISTANCE_;

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

    double PRIORITY_WEIGHT_;
    double TRANSITION_WEIGHT_;
    double LAT_WEIGHT_;
    double LONG_WEIGHT_;

    // Params
    std::vector<double> PARAMS_;

    // subscriber and publishers
    ros::Subscriber odom_sub;
    ros::Subscriber obstacles_sub;
    ros::Subscriber global_path_sub;

    ros::Publisher global_path_rviz_pub;
    ros::Publisher extracted_path_rviz_pub;
    ros::Publisher current_pose_rviz_pub;
    ros::Publisher roll_outs_rviz_pub;
    ros::Publisher weighted_trajectories_rviz_pub;
    ros::Publisher safety_box_rviz_pub;
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
    void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
    void globalPathCallback(const karcher_local_planner::WaypointArray::ConstPtr& global_path_msg);

    // Functions for publishing results
    void visualizeGlobalPath();
    void visualizeExtractedPath(const std::vector<Waypoint>& extracted_path);
    void publishCmdVel();
    void visualizeCurrentPose();
    void visualizeRollOuts(const std::vector<std::vector<Waypoint>>& roll_outs);
    void visualizeWeightedTrajectories(const std::vector<std::vector<Waypoint>>& paths, const std::vector<TrajectoryCost>& traj_costs, 
                                        const int& iClosest);
    void visualizeSafetyBox(const std::vector<Waypoint>& safety_box);

    // Local Planning functions
    void extractGlobalPathSection(std::vector<Waypoint>& extracted_path);
    int getClosestNextWaypointIndex(const std::vector<Waypoint>& path, const Waypoint& current_pos);
    double angleBetweenTwoAnglesPositive(const double& a1, const double& a2);
    double fixNegativeAngle(const double& a);
    void fixPathDensity(std::vector<Waypoint>& path);
    void smoothPath(std::vector<Waypoint>& path);
    double calculateAngleAndCost(std::vector<Waypoint>& path);
    void generateRollOuts(const std::vector<Waypoint>& path, std::vector<std::vector<Waypoint>>& roll_outs);
    bool getRelativeInfo(const std::vector<Waypoint>& path, const Waypoint& current_pos, RelativeInfo& info);
    void predictConstantTimeCostForTrajectory(std::vector<Waypoint>& path);
    TrajectoryCost doOneStepStatic(const std::vector<std::vector<Waypoint>>& roll_outs, const std::vector<Waypoint>& extracted_path, std::vector<TrajectoryCost>& trajectory_costs);
    void calculateTransitionCosts(std::vector<TrajectoryCost>& trajectory_costs, const int& curr_trajectory_index);
    void calculateLateralAndLongitudinalCostsStatic(std::vector<TrajectoryCost>& trajectory_costs, const std::vector<std::vector<Waypoint>>& roll_outs, 
                                                    const std::vector<Waypoint>& extracted_path);
    double getExactDistanceOnTrajectory(const std::vector<Waypoint>& trajectory, const RelativeInfo& p1, const RelativeInfo& p2);
    void normalizeCosts(std::vector<TrajectoryCost>& trajectory_costs);

    // Variables
    bool b_global_path;
    bool b_vehicle_state;
    bool b_obstacles;

    std::vector<Waypoint> global_path;
    VehicleState current_state;
    std::vector<CircleObstacle> circle_obstacles;
    std::vector<Waypoint> obstacle_waypoints;
    int prev_closest_index;
    double prev_cost;
    Polygon safety_box;
};

// Constructor
KarcherLocalPlannerNode::KarcherLocalPlannerNode() : tf_listener(tf_buffer)
{
    ros::NodeHandle private_nh("~");

    // topics
    std::string odom_topic_;
    std::string obstacles_topic_;
    std::string global_path_topic_;

    std::string global_path_rviz_topic_;
    std::string extracted_path_rviz_topic_;
    std::string current_pose_rviz_topic_;
    std::string roll_outs_rviz_topic_;
    std::string weighted_trajectories_rviz_topic_;
    std::string safety_box_rviz_topic_;
    std::string cmd_vel_topic_;

    // // Parameters from launch file: topic names
    ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
    ROS_ASSERT(private_nh.getParam("obstacles_topic", obstacles_topic_));
    ROS_ASSERT(private_nh.getParam("global_path_topic", global_path_topic_));

    ROS_ASSERT(private_nh.getParam("global_path_rviz_topic", global_path_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("extracted_path_rviz_topic", extracted_path_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("current_pose_rviz_topic", current_pose_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("roll_outs_rviz_topic", roll_outs_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("weighted_trajectories_rviz_topic", weighted_trajectories_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("safety_box_rviz_topic", safety_box_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("cmd_vel_topic", cmd_vel_topic_));

    // Hyperparameters
    ROS_ASSERT(private_nh.getParam("planning_frequency", planning_frequency_));

    // Parameters from launch file: Planner Parameters
    ROS_ASSERT(private_nh.getParam("max_speed", MAX_SPEED_));    
    ROS_ASSERT(private_nh.getParam("max_local_plan_distance", MAX_LOCAL_PLAN_DISTANCE_));   
    ROS_ASSERT(private_nh.getParam("path_density", PATH_DENSITY_));        
    ROS_ASSERT(private_nh.getParam("roll_outs_number", ROLL_OUTS_NUMBER_));           
    ROS_ASSERT(private_nh.getParam("sampling_tip_margin", SAMPLING_TIP_MARGIN_));     
    ROS_ASSERT(private_nh.getParam("sampling_out_margin", SAMPLING_OUT_MARGIN_));     
    ROS_ASSERT(private_nh.getParam("roll_out_density", ROLL_OUT_DENSITY_)); 
    ROS_ASSERT(private_nh.getParam("roll_in_speed_factor", ROLL_IN_SPEED_FACTOR_)); 
    ROS_ASSERT(private_nh.getParam("roll_in_margin", ROLL_IN_MARGIN_)); 
    ROS_ASSERT(private_nh.getParam("lane_change_speed_factor", LANE_CHANGE_SPEED_FACTOR_)); 
    ROS_ASSERT(private_nh.getParam("horizon_distance", HORIZON_DISTANCE_)); 

    ROS_ASSERT(private_nh.getParam("horizontal_safety_distance", HORIZONTAL_SAFETY_DISTANCE_)); 
    ROS_ASSERT(private_nh.getParam("vertical_safety_distance", VERTICAL_SAFETY_DISTANCE_)); 
    ROS_ASSERT(private_nh.getParam("max_steer_angle", MAX_STEER_ANGLE_)); 
    ROS_ASSERT(private_nh.getParam("min_speed", MIN_SPEED_)); 
    ROS_ASSERT(private_nh.getParam("lateral_skip_distance", LATERAL_SKIP_DISTANCE_)); 
    
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

    ROS_ASSERT(private_nh.getParam("priority_weight", PRIORITY_WEIGHT_));
    ROS_ASSERT(private_nh.getParam("transition_weight", TRANSITION_WEIGHT_));
    ROS_ASSERT(private_nh.getParam("lat_weight", LAT_WEIGHT_));
    ROS_ASSERT(private_nh.getParam("long_weight", LONG_WEIGHT_));

    // Subscribe & Advertise
    odom_sub = nh.subscribe(odom_topic_, 1, &KarcherLocalPlannerNode::odomCallback, this);
    obstacles_sub = nh.subscribe(obstacles_topic_, 1, &KarcherLocalPlannerNode::obstaclesCallback, this);
    global_path_sub = nh.subscribe(global_path_topic_, 1, &KarcherLocalPlannerNode::globalPathCallback, this);

    global_path_rviz_pub = nh.advertise<nav_msgs::Path>(global_path_rviz_topic_, 1, true);
    extracted_path_rviz_pub = nh.advertise<nav_msgs::Path>(extracted_path_rviz_topic_, 1, true);
    current_pose_rviz_pub = nh.advertise<geometry_msgs::PoseStamped>(current_pose_rviz_topic_, 1, true);
    roll_outs_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>(roll_outs_rviz_topic_, 1, true);
    weighted_trajectories_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>(weighted_trajectories_rviz_topic_, 1, true);
    safety_box_rviz_pub = nh.advertise<visualization_msgs::Marker>(safety_box_rviz_topic_, 1, true);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

    // timer
    timer = nh.createTimer(ros::Duration(1.0 / planning_frequency_), &KarcherLocalPlannerNode::mainTimerCallback, this);

    b_global_path = false;
    b_vehicle_state = false;
    b_obstacles = false;
    prev_closest_index = 0;
    prev_cost = 0;
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

    if(!b_obstacles)
    {
        ROS_WARN("Karcher Local Planner Node: Obstacles data not received!");
        return;
    }

    std::vector<Waypoint> extracted_path;
    extractGlobalPathSection(extracted_path);

    std::vector<std::vector<Waypoint>> roll_outs;
    generateRollOuts(extracted_path, roll_outs);

    std::vector<TrajectoryCost> trajectory_costs;
    doOneStepStatic(roll_outs, extracted_path, trajectory_costs);
}

void KarcherLocalPlannerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    b_vehicle_state = true;

    current_state.speed = odom_msg->twist.twist.linear.x;
    if(fabs(odom_msg->twist.twist.linear.x) > 0.25)
        current_state.steer = atan(WHEELBASE_LENGTH_ * odom_msg->twist.twist.angular.z/odom_msg->twist.twist.linear.x);

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

void KarcherLocalPlannerNode::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
    b_obstacles = true;

    circle_obstacles.clear();
    obstacle_waypoints.clear();

    // std::cout << "Number of obstacles detected: " << obstacle_msg->circles.size() << std::endl;

    for(int i = 0; i < obstacle_msg->circles.size(); i++)
    {
        CircleObstacle obs;
        obs.x = obstacle_msg->circles[i].center.x;
        obs.y = obstacle_msg->circles[i].center.y;
        obs.vx = obstacle_msg->circles[i].velocity.x;
        obs.vy = obstacle_msg->circles[i].velocity.y;
        obs.radius = obstacle_msg->circles[i].radius;
        obs.true_radius = obstacle_msg->circles[i].true_radius;
        circle_obstacles.push_back(obs);

        Waypoint wp;
        wp.x = obstacle_msg->circles[i].center.x;
        wp.y = obstacle_msg->circles[i].center.y;
        wp.left_width = obstacle_msg->circles[i].radius;            // TODO: for now, left width attribute represents radius of circle obstacle
        wp.right_width = obstacle_msg->circles[i].true_radius;      // TODO: for now, right width attribute represents true radius of circle obstacle
        obstacle_waypoints.push_back(wp);
    }
}

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

void KarcherLocalPlannerNode::visualizeRollOuts(const std::vector<std::vector<Waypoint>>& roll_outs)
{
    // ROS_INFO_STREAM("Number of roll outs: " << roll_outs.size());
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = "map";
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "local_roll_outs_marker";
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.01;
    lane_waypoint_marker.scale.y = 0.01;
    //lane_waypoint_marker.scale.z = 0.1;
    lane_waypoint_marker.frame_locked = false;
    // std_msgs::ColorRGBA  total_color, curr_color;

    int count = 0;
    for(int i = 0; i < roll_outs.size(); i++)
    {
        lane_waypoint_marker.points.clear();
        lane_waypoint_marker.id = count;
        // ROS_INFO_STREAM("Number of points in roll out " << i << ": " << roll_outs[i].size());

        for(int j = 0; j < roll_outs[i].size(); j++)
        {            
            geometry_msgs::Point point;
            point.x = roll_outs[i][j].x;
            point.y = roll_outs[i][j].y;
            point.z = 0;
            lane_waypoint_marker.points.push_back(point);
        }

        lane_waypoint_marker.color.a = 0.9;
        lane_waypoint_marker.color.r = 0.0;
        lane_waypoint_marker.color.g = 0.0;
        lane_waypoint_marker.color.b = 1.0;

        markerArray.markers.push_back(lane_waypoint_marker);
        count++;
    }

    roll_outs_rviz_pub.publish(markerArray);
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

void KarcherLocalPlannerNode::visualizeWeightedTrajectories(const std::vector<std::vector<Waypoint>>& paths, const std::vector<TrajectoryCost>& traj_costs, const int& iClosest)
{
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = "map";
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "weighted_trajectories_colored";
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.05;
    lane_waypoint_marker.scale.y = 0.05;
    //lane_waypoint_marker.scale.z = 0.1;
    lane_waypoint_marker.color.a = 0.9;
    lane_waypoint_marker.color.r = 1.0;
    lane_waypoint_marker.color.g = 1.0;
    lane_waypoint_marker.color.b = 1.0;
    lane_waypoint_marker.frame_locked = false;

    int count = 0;
    for(int i = 0; i < paths.size(); i++)
    {
        lane_waypoint_marker.points.clear();
        lane_waypoint_marker.id = count;

        for(int j = 0; j < paths[i].size(); j++)
        {
            geometry_msgs::Point point;

            point.x = paths[i][j].x;
            point.y = paths[i][j].y;
            // point.z = paths.at(i).at(j).pos.z;

            lane_waypoint_marker.points.push_back(point);
        }

        lane_waypoint_marker.color.b = 0;

        if(traj_costs.size() == paths.size())
        {
            float norm_cost = traj_costs[i].cost * paths.size();
            if(norm_cost <= 1.0)
            {
                lane_waypoint_marker.color.r = norm_cost;
                lane_waypoint_marker.color.g = 1.0;
            }
            else if(norm_cost > 1.0)
            {
                lane_waypoint_marker.color.r = 1.0;
                lane_waypoint_marker.color.g = 2.0 - norm_cost;
            }
        }
        else
        {
            lane_waypoint_marker.color.r = 1.0;
            lane_waypoint_marker.color.g = 0.0;
        }

        if(traj_costs[i].bBlocked)
        {
            lane_waypoint_marker.color.r = 1.0;
            lane_waypoint_marker.color.g = 0.0;
            lane_waypoint_marker.color.b = 0.0;
        }

        if(i == iClosest)
        {
            lane_waypoint_marker.color.r = 1.0;
            lane_waypoint_marker.color.g = 0.0;
            lane_waypoint_marker.color.b = 1.0;
        }

        markerArray.markers.push_back(lane_waypoint_marker);
        count++;
    }
    weighted_trajectories_rviz_pub.publish(markerArray);
}

void KarcherLocalPlannerNode::visualizeSafetyBox(const std::vector<Waypoint>& safety_box)
{
    visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = "map";
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "safety_box_rviz";
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.05;
    lane_waypoint_marker.scale.y = 0.05;
    //lane_waypoint_marker.scale.z = 0.1;
    lane_waypoint_marker.frame_locked = false;
    lane_waypoint_marker.color.r = 0.0;
    lane_waypoint_marker.color.g = 1.0;
    lane_waypoint_marker.color.b = 1.0;
    lane_waypoint_marker.color.a = 0.6;

    for(int i = 0; i < safety_box.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = safety_box[i].x;
        p.y = safety_box[i].y;
        // p.z = safety_rect.at(i).z;

        lane_waypoint_marker.points.push_back(p);
    }

    if(safety_box.size() > 0)
    {
        geometry_msgs::Point p;
        p.x = safety_box[0].x;
        p.y = safety_box[0].y;
        // p.z = safety_rect.at(0).z;
        lane_waypoint_marker.points.push_back(p);
    }

   safety_box_rviz_pub.publish(lane_waypoint_marker);
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
        wp.speed = 0;

        global_path.push_back(wp);
    }

    visualizeGlobalPath();
}

void KarcherLocalPlannerNode::extractGlobalPathSection(std::vector<Waypoint>& extracted_path)
{
    if(global_path.size() < 2) return;

    extracted_path.clear();

    Waypoint car_pos;
    car_pos.x = current_state.x; car_pos.y = current_state.y; car_pos.heading = current_state.yaw;
    int closest_index = getClosestNextWaypointIndex(global_path, car_pos);

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
    visualizeExtractedPath(extracted_path);
}

int KarcherLocalPlannerNode::getClosestNextWaypointIndex(const std::vector<Waypoint>& path, const Waypoint& current_pos)
{   
    double d = 0, min_d = DBL_MAX;
    int min_index = prev_closest_index;

    for(int i = prev_closest_index; i < path.size(); i++)
    {
        d = distance2pointsSqr(path[i], current_pos);
        double angle_diff = angleBetweenTwoAnglesPositive(path[i].heading, current_pos.heading)*RAD2DEG;

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

//   ROS_INFO("Number of iterations: %d", nIterations);

  path = smoothed_path;
}

double KarcherLocalPlannerNode::calculateAngleAndCost(std::vector<Waypoint> &path)
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

void KarcherLocalPlannerNode::generateRollOuts(const std::vector<Waypoint>& path, std::vector<std::vector<Waypoint>>& roll_outs)
{
    // std::cout << "path size: " << path.size() << std::endl;
    if(path.size() == 0) return;
    if(MAX_LOCAL_PLAN_DISTANCE_ <= 0) return;
    roll_outs.clear();

    int i_limit_index = (SAMPLING_TIP_MARGIN_/0.3)/PATH_DENSITY_;
    if(i_limit_index >= path.size())
        i_limit_index = path.size() - 1;
    // std::cout << "i_limit_index: " << i_limit_index << std::endl;

    int closest_index;
    double initial_roll_in_distance;
    
    RelativeInfo info;
    Waypoint car_pos;
    car_pos.x = current_state.x; car_pos.y = current_state.y; car_pos.heading = current_state.yaw;
    getRelativeInfo(path, car_pos, info);
    initial_roll_in_distance = info.perp_distance;
    // std::cout << "closest_index: " << closest_index << std::endl;
    // std::cout << "initial_roll_in_distance: " << initial_roll_in_distance << std::endl;

    double remaining_distance = 0;
    for(int i = 0; i < path.size()-1; i++)
    {
        remaining_distance += distance2points(path[i], path[i+1]);
    }
    // std::cout << "remaining_distance: " << remaining_distance << std::endl;

    // calculate the starting index
    double d_limit = 0;
    int start_index = 0; 
    int end_index = 0;
    // int far_index = closest_index;

    // calculate end index
    double start_distance = ROLL_IN_SPEED_FACTOR_ * current_state.speed + ROLL_IN_MARGIN_;
    if(start_distance > remaining_distance)
        start_distance = remaining_distance;
    // std::cout << "start_distance: " << start_distance << std::endl;

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
    // std::cout << "start_index: " << start_index << ", end_index: " << end_index << ", smoothing_start_index: " 
    //             << smoothing_start_index << ", smoothing_end_index: " << smoothing_end_index << std::endl;

    int nSteps = end_index - smoothing_start_index;
    // std::cout << "nSteps: " << nSteps << std::endl;

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
        predictConstantTimeCostForTrajectory(roll_outs[i]);
    }

    visualizeRollOuts(roll_outs);
}

bool KarcherLocalPlannerNode::getRelativeInfo(const std::vector<Waypoint>& path, const Waypoint& current_pos, RelativeInfo& info)
{
    if(path.size() < 2) return false;

    Waypoint p0, p1;
    if(path.size() == 2)
    {
        p0 = path[0];
        p1.x = (path[0].x+path[1].x)/2.0;
        p1.y = (path[0].y+path[1].y)/2.0;
        p1.heading = path[0].heading;
        info.front_index = 1;
        info.back_index = 0;
    }
    else
    {
        info.front_index = getClosestNextWaypointIndex(path, current_pos);

        if(info.front_index > 0)
            info.back_index = info.front_index - 1;
        else
            info.back_index = 0;

        if(info.front_index == 0)
        {
            p0 = path[info.front_index];
            p1 = path[info.front_index+1];
        }
        else if(info.front_index > 0 && info.front_index < path.size()-1)
        {
            p0 = path[info.front_index-1];
            p1 = path[info.front_index];
        }
        else
        {
            p0 = path[info.front_index-1];
            p1.x = (p0.x+path[info.front_index].x)/2.0;
            p1.y = (p0.y+path[info.front_index].y)/2.0;
            p1.heading = p0.heading;
        }
    }

    Waypoint prevWP = p0;
    Mat3 rotationMat(-p1.heading);
    Mat3 translationMat(-current_pos.x, -current_pos.y);
    Mat3 invRotationMat(p1.heading);
    Mat3 invTranslationMat(current_pos.x, current_pos.y);

    p0 = translationMat*p0;
    p0 = rotationMat*p0;
    p1 = translationMat*p1;
    p1 = rotationMat*p1;

    // p0.x = p0.x - current_state.x;
    // p0.y = p0.y - current_state.y;
    // // std::cout << "After translation: p0.x: " << p0.x << ", p0.y: " << p0.y << std::endl;
    // double x = p0.x;
    // p0.x = cos(-p1.heading)*p0.x - sin(-p1.heading)*p0.y;
    // p0.y = sin(-p1.heading)*x + cos(-p1.heading)*p0.y;
    // // std::cout << "After rotation: p0.x: " << p0.x << ", p0.y: " << p0.y << std::endl;
    // // std::cout << "p1.heading: " << p1.heading << std::endl;
    
    // p1.x = p1.x - current_state.x;
    // p1.y = p1.y - current_state.y;
    // x = p1.x;
    // p1.x = cos(-p1.heading)*p1.x - sin(-p1.heading)*p1.y;
    // p1.y = sin(-p1.heading)*x + cos(-p1.heading)*p1.y;

    double m = (p1.y-p0.y)/(p1.x-p0.x);
    info.perp_distance = p1.y - m*p1.x; // solve for x = 0
    // std::cout << "m: " << m << std::endl;

    if(std::isnan(info.perp_distance) || std::isinf(info.perp_distance)) info.perp_distance = 0;

    info.to_front_distance = fabs(p1.x); // distance on the x axes

    info.perp_point = p1;
    info.perp_point.x = 0; // on the same y axis of the car
    info.perp_point.y = info.perp_distance; //perp distance between the car and the trajectory

    info.perp_point = invRotationMat * info.perp_point;
    info.perp_point = invTranslationMat * info.perp_point;

    info.from_back_distance = hypot(info.perp_point.y - prevWP.y, info.perp_point.x - prevWP.x);

    info.angle_diff = angleBetweenTwoAnglesPositive(p1.heading, current_pos.heading)*RAD2DEG;

    return true;
}

void KarcherLocalPlannerNode::predictConstantTimeCostForTrajectory(std::vector<Waypoint>& path)
{
    if(path.size() == 0) return;

    for(int i = 0 ; i < path.size(); i++)
        path[i].time_cost = -1;

    // TODO: less than ?? (min threshold)    
    if(current_state.speed < 0.1) return;

    RelativeInfo info;
    Waypoint car_pos;
    car_pos.x = current_state.x; car_pos.y = current_state.y; car_pos.heading = current_state.yaw;
    getRelativeInfo(path, car_pos, info);

    double total_distance = 0;
    double accum_time = 0;

    path[info.front_index].time_cost = 0;
    if(info.front_index == 0) info.front_index++;

    for(int i = info.front_index; i < path.size(); i++)
    {
        total_distance += hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
        accum_time = total_distance / current_state.speed;
        path[i].time_cost = accum_time;
    }
}

TrajectoryCost KarcherLocalPlannerNode::doOneStepStatic(const std::vector<std::vector<Waypoint>>& roll_outs, const std::vector<Waypoint>& extracted_path, std::vector<TrajectoryCost>& trajectory_costs)
{
    TrajectoryCost bestTrajectory;
    bestTrajectory.bBlocked = true;
    bestTrajectory.closest_obj_distance = HORIZON_DISTANCE_;
    bestTrajectory.closest_obj_velocity = 0;
    bestTrajectory.index = -1;

    RelativeInfo car_info;
    Waypoint car_pos;
    car_pos.x = current_state.x; car_pos.y = current_state.y; car_pos.heading = current_state.yaw;
    getRelativeInfo(extracted_path, car_pos, car_info);
    int curr_index = ROLL_OUTS_NUMBER_/2 + floor(car_info.perp_distance/ROLL_OUT_DENSITY_);
    //std::cout <<  "Current Index: " << curr_index << std::endl;
    if(curr_index < 0)
        curr_index = 0;
    else if(curr_index > ROLL_OUTS_NUMBER_)
        curr_index = ROLL_OUTS_NUMBER_;

    trajectory_costs.clear();
    if(roll_outs.size() > 0)
    {
        TrajectoryCost tc;
        int central_index = ROLL_OUTS_NUMBER_/2;
        tc.lane_index = 0;
        for(int it = 0; it < roll_outs.size(); it++)
        {
            tc.index = it;
            tc.relative_index = it - central_index;
            tc.distance_from_center = ROLL_OUT_DENSITY_*tc.relative_index;
            tc.priority_cost = fabs(tc.distance_from_center);
            tc.closest_obj_distance = HORIZON_DISTANCE_;
            // if(roll_outs[it].size() > 0)
            //     tc.lane_change_cost = roll_outs[it][0].lane_change_cost;
            trajectory_costs.push_back(tc);
        }
    }

    calculateTransitionCosts(trajectory_costs, curr_index);

    // // obstacle contour points
    // Waypoint p;
    // std::vector<Waypoint> contour_points;
    // // m_AllContourPoints.clear();
    // for(int io = 0; io < obj_list.size(); io++)
    // {
    //     for(int icon = 0; icon < obj_list[io].contour.size(); icon++)
    //     {
    //         p.pos = obj_list[io].contour[icon];
    //         p.v = obj_list[io].center.v;
    //         p.id = io;
    //         p.cost = sqrt(obj_list.at(io).w*obj_list.at(io).w + obj_list.at(io).l*obj_list.at(io).l);
    //         contour_points.push_back(p);
    //     }
    // }

    calculateLateralAndLongitudinalCostsStatic(trajectory_costs, roll_outs, extracted_path);

    normalizeCosts(trajectory_costs);

    int smallestIndex = -1;
    double smallestCost = DBL_MAX;
    double smallestDistance = DBL_MAX;
    double velo_of_next = 0;

    //cout << "Trajectory Costs Log : CurrIndex: " << currIndex << " --------------------- " << endl;
    for(int ic = 0; ic < trajectory_costs.size(); ic++)
    {
        //cout << m_TrajectoryCosts.at(ic).ToString();
        if(!trajectory_costs[ic].bBlocked && trajectory_costs[ic].cost < smallestCost)
        {
            smallestCost = trajectory_costs[ic].cost;
            smallestIndex = ic;
        }

        if(trajectory_costs[ic].closest_obj_distance < smallestDistance)
        {
            smallestDistance = trajectory_costs[ic].closest_obj_distance;
            velo_of_next = trajectory_costs[ic].closest_obj_velocity;
        }
    }
    //cout << "Smallest Distance: " <<  smallestDistance << "------------------------------------------------------------- " << endl;

    if(smallestIndex == -1)
    {
        bestTrajectory.bBlocked = true;
        bestTrajectory.lane_index = 0;
        bestTrajectory.index = -1; // TODO
        bestTrajectory.closest_obj_distance = smallestDistance;
        bestTrajectory.closest_obj_velocity = velo_of_next;
    }
    else if(smallestIndex >= 0)
    {
        bestTrajectory = trajectory_costs[smallestIndex];
    }

    visualizeWeightedTrajectories(roll_outs, trajectory_costs, smallestIndex);

    // m_PrevIndex = currIndex;
    return bestTrajectory;
}

void KarcherLocalPlannerNode::calculateTransitionCosts(std::vector<TrajectoryCost>& trajectory_costs, const int& curr_trajectory_index)
{
    for(int ic = 0; ic < trajectory_costs.size(); ic++)
    {
        trajectory_costs[ic].transition_cost = fabs(ROLL_OUT_DENSITY_ * (ic - curr_trajectory_index));
    }
}

void KarcherLocalPlannerNode::calculateLateralAndLongitudinalCostsStatic(std::vector<TrajectoryCost>& trajectory_costs,
    const std::vector<std::vector<Waypoint>>& roll_outs, const std::vector<Waypoint>& extracted_path)
{
    double critical_lateral_distance = VEHICLE_WIDTH_/2.0 + HORIZONTAL_SAFETY_DISTANCE_;
    double critical_long_front_distance = WHEELBASE_LENGTH_/2.0 + VEHICLE_LENGTH_/2.0 + VERTICAL_SAFETY_DISTANCE_;
    double critical_long_back_distance = VEHICLE_LENGTH_/2.0 + VERTICAL_SAFETY_DISTANCE_ - WHEELBASE_LENGTH_/2.0;

    Mat3 invRotationMat(current_state.yaw - M_PI_2);
    Mat3 invTranslationMat(current_state.x, current_state.y);

    double corner_slide_distance = critical_lateral_distance/2.0;
    double ratio_to_angle = corner_slide_distance/MAX_STEER_ANGLE_;
    double slide_distance = current_state.steer * ratio_to_angle;

    Waypoint bottom_left;
    bottom_left.x = -critical_lateral_distance;
    bottom_left.y = -critical_long_back_distance;
    bottom_left.heading = 0;

    Waypoint bottom_right;
    bottom_right.x = critical_lateral_distance;
    bottom_right.y = -critical_long_back_distance;
    bottom_right.heading = 0;

    Waypoint top_right_car;
    top_right_car.x = critical_lateral_distance;
    top_right_car.y = WHEELBASE_LENGTH_/3.0 + VEHICLE_LENGTH_/3.0;
    top_right_car.heading = 0;

    Waypoint top_left_car;
    top_left_car.x = -critical_lateral_distance;
    top_left_car.y = WHEELBASE_LENGTH_/3.0 + VEHICLE_LENGTH_/3.0;
    top_left_car.heading = 0;

    Waypoint top_right;
    top_right.x = critical_lateral_distance - slide_distance;
    top_right.y = critical_long_front_distance;
    top_right.heading = 0;

    Waypoint top_left;
    top_left.x = -critical_lateral_distance - slide_distance;
    top_left.y = critical_long_front_distance;
    top_left.heading = 0;

    bottom_left = invRotationMat*bottom_left;
    bottom_left = invTranslationMat*bottom_left;

    top_right = invRotationMat*top_right;
    top_right = invTranslationMat*top_right;

    bottom_right = invRotationMat*bottom_right;
    bottom_right = invTranslationMat*bottom_right;

    top_left = invRotationMat*top_left;
    top_left = invTranslationMat*top_left;

    top_right_car = invRotationMat*top_right_car;
    top_right_car = invTranslationMat*top_right_car;

    top_left_car = invRotationMat*top_left_car;
    top_left_car = invTranslationMat*top_left_car;

    safety_box.points.clear();
    safety_box.points.push_back(bottom_left);
    safety_box.points.push_back(bottom_right);
    safety_box.points.push_back(top_right_car);
    safety_box.points.push_back(top_right);
    safety_box.points.push_back(top_left);
    safety_box.points.push_back(top_left_car);

    visualizeSafetyBox(safety_box.points);

    // int iCostIndex = 0;
    if(roll_outs.size() > 0 && roll_outs[0].size() > 0)
    {
        RelativeInfo car_info;
        Waypoint car_pos;
        car_pos.x = current_state.x; car_pos.y = current_state.y; car_pos.heading = current_state.yaw;
        getRelativeInfo(extracted_path, car_pos, car_info);

        for(int it = 0; it < roll_outs.size(); it++)
        {
            // int skip_id = -1;
            for(int io = 0; io < obstacle_waypoints.size(); io++)
            {
            //    if(skip_id == contour_points[icon].id) continue;
                std::cout << "Checking obstacles..." << std::endl;
                RelativeInfo obj_info;
                obstacle_waypoints[io].heading = car_pos.heading;
                getRelativeInfo(extracted_path, obstacle_waypoints[io], obj_info);
                double longitudinalDist = getExactDistanceOnTrajectory(extracted_path, car_info, obj_info);
                // std::cout << "Car_info front_index: " << car_info.front_index << std::endl;
                // std::cout << "Obj_info front_index: " << obj_info.front_index << std::endl;
                // std::cout << "Longitudinal distance: " << longitudinalDist << std::endl;
                if(obj_info.front_index == 0 && longitudinalDist > 0)
                    longitudinalDist = -longitudinalDist;
                // std::cout << "Longitudinal distance: " << longitudinalDist << std::endl;

                double direct_distance = hypot(obj_info.perp_point.y-obstacle_waypoints[io].y, obj_info.perp_point.x-obstacle_waypoints[io].x);
                // if(contour_points[icon].v < MIN_SPEED_ && direct_distance > (LATERAL_SKIP_DISTANCE_+contour_points[icon].cost))
                // {
                //     skip_id = contour_pPoints[icon].id;
                //     continue;
                // }

                // double close_in_percentage = 1;
                // close_in_percentage = ((longitudinalDist- critical_long_front_distance)/params.rollInMargin)*4.0;
    
                // if(close_in_percentage <= 0 || close_in_percentage > 1) close_in_percentage = 1;

                double distance_from_center = trajectory_costs[it].distance_from_center;

                // if(close_in_percentage < 1)
                //     distance_from_center = distance_from_center - distance_from_center * (1.0 - close_in_percentage);

                double lateralDist = fabs(obj_info.perp_distance - distance_from_center);
                std::cout << "Lateral distance: " << lateralDist << std::endl;

                if(longitudinalDist < -VEHICLE_LENGTH_ || longitudinalDist > MIN_FOLLOWING_DISTANCE_|| lateralDist > LATERAL_SKIP_DISTANCE_)
                {
                    continue;
                }

                longitudinalDist = longitudinalDist - critical_long_front_distance;
                std::cout << "Longitudinal distance: " << longitudinalDist << std::endl;

                if(safety_box.PointInsidePolygon(safety_box, obstacle_waypoints[io]) == true)
                    std::cout << "Point inside polygon!!" << std::endl;
                    trajectory_costs[it].bBlocked = true;

                if(lateralDist <= critical_lateral_distance && longitudinalDist >= -VEHICLE_LENGTH_/1.5 && longitudinalDist < MIN_FOLLOWING_DISTANCE_)
                    trajectory_costs[it].bBlocked = true;

                if(lateralDist != 0)
                    trajectory_costs[it].lateral_cost = 1.0/lateralDist;

                if(longitudinalDist != 0)
                    trajectory_costs[it].longitudinal_cost = 1.0/fabs(longitudinalDist);
                    std::cout << "Longitudinal cost: " << trajectory_costs[it].longitudinal_cost << std::endl;


                if(longitudinalDist >= -critical_long_front_distance && longitudinalDist < trajectory_costs[it].closest_obj_distance)
                {
                    trajectory_costs[it].closest_obj_distance = longitudinalDist;
                    trajectory_costs[it].closest_obj_velocity = 0; // TODO: obstacle_waypoints[io].v;
                }
            }

            // iCostIndex++;
        }
    }
}

double KarcherLocalPlannerNode::getExactDistanceOnTrajectory(const std::vector<Waypoint>& trajectory, const RelativeInfo& p1, const RelativeInfo& p2)
{
    if(trajectory.size() == 0) return 0;

    if(p2.front_index == p1.front_index && p2.back_index == p1.back_index)
    {
        return p2.to_front_distance - p1.to_front_distance;
    }
    else if(p2.back_index >= p1.front_index)
    {
        double d_on_path = p1.to_front_distance + p2.from_back_distance;
        for(int i = p1.front_index; i < p2.back_index; i++)
            d_on_path += hypot(trajectory[i+1].y - trajectory[i].y, trajectory[i+1].x - trajectory[i].x);

        return d_on_path;
    }
    else if(p2.front_index <= p1.back_index)
    {
        double d_on_path = p1.from_back_distance + p2.to_front_distance;
        for(int i = p2.front_index; i < p1.back_index; i++)
            d_on_path += hypot(trajectory[i+1].y - trajectory[i].y, trajectory[i+1].x - trajectory[i].x);

        return -d_on_path;
    }

    return 0;
}

void KarcherLocalPlannerNode::normalizeCosts(std::vector<TrajectoryCost>& trajectory_costs)
{
    //Normalize costs
    double totalPriorities = 0;
    double totalChange = 0;
    double totalLateralCosts = 0;
    double totalLongitudinalCosts = 0;
    double transitionCosts = 0;

    for(int ic = 0; ic < trajectory_costs.size(); ic++)
    {
        totalPriorities += trajectory_costs[ic].priority_cost;
        transitionCosts += trajectory_costs[ic].transition_cost;
    }

    for(int ic = 0; ic < trajectory_costs.size(); ic++)
    {
        // totalChange += trajectory_costs[ic].lane_change_cost;
        totalLateralCosts += trajectory_costs[ic].lateral_cost;
        totalLongitudinalCosts += trajectory_costs[ic].longitudinal_cost;
    }

    //  cout << "------ Normalizing Step " << endl;
    for(int ic = 0; ic < trajectory_costs.size(); ic++)
    {
        if(totalPriorities != 0)
            trajectory_costs[ic].priority_cost = trajectory_costs[ic].priority_cost / totalPriorities;
        else
            trajectory_costs[ic].priority_cost = 0;

        if(transitionCosts != 0)
            trajectory_costs[ic].transition_cost = trajectory_costs[ic].transition_cost / transitionCosts;
        else
            trajectory_costs[ic].transition_cost = 0;

        // if(totalChange != 0)
        //     trajectory_costs[ic].lane_change_cost = trajectory_costs[ic].lane_change_cost / totalChange;
        // else
        //     trajectory_costs[ic].lane_change_cost = 0;

        if(totalLateralCosts != 0)
            trajectory_costs[ic].lateral_cost = trajectory_costs[ic].lateral_cost / totalLateralCosts;
        else
            trajectory_costs[ic].lateral_cost = 0;

        if(totalLongitudinalCosts != 0)
            trajectory_costs[ic].longitudinal_cost = trajectory_costs[ic].longitudinal_cost / totalLongitudinalCosts;
        else
            trajectory_costs[ic].longitudinal_cost = 0;

        trajectory_costs[ic].cost = (PRIORITY_WEIGHT_*trajectory_costs[ic].priority_cost + TRANSITION_WEIGHT_*trajectory_costs[ic].transition_cost + LAT_WEIGHT_*trajectory_costs[ic].lateral_cost + LONG_WEIGHT_*trajectory_costs[ic].longitudinal_cost)/4.0;

        std::cout << "Index: " << ic
               << ", Priority: " << trajectory_costs[ic].priority_cost
               << ", Transition: " << trajectory_costs[ic].transition_cost
               << ", Lat: " << trajectory_costs[ic].lateral_cost
               << ", Long: " << trajectory_costs[ic].longitudinal_cost
            //    << ", Change: " << trajectory_costs.at(ic).lane_change_cost
               << ", Avg: " << trajectory_costs[ic].cost
               << std::endl;
    }

    std::cout << "------------------------ " << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "karcher_local_planner_node");
    KarcherLocalPlannerNode karcher_local_planner_obj;  
    ros::spin();
    return 0;
}