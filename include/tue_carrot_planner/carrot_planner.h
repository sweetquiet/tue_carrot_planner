#ifndef CARROT_PLANNER_H_
#define CARROT_PLANNER_H_
#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>

class CarrotPlanner
{

public:

    CarrotPlanner(const std::string &name, double max_vel_lin = 0.75, double max_vel_rot = 0.4, double dist_wall = 0.65, bool allow_rotate_only = true);

    ~CarrotPlanner();

    bool MoveToGoal(geometry_msgs::PoseStamped &goal);

    void freeze();

private:

    bool setGoal(geometry_msgs::PoseStamped& goal);

    bool computeVelocityCommand(geometry_msgs::Twist& cmd_vel);

    void setZeroVelocity(geometry_msgs::Twist& cmd_vel);

    double sign(double x){
        return (x < 0.0)?-1.0:1.0;
    }

    void determineDesiredVelocity(double dt, geometry_msgs::Twist& cmd_vel);

    double determineReference(double error_x, double vel, double max_vel, double max_acc, double dt);

    bool isClearLine();

    void laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_scan);

    double calculateHeading(const tf::Vector3& goal);

    void publishCarrot(const tf::Vector3& carrot, ros::Publisher& pub);

    void publishCmdVel(const geometry_msgs::Twist& cmd_vel, ros::Publisher& pub);

    //! ROS parameters
    double MAX_VEL;
    double MAX_ACC;
    double MAX_VEL_THETA;
    double MAX_ACC_THETA;
    double MIN_VEL_THETA;
    double GAIN;
    double MAX_ANGLE;
    double DISTANCE_VIRTUAL_WALL;
    double RADIUS_ROBOT;
    double MIN_ANGLE_ZERO_TRANS;

    //! Tracking frame and transform listener
    std::string tracking_frame_;
    tf::TransformListener* tf_listener_;

    //! Goal position and angle
    tf::Vector3 goal_;
    double goal_angle_;

    //! Timestamp and value of last time cmd_vel was published
    double t_last_cmd_vel_;
    geometry_msgs::Twist last_cmd_vel_;
    bool allow_rotate_only_;

    //! Comminucation
    ros::Publisher carrot_pub_, cmd_vel_pub_, virt_wall_pub_;
    ros::Subscriber laser_scan_sub_;

    //! Laser data
    sensor_msgs::LaserScan laser_scan_;
    bool laser_data_available_;

    //! Visualization
    bool visualization_;

};

#endif
