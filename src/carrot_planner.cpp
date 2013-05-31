#include "tue_carrot_planner/carrot_planner.h"

CarrotPlanner::CarrotPlanner(const std::string &name) :
    tracking_frame_("/base_link"), t_last_cmd_vel_(ros::Time::now().toSec()), laser_data_available_(false), visualization_(true) {

    ros::NodeHandle private_nh("~/" + name);

    //! Get parameters from the ROS parameter server
    private_nh.param("max_vel_translation", MAX_VEL, 0.5);
    private_nh.param("max_acc_translation", MAX_ACC, 0.15);
    private_nh.param("max_vel_rotation", MAX_VEL_THETA, 0.4);
    private_nh.param("max_acc_rotation", MAX_ACC_THETA, 0.35);
    private_nh.param("gain", GAIN, 0.9);
    private_nh.param("max_angle", MAX_ANGLE, 1.0/2.0*3.14159);
    private_nh.param("dist_vir_wall", DISTANCE_VIRTUAL_WALL, 0.65);
    private_nh.param("radius_robot", RADIUS_ROBOT, 0.5);

    //! Publishers
    if (visualization_) {
        virt_wall_pub_ = private_nh.advertise<sensor_msgs::LaserScan>("virtual_wall", 1);
        carrot_pub_ = private_nh.advertise<visualization_msgs::Marker>("carrot", 1);
    }
    cmd_vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    //! Listen to laser data
    laser_scan_sub_ = private_nh.subscribe("/base_scan", 10, &CarrotPlanner::laserScanCallBack, this);
    
    tf_listener_ = new tf::TransformListener();

}


CarrotPlanner::~CarrotPlanner() {
	
	delete tf_listener_;
}


bool CarrotPlanner::MoveToGoal(geometry_msgs::PoseStamped &goal){

    //! Velocity that will be published
    geometry_msgs::Twist cmd_vel;

    //! Set goal
    if (setGoal(goal)) {

        //! Compute velocity command and publish this command
        if(computeVelocityCommand(cmd_vel)) {

            ROS_DEBUG("Publishing velocity command: (x,y,th) = (%f.%f,%f)", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
            cmd_vel_pub_.publish(cmd_vel);

            return true;
        }
    }

    return false;
}

bool CarrotPlanner::setGoal(geometry_msgs::PoseStamped &goal){

    //! Check frame of the goal
    if (goal.header.frame_id != tracking_frame_){
        ROS_ERROR("Expecting goal in frame %s, no planning possible", tracking_frame_.c_str());
        return false;
    }

    //! Determine pose of the goal
    goal_angle_ = tf::getYaw(goal.pose.orientation);
    goal_.setX(goal.pose.position.x);
    goal_.setY(goal.pose.position.y);
    goal_.setZ(goal.pose.position.z);

    // TODO: Check if this makes sense
    //! Check for maximum angle, if needed move y-position goal such that truncated angle matches desired (x,y)-position
    if (goal_angle_ > MAX_ANGLE) {
        ROS_WARN("Angle %f > %f: will be truncated", goal_angle_, MAX_ANGLE);
        //ROS_WARN("CHECK IF THIS IS DESIRED: Update y from %f to %f", goal_.getY(), goal.pose.position.x*tan(MAX_ANGLE));
        //goal_angle_ = MAX_ANGLE;
        //goal_.setY(goal.pose.position.x*tan(MAX_ANGLE));
    } else if (goal_angle_ < -MAX_ANGLE) {
		ROS_WARN("Angle %f < %f: will be truncated", goal_angle_, -MAX_ANGLE);
        //ROS_WARN("CHECK IF THIS IS DESIRED: Update y from %f to %f", goal_.getY(), goal.pose.position.x*tan(-MAX_ANGLE));
        //goal_angle_ = -MAX_ANGLE;
        //goal_.setY(-goal.pose.position.x*tan(MAX_ANGLE));
	}

    ROS_DEBUG("CarrotPlanner::setGoal: (x,y,th) = (%f,%f,%f)", goal_.getX(), goal_.getY(), goal_angle_);

    //! Publish marker
    if (visualization_) publishCarrot(goal_, carrot_pub_);

    return true;

}

bool CarrotPlanner::computeVelocityCommand(geometry_msgs::Twist &cmd_vel){

    //! Determine dt since last callback
    double time = ros::Time::now().toSec() + ros::Time::now().toNSec() / 1e9;
    double dt = 0;
    if (t_last_cmd_vel_ > 0) {
        dt = time - t_last_cmd_vel_;
    }
    t_last_cmd_vel_ = time;
    ROS_DEBUG("Goal before is clear line (%f,%f,%f)", goal_.getX(), goal_.getY(), goal_angle_);

    //! Check if the path is free
    if(!isClearLine()) {
        ROS_WARN("Path is not free: only consider rotation");
        setZeroVelocity(cmd_vel);
        goal_.setX(0);
        goal_.setY(0);
        goal_.setZ(0);
    }

    ROS_DEBUG("Goal before normalization is (%f,%f,%f)", goal_.getX(), goal_.getY(), goal_angle_);

    //! Normalize position
    tf::Vector3 goal_norm;
    if (goal_.length() > 1e-6) {
        goal_norm = goal_.normalized();
    } else {
        goal_norm = goal_;
    }

    //! Convert to message
    geometry_msgs::Twist goal_norm_msg;
    tf::vector3TFToMsg(goal_norm, goal_norm_msg.linear);

    //! Determine velocity
    determineDesiredVelocity(dt, cmd_vel);
    last_cmd_vel_ = cmd_vel;

    return true;
}

bool CarrotPlanner::isClearLine(){

    //! Check if laser data is avaibale
    if (!laser_data_available_) {
        ROS_INFO("No laser data available: path considered blocked");
        return false;
    }
    
    //! Transform goal angle to frame base laser
    // TODO: This causes unstable behavior
    double angle_goal = goal_angle_;
    /*
    tf::Stamped<tf::Point> goal_pos(tf::Point(goal_.getX(), goal_.getY(), 0), ros::Time(), tracking_frame_);
    try{
        tf::Stamped<tf::Point> goal_pos_trans;
        tf_listener_->transformPoint("/front_laser", goal_pos, goal_pos_trans);
        angle_goal = atan2(goal_pos_trans.getY(), goal_pos_trans.getX());
        ROS_INFO("After transformation: updated angle from %f to %f", goal_angle_, angle_goal);
        

    } catch (tf::TransformException ex){
        ROS_WARN("Path check in carrot planner - transformPosition(): %s",ex.what());
    }*/
    
    //! Get number of beams
    int num_readings = laser_scan_.ranges.size();

    //! Calculate the index corresponding to the beam that intersects with the target position
    int num_incr = angle_goal/laser_scan_.angle_increment; // Both in rad
    ROS_DEBUG("wall: angle %f corresponds to %d increments", angle_goal, num_incr);
    int index_beam_target_pos = std::max(0, num_readings/2 + num_incr);

    //! Check for collisions with virtual wall in front of the robot
    double dth = atan2(RADIUS_ROBOT, DISTANCE_VIRTUAL_WALL);
    int d_step = dth/laser_scan_.angle_increment;


    //double angle_beam_obs = laser_scan_.angle_min + index_beam_target_pos * laser_scan_.angle_increment;
    //ROS_INFO("Angle beam obstable is %f [deg] or %f [rad]", angle_beam_obs/3.1415*180.0, angle_beam_obs);
    //ROS_INFO("d_step is %d", d_step);
    //ROS_INFO("Middle beam has angle %f", laser_scan_.angle_min + num_readings/2 * laser_scan_.angle_increment);

    //! For visualizing the virtual wall
    sensor_msgs::LaserScan wall_msg;
    if (visualization_) {
        wall_msg = laser_scan_;
        wall_msg.ranges.clear();
        wall_msg.intensities.clear();
        ROS_DEBUG("wall: index beam is %d, d_step is %d, num_readings is %d", index_beam_target_pos, d_step, num_readings);
        ROS_DEBUG("wall: offsets are %d and %d", std::max(index_beam_target_pos - d_step,0), std::min(num_readings, index_beam_target_pos + d_step));
        wall_msg.angle_min = laser_scan_.angle_min + std::max(index_beam_target_pos - d_step,0) * laser_scan_.angle_increment;
        wall_msg.angle_max = laser_scan_.angle_min + std::min(num_readings, index_beam_target_pos + d_step) * laser_scan_.angle_increment;
        ROS_DEBUG("wall: from angle %f to %f", wall_msg.angle_min, wall_msg.angle_max);
    }

    // TODO: virtual force: decrease/increase y-coordinate goal using goal_.setY(SOME_GAIN*(goal_.getY()-dy));

    //! Check for objects in front of virtual wall
    bool path_free = true;
    for (int j = std::max(index_beam_target_pos - d_step,0); j < index_beam_target_pos + d_step; ++j) {
        if (j < num_readings) {
            double dist_to_obstacle = laser_scan_.ranges[j];

			if (visualization_) {
				wall_msg.ranges.push_back(DISTANCE_VIRTUAL_WALL);
				wall_msg.intensities.push_back(100);
			}

            if (dist_to_obstacle > 0.001 && dist_to_obstacle < DISTANCE_VIRTUAL_WALL) {

                double angle = laser_scan_.angle_min + j * laser_scan_.angle_increment;
                double dy = sin(angle)*dist_to_obstacle;
                if (path_free) ROS_WARN("Object too close: %f [m], dy = %f", dist_to_obstacle, dy);
                path_free = false;
            }
        }
    }

    if (visualization_) virt_wall_pub_.publish(wall_msg);

    return path_free;
}



void CarrotPlanner::laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_scan){

    //! Only consider front laser (isn't this covered by selecting topic?)
    if(laser_scan->header.frame_id == "/front_laser"){
        laser_scan_ = *laser_scan;
        laser_data_available_ = true;
    }
}


double CarrotPlanner::calculateHeading(const tf::Vector3 &goal) {
    return atan2(goal.getY(), goal.getX());
}

void CarrotPlanner::determineDesiredVelocity(double dt, geometry_msgs::Twist &cmd_vel) {

    //! Determine errors
    tf::Vector3 error_lin = goal_;
    double error_ang = goal_angle_;
    ROS_DEBUG("Desired angle is %f", goal_angle_);

    //! Current command vel
    const geometry_msgs::Twist current_vel = last_cmd_vel_;
    tf::Vector3 current_vel_trans;
    tf::vector3MsgToTF(current_vel.linear, current_vel_trans);

    //! Normalize error
    double error_lin_norm = error_lin.length();

    //! Determine normalized velocity
    double v_desired_norm = 0;
    if (error_lin_norm > 0) {
        v_desired_norm = std::min(MAX_VEL, GAIN * sqrt(2 * error_lin_norm * MAX_ACC));
        ROS_DEBUG(" updated v_wanted_norm to %f", v_desired_norm);
    } else {
        ROS_DEBUG(" zeros normalized error: v_wanted_norm is 0 too");
    }

    //! Make sure the desired velocity has the direction towards the goal and the magnitude of v_desired_norm
    tf::Vector3 vel_desired;
    if (error_lin.length() > 0) {
        vel_desired = error_lin.normalized();
    } else {
        vel_desired.setX(0);
        vel_desired.setY(0);
        vel_desired.setZ(0);
    }
    vel_desired *= v_desired_norm;

    //! Check if the acceleration bound is violated
    tf::Vector3 vel_diff = vel_desired - current_vel_trans;
    double acc_desired = vel_diff.length() / dt;
    ROS_DEBUG(" vel_diff = (%f,%f,%f), acc_desired = %f", vel_diff.getX(), vel_diff.getY(), vel_diff.getZ(), acc_desired);
    if (acc_desired > MAX_ACC) {
        tf::vector3TFToMsg(current_vel_trans + vel_diff.normalized() * MAX_ACC * dt, cmd_vel.linear);
    } else {

        //! P-action: scale velocity with distance
        double distance = sqrt(error_lin.getX()*error_lin.getX() + error_lin.getY()*error_lin.getY());
        vel_desired = std::min(vel_desired, distance * 2.0/3.0 * vel_desired);
        tf::vector3TFToMsg(vel_desired, cmd_vel.linear);
        if (std::min(distance * 2.0/3.0, 1.0) < 1.0) {
            ROS_DEBUG("Lowered velocity by a factor %f", std::min(distance * 2.0/3.0, 1.0));
        }
    }

    //! The rotation is always controlled
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = determineReference(error_ang, current_vel.angular.z, MAX_VEL_THETA, MAX_ACC_THETA, dt);

    //! P-action: scale angular velocity with distance
    double angular_vel_calc = cmd_vel.angular.z;
    cmd_vel.angular.z = std::min(fabs(angular_vel_calc), fabs(error_ang*10.0/3.1415*angular_vel_calc));
    if (angular_vel_calc < 0) cmd_vel.angular.z *= -1;
    ROS_DEBUG("Adapted angular velocity from %f to %f for theta is %f", angular_vel_calc, cmd_vel.angular.z, goal_angle_);

    ROS_DEBUG("Final velocity command: (x:%f, y:%f, th:%f)", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
}

// PARTLY TAKEN FROM amigo_ref_interpolator
double CarrotPlanner::determineReference(double error_x, double vel, double max_vel, double max_acc, double dt) {
    double EPS = 0.5 * max_acc*dt;

    //initial state
    bool still = false;
    bool move = false;
    bool dec = false;
    bool con = false;
    bool acc = false;

    //double a = 0.0;
    double vel_mag = fabs(vel);

    //compute deceleration distance
    double delta_t1=vel_mag/max_acc; //deceleration segment time
    double dec_dist = 0.5*max_acc * (delta_t1) * (delta_t1); //deceleration distance

    //determine magnitude and sign of error vector
    double delta_x = fabs(error_x);
    int sign_x = sign(vel);

    //decide whether to move or stand still
    if (vel_mag!=0.0 || delta_x > EPS){
        move = true;
    } else {
        still = true;
        error_x = 0;
    }
    double dir = sign(error_x);

    //move: decide whether to stop, decelerate, constant speed or accelerate
    if (move){
        //		if (stopping){
        //			acc = false;
        //			con = false;
        //			still = false;
        //			dec = true;
        //			///ROS_ERROR("stopping");
        //       	} else
        if (fabs(dec_dist) >= fabs(delta_x)){
            dec = true;
            // ROS_INFO("go to dec");
        }
        else if (sign_x * error_x < 0 && vel_mag != 0.0){
            dec = true;
            // ROS_INFO("setpoint behind");
        }
        else if (fabs(dec_dist) < fabs(delta_x) && vel_mag >= max_vel){
            con = true;
            // ROS_INFO("go to con");
        }
        else {
            acc = true;
            // ROS_INFO("go to acc");
        }
        //move: reference value computations
        if (acc){
            vel_mag += max_acc * dt;
            vel_mag = std::min<double>(vel_mag, max_vel);
            //x+= dir * vel_mag * dt;
            //a = dir * max_acc;
        }
        if (con){
            //x+= dir * vel_mag * dt;
            //a = 0.0;
        }
        if (dec){
            vel_mag -= max_acc * dt;
            vel_mag = std::max<double>(vel_mag, 0.0);
            //x+= dir * vel_mag * dt;
            //a = - dir * max_acc;
            if (vel_mag < (0.5 * max_acc * dt)){
                vel_mag = 0.0;
                //reset = true;
                ///ROS_WARN("reset");
            }
        }
        //ready = false;
    }

    //stand still: reset values
    else if (still){
        vel = 0;
        //a = 0.0;
        sign_x = 0;
        //reset = true;
        //ready = true;
    }
    else {
    }

    vel = dir * vel_mag;
    ROS_DEBUG("Rotational velocity is %f", vel);

    return vel;
}

void CarrotPlanner::setZeroVelocity(geometry_msgs::Twist& cmd_vel) {
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
}

void CarrotPlanner::publishCarrot(const tf::Vector3& carrot, ros::Publisher& pub) {

    //! Create a marker message for the plan
    visualization_msgs::Marker marker;
    marker.header.frame_id = tracking_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "carrot";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.05;
    marker.color.r = 1;
    marker.color.g = 0.5;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(0.0);
    geometry_msgs::Point p1,p2;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0.05;
    p2.x = carrot.getX();
    p2.y = carrot.getY();
    p2.z = 0.05;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    pub.publish(marker);

}
