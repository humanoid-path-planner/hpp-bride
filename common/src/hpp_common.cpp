// ROS message includes
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Empty.h>

/* protected region user include files on begin */
/* protected region user include files end */

class hpp_config
{
public:
    std::string urdfDescription;
    std::string srdfDescription;
};

class hpp_data
{
// autogenerated: don't touch this class
public:
    //input data
    sensor_msgs::JointState in_initConfig;
    sensor_msgs::JointState in_goalConfig;
    //output data
    sensor_msgs::JointState out_path;
    bool out_path_active;
};

class hpp_impl
{
    /* protected region user member variables on begin */
    /* protected region user member variables end */

public:
    hpp_impl() 
    {
        /* protected region user constructor on begin */
        /* protected region user constructor end */
    }

    void configure(hpp_config config) 
    {
        /* protected region user configure on begin */
        /* protected region user configure end */
    }

    void update(hpp_data &data, hpp_config config)
    {
        /* protected region user update on begin */
        /* protected region user update end */
    }

    bool callback_loadRobotModel(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res , hpp_config config)
    {
        /* protected region user implementation of service callback for loadRobotModel on begin */
        /* protected region user implementation of service callback for loadRobotModel end */
        return true;
    }
    bool callback_solve(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res , hpp_config config)
    {
        /* protected region user implementation of service callback for solve on begin */
        /* protected region user implementation of service callback for solve end */
        return true;
    }
    bool callback_loadObstacle(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res , hpp_config config)
    {
        /* protected region user implementation of service callback for loadObstacle on begin */
        /* protected region user implementation of service callback for loadObstacle end */
        return true;
    }

    /* protected region user additional functions on begin */
    /* protected region user additional functions end */
};
