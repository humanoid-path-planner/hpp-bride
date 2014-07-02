// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <hpp_node/hppConfig.h>

// ROS message includes
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Empty.h>

// other includes
#include <hpp_common.cpp>


class hpp_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<hpp_node::hppConfig> server;
    dynamic_reconfigure::Server<hpp_node::hppConfig>::CallbackType f;

    ros::Publisher path_;
    ros::Subscriber initConfig_;
    ros::Subscriber goalConfig_;
    ros::ServiceServer loadRobotModel_;
    ros::ServiceServer solve_;
    ros::ServiceServer loadObstacle_;

    hpp_data component_data_;
    hpp_config component_config_;
    hpp_impl component_implementation_;

    hpp_ros() : np_("~")
    {
        f = boost::bind(&hpp_ros::configure_callback, this, _1, _2);
        server.setCallback(f);

        std::string loadRobotModel_remap;
        n_.param("loadRobotModel_remap", loadRobotModel_remap, (std::string)"loadRobotModel");
        loadRobotModel_ = n_.advertiseService<std_srvs::Empty::Request , std_srvs::Empty::Response>(loadRobotModel_remap, boost::bind(&hpp_impl::callback_loadRobotModel, &component_implementation_,_1,_2,component_config_));
        std::string solve_remap;
        n_.param("solve_remap", solve_remap, (std::string)"solve");
        solve_ = n_.advertiseService<std_srvs::Empty::Request , std_srvs::Empty::Response>(solve_remap, boost::bind(&hpp_impl::callback_solve, &component_implementation_,_1,_2,component_config_));
        std::string loadObstacle_remap;
        n_.param("loadObstacle_remap", loadObstacle_remap, (std::string)"loadObstacle");
        loadObstacle_ = n_.advertiseService<std_srvs::Empty::Request , std_srvs::Empty::Response>(loadObstacle_remap, boost::bind(&hpp_impl::callback_loadObstacle, &component_implementation_,_1,_2,component_config_));

        path_ = n_.advertise<trajectory_msgs::JointTrajectory>("path", 1);
        initConfig_ = n_.subscribe("initConfig", 1, &hpp_ros::topicCallback_initConfig, this);
        goalConfig_ = n_.subscribe("goalConfig", 1, &hpp_ros::topicCallback_goalConfig, this);

        np_.param("urdfDescription", component_config_.urdfDescription, (std::string)"");
        np_.param("srdfDescription", component_config_.srdfDescription, (std::string)"");



    }
    void topicCallback_initConfig(const sensor_msgs::JointState::ConstPtr& msg)
    {
        component_data_.in_initConfig = *msg;
    }
    void topicCallback_goalConfig(const sensor_msgs::JointState::ConstPtr& msg)
    {
        component_data_.in_goalConfig = *msg;
    }

    void configure_callback(hpp_node::hppConfig &config, uint32_t level)
    {
        component_config_.urdfDescription = config.urdfDescription;
        component_config_.srdfDescription = config.srdfDescription;
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_path_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_path_active)
            path_.publish(component_data_.out_path);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "hpp");

    hpp_ros node;
    node.configure();

 // if cycle time == 0 do a spin() here without calling node.update()
    ros::spin();

    return 0;
}
