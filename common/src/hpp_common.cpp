// ROS message includes
#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Empty.h>

/* protected region user include files on begin */
#include <ros/console.h>
#include <hpp/model/device.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/model/urdf/util.hh>
#include <hpp/ros/joint-state.hh>
#include <hpp/ros/joint-trajectory.hh>

using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::Configuration_t;
using hpp::core::ConfigurationPtr_t;
using hpp::core::PathVectorPtr_t;
using hpp::core::PathVectors_t;
using hpp::core::ProblemSolver;
using hpp::core::ProblemSolverPtr_t;

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
  trajectory_msgs::JointTrajectory out_path;
  bool out_path_active;
};

class hpp_impl
{
  /* protected region user member variables on begin */
private:
  ProblemSolverPtr_t problemSolver_;
  ros::Time lastUpdateInitConfig_;
  ros::Time lastUpdateGoalConfig_;
  ConfigurationPtr_t initConfig_;
  ConfigurationPtr_t goalConfig_;
  bool needToExportPath_;
  /* protected region user member variables end */

public:
  hpp_impl()
  {
    /* protected region user constructor on begin */
    problemSolver_ = new ProblemSolver;
    lastUpdateInitConfig_.sec = 0;
    lastUpdateInitConfig_.nsec = 0;
    lastUpdateGoalConfig_.sec = 0;
    lastUpdateGoalConfig_.nsec = 0;
    needToExportPath_ = false;
     /* protected region user constructor end */
  }

  void configure(hpp_config)
  {
    /* protected region user configure on begin */
    /* protected region user configure end */
  }

  void update(hpp_data &data, hpp_config)
  {
    /* protected region user update on begin */
    data.out_path_active = false;
    /// Update initial and goal configurations if needed
    DevicePtr_t robot = problemSolver_->robot ();
    if (robot) {
      if (data.in_initConfig.header.stamp != lastUpdateInitConfig_) {
        hpp::ros::jointStateToConfig (robot, data.in_initConfig, *initConfig_);
      }
      if (data.in_goalConfig.header.stamp != lastUpdateGoalConfig_) {
        hpp::ros::jointStateToConfig (robot, data.in_goalConfig, *goalConfig_);
      }
    }
    // Export path if Service solve has been called with success
    if (needToExportPath_) {
      // Get latest computed path
      const PathVectors_t& paths = problemSolver_->paths ();
      PathVectorPtr_t path = paths [paths.size () - 1];
      hpp::ros::pathVectorToJointTrajectory (robot, path, data.out_path);
      data.out_path.header.stamp = ros::Time::now();
      ++data.out_path.header.seq;
      data.out_path_active = true;
      needToExportPath_ = false;
    }
    /* protected region user update end */
  }

  bool callback_loadRobotModel(std_srvs::Empty::Request &,
      std_srvs::Empty::Response &, hpp_config config)
  {
    /* protected region user implementation of service callback for loadRobotModel on begin */
    DevicePtr_t robot = Device::create("robot");
    try
    {
      hpp::model::urdf::loadRobotModelFromParameter(robot, "anchor",
          config.urdfDescription, config.srdfDescription);
      problemSolver_->robot(robot);
      // Allocate init and goal configurations
      initConfig_ = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
      initConfig_->setZero ();
      goalConfig_->setZero ();
      goalConfig_ = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
    } catch (const std::exception& exc)
    {
      ROS_DEBUG("%s", exc.what());
      return false;
    }
    /* protected region user implementation of service callback for loadRobotModel end */
    return true;
  }
  bool callback_solve(std_srvs::Empty::Request &,
      std_srvs::Empty::Response &, hpp_config)
  {
    /* protected region user implementation of service callback for solve on begin */
    try
    {
      problemSolver_->solve();
    } catch (const std::exception& exc)
    {
      ROS_DEBUG("%s", exc.what());
      return false;
    }
    needToExportPath_ = true;
    /* protected region user implementation of service callback for solve end */
    return true;
  }
  bool callback_loadObstacle(std_srvs::Empty::Request &,
      std_srvs::Empty::Response &, hpp_config)
  {
    /* protected region user implementation of service callback for loadObstacle on begin */
    /* protected region user implementation of service callback for loadObstacle end */
    return true;
  }

  /* protected region user additional functions on begin */
  /* protected region user additional functions end */
};
